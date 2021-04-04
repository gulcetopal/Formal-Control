#!/usr/bin/env python

import rospy
import rosparam
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
import math
import numpy as np
from numpy import linalg as la
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
import os

from policy_executor import PolicyExecutor
from action_executor import ActionExecutor
from prism_talker import PrismTalker
import math
import numpy as np
import tf
import csv
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import LaserScan, Imu
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelStates
from formal_control.msg import SelfStateMsg, PathMsg
from visualization_msgs.msg import Marker, MarkerArray

class pure_pursuit:

    def __init__(self):
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.StateCallback)
        rospy.Subscriber("/traffic_topic", SelfStateMsg, self.TrafficCallback)
        rospy.Subscriber("/path_data", PathMsg, self.PathCallback)

        self.VELOCITY    = 1.5 # m/s
        self.H2_VELOCITY = 0.5 # m/s
        self.read_waypoints()

        self.LOOKAHEAD_DISTANCE = 1 * self.VELOCITY # meters
        self.ld_th = 0.08

        self.angle = 0
        self.velocity = 1.0
        self.angularz = 0

        self.model_name = 'Husky_h1'
        self.latest_state_data = []
        self.policy = []
        self.old_policy = []
        self.path_data = []

        self.got_new_plan = True
        self.target_reached = False

        self.msg = Twist()
        self.twist = Twist()
        self.ts_msg = SelfStateMsg()

    	#Publishers
    	self.goal_pub = rospy.Publisher('/waypoint/goal', Point, queue_size=1)
        self.twist_pub = rospy.Publisher("/h1/husky_velocity_controller/cmd_vel", Twist, queue_size = 50)
        self.h2_pub = rospy.Publisher("/h2/husky_velocity_controller/cmd_vel", Twist, queue_size = 50)
        self.timestep_pub = rospy.Publisher("/traffic_robot_state", SelfStateMsg, queue_size = 50)

    def PathCallback(self,data):
        self.path_data = data.x_start

    def StateCallback(self,data):
        self.latest_state_data = data

    def TrafficCallback(self,data):
        self.policy = data.policy

        if self.old_policy != self.policy:
            self.got_new_plan = True
            self.old_policy = self.policy
            print("New Policy Calculated !")

    def read_waypoints(self):
        self.path_points_x = []
        self.path_points_y = []
        self.path_points_w = []
        self.dist_arr = []
        self.path_dir = rospy.get_param('Directory/path')
        filename = os.path.join(self.path_dir)
        with open(filename) as f:
            path_points = [tuple(line) for line in csv.reader(f)]

        # Turn path_points into a list of floats to eliminate the need for casts in the code below.
        self.path_points_x   = [float(point[0]) for point in path_points]
        self.path_points_y   = [float(point[1]) for point in path_points]
        self.path_points_w   = [float(point[2]) for point in path_points]

        self.dist_arr= np.zeros(len(self.path_points_y))

    def dist(self, p1, p2):
        return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def baby_driver(self):
        qx=self.latest_state_data.pose[self.latest_state_data.name.index(self.model_name)].orientation.x
        qy=self.latest_state_data.pose[self.latest_state_data.name.index(self.model_name)].orientation.y
        qz=self.latest_state_data.pose[self.latest_state_data.name.index(self.model_name)].orientation.z
        qw=self.latest_state_data.pose[self.latest_state_data.name.index(self.model_name)].orientation.w

        quaternion = (qx,qy,qz,qw)
        euler   = euler_from_quaternion(quaternion)
        yaw     = euler[2]

        x = self.latest_state_data.pose[self.latest_state_data.name.index(self.model_name)].position.x
        y = self.latest_state_data.pose[self.latest_state_data.name.index(self.model_name)].position.y

        self.path_points_x = np.array(self.path_points_x)
        self.path_points_y = np.array(self.path_points_y)

        for i in range(len(self.path_points_x)):
            self.dist_arr[i] = self.dist((self.path_points_x[i],self.path_points_y[i]),(x,y))

        goal = len(self.dist_arr)-1

        goal_arr = np.where((self.dist_arr < self.LOOKAHEAD_DISTANCE+self.ld_th)&(self.dist_arr > self.LOOKAHEAD_DISTANCE-self.ld_th))[0]
        #print("Goal Arr: " + str(goal_arr))
        for idx in goal_arr:
            v1 = [self.path_points_x[idx]-x , self.path_points_y[idx]-y]
            v2 = [np.cos(yaw), np.sin(yaw)]
            temp_angle = self.find_angle(v1,v2)
            #print("Temp Ang: " + str(temp_angle))
            if abs(temp_angle) < np.pi/2:
                goal = idx
                break


        L = self.dist_arr[goal]

        print("\n")
        print("X: " + str(x))
        print("X Goal = " + str(self.path_points_x[goal]))
        print("\n")
        print("Y: " + str(y))
        print("Y Goal = " + str(self.path_points_y[goal]))
        print("\n")
        gvcx = self.path_points_x[goal] - x
        gvcy = self.path_points_y[goal] - y
        goal_x_veh_coord = gvcx*np.cos(yaw) + gvcy*np.sin(yaw)
        goal_y_veh_coord = gvcy*np.cos(yaw) - gvcx*np.sin(yaw)

        #alpha = self.path_points_w[goal] - (yaw)
        alpha = math.atan(gvcy/gvcx) - (yaw)

        k = 2 * math.sin(alpha)/L
        angle_i = math.atan(k*0.4)
        self.angularz = k*self.velocity
        #print("\n")
        #print("Alpha: " + str(math.degrees(alpha)))
        print("Angular Velocity: " + str(math.degrees(self.angularz)))

        angle = angle_i*2
        #angle = np.clip(angle, -0.4189, 0.4189) # 0.4189 radians = 24 degrees because car can only turn 24 degrees max

        left_mid_th = 0.3
        right_mid_th = 0.3
        stp = 0.1

        if abs(y + 2.745) < right_mid_th or y < -2.9:
            self.ts_msg.timestep = 3
            print("Current Lane: Right")
        elif abs(y + 2.1) < right_mid_th:
            self.ts_msg.timestep = 6
            print("Current Lane: Mid")
        elif abs(y + 1.45) < right_mid_th or y > -1.2:
            self.ts_msg.timestep = 9
            print("Current Lane: Left")
        else:
            print("Current Lane: -")
            self.ts_msg.got_new_plan = self.got_new_plan
        self.timestep_pub.publish(self.ts_msg)


        #self.set_speed(angle)
        self.const_speed(angle)

    	goalPoint = Point(float(goal_x_veh_coord),float(goal_y_veh_coord),float(angle));
    	self.goal_pub.publish(goalPoint)

        # print functions for DEBUGGING
       # print(self.path_points_x[goal],self.path_points_y[goal],self.path_points_w[goal])
       # print(x,y,180*yaw/math.pi)
       # print(goal_y_veh_coord,angle)
       # print(self.LOOKAHEAD_DISTANCE,self.msg.velocity)
       # print("*******")

    def send_command(self):
        self.msg.linear.x = self.velocity
        self.msg.linear.y = 0
        self.msg.angular.z = self.angularz

    	self.twist_pub.publish(self.msg)
        self.go_forward(self.H2_VELOCITY)

    def go_forward(self,speed = 0.5, secs = 0.15 ):
        rate = rospy.Rate(10)
        self.twist.linear.x = speed
        self.twist.angular.z = 0
        for i in range(int(secs * 10)):
            self.h2_pub.publish(self.twist)
            rate.sleep()

    def set_speed(self,angle):
        if (abs(angle)>0.2018):
            self.LOOKAHEAD_DISTANCE = 2
            # self.msg.velocity = 1.5
            self.angle = angle

            if self.velocity - 1.0 >= 0.5:
                self.velocity -= 0.5

        else:
            self.LOOKAHEAD_DISTANCE = 1.0
            # self.msg.velocity = 3.0
            self.angle = angle

            if self.VELOCITY - self.velocity > 0.2:
                self.velocity += 0.2

    def const_speed(self,angle):
        self.LOOKAHEAD_DISTANCE = 2
        self.angle = angle
        self.velocity = self.VELOCITY

    def find_angle(self, v1, v2):
        cosang = np.dot(v1, v2)
        sinang = la.norm(np.cross(v1, v2))
        return np.arctan2(sinang, cosang)


if __name__ == '__main__':
    rospy.init_node('main_driver') # Main driver with PRISM and PP application
    PP = pure_pursuit()
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        if PP.latest_state_data != []:
            #if PP.got_new_plan:
            if PP.path_data != []:
                PP.read_waypoints()
                PP.baby_driver()

                PP.send_command()
        rate.sleep()
