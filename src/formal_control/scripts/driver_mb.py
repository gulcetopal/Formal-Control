#!/usr/bin/env python

import rospy
import os
from policy_executor import PolicyExecutor
from action_executor import ActionExecutor
from prism_talker import PrismTalker
import math
import numpy as np
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import LaserScan, Imu
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelStates
from formal_control.msg import SelfStateMsg, PathMsg
from visualization_msgs.msg import Marker, MarkerArray
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
import actionlib


class Robot():
    def __init__(self, twist_pub = "/h1/husky_velocity_controller/cmd_vel", scan_topic = " ", odom_topic = " ", imu_topic = " ", model_name = "Husky_h1", entity_name = "base_link"):
        self.marker_pub = rospy.Publisher("/marker", Marker, queue_size = 50)
        self.path_est_pub = rospy.Publisher("/est_path_data", PathMsg, queue_size = 50)
        self.timestep_pub = rospy.Publisher("/traffic_robot_state", SelfStateMsg, queue_size = 50)
        self.twist_pub = twist_pub
        self.scan_topic = scan_topic
        self.odom_topic = odom_topic
        self.imu_topic = imu_topic

        self.coeff = 1
        self.v_refx = 1
        self.l = 2

        self.goal_tolerance = 0.2
        self.x = 0
        self.y = 0
        self.x_target = 0
        self.y_target = 0
        self.xr = 0
        self.yr = 0

        self.dist = 0
        self.yaw = 0
        self.angularz = 0
        self.msg = PathMsg()
        self.twist = Twist()
        self.ts_msg = SelfStateMsg()
        self.latest_scan = None
        self.dist = 0
        self.model_name = model_name
        self.entity_name = entity_name
        self.fdist = 0
        self.bdist = 0
        self.vx = 0
        self.v = 0
        self.v_refy = 0
        self.policy = []
        self.timestep = 0
        self.y_tol = 0.1
        self.m = []
        self.x_start = []
        self.x_finish = []
        self.y_start = []
        self.y_finish = []
        self.xr_list = []
        self.yr_list = []
        self.x_est_array = [0]
        self.y_est_array = [-2.745]
        self.stage = 0

        #Move base
        global client
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        
        self.main()


    #Callbacks
    def Odomcallback(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        self.x = x
        self.y = y

    #Actions
    def fix_yaw(self, yaw):
        while not (yaw < math.pi and yaw >= -math.pi):
            if yaw >= math.pi:
                yaw -= math.pi * 2
            elif yaw < -math.pi:
                yaw += math.pi * 2
            else:
                break
        return yaw

    def control_dist(self, goal):
        self.dist = float(math.sqrt(math.pow((goal[0] - self.x), 2) + math.pow((goal[1] - self.y), 2)))
        return self.dist > self.goal_tolerance
    
    def go_forward(self,speed = 0.5, secs = 0.15 ):
        rate = rospy.Rate(10)
        self.twist.linear.x = speed
        self.twist.angular.z = 0
        for i in range(int(secs * 10)):
                self.twist_pub.publish(self.twist)
                rate.sleep()
        #print("Distance to target: " + str(self.dist))
    '''
    def yanal_go_forward(self, angularz, speedx = 2 , speedy = 0, secs = 0.15 ):
        rate = rospy.Rate(5)
        self.twist.linear.x = speedx
        self.twist.linear.y = speedy
        self.twist.angular.z = angularz

        for i in range(int(secs * 10)):
            #print("Angular Z: " + str(math.degrees(self.twist.angular.z)))
            self.twist_pub.publish(self.twist)
            rate.sleep()
    '''

    def goal_point(self):
        rate = rospy.Rate(5)
        m = 0

        min_dist = 100

        for i in range(len(self.x_start)-1):
            x = self.x_start[i]
            y = self.y_start[i]
            #self.dist = abs((-m*self.x + self.y + m*x - y))/math.sqrt(m*m+1)
            self.dist = math.sqrt((x-self.x)*(x-self.x) + (y-self.y)*(y-self.y))
            if self.dist< min_dist:
                min_dist = self.dist

        if self.l < min_dist:
            l = min_dist+0.1
        else:
            l = self.l

        x = 0
        for i in range(len(self.x_start)-1):
            x = self.x_start[i]
            y = self.y_start[i]
            #self.dist = abs((-m*self.x + self.y + m*x - y))/math.sqrt(m*m+1)
            self.dist = math.sqrt((x-self.x)*(x-self.x) + (y-self.y)*(y-self.y))
            #print("Dist: "+ str(self.dist))
            if abs(self.dist-self.l)<0.08:
                self.xr_list.append(x)
                self.yr_list.append(y)

        #print("Xr List: " + str(self.xr_list))
        #print("Yr List: " + str(self.yr_list))

        if len(self.xr_list) is not 0:
            self.xr = max(self.xr_list)
            self.yr = self.yr_list[self.xr_list.index(self.xr)]
        else:
            rate.sleep()
            #self.xr = self.x_start[len(self.x_start)-1]
            #self.yr = self.y_start[len(self.y_start)-1]

        self.v = self.v_refx
        self.x_est_array.append(self.xr)
        self.y_est_array.append(self.yr)

        self.msg.x_start = self.x_est_array
        self.msg.y_start = self.y_est_array

        print("Current X: " + str(self.x))
        print("Current Y: " + str(self.y))
        print("\n")
    '''
    def pure_pursuit(self):
        self.goal_point()
        print("XR: " + str(self.xr))
        print("YR: " + str(self.yr))
        print("\n")

        atan = (math.atan2((self.yr-self.y),(self.xr-self.x)))
        alpha =  atan - self.yaw + math.pi/2
        alpha = self.fix_yaw(alpha)

        #print("Yaw_deg: " + str(math.degrees(self.yaw)))
        #print("atan_deg: " + str(math.degrees(atan)))
        #print("ALPHA_deg: " + str(math.degrees(alpha)))

        #R = self.l/(2*math.sin(alpha))

        if abs(self.yr - self.y) < 0.001:
            R = 100000
        else:
            R = self.l*self.l/(2*(self.yr-self.y))

        #print("R: " + str(R))

        self.angularz =  self.v/R
        #self.angularz =  2*self.v*math.sin(alpha)/self.l

        #print("V: " + str(self.v))
        if self.xr != 0 and self.yr != 0:
            self.yanal_go_forward(self.angularz, self.v_refx)
    ''' 

    def send_move_base(self, x, y , yaw ):
        
        goal=MoveBaseGoal()
        goal.target_pose.header.frame_id = "/odom"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0

        q = tf.transformations.quaternion_from_euler(0,0,yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3] 
      
        client.send_goal(goal)
        wait = client.wait_for_result()
        if wait:
            rospy.loginfo("Goal execution done!")
        else:
            rospy.loginfo("Problem!")

    def pause_move_base():
        global client
        goal=MoveBaseGoal()
        goal.target_pose.header.frame_id = "/odom"
        client.cancel_all_goals()
        client.cancel_goal()

    def action_plan_exec(self,path,policy):
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        
        self.timestep = 0
        rate = rospy.Rate(5)
        self.m = path.m
        self.x_start = path.x_start
        self.x_finish = path.x_finish
        self.y_start = path.y_start
        self.y_finish = path.y_finish
#        self.pure_pursuit()
        
        self.goal_point()
        print("XR: " + str(self.xr))
        print("YR: " + str(self.yr))
        print("\n")
        print("Policy: " + str(policy))

        print("Client is waiting")
        client.wait_for_server()
        print("Client is on")

        yaw = self.fix_yaw((math.atan2((self.yr-self.y),(self.xr-self.x))))
        self.send_move_base(self.xr, self.yr + 2.745, yaw)
        
        a = 1
        right_array = [0, 1, 2, 3, 4, 5]
        mid_array = [6, 7, 8]
        left_array = [9, 10, 11]

        left_mid_th = 0.2
        right_mid_th = 0.2
        stp = 0.1

        if abs(self.y + 2.745) < right_mid_th or self.y < -2.9:
            self.ts_msg.timestep = 0
            print("Current Lane: Right")
        elif abs(self.y + 2.1) < right_mid_th:
            self.ts_msg.timestep = 6
            print("Current Lane: Mid")
        elif abs(self.y + 1.45) < right_mid_th or self.y > -1.2:
            self.ts_msg.timestep = 9
            print("Current Lane: Left")
        else:
            print("Current Lane: Idk Dude :>")

        self.timestep_pub.publish(self.ts_msg)

    def main(self):        
        self.path_est_pub.publish(self.msg)


class RobotsAction():
    def __init__(self):
        rospy.init_node("driver")
        self.policy = []
        self.latest_state_data = []
        self.got_new_plan = False
        self.check = False
        self.yaw = 0
        self.ang_x = 0
        self.ang_y = 0
        self.ang_z = 0
        self.path_data = PathMsg()
        rospy.Subscriber("/traffic_topic", SelfStateMsg, self.policyCallback)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.StateCallback)
        rospy.Subscriber("/path_data", PathMsg, self.PathCallback)
        self.main()

    def StateCallback(self,data):
        self.latest_state_data = data

    def PathCallback(self,data):
        self.path_data = data

    def policyCallback(self,data):
        #print("Policy Sub OK")
        self.policy = data.policy
        self.got_new_plan = data.got_new_plan
        #self.v_refx = data.v_refx
        #self.yaw_ref = data.yaw_ref

    def fix_yaw(self, yaw):
        while not (yaw < math.pi and yaw >= -math.pi):
            if yaw >= math.pi:
                yaw -= math.pi * 2
            elif yaw < -math.pi:
                yaw += math.pi * 2
            else:
                break
        return yaw

    def get_params(self,robot_1,robot_2):
        robot_1.x = self.latest_state_data.pose[self.latest_state_data.name.index(robot_1.model_name)].position.x
        robot_2.x = self.latest_state_data.pose[self.latest_state_data.name.index(robot_2.model_name)].position.x
        robot_1.y = self.latest_state_data.pose[self.latest_state_data.name.index(robot_1.model_name)].position.y
        robot_2.y = self.latest_state_data.pose[self.latest_state_data.name.index(robot_2.model_name)].position.y
        robot_1.vx = self.latest_state_data.twist[self.latest_state_data.name.index(robot_1.model_name)].linear.x
        robot_2.vx = self.latest_state_data.twist[self.latest_state_data.name.index(robot_2.model_name)].linear.x
        i = self.latest_state_data.name.index(robot_1.model_name)
        [roll1, pitch1, robot_1.yaw] = euler_from_quaternion([self.latest_state_data.pose[i].orientation.x,self.latest_state_data.pose[i].orientation.y,self.latest_state_data.pose[i].orientation.z,self.latest_state_data.pose[i].orientation.w])
        robot_1.yaw = self.fix_yaw(robot_1.yaw)
        i = self.latest_state_data.name.index(robot_2.model_name)
        [roll2, pitch2, robot_2.yaw] = euler_from_quaternion([self.latest_state_data.pose[i].orientation.x,self.latest_state_data.pose[i].orientation.y,self.latest_state_data.pose[i].orientation.z,self.latest_state_data.pose[i].orientation.w])
        robot_2.yaw = self.fix_yaw(robot_2.yaw)
        print(robot_1.model_name)
        

    def check_risk(self, robot_1, robot_2):
        v_relative = robot_1.vx - robot_2.vx
        robot_1.msg.check = True

    def main(self):
        rate = rospy.Rate(5)
        husky_1_pub = rospy.Publisher("/h1/husky_velocity_controller/cmd_vel", Twist, queue_size = 50)
        husky_2_pub = rospy.Publisher("/h2/husky_velocity_controller/cmd_vel", Twist, queue_size = 50)
        robot_state_pub = rospy.Publisher("/traffic_robot_state", SelfStateMsg, queue_size = 50) 

        husky_1 = Robot(husky_1_pub, "/h1/scan", "/h1/odometry/filtered", "/h1/imu/data","Husky_h1")
        husky_2 = Robot(husky_2_pub, "/h2/scan", "/h2/odometry/filtered", "/h2/imu/data","Husky_h2")

        msg = SelfStateMsg()
        

        while not rospy.is_shutdown():
            
            if self.latest_state_data == []:
                rate.sleep()
            else:
                if self.policy == []:
                    rate.sleep()
                else:
                    self.get_params(husky_1,husky_2)
                    husky_1.action_plan_exec(self.path_data, self.policy)
                    #husky_2.go_forward(0.5)
                    #self.check_risk(husky_1, husky_2)

if __name__ == '__main__':
    RobotsAction()
