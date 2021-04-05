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
from formal_control.msg import SelfStateMsg
from visualization_msgs.msg import Marker, MarkerArray

class Robot():
    def __init__(self, twist_pub = "/h1/husky_velocity_controller/cmd_vel", scan_topic = " ", odom_topic = " ", imu_topic = " ", model_name = "Husky_h1", entity_name = "base_link"):
        self.marker_pub = rospy.Publisher("/marker", Marker, queue_size = 50)
        self.twist_pub = twist_pub
        self.scan_topic = scan_topic
        self.odom_topic = odom_topic
        self.imu_topic = imu_topic
        self.goal_tolerance = 0.2
        self.x = 0
        self.y = 0
        self.x_target = 0
        self.y_target = 0
        self.xr = 0
        self.yr = 0
        self.l = 5
        self.yaw = 0
        self.twist = Twist()
        self.latest_scan = None
        self.dist = 0
        self.model_name = model_name
        self.entity_name = entity_name
        self.fdist = 0
        self.bdist = 0
        self.vx = 0
        self.coeff = 0
        self.v_refx = 5
        self.v_refy = 0
        self.policy = []
        self.timestep = 0
        self.y_tol = 0.2

        self.main()

# Callback Funcs

    def Odomcallback(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        self.x = x
        self.y = y

##################

    def fix_yaw(self, yaw):
        while not (yaw < math.pi and yaw >= -math.pi):
            if yaw >= math.pi:
                yaw -= math.pi * 2
            elif yaw < -math.pi:
                yaw += math.pi * 2
            else:
                break
        return yaw

    def visualize_points(self):
        node_point = Marker()
        node_point.type = 2
        node_point.header.frame_id = "odom"
        node_point.ns = "lookahead_points"
        #node_point.id = node.index
        node_point.action = 0
        node_point.lifetime.secs = 0
        node_point.pose.position.x = self.xr
        node_point.pose.position.y = self.yr
        node_point.pose.position.z = 0.1
        node_point.pose.orientation.w = 1
        node_point.scale.x, node_point.scale.y, node_point.scale.z = 0.2, 0.2, 0.2
        node_point.color.r = 255
        node_point.color.g = 0

        self.marker_pub.publish(node_point)

# self Actions
    def reset_twist(self):
        self.twist.linear.x  = 0
        self.twist.linear.y  = 0
        self.twist.linear.z  = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0
        self.twist_pub.publish(self.twist)

    def control_dist(self, goal):
        self.dist = float(math.sqrt(math.pow((goal[0] - self.x), 2) + math.pow((goal[1] - self.y), 2)))
        return self.dist > self.goal_tolerance

    def turn(self, speed, secs = 0.15):
        rate = rospy.Rate(10)
        self.twist.angular.z = speed
        for i in range(int(secs * 10)):
            self.twist_pub.publish(self.twist)
            rate.sleep()
        self.reset_twist()

    def go_forward(self,speed = 0.5, secs = 0.15 ):
        rate = rospy.Rate(10)
        self.twist.linear.x = speed
        self.twist.angular.z = 0
        for i in range(int(secs * 10)):
                self.twist_pub.publish(self.twist)
                rate.sleep()
        #print("Distance to target: " + str(self.dist))

    def yanal_go_forward(self, angularz, speedx = 0.5, speedy = 0.2, secs = 0.15 ):
        rate = rospy.Rate(5)
        self.twist.linear.x = speedx
        self.twist.linear.y = speedy
        self.twist.angular.z = angularz

        #for i in range(int(secs * 10)):
        #print("Angular Z: "+str(math.degrees(self.twist.angular.z)))
        self.twist_pub.publish(self.twist)
        rate.sleep()
        #print("Distance to target: " + str(self.dist))

##################

# self Behaviours
    def free_ride(self):
        self.go_forward(self.v_refx)

    def sheer_in(self,angularz):
        self.yanal_go_forward(angularz,self.v_refx,self.v_refy,)

    def sheer_out(self,angularz):
        self.yanal_go_forward(angularz,self.v_refx,self.v_refy)

    def faster(self):
        inc = 0.2
        self.v_refx = self.v_refx+inc
        self.go_forward(self.v_refx)

    def slower(self):
        inc = -0.2
        self.v_refx = self.v_refx+inc
        self.go_forward(self.v_refx)

    def goal_point(self):
        #print("X Target: " + str(self.x_target))
        #print("Y Target: " + str(self.y_target))
        #print("\n")
        print("X: " + str(self.x))
        print("Y: " + str(self.y))
        print("\n")
        #print("Egim: "+str((self.y_target-self.y)/(self.x_target-self.x)))
        #print("ATAN: "+ str(math.degrees(self.fix_yaw(math.atan2((self.y_target-self.y),(self.x_target-self.x))))))
        self.yr = self.l*math.cos(self.fix_yaw(math.atan2((self.y_target-self.y),(self.x_target-self.x)))-math.pi/2)+ self.y
        if self.yr > -1.45:
            self.yr = -1.45
        elif self.yr < -2.75:
            self.yr = -2.75
        self.xr = self.l*math.sin(self.fix_yaw(math.atan2((self.y_target-self.y),(self.x_target-self.x))))+ self.x

        print("XR: " + str(self.xr))
        print("YR: " + str(self.yr))
        print("\n")
        print("Atan: " + str(math.sin(self.fix_yaw(math.atan2((self.y_target-self.y),(self.x_target-self.x))))))
        #self.visualize_points()

    def pure_pursuit(self):
        self.goal_point()
        alpha = self.yaw-self.fix_yaw((math.atan2((self.yr-self.y),(self.xr-self.x))))
        R = self.l/(2*math.sin(alpha+math.pi/2))
        angularz =  self.vx/R*1.3
        return angularz

    def action_plan_exec(self, timestep):
        self.timestep = timestep
        step = 8
        t = 0.15
        print("Current Timestep: " + str(self.timestep))
        print("Current State: " + str(self.policy[self.timestep]))
        if self.policy[self.timestep+1] == 0:
            if self.policy[self.timestep] == 0:
                self.v_refx = 1.2
                self.x_target = self.x + self.v_refx*t*step
                self.y_target = -2.745
                self.l = 10 #10
                print("AAA\n")
                #self.free_ride()
                self.sheer_out(self.pure_pursuit())
                self.timestep = self.timestep+1
            else:
                self.v_refx = 1.2
                self.x_target = self.x + self.v_refx*t*step
                self.y_target = -2.745
                self.l = 10 #10
                print("BBB\n")
                #self.slower()
                self.sheer_out(self.pure_pursuit())
                self.timestep = self.timestep+1

        elif self.policy[self.timestep+1] == 1:
            if self.policy[self.timestep] == 0:
                self.v_refx = 1.4
                self.x_target = self.x + self.v_refx*t*step
                self.y_target = -2.745
                self.l = 10 #10
                print("CCC\n")
                #self.faster()
                self.sheer_out(self.pure_pursuit())

            elif self.policy[self.timestep] == 1:
                self.v_refx = 1.2
                self.x_target = self.x + self.v_refx*t*step
                self.y_target = -2.745
                self.l = 12 #6
                print("DDD\n")
                #self.slower()
                self.sheer_out(self.pure_pursuit())
            else:
                self.v_refx = 1.2
                self.x_target = self.x + self.v_refx*t*step
                self.y_target = -2.745
                self.l = 12 #6
                print("EEE\n")
                self.sheer_in(self.pure_pursuit())

            if -2.745-self.y_tol <self.y<-2.745+self.y_tol :
                self.timestep = self.timestep+1

        elif self.policy[self.timestep+1] == 2:
            if self.policy[self.timestep] == 1:
                self.v_refx = 1.2
                self.x_target = self.x + self.v_refx*t*step
                self.y_target = -1.55
                self.l = 12 #6
                print("FFF\n")
                self.sheer_out(self.pure_pursuit())
            else:
                self.v_refx = 1.2
                self.x_target = self.x + self.v_refx*t*step
                self.y_target = -2.745
                self.l = 16 #6
                print("GGG\n")
                self.sheer_in(self.pure_pursuit())
            if -2.1-self.y_tol <self.y<-2.1+self.y_tol :
                self.timestep = self.timestep+1

        elif self.policy[self.timestep+1] == 3:
            if self.policy[self.timestep] == 2:
                self.v_refx = 1.2
                self.x_target = self.x + self.v_refx*t*step
                self.y_target = -1.45
                self.l = 12 #6
                print("HHH\n")
                self.sheer_out(self.pure_pursuit())
            else:
                self.v_refx = 1.4
                self.x_target = self.x + self.v_refx*t*step
                self.y_target = -1.45
                self.l = 10 #6
                #self.faster()
                print("III\n")
                self.sheer_out(self.pure_pursuit())
            if -1.45-self.y_tol <self.y<-1.45+self.y_tol :
                self.timestep = self.timestep+1


    def main(self):
        x = 0

class RobotsAction():
    def __init__(self):
        rospy.init_node("Driver_Mahmut")
        self.policy = []
        self.got_new_plan = False
        rospy.Subscriber("/traffic_topic", SelfStateMsg, self.policyCallback)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.StateCallback)
        self.main()

    def StateCallback(self,data):
        self.latest_state_data = data

    def policyCallback(self,data):
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

    def main(self):
        rate = rospy.Rate(1)
        #rospy.Subscriber(self.odom_topic, Odometry, self.Odomcallback)
        husky_1_pub = rospy.Publisher("/h1/husky_velocity_controller/cmd_vel", Twist, queue_size = 50)
        husky_2_pub = rospy.Publisher("/h2/husky_velocity_controller/cmd_vel", Twist, queue_size = 50)
        robot_state_pub = rospy.Publisher("/traffic_robot_state", SelfStateMsg, queue_size = 50)

        husky_1 = Robot(husky_1_pub, "/h1/scan", "/h1/odometry/filtered", "/h1/imu/data","Husky_h1")
        husky_2 = Robot(husky_2_pub, "/h2/scan", "/h2/odometry/filtered", "/h2/imu/data","Husky_h2")

        msg = SelfStateMsg()

        while not rospy.is_shutdown():
            if self.policy == []:
                rate.sleep()
            else:
                if self.got_new_plan or husky_1.timestep >= len(self.policy):
                    husky_1.timestep = 0

                if (husky_1.timestep != len(self.policy)-1):
                    self.get_params(husky_1,husky_2)
                    husky_1.policy = self.policy
                    husky_1.action_plan_exec(husky_1.timestep)
                    husky_2.go_forward(0.8)

                    msg.timestep = husky_1.timestep
                    robot_state_pub.publish(msg)
                    print("Current State: "+str(husky_1.policy[husky_1.timestep]))
                    #print("Target State: "+str(husky_1.policy[husky_1.timestep + 1]))
                    print("--------------------------------------")
                else:
                    self.get_params(husky_1,husky_2)
                    husky_1.timestep = husky_1.timestep-1
                    husky_1.action_plan_exec(husky_1.timestep)
                    husky_2.go_forward(0.8)

if __name__ == '__main__':
    RobotsAction()
