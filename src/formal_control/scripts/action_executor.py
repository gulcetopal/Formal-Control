#!/usr/bin/env python

import rospy
import os
from policy_executor import PolicyExecutor
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

class Robot():
    def __init__(self, twist_pub = "/h1/husky_velocity_controller/cmd_vel", scan_topic = " ", odom_topic = " ", imu_topic = " ", model_name = "Husky_h1", entity_name = "base_link"):
        self.twist_pub = twist_pub
        self.scan_topic = scan_topic
        self.odom_topic = odom_topic
        self.imu_topic = imu_topic

        self.x = 0
        self.y = 0
        self.yaw = 0
        self.twist = Twist()

        self.model_name = model_name
        self.entity_name = entity_name

        self.rfdist = 0
        self.lfdist = 0
        self.bdist = 0

        # Th değerleri tanım için var, adjust_threshold()dan değiştirilecek
        self.rfdist_th = 0
        self.lfdist_th = -1
        self.bdist_th = 0

        self.vx = 0
        self.vy = 0
        self.angularz = 0
        self.v_refx = 5
        self.v_refy = 0

        self.main()

#   Main
    def main(self):
        main_start = 1


class ActionExecutor():
    def __init__(self,policy = [0] ):
        rospy.init_node('action_executor')
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.StateCallback)
        rospy.Subscriber("/traffic_robot_state", SelfStateMsg, self.TimestepCallback)

        self.policy = policy
        self.policy_pub = rospy.Publisher("/traffic_topic", SelfStateMsg, queue_size = 50)

        self.old_policy = []
        self.v = 0

        self.timestep = 0
        self.step = 0
        self.state = 2 # start from state: wait for order

        self.got_new_plan = False
        self.check = False
        self.lane = 1

        self.FV = None
        self.BV = None

        self.latest_scan = None
        self.latest_state_data = None
        self.talker = PrismTalker()
        self.policy_exe = PolicyExecutor()
        self.msg = SelfStateMsg()

        self.main()

#   Callbacks and fixes
    def StateCallback(self,data):
        self.latest_state_data = data

    def TimestepCallback(self,data):
        self.timestep = data.timestep

    def fix_yaw(self, yaw):
        while not (yaw < math.pi and yaw >= -math.pi):
            if yaw >= math.pi:
                yaw -= math.pi * 2
            elif yaw < -math.pi:
                yaw += math.pi * 2
            else:
                break
        return yaw

#   Get parameters and policy
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

    def get_policy(self, robot_1):
        self.old_policy = self.policy
        self.got_new_plan = True
        self.policy_exe = PolicyExecutor(robot_1.rfdist,robot_1.lfdist,robot_1.bdist,self.msg.v_relative)
        self.policy_exe.SetInit(self.step, self.lane)
        self.policy_exe.Policy()
        self.policy = self.policy_exe.policy
        for i in range(10):
            self.policy.append(self.policy[len(self.policy)-1])
        self.msg.policy = self.policy
        self.policy_pub.publish(self.msg)

#   Check variables - environmental variables are controlled
    def check_distance(self,robot_1,robot_2):
        if robot_1.x >= robot_2.x:
            self.FV = robot_1
            self.BV = robot_2
        else:
            self.FV = robot_2
            self.BV = robot_1
        self.FV.rfdist = 100
        self.FV.bdist =  abs(self.FV.x- self.BV.x) #float(math.sqrt(math.pow((self.FV.x- self.BV.x), 2) + math.pow((self.BV.y - self.FV.y), 2)))
        self.BV.rfdist = self.FV.bdist
        self.BV.bdist = 100
        self.msg.rfdist = robot_1.rfdist
        self.msg.bdist = robot_1.bdist

        print("Front distance: "+ str(robot_1.rfdist))
        print("Back distance : "+ str(robot_1.bdist))
        print("\n")

    def check_velocity(self,robot_1,robot_2):
    	robot1v = robot_1.vx
        robot2v = robot_2.vx
        self.v = robot1v-robot2v
        self.msg.v_relative = self.v
        print("Husky 1 vx: "+ str(robot_1.vx))
        print("Husky 1 vy: "+ str(robot_1.vy))
        print("Relative vel: "+ str(self.v))

    def adjust_threshold(self,robot_1,robot_2):
        PRISM_TIME = 7
        OVERTAKE_TIME = 1

        prism_th = PRISM_TIME * self.v
        overtake_th = OVERTAKE_TIME * self.v #robot_1.vx #robot_1.vx * robot_1.vx / robot_2.vx #self.v
        robot_1.rfdist_th = prism_th + overtake_th

        if prism_th < robot_1.bdist_th:
            robot_1.bdist_th = 1 
        else:
            robot_1.bdist_th = 0.01
        

    def check_state(self, robot_1, timestep):
        right_array = [0, 1, 2, 3, 4, 5]
        mid_array = [6, 7, 8]
        left_array = [9, 10, 11]
        i = timestep

        if i in right_array:
            right_check = True
        else:
            right_check = False

        if i in mid_array:
            mid_check = True
        else:
            mid_check = False

        if i in left_array:
            left_check = True
        else:
            left_check = False

        if right_check:
            self.lane = 1
            if (robot_1.rfdist > robot_1.rfdist_th and robot_1.lfdist > robot_1.lfdist_th and robot_1.bdist > robot_1.bdist_th):
                self.step = 0
            elif(robot_1.rfdist > robot_1.rfdist_th and robot_1.lfdist > robot_1.lfdist_th and robot_1.bdist < robot_1.bdist_th):
                self.step = 1
            elif(robot_1.rfdist > robot_1.rfdist_th and robot_1.lfdist < robot_1.lfdist_th and robot_1.bdist > robot_1.bdist_th):
                self.step = 2
            elif(robot_1.rfdist < robot_1.rfdist_th and robot_1.lfdist > robot_1.lfdist_th and robot_1.bdist > robot_1.bdist_th):
                self.step = 3
            elif(robot_1.rfdist < robot_1.rfdist_th and robot_1.lfdist > robot_1.lfdist_th and robot_1.bdist < robot_1.bdist_th):
                self.step = 4
            elif(robot_1.rfdist < robot_1.rfdist_th and robot_1.lfdist < robot_1.lfdist_th and robot_1.bdist > robot_1.bdist_th):
                self.step = 5
        if mid_check:
            self.lane = 2
            if(robot_1.rfdist > robot_1.rfdist_th and robot_1.lfdist > robot_1.lfdist_th and robot_1.bdist > robot_1.bdist_th):
                self.step = 6
            elif(robot_1.rfdist > robot_1.rfdist_th and robot_1.lfdist > robot_1.lfdist_th and robot_1.bdist < robot_1.bdist_th):
                self.step = 7
            elif(robot_1.rfdist < robot_1.rfdist_th and robot_1.lfdist > robot_1.lfdist_th and robot_1.bdist > robot_1.bdist_th):
                self.step = 8
        if left_check:
            self.lane = 3
            if(robot_1.rfdist > robot_1.rfdist_th and robot_1.lfdist > robot_1.lfdist_th and robot_1.bdist > robot_1.bdist_th):
                self.step = 9
            elif(robot_1.rfdist > robot_1.rfdist_th and robot_1.lfdist > robot_1.lfdist_th and robot_1.bdist < robot_1.bdist_th):
                self.step = 10
            elif(robot_1.rfdist < robot_1.rfdist_th and robot_1.lfdist > robot_1.lfdist_th and robot_1.bdist > robot_1.bdist_th):
                self.step = 11

# Main
    def main(self):
        rate = rospy.Rate(3)

        while self.latest_state_data is None:
            rospy.sleep(0.1)

        husky_1 = Robot("/h1/husky_velocity_controller/cmd_vel", "/h1/scan", "/h1/odometry/filtered", "/h1/imu/data","Husky_h1")
        husky_2 = Robot("/h2/husky_velocity_controller/cmd_vel", "/h2/scan", "/h2/odometry/filtered", "/h2/imu/data","Husky_h2")

        left_array = [9, 10, 11]

        self.get_params(husky_1,husky_2)
        self.check_distance(husky_1,husky_2)
        self.get_policy(husky_1)

        counter = 0

        while not rospy.is_shutdown():
            self.get_params(husky_1,husky_2)
            self.check_distance(husky_1,husky_2)
            self.check_velocity(husky_1,husky_2)
            self.adjust_threshold(husky_1,husky_2)
            self.check_state(husky_1, self.timestep)

            #print("Timestepim: " + str(self.timestep))
            print("Policy Step: " + str(self.step))
            print("\n")

            # States: 0 - Calculate Policy, 1 - Execute Policy, 2 - Wait for order

            left_array = [9, 10, 11]
            i = self.timestep

            if i in left_array:
                left_check = True
            else:
                left_check = False

            if self.state == 0:
                self.get_policy(husky_1)
                self.state = 2
            elif self.state == 1:
                check_request1 = (0 < husky_1.bdist < husky_1.vx) or (husky_1.rfdist_th-husky_1.vx < husky_1.rfdist < husky_1.rfdist_th) #or left_check
                #check_request1 = 80 > husky_1.bdist > husky_1.bdist_th*self.msg.v_relative or husky_1.rfdist_th*self.msg.v_relative-0.1 < husky_1.rfdist < husky_1.rfdist_th*self.msg.v_relative
                #check_request2 =  or (self.policy[len(self.policy)-1] == self.step) or not (self.step in self.policy)
                print("bdist: " + str(husky_1.bdist))
                print("bdistth: " + str(husky_1.bdist_th))
                print("\n")
                #print("bdistth min: " + str(husky_1.bdist_th-husky_1.vx/5))
                #print("bdistth max: " + str(husky_1.bdist_th+husky_1.vx/5))
                print("fdist: " + str(husky_1.rfdist))
                print("fdistth: " + str(husky_1.rfdist_th))
                #print("Policy fdistthhh: " + str(husky_1.rfdist_th*self.msg.v_relative))
                print("\n")
                if check_request1:
                    self.state = 0
            elif self.state == 2:
                if self.got_new_plan:
                    self.state = 1
                    self.got_new_plan = False

            self.msg.timestep = self.state
            self.msg.old_policy = self.old_policy
            self.msg.got_new_plan = self.got_new_plan
            self.policy_pub.publish(self.msg)

            rate.sleep()

if __name__ == '__main__':
    ActionExecutor()
