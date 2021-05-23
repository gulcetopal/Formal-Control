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
        self.rbdist = 0
        self.lbdist = 0


# ths to go risky
        self.rfdist_th = 4.5
        self.lfdist_th = 4.5
        self.bdist_th = 4.5

# critical ths to go emg center to center dist
        self.rfdist_crit_th = 1.55
        self.lfdist_crit_th = 1.5
        self.bdist_crit_th = 1.5

# Safety ths after emg - center to center dist
        self.emg_rfdist_th = 2.3
        self.emg_lfdist_th = 2.3
        self.emg_bdist_th = 2.3

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
    def __init__(self,policy = [0,0]):
        rospy.init_node('action_executor')
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.StateCallback)
        rospy.Subscriber("/traffic_timestep", SelfStateMsg, self.TimestepCallback)
        rospy.Subscriber("/traffic_robot_state", SelfStateMsg, self.LaneCallback)
        rospy.Subscriber("/policy_topic", SelfStateMsg, self.PolicyCallback)

        self.policy = policy
        self.policy_pub = rospy.Publisher("/traffic_topic", SelfStateMsg, queue_size = 50)
        self.calc_policy = [0]
        self.calc_actions = []
        self.actions = []
        self.old_policy = []
        self.v_right = 0
        self.v_left = 0

        self.lane_data = 0
        self.step = 0
        self.state = 0 # start from state: wait for order
        self.timestep = 0

        self.emergency = 0
        self.v_emg = 1 #PARAMETRIKKKKKKK
        self.FV_vel = 0
        self.got_new_plan = False
        self.request_check = False
        self.check = False
        self.lane = 1

        self.rfcrit_check = False
        self.lfcrit_check = False
        self.bcrit_check = False
        self.crit_check = "None"

        self.FV = None
        self.BV = None
        self.LFV = None
        self.LBV = None

        self.latest_scan = None
        self.latest_state_data = None
        self.talker = PrismTalker()
        self.policy_exe = PolicyExecutor()
        self.msg = SelfStateMsg()

        self.main()

#   Callbacks and fixes
    def StateCallback(self,data):
        self.latest_state_data = data

    def LaneCallback(self,data):
        self.lane_data = data.timestep

    def TimestepCallback(self,data):
        self.timestep = data.timestep

    def PolicyCallback(self,data):
        self.calc_policy = data.policy
        self.calc_actions = data.actions
        self.old_policy = data.old_policy

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

    def get_params(self,robot_1,robot_2,robot_3):
        robot_1.x = self.latest_state_data.pose[self.latest_state_data.name.index(robot_1.model_name)].position.x
        robot_2.x = self.latest_state_data.pose[self.latest_state_data.name.index(robot_2.model_name)].position.x
        robot_3.x = self.latest_state_data.pose[self.latest_state_data.name.index(robot_3.model_name)].position.x
        robot_1.y = self.latest_state_data.pose[self.latest_state_data.name.index(robot_1.model_name)].position.y
        robot_2.y = self.latest_state_data.pose[self.latest_state_data.name.index(robot_2.model_name)].position.y
        robot_3.y = self.latest_state_data.pose[self.latest_state_data.name.index(robot_3.model_name)].position.y

        robot_1.vx = self.latest_state_data.twist[self.latest_state_data.name.index(robot_1.model_name)].linear.x
        robot_2.vx = self.latest_state_data.twist[self.latest_state_data.name.index(robot_2.model_name)].linear.x
        robot_3.vx = self.latest_state_data.twist[self.latest_state_data.name.index(robot_3.model_name)].linear.x

        i = self.latest_state_data.name.index(robot_1.model_name)
        [roll1, pitch1, robot_1.yaw] = euler_from_quaternion([self.latest_state_data.pose[i].orientation.x,self.latest_state_data.pose[i].orientation.y,self.latest_state_data.pose[i].orientation.z,self.latest_state_data.pose[i].orientation.w])
        robot_1.yaw = self.fix_yaw(robot_1.yaw)
        i = self.latest_state_data.name.index(robot_2.model_name)
        [roll2, pitch2, robot_2.yaw] = euler_from_quaternion([self.latest_state_data.pose[i].orientation.x,self.latest_state_data.pose[i].orientation.y,self.latest_state_data.pose[i].orientation.z,self.latest_state_data.pose[i].orientation.w])
        robot_2.yaw = self.fix_yaw(robot_2.yaw)
        i = self.latest_state_data.name.index(robot_3.model_name)
        [roll2, pitch2, robot_3.yaw] = euler_from_quaternion([self.latest_state_data.pose[i].orientation.x,self.latest_state_data.pose[i].orientation.y,self.latest_state_data.pose[i].orientation.z,self.latest_state_data.pose[i].orientation.w])
        robot_2.yaw = self.fix_yaw(robot_3.yaw)
    
    def get_policy(self, robot_1):
        self.request_check = True
        self.msg.request = True
        #self.policy_pub.publish(self.msg)
        #self.msg.request = False


#   Check variables - environmental variables are controlled
    def check_distance(self,robot_1,robot_2,robot_3):
        tol = 0
        if robot_1.x >= robot_2.x + tol:
            self.FV = robot_1
            self.BV = robot_2
        else:
            self.FV = robot_2
            self.BV = robot_1
        self.FV.rfdist = 100
        self.FV.rbdist =  abs(self.FV.x- self.BV.x) #float(math.sqrt(math.pow((self.FV.x- self.BV.x), 2) + math.pow((self.BV.y - self.FV.y), 2)))
        self.BV.rfdist = self.FV.rbdist
        self.BV.rbdist = 100
        self.msg.rfdist = robot_1.rfdist
        self.msg.lfdist = robot_1.lfdist
        self.msg.bdist = robot_1.rbdist

        if robot_1.x >= robot_3.x + tol:
            self.LFV = robot_1
            self.LBV = robot_3
        else:
            self.LFV = robot_3
            self.LBV = robot_1
        self.LFV.lfdist = 100
        self.LFV.lbdist =  abs(self.LFV.x- self.LBV.x) #float(math.sqrt(math.pow((self.FV.x- self.BV.x), 2) + math.pow((self.BV.y - self.FV.y), 2)))
        self.LBV.lfdist = self.LFV.lbdist
        self.LBV.lbdist = 100



        x = (float(math.sqrt(math.pow((self.FV.x- self.BV.x), 2) + math.pow((self.BV.y - self.FV.y), 2))))
        y = (float(math.sqrt(math.pow((self.LFV.x- self.LBV.x), 2) + math.pow((self.LBV.y - self.LFV.y), 2))))

        if self.rfcrit_check:
            self.msg.crit_check = "right"
        elif self.lfcrit_check:
            self.msg.crit_check = "left"
        elif self.bcrit_check:
            self.msg.crit_check = "back"
        else:
            self.msg.crit_check = "none"

        #print("RD: "+ str(x) + " || LD: " + str(y))
        #print("---------------------------------")
        

    def check_right_crit(self,robot_1,robot_2):
        # Lane ->> 1 = Right, 2 = Mid, 3 = Left
        yaw_th = 0.1

        if robot_1.rfdist > robot_1.rfdist_crit_th:
            self.rfcrit_check = False
        else:
            if (self.lane == 2 or self.lane == 3):
                if abs(robot_1.yaw-robot_2.yaw) < yaw_th:
                    self.rfcrit_check = False
                else:
                    self.rfcrit_check = (float(math.sqrt(math.pow((self.FV.x- self.BV.x), 2) + math.pow((self.BV.y - self.FV.y), 2)))) < 0.75
                     #robot_1.rfdist_crit_th
            elif self.lane == 1:
                self.rfcrit_check = True

    def check_left_crit(self,robot_1,robot_2):
        # Lane ->> 1 = Right, 2 = Mid, 3 = Left
        yaw_th = 0.1

        if robot_1.lfdist > robot_1.lfdist_crit_th:
            self.lfcrit_check = False
        else:
            if (self.lane == 1 or self.lane == 2):
                if abs(robot_1.yaw-robot_2.yaw) < yaw_th:
                    self.lfcrit_check = False
                else:
                    self.lfcrit_check = (float(math.sqrt(math.pow((self.LFV.x- self.LBV.x), 2) + math.pow((self.LBV.y - self.LFV.y), 2)))) < robot_1.lfdist_crit_th
            elif self.lane == 3:
                self.lfcrit_check = True

    def check_back_crit(self,robot_1,robot_2):
        # Lane ->> 1 = Right, 2 = Mid, 3 = Left
        yaw_th = 0.1

        if robot_1.rbdist > robot_1.bdist_crit_th:
            self.bcrit_check = False
        else:
            if abs(robot_1.yaw-robot_2.yaw) < yaw_th:
                self.bcrit_check = False
            else:
                self.bcrit_check = (float(math.sqrt(math.pow((self.FV.x- self.BV.x), 2) + math.pow((self.BV.y - self.FV.y), 2)))) < 0.75 #robot_1.bdist_crit_th

    def check_crits(self,robot_1):
        self.rfcrit_check = False
        self.lfcrit_check = False
        self.bcrit_check = False
        if self.BV == robot_1:
            self.check_right_crit(self.BV,self.FV)
        else:
            self.check_back_crit(self.FV,self.BV)
        if self.LBV == robot_1:
            self.check_left_crit(self.LBV,self.LFV)
        else:
            self.check_back_crit(self.LFV,self.LBV)

    def check_velocity(self,robot_1,robot_2, robot_3):
    	robot1v = 1 #robot_1.vx  #PARAMETRIK ATANCAK
        robot2v = robot_2.vx
        robot3v = robot_3.vx
        self.v_right = robot1v-robot2v
        self.v_left = robot1v-robot3v
        self.msg.v_relative = self.v_right
        
        self.FV_vel = robot2v
 
        #print("\n")
        #print("Husky 1 vx: "+ str(robot_1.vx))
        #print("Husky 1 vy: "+ str(robot_1.vy))
        #print("Husky 1 v: "+ str(float(math.sqrt(math.pow(robot_1.vx, 2) + math.pow(robot_1.vy, 2)))))
        #print("\n")
        #print("Relative vel: "+ str(self.v_left))
        

    def adjust_threshold(self,robot_1,robot_2):
        PRISM_TIME = 7
        OVERTAKE_TIME = 3


        robot_1.rfdist_th = PRISM_TIME * 0.5 + OVERTAKE_TIME * 0.5
        robot_1.lfdist_th = PRISM_TIME * 1.5 + OVERTAKE_TIME * 1.5 + robot_1.rfdist_th
        #print("RT: " + str(robot_1.rfdist_th) + " || LT: " + str(robot_1.lfdist_th))
        #print("\n")

        prism_th = PRISM_TIME * self.v_right
        if prism_th < robot_1.bdist_th:
            robot_1.bdist_th = PRISM_TIME * self.v_right
        else:
            robot_1.bdist_th = 1 #0.01
        #print("Right Back Distance : "+ str(robot_1.rbdist))
        #print("Back Threshold : "+ str(robot_1.bdist_th))
        #print("\n")


        # EMG Thresholds
        self.emg_rfdist_th = PRISM_TIME * self.v_right
        self.emg_lfdist_th = PRISM_TIME * self.v_left
        self.emg_bdist_th = 1.5 #PRISM_TIME * self.v


    def check_state(self, robot_1, lane):

        if self.emergency != 0 and self.saferf and self.safelf and self.safeb:
            self.emergency = 0

        right_array = list(range(0,8))
        mid_array = list(range(8,16))
        left_array = list(range(16,24))
        i = lane

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
            if (robot_1.rfdist > robot_1.rfdist_th and robot_1.lfdist > robot_1.lfdist_th and robot_1.rbdist > robot_1.bdist_th):
                self.step = 0
                if self.rfcrit_check:
                    self.emergency = 1
            elif(robot_1.rfdist > robot_1.rfdist_th and robot_1.lfdist > robot_1.lfdist_th and robot_1.rbdist < robot_1.bdist_th):
                self.step = 1
                if self.rfcrit_check:
                    self.emergency = 1
            elif(robot_1.rfdist > robot_1.rfdist_th and robot_1.lfdist < robot_1.lfdist_th and robot_1.rbdist > robot_1.bdist_th):
                self.step = 2
                if self.rfcrit_check:
                    self.emergency = 1
            elif(robot_1.rfdist > robot_1.rfdist_th and robot_1.lfdist < robot_1.lfdist_th and robot_1.rbdist < robot_1.bdist_th):
                self.step = 3
                if self.rfcrit_check:
                    self.emergency = 1
            elif(robot_1.rfdist < robot_1.rfdist_th and robot_1.lfdist > robot_1.lfdist_th and robot_1.rbdist > robot_1.bdist_th):
                self.step = 4
                if self.rfcrit_check:
                    self.emergency = 1
            elif(robot_1.rfdist < robot_1.rfdist_th and robot_1.lfdist > robot_1.lfdist_th and robot_1.rbdist < robot_1.bdist_th):
                self.step = 5
                if self.rfcrit_check or self.bcrit_check:
                    self.emergency = 3
            elif(robot_1.rfdist < robot_1.rfdist_th and robot_1.lfdist < robot_1.lfdist_th and robot_1.rbdist > robot_1.bdist_th):
                self.step = 6
                if self.rfcrit_check:
                    self.emergency = 1
            elif(robot_1.rfdist < robot_1.rfdist_th and robot_1.lfdist < robot_1.lfdist_th and robot_1.rbdist < robot_1.bdist_th):
                self.step = 7
                if self.rfcrit_check or self.bcrit_check:
                    self.emergency = 3
        if mid_check:
            self.lane = 2
            if (robot_1.rfdist > robot_1.rfdist_th and robot_1.lfdist > robot_1.lfdist_th and robot_1.rbdist > robot_1.bdist_th):
                self.step = 8
                if self.rfcrit_check:
                    self.emergency = 2
            elif(robot_1.rfdist > robot_1.rfdist_th and robot_1.lfdist > robot_1.lfdist_th and robot_1.rbdist < robot_1.bdist_th):
                self.step = 9
                if self.bcrit_check:
                    self.emergency = 2
            elif(robot_1.rfdist > robot_1.rfdist_th and robot_1.lfdist < robot_1.lfdist_th and robot_1.rbdist > robot_1.bdist_th):
                self.step = 10
                if self.lfcrit_check:
                    self.emergency = 3
            elif(robot_1.rfdist > robot_1.rfdist_th and robot_1.lfdist < robot_1.lfdist_th and robot_1.rbdist < robot_1.bdist_th):
                self.step = 11
                if self.lfcrit_check:
                    self.emergency = 3
            elif(robot_1.rfdist < robot_1.rfdist_th and robot_1.lfdist > robot_1.lfdist_th and robot_1.rbdist > robot_1.bdist_th):
                self.step = 12
                if self.rfcrit_check:
                    self.emergency = 2
            elif(robot_1.rfdist < robot_1.rfdist_th and robot_1.lfdist > robot_1.lfdist_th and robot_1.rbdist < robot_1.bdist_th):
                self.step = 13
                if self.rfcrit_check:
                    self.emergency = 2
            elif(robot_1.rfdist < robot_1.rfdist_th and robot_1.lfdist < robot_1.lfdist_th and robot_1.rbdist > robot_1.bdist_th):
                self.step = 14
                if self.rfcrit_check or self.lfcrit_check:
                    self.emergency = 3
            elif(robot_1.rfdist < robot_1.rfdist_th and robot_1.lfdist < robot_1.lfdist_th and robot_1.rbdist < robot_1.bdist_th):
                self.step = 15
                if self.rfcrit_check or self.lfcrit_check:
                    self.emergency = 2
        if left_check:
            self.lane = 3
            if (robot_1.rfdist > robot_1.rfdist_th and robot_1.lfdist > robot_1.lfdist_th and robot_1.rbdist > robot_1.bdist_th):
                self.step = 16
                if self.lfcrit_check:
                    self.emergency = 2
            elif(robot_1.rfdist > robot_1.rfdist_th and robot_1.lfdist > robot_1.lfdist_th and robot_1.rbdist < robot_1.bdist_th):
                self.step = 17
                if self.bcrit_check:
                    self.emergency = 2
            elif(robot_1.rfdist > robot_1.rfdist_th and robot_1.lfdist < robot_1.lfdist_th and robot_1.rbdist > robot_1.bdist_th):
                self.step = 18
                if self.lfcrit_check:
                    self.emergency = 2
            elif(robot_1.rfdist > robot_1.rfdist_th and robot_1.lfdist < robot_1.lfdist_th and robot_1.rbdist < robot_1.bdist_th):
                self.step = 19
                if self.lfcrit_check:
                    self.emergency = 2
            elif(robot_1.rfdist < robot_1.rfdist_th and robot_1.lfdist > robot_1.lfdist_th and robot_1.rbdist > robot_1.bdist_th):
                self.step = 20
                if self.rfcrit_check or self.lfcrit_check:
                    self.emergency = 2
            elif(robot_1.rfdist < robot_1.rfdist_th and robot_1.lfdist > robot_1.lfdist_th and robot_1.rbdist < robot_1.bdist_th):
                self.step = 21
                if self.rfcrit_check or self.lfcrit_check:
                    self.emergency = 2
            elif(robot_1.rfdist < robot_1.rfdist_th and robot_1.lfdist < robot_1.lfdist_th and robot_1.rbdist > robot_1.bdist_th):
                self.step = 22
                if self.lfcrit_check:
                    self.emergency = 2
            elif(robot_1.rfdist < robot_1.rfdist_th and robot_1.lfdist < robot_1.lfdist_th and robot_1.rbdist < robot_1.bdist_th):
                self.step = 23
                if self.lfcrit_check:
                    self.emergency = 2
        self.msg.lane = self.lane
# Main
    def main(self):
        rate = rospy.Rate(3)

        while self.latest_state_data is None:
            rospy.sleep(0.1)

        husky_1 = Robot("/h1/husky_velocity_controller/cmd_vel", "/h1/scan", "/h1/odometry/filtered", "/h1/imu/data","Husky_h1")
        husky_2 = Robot("/h2/husky_velocity_controller/cmd_vel", "/h2/scan", "/h2/odometry/filtered", "/h2/imu/data","Husky_h2")
        husky_3 = Robot("/h3/husky_velocity_controller/cmd_vel", "/h3/scan", "/h3/odometry/filtered", "/h3/imu/data","Husky_h3")

        left_array = list(range(16,24))

        self.get_params(husky_1,husky_2,husky_3)
        self.check_distance(husky_1,husky_2,husky_3)
        self.get_policy(husky_1)

        """        self.policy = self.calc_policy
        self.actions = self.calc_actions"""

        counter = 0

        while not rospy.is_shutdown():
            self.get_params(husky_1,husky_2,husky_3)
            self.check_distance(husky_1,husky_2,husky_3)
            self.check_crits(husky_1)
            self.check_velocity(husky_1,husky_2, husky_3)
            self.adjust_threshold(husky_1,husky_2)
            self.check_state(husky_1, self.lane_data)

            print("-------------------------------------")
            #print("Policy:" + str(self.policy))

            print("Calc Policy: " + str(self.calc_policy ))
            print("Policy: " + str(self.policy))
            print("Old Policy: " + str(self.old_policy))
            if self.calc_policy != self.policy:
                self.policy = self.calc_policy
                self.actions = self.calc_actions
                self.msg.policy = self.policy
                self.msg.actions = self.actions
                self.got_new_plan = True
            else:
                if self.policy == self.old_policy:
                    self.got_new_plan = True
                else:
                    self.got_new_plan = False

#           #######################################

            self.saferf = (float(math.sqrt(math.pow((self.FV.x- self.BV.x), 2) + math.pow((self.BV.y - self.FV.y), 2)))) > husky_1.emg_rfdist_th
            self.safelf = (float(math.sqrt(math.pow((self.LFV.x- self.LBV.x), 2) + math.pow((self.LBV.y - self.LFV.y), 2)))) > husky_1.emg_lfdist_th
            self.safeb =(float(math.sqrt(math.pow((self.FV.x- self.BV.x), 2) + math.pow((self.BV.y - self.FV.y), 2)))) > husky_1.emg_bdist_th


            # States: 0 - Calculate Policy, 1 - Execute Policy, 2 - Wait for order, 3 - EMG
#           #######################################
            if self.emergency != 0:
                self.state = 3

            if self.state == 0:
                print("State: " + str(self.state))
                print("\n")
                print("PRISM STARTED")
                self.get_policy(husky_1)
                #self.request_check == False
                self.state = 2     

            elif self.state == 1:
                self.got_new_plan = False
                print("State: " + str(self.state))
                self.msg.request = False
                #self.request_check == False
                check_request1 = (0 < husky_1.rbdist < husky_1.bdist_th+husky_1.vx/3) or (husky_1.rfdist_th-husky_1.vx < husky_1.rfdist < husky_1.rfdist_th) #or left_check
                check_request2 =  not (self.step in self.policy) #([len(self.policy)-1] == self.timestep) or

                if check_request1 or check_request2:
                    self.state = 0
                    print("CR1: " + str(check_request1))
                    print("CR2: " + str(check_request2))
                    
            elif self.state == 2:
                print("State: " + str(self.state))
                self.msg.request = False
                self.request_check == False
                if self.got_new_plan:
                    self.v_emg = 1
                    self.state = 1
                    self.got_new_plan = False

            elif self.state == 3:
                print("State: " + str(self.state))
                self.msg.request = False
                #self.request_check == False
                self.check_state(husky_1, self.lane_data)
                #self.emergency != 0 and
                if (self.saferf and self.safelf and self.safeb):
                    self.state = 0
                    self.v_emg = self.FV_vel #/2
                    print("EMG Step: " + str(self.step))
            
            ########################################

            self.msg.current_state = self.step
            print("Step: " + str(self.step))
            print("Req Check: " + str(self.request_check))
            print("\n")
            self.msg.emergency = self.emergency
            self.msg.timestep = self.state
            self.msg.old_policy = self.old_policy
            self.msg.got_new_plan = self.got_new_plan
            self.msg.v_emg = self.v_emg
            self.policy_pub.publish(self.msg)

            rate.sleep()

if __name__ == '__main__':
    ActionExecutor()
