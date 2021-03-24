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
        self.goal_tolerance = 0.2
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.twist = Twist()
        self.latest_scan = None
        self.dist = 0
        self.model_name = model_name
        self.entity_name = entity_name

        self.rfdist = 0
        self.lfdist = 0
        self.bdist = 0

        self.rfdist_th = 4.3
        self.lfdist_th = -1
        self.bdist_th = 1

        self.vx = 0
        self.v_refx = 5
        self.v_refy = 0

        self.main()

# Callback Funcs
    def imuCallback(self, data):
	    [roll, pitch, self.yaw] = euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]) #Convert quaternion to euler angles
	    self.yaw = self.fix_yaw(self.yaw)



    def Odomcallback(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        self.x = x
        self.y = y

##################

# Robot Actions
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

    def yanal_go_forward(self, speedx = 0.5, speedy = 0.2, angularz = math.pi/10, secs = 0.15 ):
        rate = rospy.Rate(5)
        self.twist.linear.x = speedx
        self.twist.linear.y = speedy
        self.twist.angular.z = angularz
        for i in range(int(secs * 10)):
            if abs(self.yaw-self.yaw_ref)<=0.1:
                self.twist.angular.z = 0
            else:
                self.twist.angular.z = angularz
            self.twist_pub.publish(self.twist)
            rate.sleep()
        #print("Distance to target: " + str(self.dist))

##################

# Robot Behaviours
    def free_ride(self):
        self.go_forward(self.v_refx)

    def sheer_in(self):
        self.yanal_go_forward(self.v_refx,-self.v_refy,self.yaw_ref)

    def sheer_out(self):
        self.yanal_go_forward(self.v_refx,self.v_refy, self.yaw_ref)

    def faster(self):
        inc = 0.2
        self.v_refx = self.v_refx+inc
        self.go_forward(self.v_refx)

    def slower(self):
        inc = -0.2
        self.v_refx = self.v_refx+inc
        self.go_forward(self.v_refx)

##################

# Main Func
    def main(self):
        #rospy.Subscriber(self.odom_topic, Odometry, self.Odomcallback)
        #rospy.Subscriber(self.imu_topic, Imu, self.imuCallback)
        x = 2

##################

class ActionExecutor():
    def __init__(self,policy = [0] ):
        rospy.init_node('Calculator_Mahmut')
        rospy.Subscriber("/h1/scan", LaserScan, self.LidarCallback)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.StateCallback)
        rospy.Subscriber("/traffic_robot_state", SelfStateMsg, self.TimestepCallback)

        self.policy = policy
        self.policy_pub = rospy.Publisher("/traffic_topic", SelfStateMsg, queue_size = 50)
        self.husky_1_pub = rospy.Publisher("/h1/husky_velocity_controller/cmd_vel", Twist, queue_size = 50)
        self.husky_2_pub = rospy.Publisher("/h2/husky_velocity_controller/cmd_vel", Twist, queue_size = 50)

        self.old_policy = []
        self.v = 0

        self.timestep = 0
        self.step = 0

        self.got_new_plan = False
        self.check = False

        self.FV = None
        self.BV = None

        self.latest_scan = None
        self.latest_state_data = None
        self.talker = PrismTalker()
        self.policy_exe = PolicyExecutor()
        self.msg = SelfStateMsg()

        self.main()

# Callback Funcs
    def LidarCallback(self,data):
        self.latest_scan = data.ranges

    def StateCallback(self,data):
        self.latest_state_data = data

    def TimestepCallback(self,data):
        self.timestep = data.timestep

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
# Gets
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
        self.policy_exe.SetInit(self.step)
        #self.timestep = self.step
        self.policy_exe.Policy()
        self.policy = self.policy_exe.policy
        for i in range(10):
            self.policy.append(self.policy[len(self.policy)-1])
        self.msg.policy = self.policy
        self.policy_pub.publish(self.msg)

##################

# Checks
    def check_distance(self,robot_1,robot_2):
        if robot_1.x >= robot_2.x:
            self.FV = robot_1
            self.BV = robot_2
        else:
            self.FV = robot_2
            self.BV = robot_1
        self.FV.rfdist = 100
        self.FV.bdist = float(math.sqrt(math.pow((self.FV.x- self.BV.x), 2) + math.pow((self.BV.y - self.FV.y), 2)))
        self.BV.rfdist = self.FV.bdist
        self.BV.bdist = 100
        self.msg.rfdist = robot_1.rfdist
        self.msg.bdist = robot_1.bdist

        #print("Forward Vehicle: " + self.FV.model_name)
        #print("Backward Vehicle: " + self.BV.model_name)
        print("Front distance: "+ str(robot_1.rfdist))
        print("Back distance : "+ str(robot_1.bdist))
        print("\n")
        #print("Husky 1 X = " + str(robot_1.x))
        #print("Husky 1 Y = " + str(robot_1.y))
        """
        print("F_dist = " + str(self.BV.rfdist))
        print("B_dist = " + str(self.FV.bdist))

        print("x1 = " + str(robot_1.x))
        print("x2 = " + str(robot_2.x))
        """

    def check_velocity(self,robot_1,robot_2):
        self.v =robot_1.vx-robot_2.vx
        self.msg.v_relative = self.v

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
            if(robot_1.rfdist > robot_1.rfdist_th and robot_1.lfdist > robot_1.lfdist_th and robot_1.bdist > robot_1.bdist_th):
                self.step = 6
            elif(robot_1.rfdist > robot_1.rfdist_th and robot_1.lfdist > robot_1.lfdist_th and robot_1.bdist < robot_1.bdist_th):
                self.step = 7
            elif(robot_1.rfdist < robot_1.rfdist_th and robot_1.lfdist > robot_1.lfdist_th and robot_1.bdist > robot_1.bdist_th):
                self.step = 8
        if left_check:
            if(robot_1.rfdist > robot_1.rfdist_th and robot_1.lfdist > robot_1.lfdist_th and robot_1.bdist > robot_1.bdist_th):
                self.step = 9
            elif(robot_1.rfdist > robot_1.rfdist_th and robot_1.lfdist > robot_1.lfdist_th and robot_1.bdist < robot_1.bdist_th):
                self.step = 10
            elif(robot_1.rfdist < robot_1.rfdist_th and robot_1.lfdist > robot_1.lfdist_th and robot_1.bdist > robot_1.bdist_th):
                self.step = 11

##################
    def main(self):
        rate = rospy.Rate(1)
        self.got_new_plan = False
        while self.latest_state_data is None:
            rospy.sleep(0.1)

        husky_1 = Robot(self.husky_1_pub, "/h1/scan", "/h1/odometry/filtered", "/h1/imu/data","Husky_h1")
        husky_2 = Robot(self.husky_2_pub, "/h2/scan", "/h2/odometry/filtered", "/h2/imu/data","Husky_h2")

        left_array = [9, 10, 11]

        self.get_params(husky_1,husky_2)
        self.check_distance(husky_1,husky_2)
        self.get_policy(husky_1)

        counter = 0

        while not rospy.is_shutdown():
            self.get_params(husky_1,husky_2)
            self.check_distance(husky_1,husky_2)
            self.check_velocity(husky_1,husky_2)
            self.check_state(husky_1, self.timestep)

            #print("Timestepim: " + str(self.timestep))
            print("Step: " + str(self.step))
            print("\n")

            if self.step in self.policy:
                if (self.policy[len(self.policy)-1] != self.step):
                    if self.v != 0:
                        if self.step in left_array:
                                left_check = True
                        else:
                                left_check = False
                        if (1 < husky_1.bdist < 4) or (math.floor(husky_1.rfdist_th) < husky_1.rfdist < husky_1.rfdist_th) or left_check:
                            self.get_policy(husky_1)
                            rate.sleep()
                            #husky_1.bdist_th

                else:
                    self.get_policy(husky_1)
                    rate.sleep()
            else:
                if len(self.policy) != 0:
                    self.get_policy(husky_1)
                    print("Error!")
                    rate.sleep()
                else:
                    rate.sleep()

            #print("Old pal: " + str(self.old_policy))
            #print("Rook: " + str(self.policy))

            if self.old_policy != self.policy:
                self.got_new_plan = True
                self.old_policy = self.policy
                print("New Policy Calculated !")

            self.msg.got_new_plan = self.got_new_plan
            self.policy_pub.publish(self.msg)

            if self.got_new_plan == True:
                self.got_new_plan = False


if __name__ == '__main__':
    ActionExecutor()
