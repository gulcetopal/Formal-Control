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

class GetPolicy():
    def __init__(self,policy = [0] ):
        rospy.init_node('get_policy')
        rospy.Subscriber("/traffic_topic", SelfStateMsg, self.StateCallback)

        self.talker = PrismTalker()
        self.policy_exe = PolicyExecutor()
        self.msg = SelfStateMsg()

        self.rfdist = 0
        self.lfdist = 0
        self.bdist = 0
        self.v_relative = 0
        self.step = 2

        self.request = False
        self.old_policy = [0]
        self.policy = policy
        self.actions = []
        self.policy_pub = rospy.Publisher("/policy_topic", SelfStateMsg, queue_size = 50)

        self.main()

    def StateCallback(self,data):
        self.rfdist = data.rfdist
        self.lfdist = data.lfdist
        self.bdist = data.bdist
        self.v_relative = data.rfdist
        self.request = data.request
        self.step = data.current_state
        self.lane = data.lane

    def get_policy(self):
        self.actions = []
        self.old_policy = self.policy
        self.policy_exe = PolicyExecutor(self.rfdist,self.lfdist,self.bdist,self.v_relative)
        self.policy_exe.SetInit(self.step, self.lane)
        self.policy_exe.Policy()
        self.policy = self.policy_exe.policy
        self.actions = self.policy_exe.actions
        # Tam sollama cozuldugunde
        for i in range(len(self.policy)):
            if self.policy[i] in (range(16,24)):
                for k in range(3): #3
                    self.policy.insert(i,self.policy[i])
                break

        for i in range(10):
            self.policy.append(self.policy[len(self.policy)-1])

    def main(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.msg.policy = self.policy
            self.msg.old_policy = self.old_policy
            self.msg.actions = self.actions
            if self.request:
                print("*********************************************************************************************************************************************************************************************************************************************")
                self.get_policy()
            self.policy_pub.publish(self.msg)
            rate.sleep()

if __name__ == '__main__':
    GetPolicy()
