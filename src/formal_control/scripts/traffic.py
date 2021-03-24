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
        rospy.init_node("others_driver")
        self.twist = Twist()
        self.husky_2_pub = rospy.Publisher("/h2/husky_velocity_controller/cmd_vel", Twist, queue_size = 50)

        self.main()

    def main(self):  
        self.twist.linear.x = 0.25
        self.twist.angular.z = 0
        while not rospy.is_shutdown():      
            self.husky_2_pub.publish(self.twist)

if __name__ == '__main__':
    Robot()
