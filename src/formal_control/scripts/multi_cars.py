#!/usr/bin/env python

import rospy
import os
import math
import tf
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelStates


class Robot():
    def __init__(self, twist_pub = "/h1/husky_velocity_controller/cmd_vel", scan_topic = " ", odom_topic = " ", imu_topic = " ", model_name = "Husky_h1", entity_name = "base_link"):
        rospy.init_node("environmental_driver")

        # Control variable to print velocity information
        self.PRINT_DEBUG = 0

        # Adjustment of other vehicles' velocity
        self.RIGHT_LANE_VELOCITY = 0.5
        self.LEFT_LANE_VELOCITY = 0.5


        self.h2_vx = 0
        self.h3_vx = 0

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.StateCallback)
        self.twist_right = Twist()
        self.husky_2_pub = rospy.Publisher("/h2/husky_velocity_controller/cmd_vel", Twist, queue_size = 50)
        self.twist_left = Twist()
        self.husky_3_pub = rospy.Publisher("/h3/husky_velocity_controller/cmd_vel", Twist, queue_size = 50)

        self.main()

    def StateCallback(self,data):
        self.state_data = data
        self.h2_vx = self.state_data.twist[self.state_data.name.index("Husky_h2")].linear.x
        self.h3_vx = self.state_data.twist[self.state_data.name.index("Husky_h3")].linear.x

    def main(self):  
        self.twist_right.linear.x = self.RIGHT_LANE_VELOCITY
        self.twist_left.linear.x = self.LEFT_LANE_VELOCITY

        self.twist_right.angular.z = 0
        self.twist_left.angular.z = 0

        rate = rospy.Rate(5)

        while not rospy.is_shutdown():      
            self.husky_2_pub.publish(self.twist_right)
            self.husky_3_pub.publish(self.twist_left)

            if self.PRINT_DEBUG == 1:
                print("Linear Velocities")
                print("Right Lane Vehicle: " + str(self.h2_vx))
                print("Left Lane Vehicle: " + str(self.h3_vx))
            
            rate.sleep()

if __name__ == '__main__':
    Robot()
