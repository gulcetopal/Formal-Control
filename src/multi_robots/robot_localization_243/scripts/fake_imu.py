#!/usr/bin/env python

import rospy
import os
import math
import numpy as np
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import LaserScan, Imu
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelStates

class RobotsAction():
    def __init__(self):
        rospy.init_node("imu_node")
        self.ang_x = 0
        self.ang_y = 0
        self.ang_z = 0
        self.flag = 0
        self.imu = Imu()
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.StateCallback)
        self.fake_imu_pub = rospy.Publisher("/imu_h1/data", Imu, queue_size = 50)
        self.main()

    def StateCallback(self,data):
        self.latest_state_data = data
        if self.latest_state_data != None:
            self.flag = 1
        else: 
            self.flag = 0
        i = 17

        self.ang_x = self.latest_state_data.twist[i].angular.x
        self.ang_y = self.latest_state_data.twist[i].angular.y
        self.ang_z = self.latest_state_data.twist[i].angular.z

        self.imu.orientation.x = self.latest_state_data.pose[i].orientation.x
        self.imu.orientation.y = self.latest_state_data.pose[i].orientation.y
        self.imu.orientation.z = self.latest_state_data.pose[i].orientation.z
        self.imu.orientation.w = self.latest_state_data.pose[i].orientation.w
        self.imu.angular_velocity.x = self.ang_x
        self.imu.angular_velocity.y = self.ang_y
        self.imu.angular_velocity.z = self.ang_z

        self.fake_imu_pub.publish(self.imu)

    def main(self):
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            if self.flag == 1:
                rospy.loginfo("Imu data generated manually!")
            else: 
                rospy.loginfo("Error: No Imu data!")
            rate.sleep()
            
if __name__ == '__main__':
    RobotsAction()