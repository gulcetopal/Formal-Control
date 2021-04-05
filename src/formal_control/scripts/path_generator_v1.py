#!/usr/bin/env python

import rospy
import rosparam
from formal_control.msg import SelfStateMsg, PathMsg
import matplotlib.pyplot as plt
import matplotlib as mpl
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import csv
import numpy as np

class Point():
    def __init__(self,xs=0,xf=0,ys=0,yf=0,k=0):
        self.xs = xs
        self.xf = xf
        self.ys = ys
        self.yf = yf
        self.k = k

class PathGenerator():
    def __init__(self):
        rospy.init_node("PathGenerator")
        self.latest_state_data = ModelStates()
        self.traj = []
        self.policy = []

        self.path_dir = rospy.get_param('Directory/path')
        #self.policy = [0,0,0,0]
        #self.policy = [0,0,0,0,1,1,1,2,3,3,3,3,3,3,3,3,3,3,3,3]
        #self.policy = [3,3,3,8,11,11,11,6,0,0,0]
        #self.policy = [11,11,11,11,11,11]

        self.v_relative = 0
        self.old_policy = []
        self.trajm_lines = []
        self.trajys_lines = []
        self.trajyf_lines = []
        self.line_step = 10
        self.robot_x = 0
        self.robot_y = 0
        self.x_array = []
        self.y_array = []
        self.trajx_array = []
        self.trajy_array = []
        self.path_x = []
        self.path_y = []
        self.w = []
        self.got_new_plan = False
        self.policy_check = False

        rospy.Subscriber("/traffic_topic", SelfStateMsg, self.policyCallback)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.stateCallback)
        self.path_pub = rospy.Publisher("/path_data", PathMsg, queue_size = 50)

        self.main()

    def stateCallback(self,data):
        self.latest_state_data = data

    def policyCallback(self,data):
        self.policy = data.policy
        self.v_relative = data.v_relative
        self.got_new_plan = data.got_new_plan

    def trajectoryPlanner(self):
        right_array = [0, 1, 2, 3, 4, 5]
        mid_array = [6, 7, 8]
        left_array = [9, 10, 11]

        timestep = 15
        pt = Point()

        x = self.robot_x

        self.traj = []
        self.trajx = []
        self.trajy = []
        mid_check_array = []
        if len(self.policy) == 1:
            self.policy = [self.policy[0],self.policy[0]]

        print("Policy: " + str(self.policy))

        for i in self.policy:
            for k in right_array:
                if i == k:
                    right_check = True
                    break
                else:
                    right_check = False
            for k in mid_array:
                if i == k:
                    mid_check = True
                    break
                else:
                    mid_check = False
            mid_check_array.append(mid_check)
            for k in left_array:
                if i == k:
                    left_check = True
                    break
                else:
                    left_check = False

            if right_check:
                y = -2.745
            elif mid_check:
                y = -2.1
            elif left_check:
                y = -1.45
            else:
                y = self.robot_y

            pt.ys = y
            self.trajy.append(pt.ys)

        pt.xs = self.robot_x + 0.1
        self.trajx.append(pt.xs)

        for i in range(len(self.policy)-1):
            if (((self.policy[i+1]) == 6) or ((self.policy[i+1]) == 7) or ((self.policy[i+1]) == 8) or mid_check_array[i]):
                pt.xs = pt.xs + self.robot_vx*3/2   #1.5
            else:
                pt.xs = pt.xs + self.robot_vx       #1
            self.trajx.append(pt.xs)

        line = Point()
        self.trajm_lines = []
        self.trajxs_lines = []
        self.trajxf_lines = []
        self.trajys_lines = []
        self.trajyf_lines = []
        self.path_x = []
        self.path_y = []

        xx = self.robot_x
        yy = self.robot_y
        self.path_x.append(xx)
        self.path_y.append(yy)
        for i in range(len(self.trajx)-1):
            line.ys = self.trajy[i]
            line.yf = self.trajy[i+1]
            line.xs = self.trajx[i]
            line.xf = self.trajx[i+1]

            self.trajys_lines.append(line.ys)
            self.trajyf_lines.append(line.yf)

            self.trajxs_lines.append(line.xs)
            self.trajxf_lines.append(line.xf)
        #print("Trajx: " + str(self.trajx))
        #print("Trajy: " + str(self.trajy))
        #print("Trajxs: " + str(self.trajxs_lines))
        #print("Trajys: " + str(self.trajys_lines))

        temp_x = [xx]
        temp_y = [yy]
        temp2_x = [self.trajxs_lines[0]]
        temp2_y = [self.trajys_lines[0]]

        if len(self.trajxs_lines) is not 0:
            for i in range(len(self.trajxf_lines)):
                temp2_x.append(self.trajxf_lines[i])
                temp2_y.append(self.trajyf_lines[i])
            self.trajxf_lines = temp2_x
            self.trajyf_lines = temp2_y

            for i in range(len(self.trajxs_lines)):
                temp_x.append(self.trajxs_lines[i])
                temp_y.append(self.trajys_lines[i])
            self.trajxs_lines = temp_x
            self.trajys_lines = temp_y

            #print("Trajxs: " + str(self.trajxs_lines))
            #print("Trajys: " + str(self.trajys_lines))

            for i in range(len(self.trajxf_lines)):
                for k in range(self.line_step):
                    xx = xx + (self.trajxf_lines[i] - self.trajxs_lines[i])/self.line_step
                    yy = yy + (self.trajyf_lines[i] - self.trajys_lines[i])/self.line_step
                    self.path_x.append(xx)
                    self.path_y.append(yy)
            #print("Path X: " + str(self.path_x))
            #print("Path Y: " + str(self.path_y))

    def fix_yaw(self, yaw):
        while not (yaw < math.pi and yaw >= -math.pi):
            if yaw >= math.pi:
                yaw -= math.pi * 2
            elif yaw < -math.pi:
                yaw += math.pi * 2
            else:
                break
        return yaw

    def get_params(self):
        model_name = 'Husky_h1'
        if len(self.latest_state_data.name)>0:
            i = self.latest_state_data.name.index(model_name)
            self.robot_x = self.latest_state_data.pose[i].position.x
            self.robot_y = self.latest_state_data.pose[i].position.y
            self.robot_vx = self.latest_state_data.twist[i].linear.x
            self.robot_vw = self.latest_state_data.twist[i].angular.z
            [roll1, pitch1, self.robot_yaw] = euler_from_quaternion([self.latest_state_data.pose[i].orientation.x,self.latest_state_data.pose[i].orientation.y,self.latest_state_data.pose[i].orientation.z,self.latest_state_data.pose[i].orientation.w])
            self.robot_yaw = self.fix_yaw(self.robot_yaw)
            self.x_array.append(self.robot_x)
            self.y_array.append(self.robot_y)
        else:
            print("No gazebo")

    def main(self):
        rate = rospy.Rate(5)

        msg = PathMsg()
        old_trajx = []
        counter = 1
        while not rospy.is_shutdown():
            self.get_params()
            #if self.got_new_plan:
            if self.robot_x == False or self.policy == []:
                rate.sleep()
                print("No policy")
            else:
                if self.policy != self.old_policy:
                    self.old_policy = self.policy
                    self.policy_check = True
                else:
                    self.policy_check = False

                if self.policy_check:
                    self.w = []
                    print("Policy updated")
                    self.get_params()
                    self.trajectoryPlanner()
                    old_trajx = self.trajx
                    msg.m = self.trajm_lines
                    msg.y_start = self.path_y
                    msg.y_finish = self.trajyf_lines
                    msg.x_start = self.path_x
                    msg.x_finish = self.trajxf_lines
                    self.policy_check = False
                    for i in range(len(self.path_x)-1):
                        if self.path_x[i]-self.path_x[i+1] == 0:
                            yaw = 0
                        else:
                            yaw = math.atan((self.path_y[i]-self.path_y[i+1])/(self.path_x[i]-self.path_x[i+1]))

                        self.w.append(yaw)
                    self.w.append(self.w[len(self.w)-1])
                    with open(self.path_dir,"w") as path:
                        path_writer = csv.writer(path, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                        for i in range(len(msg.x_start)):
                            path_writer.writerow([msg.x_start[i],msg.y_start[i],self.w[i]])


                if counter < 2:
                    self.w = []
                    self.get_params()
                    self.trajectoryPlanner()

                    msg.m = self.trajm_lines
                    msg.y_start = self.path_y
                    msg.y_finish = self.trajyf_lines
                    msg.x_start = self.path_x
                    msg.x_finish = self.trajxf_lines
                    self.policy_check = False
                    for i in range(len(self.path_x)-1):
                        if self.path_x[i]-self.path_x[i+1] == 0:
                            yaw = 0
                        else:
                            yaw = math.atan((self.path_y[i]-self.path_y[i+1])/(self.path_x[i]-self.path_x[i+1]))
                        self.w.append(yaw)
                    self.w.append(self.w[len(self.w)-1])
                    with open(self.path_dir,"w") as path:
                        path_writer = csv.writer(path, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                        for i in range(len(msg.x_start)):
                            path_writer.writerow([msg.x_start[i],msg.y_start[i],self.w[i]])

                #print("Path X: " + str(self.path_x))
                #print("Path Y: " + str(self.path_y))

                self.path_pub.publish(msg)
                rate.sleep()

                #if self.policy_check:

                #plt.close('all')
                #if self.robot_x > 15:
                #    plt.plot(self.path_x,self.path_y,'r')
                    #print("Est. " + str(self.est_path_data.y_start))
                #    plt.plot(self.x_array,self.y_array,'b')
                    #plt.plot(self.est_path_data.x_start,self.est_path_data.y_start,'k')
                    #plt.plot(self.est_path2_data.x_start,self.est_path2_data.y_start,'g')

                            #plt.axis([max(self.trajx)+1,min(self.trajx)-1,max(self.trajy)+1,min(self.trajy)-1])
                #    plt.show()
                    #rate.sleep()
                #else:
                #    a = 2
                counter = counter + 1

if __name__ == '__main__':
    PathGenerator()