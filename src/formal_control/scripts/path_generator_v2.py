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
        self.actions = []
        self.msg = PathMsg()

        self.path_dir = rospy.get_param('Directory/path')
        #self.policy = [0,0,0,0]
        #self.policy = [0,0,0,0,1,1,1,2,3,3,3,3,3,3,3,3,3,3,0,0,0,0,1,1,1,2,3,3,3,3,3,3,3,3,3,3,0,0,0,0,1,1,1,2,3]
        #self.policy = [3,3,3,8,11,11,11,6,0,0,0]
        #self.policy = [11,11,11,11,11,11]
        self.emergency = 0

        self.v_relative = 0
        self.velocity_ref = 1.0

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
        self.trajx = []
        self.trajy = []
        self.w = []
        self.got_new_plan = False
        self.policy_check = False

        self.robot_2_vx = 0
        self.robot_3_vx = 0
        self.model_name_2 = "Husky_h2"
        self.model_name_3 = "Husky_h3"
        self.v_emg_ref = self.velocity_ref

        rospy.Subscriber("/traffic_topic", SelfStateMsg, self.policyCallback)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.StateCallback)
        self.path_pub = rospy.Publisher("/path_data", PathMsg, queue_size = 50)
        self.timestep_pub = rospy.Publisher("/traffic_timestep", SelfStateMsg, queue_size = 50)

        self.timestep_msg = SelfStateMsg()

        self.main()

    def StateCallback(self,data):
        self.latest_state_data = data
        self.robot_2_vx = self.latest_state_data.twist[self.latest_state_data.name.index(self.model_name_2)].linear.x
        self.robot_3_vx = self.latest_state_data.twist[self.latest_state_data.name.index(self.model_name_3)].linear.x

    def policyCallback(self,data):
        self.policy = data.policy
        self.actions = data.actions
        self.emergency = data.emergency
        self.v_relative = data.v_relative
        self.v_emg_ref = data.v_emg
        self.got_new_plan = data.got_new_plan

    def velocity_profiler(self):
        
        i = 0
        if self.trajx != []:
            for k in self.trajx:
                if self.robot_x < k:
                    break
                else:
                    if len(self.actions) != 0:
                        if i == len(self.actions)-1:
                            break
                        else:
                            i = i+1
        else:
            i = 0

        print("Actions: " + str(self.actions))
        print("Action Step: " + str(i))   

        ii = 0
        if self.trajx != []:
            for k in self.trajx:
                if self.robot_x < k:
                    break
                else:
                    if ii == len(self.policy)-1:
                        break
                    else:
                        ii = ii+1
        else:
            ii = 0
        timestep = ii
        self.timestep_msg.timestep = timestep

        if self.emergency != 0:
            if self.emergency == 1: # EM1 -> Handbrake
                self.msg.m = 0
            elif self.emergency == 2: # EM2 -> Pull over to Left
                self.msg.m = self.velocity_ref*1.5
            elif self.emergency == 3: # EM3 -> Pull over to right
                self.msg.m = self.velocity_ref*1.5
        else:
            if len(self.actions) == 0:
                self.msg.m = self.v_emg_ref
                print("EMG Vel: " + str(self.v_emg_ref))
            else:
                if self.actions[i] == 0: # Free Ride
                    self.msg.m = self.velocity_ref
                elif self.actions[i] == 1: # Faster
                    self.msg.m = self.velocity_ref + 0.3
                elif self.actions[i] == 2: # Slower
                    self.msg.m = self.robot_2_vx - self.robot_2_vx/15 #- self.robot_2_vx/3   #self.velocity_ref - 0.3
                elif self.actions[i] == 3: # Sheer in
                    self.msg.m = self.velocity_ref
                elif self.actions[i] == 4: # Sheer out
                    self.msg.m = self.velocity_ref

    def emergency_planner(self):
        timestep = 15
        pt = Point()

        x = self.robot_x

        self.traj = []
        self.trajx = []
        self.trajy = []
        mid_check_array = []

        if self.emergency == 1:
            y = self.robot_y
        elif self.emergency == 2:
            y = -0.5
        elif self.emergency == 3:
            y = -3.5
        else:
            y = self.robot_y

        pt.ys = y
        self.trajy.append(pt.ys)

        pt.xs = x + 0.3
        self.trajx.append(pt.xs)

        for i in range(10):
            pt.xs = pt.xs + 0.3
            self.trajx.append(pt.xs)
            self.trajy.append(pt.ys)

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
            print("EMERGENCY!")
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

    def trajectoryPlanner(self):
        right_array = list(range(0,8))
        mid_array = list(range(8,16))
        left_array = list(range(16,24))

        timestep = 15
        pt = Point()

        x = self.robot_x

        self.traj = []
        self.trajx = []
        self.trajy = []
        mid_check_array = []
        if len(self.policy) == 1:
            self.policy = [self.policy[0],self.policy[0]]

        #print("Policy: " + str(self.policy))

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
            if (mid_check_array[i+1]):
                pt.xs = pt.xs + 1.5
            else:
                pt.xs = pt.xs + 1
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
        old_trajx = []
        counter = 1
        self.old_policy = self.policy

        while not rospy.is_shutdown():
            print("EMG: " + str(self.emergency))
            print("\n")
            self.get_params()
            #if self.got_new_plan:
            if self.robot_x == False or self.policy == []:
                rate.sleep()
                print("No policy")
            else:
                print("Policy:" + str(self.policy))
                self.velocity_profiler()

                
                if self.old_policy != self.policy:
                    self.old_policy = self.policy
                    self.policy_check = True
                else:
                    self.policy_check = False


                if self.emergency == 0:
                    if self.policy_check:
                        self.w = []
                        print("Policy updated")
                        self.get_params()
                        self.trajectoryPlanner()
                        old_trajx = self.trajx
                        self.msg.y_start = self.path_y
                        self.msg.y_finish = self.trajyf_lines
                        self.msg.x_start = self.path_x
                        self.msg.x_finish = self.trajxf_lines
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
                            for i in range(len(self.msg.x_start)):
                                path_writer.writerow([self.msg.x_start[i],self.msg.y_start[i],self.w[i]])
                

                else:
                    self.w = []
                    print("EMERGENCY !!***!!")
                    self.get_params()
                    self.emergency_planner()
                    old_trajx = self.trajx
                    self.msg.y_start = self.path_y
                    self.msg.y_finish = self.trajyf_lines
                    self.msg.x_start = self.path_x
                    self.msg.x_finish = self.trajxf_lines
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
                        for i in range(len(self.msg.x_start)):
                            path_writer.writerow([self.msg.x_start[i],self.msg.y_start[i],self.w[i]])
                #if counter < 2:

                #print("Path X: " + str(self.path_x))
                #print("Path Y: " + str(self.path_y))
                #print("Trajx: " + str(self.trajx))
                #print("Trajy: " + str(self.trajy))
                self.path_pub.publish(self.msg)
                self.timestep_pub.publish(self.timestep_msg)
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
