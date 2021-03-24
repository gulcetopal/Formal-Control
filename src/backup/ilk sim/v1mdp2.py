#!/usr/bin/env python

import rospy
import rosparam
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from sensor_msgs.msg import LaserScan, Imu
import actionlib
import math
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import pandas as pd
import heapq

#client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

class MDPNode():

    def __init__(self, x, y, visited = False, neighbour_list = [],previous = []):
        self.x = x
        self.y = y
        self.neighbour_list = neighbour_list
        self.previous = previous
        self.visited = visited

class MDPState():

    def __init__(self, nav, act):
        self.nav = nav
        self.act = act

class MDPEdge():

    def __init__(self, state_1, state_2, len = 0, prob = 1, door_guard = 1) :
        self.state_1 = state_1
        self.state_2 = state_2
        self.len = len
        self.prob = prob
        self.door_guard = door_guard
        self.cost = 0

    def CostFnc(self):
        cost = self.len*self.prob
        self.cost = cost
        return cost


class Robot():

    def __init__(self):
        rospy.init_node('mdp_v1')
        ####### Init,pubs and subs
        self.goal_tolerance = 0.3
        self.dist = 0
        self.policy = []
        self.path = []
        self.latest_scan = None
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.twist = Twist()
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 50)

        self.main()

    def LidarCallback(self,data):
        self.latest_scan = data.ranges

    def Odomcallback(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        self.x = x
        self.y = y

    def fix_yaw(self, yaw):
        while not (yaw < math.pi and yaw >= -math.pi):
            if yaw >= math.pi:
                yaw -= math.pi * 2
            elif yaw < -math.pi:
                yaw += math.pi * 2
            else:
                break
        return yaw

    def imuCallback(self, data):
	    [roll, pitch, self.yaw] = euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]) #Convert quaternion to euler angles
	    self.yaw = self.fix_yaw(self.yaw)

    def reset_twist(self):
        self.twist.linear.x  = 0
        self.twist.linear.y  = 0
        self.twist.linear.z  = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0
        self.twist_pub.publish(self.twist)

    def control_dist(self, state):
        self.dist = float(math.sqrt(math.pow((state.nav.x - self.x), 2) + math.pow((state.nav.y - self.y), 2)))
        return self.dist > self.goal_tolerance

    def go_forward(self, state, speed= 0.5, secs = 0.15 ):
        rate = rospy.Rate(10)
        self.twist.linear.x = speed
        for i in range(int(secs * 10)):
            if self.control_dist(state):
                self.twist_pub.publish(self.twist)
                rate.sleep()
            else:
                self.reset_twist()
        print("Distance to target: " + str(self.dist))

    def turn(self, speed, secs):
        rate = rospy.Rate(10)
        self.twist.angular.z = speed
        for i in range(int(secs * 10)):
            self.twist_pub.publish(self.twist)
            rate.sleep()
        self.reset_twist()

    def turn_to_yaw(self, target_yaw, state, angle_error_threshold = math.pi / (48)):
        print("Current yaw: {}\nTurning to: {}".format(self.yaw, target_yaw))
        yaw_dist = self.yaw - target_yaw
        yaw_dist_complement = (math.pi * 2 - yaw_dist) * -1

        if yaw_dist > math.pi or yaw_dist < -math.pi:
            yaw_dist = yaw_dist_complement

        while (yaw_dist > angle_error_threshold) or (yaw_dist < -1 * angle_error_threshold):
            if not self.control_dist(state):
                break
#            print("Yaw ayari")
            if (self.yaw < 0 and target_yaw > 0 and (yaw_dist > math.pi or yaw_dist < -math.pi)):
                self.turn(math.copysign(0.5, yaw_dist), 0.15)
            else:
                self.turn(math.copysign(0.5, -1 * yaw_dist), 0.15)
            self.yaw = self.fix_yaw(self.yaw)
            yaw_dist = self.yaw - target_yaw
            yaw_dist_complement = (math.pi * 2 - yaw_dist) * -1

            if yaw_dist > math.pi or yaw_dist < -math.pi:
                yaw_dist = yaw_dist_complement

    def CheckDoorLidar(self, edge):
        scan_data = list(self.latest_scan)
        dist = float(math.sqrt(math.pow((edge.state_2.nav.x - edge.state_1.nav.x), 2) + math.pow((edge.state_2.nav.y - edge.state_1.nav.y), 2)))
        if scan_data[314] >= dist:
            door_stat = 1;
        else:
            door_stat = 0;

        return door_stat

    def shortest(self, node, path):
        if node.neighbour_list:
            path.append(node.previous)
            print("xxxxx")
            self.shortest(node.previous, path)

    def PolicyGenerator(self, init_node, goal_node, edges):
        if init_node != goal_node:
            cost = 1000
            node = init_node
            edge_handler = None
            for nd in node.neighbour_list:
                for edge in edges:
                    if edge.state_1.nav == node and edge.state_2.nav == nd and edge.state_2.nav.visited == False:
                        if edge.CostFnc() <= cost:
                            cost = edge.CostFnc()
                            edge_handler = edge
                            self.path.append(edge_handler)
                            edge.state_2.nav.visited = True
            self.PolicyGenerator(edge_handler.state_2.nav,goal_node,edges)

    def send_move_base(self, x, y , yaw ):

    	global client
    	goal=MoveBaseGoal()
    	goal.target_pose.header.frame_id = "/odom"
    	goal.target_pose.pose.position.x = x
    	goal.target_pose.pose.position.y = y
    	goal.target_pose.pose.position.z = 0

    	q = tf.transformations.quaternion_from_euler(0,0,yaw)
    	goal.target_pose.pose.orientation.x = q[0]
    	goal.target_pose.pose.orientation.y = q[1]
    	goal.target_pose.pose.orientation.z = q[2]
    	goal.target_pose.pose.orientation.w = q[3]

    	client.send_goal(goal)

    def main(self):
        rate = rospy.Rate(10)
        rospy.Subscriber("/scan", LaserScan, self.LidarCallback)
        rospy.Subscriber("/odometry/filtered", Odometry, self.Odomcallback)
        rospy.Subscriber("/imu/data", Imu, self.imuCallback)

        while self.latest_scan is None:
            rospy.sleep(0.1)

####### Creating MDP and Objects
        v00 = MDPNode(0,0)
        v1 = MDPNode(-0.5,-5.66)
        v2 = MDPNode(-2.5,-5.66)
        v3 = MDPNode(0.8,-5.66, True)
        v4 = MDPNode(2.8,-5.66)
        v5 = MDPNode(-0.5,-0.7)
        v6 = MDPNode(-2.5,-0.7)
        v7 = MDPNode(0.8,-0.7)
        v8 = MDPNode(2.8,-0.7)

        goal_1 = v2
        goal_2 = v8

        n00 = MDPState(v00,goal_1)
        n1 = MDPState(v1,goal_1)
        n2 = MDPState(v2,goal_1)
        n3 = MDPState(v3,goal_1)
        n4 = MDPState(v4,goal_1)
        n5 = MDPState(v5,goal_1)
        n6 = MDPState(v6,goal_1)
        n7 = MDPState(v7,goal_1)
        n8 = MDPState(v8,goal_1)
        n9 = MDPState(v1,goal_2)
        n10 = MDPState(v2,goal_2)
        n11 = MDPState(v3,goal_2)
        n12 = MDPState(v4,goal_2)
        n13 = MDPState(v5,goal_2)
        n14 = MDPState(v6,goal_2)
        n15 = MDPState(v7,goal_2)
        n16 = MDPState(v8,goal_2)
    #    n17 = MDPState(v0,goal_1)
    #    n18 = MDPState(v0,goal_2)

        e1 = MDPEdge(n1,n2,2,0.9,1)
        e2 = MDPEdge(n2,n1,2,0.9,1)
        e3 = MDPEdge(n1,n7,26.65,1,0)
        e4 = MDPEdge(n7,n1,26.65,1,0)
        e5 = MDPEdge(n7,n8,2,0.9,1)
        e6 = MDPEdge(n8,n7,2,0.9,1)
        e7 = MDPEdge(n4,n3,2,0.9,1)
        e8 = MDPEdge(n3,n4,2,0.9,1)
        e9 = MDPEdge(n6,n5,2,0.9,1)
        e10 = MDPEdge(n5,n6,2,0.9,1)
        e11 = MDPEdge(n3,n5,26.65,1,1)
        e12 = MDPEdge(n5,n3,26.65,1,1)
        e13 = MDPEdge(n3,n7,4.94,1,1)
        e14 = MDPEdge(n7,n3,4.94,1,1)
        e15 = MDPEdge(n1,n5,4.94,1,1)
        e16 = MDPEdge(n5,n1,4.94,1,1)
        e17 = MDPEdge(n1,n3,0.3,1,1)
        e18 = MDPEdge(n3,n1,0.3,1,1)
        e19 = MDPEdge(n5,n7,0.3,1,1)
        e20 = MDPEdge(n7,n5,0.3,1,1)

        #e5 = MDPEdge(v1,v2), e6 = MDPEdge(v1,v2)
        #e7 = MDPState(v1,v2), e8 = MDPState(v1,v2), e9 = MDPState(v1,v2), e10 = MDPState(v1,v2), e11 = MDPState(v1,v2), e12 = MDPState(v1,v2)

        edges = [e5,e1,e2,e6,e7,e8,e9,e10,e11,e12,e13,e14,e15,e16,e17,e18,e19,e20]
#######
        self.init_node = v1
        self.goal_node = v8

        e_return = MDPEdge(self.goal_node,n00)

        for edge in edges:
            edge.state_1.nav.neighbour_list.append(edge.state_2.nav)
            edge.state_2.nav.previous.append(edge.state_1.nav)

        self.PolicyGenerator(self.init_node,self.goal_node,edges)
#        policy = [e1,e2,e3,e4] #test case
        print(len(self.path))
        policy = self.path

#######

        print("Target Y: " + str(policy[0].state_1.nav.y))
        print("Y: " + str(self.y))
        print("Target X: " + str(policy[0].state_1.nav.x))
        print("X: " + str(self.x))
        self.yaw_goal = math.atan2((policy[0].state_1.nav.y - self.y),(policy[0].state_1.nav.x - self.x))
        self.turn_to_yaw(self.yaw_goal,policy[0].state_1)
        while(self.control_dist(policy[0].state_1)):
            self.go_forward(policy[0].state_1)
        print("-->> Init Node Reached, Next stop: Wonderland.. ")

        for policy_edge in policy:
            rospy.loginfo("Current Policy: " )
            if(policy_edge.door_guard): # Kapidan gecme actioni

                print("-->> Target Y: " + str(policy_edge.state_2.nav.y))
                print("-->> Y: " + str(self.y))
                print("-->> Target X: " + str(policy_edge.state_2.nav.x))
                print("-->> X: " + str(self.x))
                self.yaw_goal = math.atan2((policy_edge.state_2.nav.y - self.y),(policy_edge.state_2.nav.x - self.x))

                self.turn_to_yaw(self.yaw_goal,policy_edge.state_2)

#                    self.send_move_base(policy_edge.state_1.nav.x,policy_edge.state_1.nav.y,self.yaw_goal)
                print("..Checking Door..")
                if(self.CheckDoorLidar(policy_edge)):
                    while(self.control_dist(policy_edge.state_2)):
                        self.go_forward(policy_edge.state_2)
                    print("-->> Node Reached, Next stop: Wonderland.. ")

#                       self.send_move_base(policy_edge.state_2.nav.x,policy_edge.state_2.nav.y, self.yaw_goal)
                else:
                    print("-->> Door Closed :(")
                    break
            else: # Engelsiz gitme actioni
                self.yaw_goal = math.atan2((policy_edge.state_2.nav.y - self.y),(policy_edge.state_2.nav.x - self.x))
                self.turn_to_yaw(self.yaw_goal,policy_edge.state_2)
                while(self.control_dist(policy_edge.state_2)):
                    self.go_forward(policy_edge.state_2)
                print("-->> Node Reached, Next stop: Wonderland.. ")
        if(self.control_dist(policy[len(policy)-1].state_2)):
            print("-->> Mission Accomplished.. ;)  ")
#                    self.send_move_base(policy_edge.state_2.nav.x,policy_edge.state_2.nav.y,self.yaw_goal)

        rospy.spin()

if __name__ == '__main__':
	Robot()
