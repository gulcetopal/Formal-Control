#!/usr/bin/env python

import rospy
import rosparam
from std_msgs.msg import String
from state_mach.msg import SelfStateMsg
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from sensor_msgs.msg import LaserScan
from mdpclass import Node, State, Edge
import actionlib
import math
import tf

client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

class MDPNode():

    def __init__(self,x ,y):
        self.x = x
        self.y = y

class MDPState():

    def __init__(self,nav, act):
        self.nav = nav
        self.act = act

class MDPEdge():

    def __init__(self,state_1,state_2, len, prob, door_guard):
        self.state_1 = state_1
        self.state_2 = state_2
        self.len = len
        self.prob = prob
        self.door_guard = door_guard

    def CostFnc(len, prob):
        cost = len*prob
        self.cost = cost
        return cost

class Robot():

    def __init__(self):
        rospy.init_node('mdp_v1')
        ####### Init,pubs and subs
        self.policy = []
        self.latest_scan = None
        #self.init_node = v1
        #self.goal_node = [v2,v8]
        self.x = 0
        self.y = 0
        self.main()

    def LidarCallback(self,data):
        self.latest_scan = data.ranges

    def Odomcallback(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        self.x = x
        self.y = y
    def CheckDoorProb(self):
        prob = [1,10]
        prob = rand(prob)

        if( 9 <= prob < 10):
            door_stat = true
        else:
            door_stat = false
        return door_stat

    def CheckDoorLidar(self, edge):
        scan_data = list(self.latest_scan)
        dist = float(math.sqrt(math.pow((edge.state_2.nav.x - edge.state_1.nav.x), 2) + math.pow((edge.state_2.nav.y - edge.state_1.nav.y), 2)))
        if scan_data[314] >= dist:
            door_stat = 1;
        else:
            door_stat = 0;

        return door_stat

    def searchPolicy(self):

        CostFnc()

        return policy

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
        while self.latest_scan is None:
            rospy.sleep(0.1)

####### Creating MDP and Objects
        v1 = MDPNode(-0.5,-5.66)
        v2 = MDPNode(-2.5,-5.66)
        v3 = MDPNode(0.8,-5.66)
        v4 = MDPNode(2.8,-5.66)
        v5 = MDPNode(-0.5,-0.7)
        v6 = MDPNode(-2.5,-0.7)
        v7 = MDPNode(0.8,-0.7)
        v8 = MDPNode(2.8,-0.7)

        n1 = MDPState(v1,v2)
        n2 = MDPState(v2,v2)
        n3 = MDPState(v3,v2)
        n4 = MDPState(v4,v2)
        n5 = MDPState(v5,v2)
        n6 = MDPState(v6,v2)
        n7 = MDPState(v7,v2)
        n8 = MDPState(v8,v2)
        n9 = MDPState(v1,v8)
        n10 = MDPState(v2,v8)
        n11 = MDPState(v3,v8)
        n12 = MDPState(v4,v8)
        n13 = MDPState(v5,v8)
        n14 = MDPState(v6,v8)
        n15 = MDPState(v7,v8)
        n16 = MDPState(v8,v8)
    #    n17 = MDPState(v0,v2)
    #    n18 = MDPState(v0,v8)

        e1 = MDPEdge(n1,n2,1,1,1)
        e2 = MDPEdge(n2,n1,1,1,1)
        e3 = MDPEdge(n1,n7,1,1,0)
        e4 = MDPEdge(n7,n8,1,1,1)

        #e5 = MDPEdge(v1,v2), e6 = MDPEdge(v1,v2)
        #e7 = MDPState(v1,v2), e8 = MDPState(v1,v2), e9 = MDPState(v1,v2), e10 = MDPState(v1,v2), e11 = MDPState(v1,v2), e12 = MDPState(v1,v2)

#######
        global client
        client.wait_for_server()
#        policy = searchPolicy(self.init_node,self.goal_node)
        policy = [e1,e2,e3,e4] #test case
        while not rospy.is_shutdown():
            for policy_edge in policy:
                if(policy_edge.door_guard):
                    self.yaw_goal = math.atan2((policy_edge.state_2.nav.y - self.y),(policy_edge.state_2.nav.x - self.x))
                    print("*******")
                    self.send_move_base(policy_edge.state_1.nav.x,policy_edge.state_1.nav.y,self.yaw_goal)
                    print("*******")
                    if(self.CheckDoorLidar(policy_edge)):
                        self.send_move_base(policy_edge.state_2.nav.x,policy_edge.state_2.nav.y, self.yaw_goal)
                    else:
                        print("Door Closed :(")
                        break
                else:
                    self.send_move_base(policy_edge.state_2.nav.x,policy_edge.state_2.nav.y,policy_edge.yaw_goal)

                while (policy_edge.state_2.nav.x != self.x and policy_edge.state_2.nav.y != self.y):
                    rate.sleep()

                if(i == len(policy)-1):
                    rospy.loginfo("Current Policy: " + str(policy_edge))
                    rospy.loginfo(str(policy_edge.state_2.nav) + " is reached," + "Next stop: Wonderland" )
                else:
                    rospy.loginfo(str(policy_edge.state_2.nav) + " is reached, Terminate...")

        rospy.spin()

if __name__ == '__main__':
	Robot()
