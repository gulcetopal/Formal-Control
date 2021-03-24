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

#client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

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
        self.goal_tolerance = 0.2
        self.dist = 0
        self.policy = []
        self.latest_scan = None
        #self.init_node = v1
        #self.goal_node = [v2,v8]
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

    def go_forward(self, state, speed= 0.5, secs = 0.5 ):
        rate = rospy.Rate(10)
        self.twist.linear.x = speed
        for i in range(int(secs * 10)):
            if self.control_dist(state):
                self.twist_pub.publish(self.twist)
                rate.sleep()
            else:
                break
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

    def searchPolicy(self):
        # Once amelece ara, Sonra optimal routelari ara
        CostFnc()

        return policy

    def main(self):
        rate = rospy.Rate(10)
        rospy.Subscriber("/scan", LaserScan, self.LidarCallback)
        rospy.Subscriber("/odometry/filtered", Odometry, self.Odomcallback)
        rospy.Subscriber("/imu/data", Imu, self.imuCallback)

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
#        global client
#        client.wait_for_server()
#        policy = searchPolicy(self.init_node,self.goal_node)
        policy = [e1,e2,e3,e4] #test case
        while not rospy.is_shutdown():
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

                    print("Target Y: " + str(policy_edge.state_2.nav.y))
                    print("Y: " + str(self.y))
                    print("Target X: " + str(policy_edge.state_2.nav.x))
                    print("X: " + str(self.x))
                    self.yaw_goal = math.atan2((policy_edge.state_2.nav.y - self.y),(policy_edge.state_2.nav.x - self.x))

                    self.turn_to_yaw(self.yaw_goal,policy_edge.state_2)

#                    self.send_move_base(policy_edge.state_1.nav.x,policy_edge.state_1.nav.y,self.yaw_goal)
                    print("Checking Door..")
                    if(self.CheckDoorLidar(policy_edge)):
                        while(self.control_dist(policy_edge.state_2)):
                            self.go_forward(policy_edge.state_2)
                        print("Node Reached, Next stop: Wonderland.. ")

#                       self.send_move_base(policy_edge.state_2.nav.x,policy_edge.state_2.nav.y, self.yaw_goal)
                    else:
                        print("Door Closed :(")
                        break
                else: # Engelsiz gitme actioni
                    self.yaw_goal = math.atan2((policy_edge.state_2.nav.y - self.y),(policy_edge.state_2.nav.x - self.x))
                    self.turn_to_yaw(self.yaw_goal,policy_edge.state_2)
                    while(self.control_dist(policy_edge.state_2)):
                        self.go_forward(policy_edge.state_2)
                    print("Node Reached, Next stop: Wonderland.. ")

#                    self.send_move_base(policy_edge.state_2.nav.x,policy_edge.state_2.nav.y,self.yaw_goal)
            rate.sleep()

if __name__ == '__main__':
	Robot()
