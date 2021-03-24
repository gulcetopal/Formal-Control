#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy

class Node():

    def __init__(self,x ,y):
        self.x = x
        self.y = y

class State():
	def __init__(self, nav_pt, act_pt)
		self.nav_pt = nav_pt
        self.act_pt = act_pt




class Edge():
	def __init__(self, init_state, goal_state, length, prob, guard)
		self.init_state = init_state
        self.goal_state = goal_state
        self.length = length
        self.prob = prob
        self.guard = guard



