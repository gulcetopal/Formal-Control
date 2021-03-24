#!/usr/bin/env python

import rospy
import rosparam
import os
from policy_generator import PolicyGenerator
from prism_talker import PrismTalker

class PolicyExecutor():
    def __init__(self, rfdist = 0, lfdist = 0, bdist = 0, v = 0, init_state = 0, lane = 1, policy = [] ):
        self.export_dir = rospy.get_param('Directory/policy')
        self.init_state = init_state
        self.policy = policy
        self.rfdist = rfdist
        self.lfdist = lfdist
        self.bdist = bdist
        self.lane = lane
        self.v = v
        self.main()

    def SetInit(self,init_state, init_lane):
        self.init_state = init_state
        self.lane = init_lane

    def Policy(self):
        talker = PrismTalker(self.rfdist,self.rfdist,self.bdist,self.v)
        talker.SetParams(self.init_state, self.lane, self.rfdist,self.lfdist,self.bdist,self.v)
        print("Initial state: " + str(self.init_state))
        talker.CallPrism()
        generator = PolicyGenerator()
        generator.export_dir = talker.export_dir
        generator.Parser()
        self.policy = generator.policy

    def main(self):
        self.Policy()

if __name__ == '__main__':
    PolicyExecutor()
