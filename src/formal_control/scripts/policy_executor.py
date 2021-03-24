#!/usr/bin/env python

import rospy
import os
from policy_generator import PolicyGenerator
from prism_talker import PrismTalker

class PolicyExecutor():
    def __init__(self, rfdist = 0, lfdist = 0, bdist = 0, v = 0, init_state = 0,export_dir = '/home/gulce/prism-games/prism-games/prism/policy_result.txt',policy = [] ):
        #rospy.init_node('policy_executor')
        self.export_dir = export_dir
        self.init_state = init_state
        self.policy = policy
        self.rfdist = rfdist
        self.lfdist = lfdist
        self.bdist = bdist
        self.v = v
        self.main()

    def SetInit(self,init_state):
        self.init_state = init_state

    def Policy(self):
        talker = PrismTalker(self.rfdist,self.rfdist,self.bdist,self.v)
        talker.SetParams(self.init_state,self.rfdist,self.lfdist,self.bdist,self.v)
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
