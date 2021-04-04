#!/usr/bin/env python

import rospy
import rosparam
import os
import sys
from formal_control.msg import SelfStateMsg


class PrismTalker():
    def __init__(self, rfdist = 0, lfdist = 0, bdist = 0, v = 0, init_state = 0, lane = 1):
        self.dir = rospy.get_param('Directory/prism')
        self.model = rospy.get_param('Prism/model')
        self.spec = rospy.get_param('Prism/spec')
        self.export_dir = rospy.get_param('Directory/policy')
        self.init_state = init_state
        self.lane = lane
        self.rfdist = rfdist
        self.lfdist = lfdist
        self.bdist = bdist
        self.v = v
        self.main()

    def SetParams(self, init_state, lane, rfdist, lfdist, bdist, v):
        self.init_state = init_state
        self.lane = lane
        self.rfdist = rfdist
        self.lfdist = lfdist
        self.bdist = bdist
        self.v = v
        #print(str(self.init_state))

    def CallPrism(self):
        rospy.loginfo("Starting PRISM...")
        rfdist = self.rfdist
        lfdist = self.lfdist
        bbdist = self.bdist
        v = self.v
        ii = self.init_state
        i_lane = self.lane
        cmd = "rrfdist="+ str(rfdist) +",llfdist="+ str(lfdist) +",bbdist="+str(bbdist)+",vv="+str(v)+ ",init_s="+ str(ii) + ",init_i="+ str(i_lane)

        os.system("cd " + self.dir)
        os.chdir(self.dir)
        os.system("bin/prism " + self.model +" "+ self.spec + " -const "+ cmd + " -simpath 8"+" "+self.export_dir)
        print("\n")
        rospy.loginfo("Behavioural plan calculated!")
        print("\n") 

    def main(self):
        rate = rospy.Rate(10)
        self.dir = rospy.get_param('Directory/prism')
        self.CallPrism()

if __name__ == '__main__':
    PrismTalker()
