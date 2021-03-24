#!/usr/bin/env python

import rospy
import os
import sys
from formal_control.msg import SelfStateMsg


class PrismTalker():
    def __init__(self, rfdist = 0, lfdist = 0, bdist = 0, v = 0, init_state = 0, dir = "/home/gulce/final_ws/src/prism", model = "mdp_v2.nm", spec = "spec.pctl", export_dir = "/home/gulce/prism-games/prism-games/prism/policy_result.txt"):
        #rospy.init_node('prism_node')
        #rospy.init_node('Mahmut')
        self.dir = dir
        self.model = model
        self.spec = spec
        self.export_dir = export_dir
        self.init_state = init_state
        self.rfdist = rfdist
        self.lfdist = lfdist
        self.bdist = bdist
        self.v = v
        self.main()

    def SetParams(self, init_state, rfdist, lfdist, bdist, v):
        self.init_state = init_state
        self.rfdist = rfdist
        self.lfdist = lfdist
        self.bdist = bdist
        self.v = v
        print(str(self.init_state))
        """
        print("F_dist: "+ str(self.fdist))
        print("B_dist: "+ str(self.bdist))
        print("Relative V: "+ str(self.v))
        """

    def CallPrism(self):
        rospy.loginfo("Starting PRISM...")
        #param = bdist
        rfdist = self.rfdist
        lfdist = self.lfdist
        bbdist = self.bdist
        v = self.v
        ii = self.init_state
        #ii = 3
        cmd = "rrfdist="+ str(rfdist) +",llfdist="+ str(lfdist) +",bbdist="+str(bbdist)+",vv="+str(v)+ ",init_s="+ str(ii)
        #cmd = "init_s="+ str(ii)
        #cmd = "init_s="+ str(3)
        os.system("cd " + self.dir)
        os.chdir(self.dir)
        os.system("bin/prism " + self.model +" "+ self.spec + " -const "+ cmd + " -simpath 20"+" "+self.export_dir)
        print("\n")
        rospy.loginfo("Behavioural Plan Calculated !")
        print("\n")

    def main(self):
        rate = rospy.Rate(10)
        self.dir = "/home/gulce/prism-games/prism-games/prism"
        self.CallPrism()

if __name__ == '__main__':
    PrismTalker()
