#!/usr/bin/env python

import rospy
import os
import sys

class PolicyGenerator():
    def __init__(self, export_dir = '/home/gulce/prism-games/prism-games/prism/policy_result.txt',policy = [] ):
        #rospy.init_node('policy_generator')
        self.export_dir = export_dir
        self.policy = policy
        self.main()

    def Parser(self):
        file = open(self.export_dir, mode = 'r')
        lines = file.readlines()
        lines.remove(lines[0]) # 0=Vehicle - 1=Timestep - 2=State
        self.policy = []
        for line in lines:
            line = line.split(' ')
            if not (line[0] == 'calculator'):
                self.policy.append(int(line[7]))

        print(self.policy)

    def main(self):
        self.Parser()

if __name__ == '__main__':
    PolicyGenerator()
