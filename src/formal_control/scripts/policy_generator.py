#!/usr/bin/env python

import rospy
import os
import sys

class PolicyGenerator():
    def __init__(self, policy = []):
        self.export_dir = rospy.get_param('Directory/policy')
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
