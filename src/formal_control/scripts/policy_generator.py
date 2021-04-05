#!/usr/bin/env python

import rospy
import os
import sys

class PolicyGenerator():
    def __init__(self, policy = [], actions = []):
        self.export_dir = rospy.get_param('Directory/policy')
        self.policy = policy
        self.main()

    def Parser(self):
        file = open(self.export_dir, mode = 'r')
        lines = file.readlines()
        lines.remove(lines[0]) # 0=Vehicle - 1=Timestep - 2=State
        self.policy = []
        self.actions = []
        for line in lines:
            line = line.split(' ')
            if not (line[0] == 'calculator'):
                self.policy.append(int(line[7]))
                if line[0] == '[fr]':
                    self.actions.append(0)
                elif line[0] == '[f]':
                    self.actions.append(1)
                elif line[0] == '[s]':
                    self.actions.append(2)
                elif line[0] == '[si]':
                    self.actions.append(3)
                elif line[0] == '[so]':
                    self.actions.append(4)

        print(self.policy)

    def main(self):
        self.Parser()

if __name__ == '__main__':
    PolicyGenerator()
