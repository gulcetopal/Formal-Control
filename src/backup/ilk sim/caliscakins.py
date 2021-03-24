#!/usr/bin/env python
import pandas as pd
import numpy as np

class MDPNode():

    def __init__(self,x ,y):
        self.x = x
        self.y = y

	
class Policy():
	def __init__(self):
		self.main()

	def searchPolicy(self, init_node, goal_node):
		policy = []
		neighbour_list = []
		self.n = 'n'
		i = 0
		self.node = self.n + str(i)
		data = {'Cost': [0], self.node: [goal_node]}
		table = pd.DataFrame(data)
		i = i + 1
		
		for neighbor in neighbour_list:
			appendSearch(neighbour_list, goal_node, i)
		#3goal_node 
			 

	#append search fonks kaldır n0'i da katıp search policyde iteratif yap
	#bitiş noktasi ekle 
	def appendSearch(self, goal_list, i):
		node = self.node
		for goal_node in goal_list:
			rows = table[table.node == goal_node]
			neighbour_list = self.neighbourSearch(goal_node)
			for j in range(int(len(neighbour_list)-1)):
				table = table.append(rows, ignore_index = True)
		newnode = n + str(i)
		table = table.assign(newnode=np.nan)
		for goal_node in goal_list:	
			table[newnode][table.node == goal_node] = self.neighbourSearch(goal_node)
		i = i + 1
		goal_list = self.neighbourSearch(goal_node)	
		print(goal_list)

	def neighborSearch(self, goal_node, edges):
		neighbour_list = []
		for edge in edges:
			if edge.state_2.nav == goal_node:
				neighbour_list = neighbour_list.append(edge.state_1.nav)
		return neighbour_list

	def main(self):
		v1 = MDPNode(-0.5,-5.66)
		v2 = MDPNode(-2.5,-5.66)
		v3 = MDPNode(0.8,-5.66)
		v4 = MDPNode(2.8,-5.66)
		v5 = MDPNode(-0.5,-0.7)
		v6 = MDPNode(-2.5,-0.7)
		v7 = MDPNode(0.8,-0.7)
		v8 = MDPNode(2.8,-0.7)

		init_node = v1
		goal_node = v8

		self.searchPolicy(v1, v8)

	if __name__ == '__main__':
		Policy()


