#!/usr/bin/env python
import pandas as pd
import numpy as np

goal_node = 'v8'

data = {'Cost': [0], 'Goal': [goal_node]}
policy = []
neighbour_list = []
table = pd.DataFrame(data)

n = 'n'
i = 1

node = n + str(i)
neighbour_list = ['v7']
rows = table[table.Goal== 'v8']
for j in range(int(len(neighbour_list)-1)):
	table = table.append(rows, ignore_index = True)
table = table.assign(n1=np.nan)
table[node][table.Goal == 'v8'] = neighbour_list
i = i + 1
print(i)
print(table)

node = n + str(i)
neighbour_list = ['v1', 'v5']
rows = table[table.n1 == 'v7']
for j in range(int(len(neighbour_list)-1)):
	table = table.append(rows, ignore_index = True)

table = table.assign(n2=np.nan)
table[node][table.n1 == 'v7'] = neighbour_list
i = i + 1
print(i)
print(table)


node = n + str(i)
neighbour_list = ['v1', 'v7', 'v3', 'v6']
rows = table[table.n2 == 'v5']
for j in range(int(len(neighbour_list)-1)):
	table = table.append(rows, ignore_index = True)

table = table.assign(n3=np.nan)
table[node][table.n2 == 'v5'] = neighbour_list
i = i + 1
print(i)
print(table)


def searchPolicy(self, init_node, goal_node):
	policy = []
	neighbour_list = []
	n = 'n'
	i = 0
	node = n + str(i)
	data = {'Cost': [0], node: [goal_node]}
	table = pd.DataFrame(data)
	i = i + 1
	

	for edge in edges:
		if edge.state_2.nav == goal_node:
			neighbour_list = neighbour_list.append(edge.state_1.nav) #neighbour_list = ['v7']
	for neighbour in neighbour_list
		rows = table[table.node == goal_node]
		node = n + str(i)
		for j in range(int(len(neighbour_list)-1)):
			table = table.append(rows, ignore_index = True)
		table = table.assign(node=np.nan)
		table[node][table.Goal == goal_node] = neighbour_list
		goal_node = neighbour
	i = i + 1



        


