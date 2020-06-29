import numpy as np
import matplotlib.pyplot as plt
import os


###########################################################
# Max arete triangle
#"""
maxDist = 0
s = np.zeros((3,3))
i = 0

with open('/home/antoine/catkin_ws_repos/src/StructuralInspectionPlanner/request/meshes/BigBen.stl', 'r') as f:
	for l in f.readlines():
		if 'vertex' in l:
			s[i] = l.strip().split()[1:]
			i += 1
		elif 'endloop' in l:
			i = 0
			d1 = np.linalg.norm(s[0]-s[1])
			d2 = np.linalg.norm(s[1]-s[2])
			d3 = np.linalg.norm(s[0]-s[2])
			maxDist = max([maxDist, d1, d2, d3])
			
print(maxDist)			
###"""
###########################################################
# Centre gravite
#"""
maxDist = 0
s = np.zeros((3,3))
i = 0

with open('/home/antoine/catkin_ws_repos/src/StructuralInspectionPlanner/request/meshes/BigBen.stl', 'r') as f:
	for l in f.readlines():
		if 'vertex' in l:
			s[i] = l.strip().split()[1:]
			i += 1
		elif 'endloop' in l:
			i = 0
			g = [(s[0][0]+s[1][0]+s[2][0])/3.0 , (s[0][1]+s[1][1]+s[2][1])/3.0, (s[0][2]+s[1][2]+s[2][2])/3.0]
			d1 = np.linalg.norm(s[0]-g)
			d2 = np.linalg.norm(s[1]-g)
			d3 = np.linalg.norm(s[2]-g)
			print(d1)
			print(d2)
			print(d3)
			maxDist = max([maxDist, d1, d2, d3])
			
print(maxDist)			
#"""
