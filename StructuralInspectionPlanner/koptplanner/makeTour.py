import numpy as np
import math
import tf

VPs = np.zeros((397, 8))

with open('/home/antoine/catkin_ws_repos/src/StructuralInspectionPlanner/koptplanner/data/planTour/goodTempTour.txt', 'r') as f:
	i = 0
	for l in f.readlines():
		VPs[i][0] = int(l.strip())-1
		i+=1
		
with open('/home/antoine/catkin_ws_repos/src/StructuralInspectionPlanner/koptplanner/data/planTour/goodTour.txt', 'r') as f:
	i = 0
	for l in f.readlines():
		if i%2 == 0:
			rot = l.strip().split(',')
			q = tf.transformations.quaternion_from_euler(float(rot[3]), float(rot[4]), float(rot[5]))
			VPs[i/2][1] = float(rot[0])
			VPs[i/2][2] = float(rot[1])
			VPs[i/2][3] = float(rot[2])
			VPs[i/2][4:] = q
		i+=1
		
with open('/home/antoine/catkin_ws_repos/src/StructuralInspectionPlanner/koptplanner/data/niceTour.txt', 'w') as f:
	for vp in VPs:
		for i in vp:
			f.write(str(i)+"\t")
		f.write("\n")

for v in range(1,397):
	with open('/home/antoine/catkin_ws_repos/src/StructuralInspectionPlanner/koptplanner/viewpoints/viewpoint_'+str(v)+'.txt', 'a') as f:
		for vp in VPs:
			if v == vp[0]:
				for i in vp[1:]:
					f.write(str(i)+"\t")
				f.write("\n")
