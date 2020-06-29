import numpy as np
import matplotlib.pyplot as plt
import os

def read_file(file_path):
	costs = []
	times = []

	with open(file_path, 'r') as f:
		for l in f.readlines():
			#if 'STARTING ITERATION' in l:
				#it = float(l[l.find('ITERATION')+10:len(l)-5].strip())
			if 'New tour cost = ' in l:
				cost = float(l[l.find('= ')+2:len(l)-5].strip())
			elif 'Calculation time was:' in l:
				time = float(l[l.find(':\t')+3:l.find('ms')].strip())
				costs.append(cost)
				times.append(float(l[l.find(':\t')+3:l.find('ms')].strip()))

	return costs, times
	
def plot(dir_path, title, param):
	files = [f for f in os.listdir(dir_path) if '.txt' in f]
	iters = np.arange(10,50)
	cmap = plt.get_cmap('gnuplot')
	colors = [cmap(i) for i in np.linspace(0,1,2*len(files)) if int(i*10)%2 == 0]
	plt.suptitle(title)
	
	for f,c in zip(files,colors):
		print(f)
		costs, times = read_file(dir_path+'/'+f)
		
		if 'incidenceAngle' or 'SpaceSize' in f:
			lab = f.replace('_','=')[f.find(param):f.find('.txt')]
			lab = lab.replace('-', '/')
		else:
			lab = f.replace('_','=')[f.find(param):f.find('_', f.find(param)+len(param)+2)]
			if '05' in f:
				lab = lab.replace('05', '0.5')

		plt.subplot(211)
		plt.title('Tour cost as function as number of iterations')
		plt.plot(iters, costs[:40], 'o-', label=lab, color=c)
		plt.xlabel('LKH number of iterations')
		plt.ylabel('LKH tour cost')
		plt.grid()
		plt.legend(loc='best')
		plt.subplot(212)		
		plt.title('Calculation time as function as number of iterations')
		plt.plot(iters, times[:40], 'o-', label=lab, color=c)
		plt.xlabel('LKH number of iterations')
		plt.ylabel('Calculation time (ms)')
		plt.grid()
		plt.legend(loc='best')
	
	plt.show()


############################################################################################
dir_path = 'minDist_test'
title = 'Fixed parameters: SpaceArea=[353.841728,245.546589,358.287254], maxDist=70, IncidenceAngle=PI/6' 
param = 'minDist'
#plot(dir_path, title, param)
# 1

dir_path = 'maxDist_test'
title = 'Fixed parameters: SpaceArea=[353.841728,245.546589,358.287254], minDist=1, IncidenceAngle=PI/6' 
param = 'maxDist'
#plot(dir_path, title, param)
# 80 or 100

dir_path = 'incidenceAngle_test'
title = 'Fixed parameters: SpaceArea=[353.841728,245.546589,358.287254], minDist=1, maxDist=80' 
#param = 'incidenceAngle'
#plot(dir_path, title, param)
# PI/9

dir_path = 'spaceSize_test'
title = 'minDist=1, maxDist=80, incidenceAngle=PI/9' 
param = 'spaceSize'
plot(dir_path, title, param)

