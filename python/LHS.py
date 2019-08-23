# Programmed by Antoine BALDO
#refined by Glen COLBY
#Generalized by Greg WILSON

import os 
import random
import numpy as np

'''
This function generates us an LHS matrix that is nV columns by nS rows.
However, it is a dictionary which is hard to work with if you don't know
python. So, I decided to refine it. check the function below for how to use.
'''
def LHS_Generator(nV,nS):
	# Var number (nV dimension) setpoint
	# You can change the dimension's size
	nV = nV

	# Sample number setpoint
	# You can change the Sample number
	nS = nS

	# Initialisation:
	k=1
	# Creation of a list dictionnary
	x = {}
	x[k] = []

	# Loop elements (part1)
	for i in range(1,(nV+1)):
			x1 = []

			for j in range(1,(nS+1)):
					a = ((float(j)-1)/nS)
					b = ((float(j))/nS)
					listesample = random.uniform(a,b)
					x1.append(listesample)

			# Select a random number nP times between each Sample and for each Var (part2)
			for k in range(1,nS+1):
					listechoice = random.choice(x1)
					x.setdefault(k, []).append(listechoice)
					x1.remove(listechoice)
			#print 'var %d:' % i
			#x2 = [ '%.6f' % elem for elem in x1]
			# print("%.2f" % x1)
			#print x2
	
	return x
'''
You need the following lines of code-

1. Make sure your main.py and this file (betterLHS.py) are in the same folder

2. at the beginning,

from betterLHS import real_LHS

3. Whenever you want to get a matrix later on,

LHS_Matrix = real_LHS(Bounds, Size)
which will give an N x n matrix of design variables.

Bounds = #Design Variable(n) x 2 matrix that is ordered in such a way that all design variables have min,max bounds
and an integer indicator 0 for real number, 1 for integer:
[[min1,max1,int?],[min2,max2,int?],...,[minn,maxn,int?]]

 Size = N (Amount of sets of designs you want)
 
 The function creates an LHS text file as well if you would rather run this function once then just read from files each time.
 Reading from a file means your LHS won't be changing with each run. You will make one true LHS so runs remain consistent.
'''
def real_LHS(Bounds,Size):
	n = len(Bounds)
	rand_matrix = LHS_Generator(n,Size)
	
	DataFile = open('LHS.txt', 'w')
	DataFile.close()

	LHS = [[0 for y in range(n)] for x in range(Size)]
	DataFile = open('LHS.txt','a')
	for i in range(Size):
		for j in range(n):
			if Bounds[j][2] == 1:
				LHS[i][j] = int(round((Bounds[j][1] - Bounds[j][0])*rand_matrix[i+1][j] + Bounds[j][0]))
				DataFile.write('%1i\t' % (LHS[i][j]))
			else:
				LHS[i][j] = (Bounds[j][1] - Bounds[j][0])*rand_matrix[i+1][j] + Bounds[j][0]
				DataFile.write('%4.4f\t' % (LHS[i][j]))
			
		DataFile.write('\n')
	DataFile.close()
	return LHS

# This function takes a dictionary where each key is a variable and the value of the key is an array with a min and max value.
def myLHS(bounds, sampleSize):

	# Create a matrix that works with real_LHS(). The 3 columns is required by real_LHS: [min value, max value, integer or not]
	numberOfVariables = len(bounds)
	matrix = np.zeros((numberOfVariables, 3))

	iter = 0
	for index, (key, value) in enumerate(bounds.items()):
		print(index, key)

		# Store the minimum and maximum values of each variable
		matrix[index][0] = value[0]
		matrix[index][1] = value[1]

		# If the variable is the number of stringers, let real_LHS() know it needs to be an integer
		if key == 'numberOfRibs':
			matrix[iter][2] = 1
		else:
			matrix[iter][2] = 0
		iter += 1

	LHS_Matrix = real_LHS(matrix, sampleSize)
	print(LHS_Matrix)
	return LHS_Matrix