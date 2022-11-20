#!/usr/bin/env python3
import numpy as np
from scipy.linalg import expm
from lab3_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
	w1 = np.array([0,0,1])
	p1 = np.array([-150, 150, 10])
	v1 = np.cross(-w1, p1) 
	O1 = np.array([[0,-1,0],[1,0,0],[0,0,0]])
	S1 = np.hstack((O1, v1.reshape(3,1)))
	S1 = np.vstack((S1, [0,0,0,0]))

	w2 = np.array([0,1,0])
	p2 = np.array([-150, 270, 162])
	v2 = np.cross(-w2, p2) 
	O2 = np.array([[0,0,1],[0,0,0],[-1,0,0]])
	S2 = np.hstack((O2, v2.reshape(3,1)))
	S2 = np.vstack((S2, [0,0,0,0]))

	w3 = np.array([0,1,0])
	p3 = np.array([94, 270, 162])
	v3 = np.cross(-w3, p3) 
	O3 = np.array([[0,0,1],[0,0,0],[-1,0,0]])
	S3 = np.hstack((O3, v3.reshape(3,1)))
	S3 = np.vstack((S3, [0,0,0,0]))

	w4 = np.array([0,1,0])
	p4 = np.array([307, 177, 162])
	v4 = np.cross(-w4, p4) 
	O4 = np.array([[0,0,1],[0,0,0],[-1,0,0]])
	S4 = np.hstack((O4, v4.reshape(3,1)))
	S4 = np.vstack((S4, [0,0,0,0]))

	w5 = np.array([1,0,0])
	p5 = np.array([307, 260, 162])
	v5 = np.cross(-w5, p5) 
	O5 = np.array([[0,0,0],[0,0,-1],[0,1,0]])
	S5 = np.hstack((O5, v5.reshape(3,1)))
	S5 = np.vstack((S5, [0,0,0,0]))

	print(S5)

	w6 = np.array([0,1,0])
	p6 = np.array([390, 260, 162])
	v6 = np.cross(-w6, p6) 
	O6 = np.array([[0,0,1],[0,0,0],[-1,0,0]])
	S6 = np.hstack((O6, v6.reshape(3,1)))
	S6 = np.vstack((S6, [0,0,0,0]))

	S = [S1, S2, S3, S4, S5, S6]


	RM = np.array([[0, -1, 0],
		 		   [0, 0, -1],
		  		   [1, 0, 0]])
	pM = np.array([390, 401, 215.5])

	M = np.hstack((RM, pM.reshape(3,1)))
	M = np.vstack((M, [0,0,0,1]))

	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	M, S = Get_MS()
	T = expm(S[0]*theta1) *  expm(S[1]*theta2) * expm(S[2]*theta3) * expm(S[3]*theta4) * \
		expm(S[4]*theta5) * expm(S[5]*theta6) * M
	
	

	# ==============================================================#

	print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value

