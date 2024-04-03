###Sidney Tsui
###lab 6
### 4/2/2024

### used lab 5 as outline
#used in class lectures, and online sources

#filter estimated values of x_c, y_c, z_c, radius b4 publishing
#use method explained in class to implement low-pass filter to each parameter.
#find proper filter gains and fine tune them

#!/usr/bin/env python3
import rospy
import numpy as np
import math 
from robot_vision_lectures.msg import XYZarray, SphereParams


filter_gains = {'xc': 0.1, 'yc': 0.1, 'zc': 0.1, 'radius': 0.1}
matrix_a = []
matrix_b = []
output = True
xOutput, yOutput, zOutput, rOutput = 0.0, 0.0, 0.0, 0.0
valParams = False

###builds matrices from receiving the data points
def build_matrices(data_points):
	global matrix_a
	global matrix_b
	###create matrices from Points array
	for i in data_points.points:
		###[2*x, 2*y, 2*z, 1]
		matrix_a.append([2 * i.x, 2 * i.y, 2 * i.z, 1])
		###[x^2 + y^2 + z^2]
		matrix_b.append([i.x ** 2 + i.y ** 2 + i.z ** 2])
###fits the sphere model to data points and finds P
def fit(matrix_a, matrix_b):
	unfiltered = SphereParams()

	A = np.array(matrix_a)
	B = np.array(matrix_b)
	P = np.array([])
	
	try: 
		##used to find P
		P = np.linalg.lstsq(A, B, rcond = None)[0]
		###set unfiltered center and radius parameters
		unfiltered.xc = P[0]
		unfiltered.yc = P[1]
		unfiltered.zc = P[2]
		### radius calculation: SQRT(P[3] + Xc^2 + Yc^2 + Zc^2)
		unfiltered.radius = math.sqrt(P[3] + P[0] ** 2 + P[1] ** 2 + P[2] ** 2)
	
		### return the parameter message
		return unfiltered
		
	except:
		###error handling
		valParams = False
###filter parameters before publishing
###added parameters to avoid using global variables 
def filtered(unfiltered, filter_gain, output, xOutput, yOutput, zOutput, rOutput):
	filParams = SphereParams()
	if output:
		##init output if true
		xOutput = unfiltered.xc
		yOutput = unfiltered.yc
		zOutput = unfiltered.zc
		rOutput = unfiltered.radius
		output = False
		
    ###filtered values using filter_gain, input, output
	xInput = unfiltered.xc
	yInput = unfiltered.yc
	zInput = unfiltered.zc
	rInput = unfiltered.radius
	###from Input
	filParams.xc = xOutput
	filParams.yc = yOutput
	filParams.zc = zOutput
	filParams.radius = rOutput
	xOutput = filter_gain * xInput + (1 - filter_gain) * xOutput
	yOutput = filter_gain * yInput + (1 - filter_gain) * yOutput
	zOutput = filter_gain * zInput + (1 - filter_gain) * zOutput
	rOutput = filter_gain * rInput + (1 - filter_gain) * rOutput
	
	return filParams
	
if __name__ == '__main__':
	#init ros node
	rospy.init_node('sphere_fit', anonymous = True)
	#init all variables
	output = True 
	xOutput, yOutput, zOutput, rOutput = 0.0, 0.0, 0.0, 0.0 
	valParams = False
	# define subscriber and publisher
	# subscribe to /xyz_croppedtopic
	img_sub = rospy.Subscriber("/xyz_cropped_ball", XYZarray, build_matrices) 
	#create publisher sphereParameters on the /sphere_params topic
	# define a publisher to publish images
	sp_pub = rospy.Publisher('/sphere_params', SphereParams, queue_size = 10)
	# set the loop frequency to 10 hz
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		#check matrices
		if len(matrix_a) > 0 and len(matrix_b) > 0:
			unfiltered = fit(matrix_a, matrix_b)
			if valParams:
				filParams = filtered(unfiltered, filter_gain, output, xOutput, yOutput, zOutput, rOutput)
			    #check P empty
				# publish sphere params
				sp_pub.publish(filParams)
		rate.sleep()
