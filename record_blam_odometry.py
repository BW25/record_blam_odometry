#!/usr/bin/env python
# Autor> Clayder Gonzalez
# Edited> Brendan Woo

# Original source> https://github.com/claydergc/loam_velodyne_kitti_ros/blob/master/scripts/record_loam_odometry.py

import time
import rospy
import math
import numpy as np
import tf

from nav_msgs.msg import Odometry     
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

import matplotlib
import matplotlib.pyplot as plt
import math

trajectoryFile = None
timeStamp = None
pitch = None

def odom_callback_loam(data):
	global trajectoryFile
	global timeStamp
	global pitch
	timeStampNum = data.header.stamp.secs + (data.header.stamp.nsecs * 10**(-9))
	timeStamp = str(timeStampNum)
	quaternion = (data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)

	#Additional code to get translation data
	transX = data.pose.position.x
	transY = data.pose.position.y
	transZ = data.pose.position.z

	#euler = tf.transformations.euler_from_quaternion(quaternion)
	#roll = euler[0]
	#pitch = euler[1]
	#yaw = euler[2]

	#Altered code to print translation data
	trajectoryFile.write(timeStamp + " " + str(transX) + " " +  str(transY) + " " + str(transZ) + " " + str(data.pose.orientation.x) + " " + str(data.pose.orientation.y) + " " + str(data.pose.orientation.z) + " " + str(data.pose.orientation.w) + " " + "\n")	

def writeFile(file):
	global trajectoryFile
	# inicializa nodo ROS
	rospy.init_node('record_blam_odometry');
	trajectoryFile = open(file, 'w')

	#Additional code to add header information
	trajectoryFile.write("Timestamp transX transY transZ quatX quatY quatZ quatW\n")

	rospy.Subscriber('/blam/blam_slam/odometry_integrated_estimate', PoseStamped, odom_callback_loam)
	rospy.spin()
	trajectoryFile.close()
        
if __name__ == '__main__':
	writeFile('BLAMTrajectory.txt')
