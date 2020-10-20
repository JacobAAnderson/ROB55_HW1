#!/usr/bin/env python3
# Jake Anderson
# ROB 599 Software deve
# Homework 1
# Oct 19, 2020

# Move Fetch Robot

import rospy
import sys
import numpy as np

from math import tanh, sqrt
from sensor_msgs.msg   import LaserScan
from geometry_msgs.msg import Twist



def kalmanFilter(x, sig, u, z, A, B, Q, R):

	X_ = A * x  +  B * u
	P_ = A * sig * A + Q

	# K = Err in estimate / ( err in eatimate + err in measurment)
	K = P_/(P_ + R)

	x = X_ + K * (z - X_)

	# sigma = (1-K) * Err in estimate
	sig = (1 - K) * P_

	return x, sig


class mover:

	def __init__(self, dist):

		# Initialize the node.
		rospy.init_node('laser_subscriber', argv=sys.argv)

		# Set up a subscriber and publisher.
		self.sub = rospy.Subscriber('base_scan', LaserScan, self.lidar_callback)
		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

		print("Calibarting")

		self.x = 0
		self.sig = 0
		self.speed = 0
		self.is_cal = False
		self.cal_set = []
		self.lidar_STD = 0
		self.dist = dist


	def lidar_callback(self, lidar_msg):
		"""
		Handel Laser scan data
		"""

		# Lidar Calibration --> Get standard deviation in the lidar measurments
		min_range = min(lidar_msg.ranges)

		if not self.is_cal:
			self.cal_set.append(min_range)

			if len(self.cal_set) >= 100:
				self.lidar_STD = np.std(self.cal_set)
				self.is_cal = True
				self.x = np.average(self.cal_set)
				self.sig = self.lidar_STD

				print("Done Calibarting")
				print('lidar_STD: {0}\n'.format(self.lidar_STD) )

			else: return



		A =  1   				# Process Model  --> Lidar measurment
		B = -1   				# Control Model  --> Robot movement
		Q =  0.05  				# Process STD    --> Error in robot Movement
		R =  self.lidar_STD 	# Measurment STD --> Lidar noise

		self.x, self.sig = kalmanFilter(self.x, self.sig, self.speed, min_range, A, B, Q, R)

		err = self.dist - self.x

		# Check to see if we can really do better
		if abs(err) > self.lidar_STD * 3:
			self.speed = tanh(err)

		else:
			self.speed = 0


		print( '\nMin Range: {0} [m]'.format(min_range) )
		print( 'X: {0}\tSIG: {1}'.format(self.x, self.sig) )
		print( 'Error: {0}'.format(err) )
		print( 'Joint Err: {0}'.format(joint_err) )
		print( 'Speed: {0} [m/s]\n'.format(self.speed) )

		cmd_msg = Twist()
		cmd_msg.linear.x = self.speed
		cmd_msg.linear.y = 0.0
		cmd_msg.linear.z = 0.0
		cmd_msg.angular.x = 0.0
		cmd_msg.angular.y = 0.0
		cmd_msg.angular.z = 0.0

		self.pub.publish(cmd_msg)


if __name__ == '__main__':

	# Instanciate the mover and giv it a distance of 1 meter
	mv = mover(1)

	# Give control to ROS.
	rospy.spin()
