#!/usr/bin/env python3
# Jake Anderson
# ROB 599 Software deve
# Homework 1
# Oct 20, 2020

# Filter out Laser Scan data that is not infront of the robot

import rospy
import sys
import numpy as np

from math import tanh, sin
from sensor_msgs.msg   import LaserScan


def callback(lidar_msg):
	"""
	Callback function to filter LaserScan Data.
	"""

	print('\n\nLidar Max Range: {0}'.format(lidar_msg.range_max) )
	print("Old Max Angle {0}".format(lidar_msg.angle_min))
	print("Old Min Angle {0}".format(lidar_msg.angle_max))

	newRanges = []
	newAngles = []
	newIntens = []

	angles = np.arange(lidar_msg.angle_min, lidar_msg.angle_max, lidar_msg.angle_increment)
	for r, a, i in zip(lidar_msg.ranges, angles, lidar_msg.intensities):

		if abs(r*sin(a)) <= 0.5:
			newRanges.append(r)
			newAngles.append(a)
			newIntens.append(i)


	lidar_msg.angle_min = min(newAngles)
	lidar_msg.angle_max = max(newAngles)
	lidar_msg.ranges = newRanges
	lidar_msg.intensities = newIntens

	print("New Max Angle {0}".format(lidar_msg.angle_min))
	print("New Min Angle {0}\n".format(lidar_msg.angle_max))

	pub.publish(lidar_msg)


if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('laser_filter')

	# Set up a subscriber and publisher.
	sub = rospy.Subscriber('base_scan', LaserScan, callback )
	pub = rospy.Publisher( 'base_scan_filterd', LaserScan, queue_size=1 )

	rospy.spin()
