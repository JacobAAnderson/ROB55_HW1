#!/usr/bin/env python3

import sys
import rospy

from math import tanh, sin
from sensor_msgs.msg   import LaserScan



def callback(lidar_msg):
	"""
	Callback function to filter LaserScan Data.
	"""

	newRanges = []
	newAngles = []
	newIntens = []

	angles = np.arange(lidar_msg.angle_min, lidar_msg.angle_max, lidar_msg.angle_increment)
	for r, a, i in zip(lidar_msg.ranges, angles, lidar_msg.intensities):

		if abs(r*sin(a)) <= 0.5: 		# If the beam is les than 0.5 meters wide, then keep it
			newRanges.append(r)
			newAngles.append(a)
			newIntens.append(i)


	lidar_msg.angle_min = min(newAngles)
	lidar_msg.angle_max = max(newAngles)
	lidar_msg.ranges = newRanges
	lidar_msg.intensities = newIntens

	pub.publish(lidar_msg)


if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('laser_timmer')

	rospy.loginfo("Starting 'laser_filter' Node")

	# Set up a subscriber and publisher.
	sub = rospy.Subscriber('base_scan', LaserScan, callback )
	pub = rospy.Publisher( 'base_scan_filterd', LaserScan, queue_size=1 )

	rospy.spin()
