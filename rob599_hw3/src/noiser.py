#!/usr/bin/env python3

import sys
import rospy

from numpy.random     import randn
from sensor_msgs.msg  import LaserScan



def callback(lidar_msg):
	"""
	Callback function to filter LaserScan Data.
	"""

	if rospy.has_param('laser_noise_variance'):
		var = rospy.get_param('laser_noise_variance')
	else:
		var = 0.01

	print("\n\nVariance: {0}\n".format(var))


	newRanges = []

	for r in lidar_msg.ranges:                       # Cycle through the range measurmetns and apply filter

		newRanges.append( r + randn() * var )       # Add the new filtered mesaurment to the list


	lidar_msg.ranges = newRanges                       # Reassign the range measurmetns

	pub.publish(lidar_msg)



if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('laser_noise')

	rospy.loginfo("Starting 'laser_noise' Node")

	# Set up a subscriber and publisher.
	sub = rospy.Subscriber('base_scan', LaserScan, callback )
	pub = rospy.Publisher( 'base_scan_noised', LaserScan, queue_size=1 )

	rospy.spin()
