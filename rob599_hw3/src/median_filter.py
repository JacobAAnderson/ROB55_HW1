#!/usr/bin/env python3

import sys
import rospy

from numpy            import median, min, max, array
from sensor_msgs.msg  import LaserScan



def callback(lidar_msg):
	"""
	Callback function to filter LaserScan Data.
	"""

	if rospy.has_param('median_filter_size'):
		filterSize = rospy.get_param('median_filter_size')
	else:
		filterSize = 10

	print("\n\nFilter Size: {0}\n".format(filterSize))


	rangeSize = len(lidar_msg.ranges) -1              # Size of the lidar message


	if filterSize < 2 or filterSize > rangeSize:      # If the filter size is less than 2 or biger than the range array, then the filter is useless, just republish and exit
		pub.publish(lidar_msg)
		return


	a = round(filterSize/2)                           # Make sure the indecies remain integers



	oldRanges = array(lidar_msg.ranges)
	newRanges = []

	for i in range(0,rangeSize):                       # Cycle through the range measurmetns and apply filter

		idx = range(i-a, i+a, 1)                      # Indecies of the measurmens being filtered

		if min(idx) < 0:                              # If i is less than 1/2 the filter size, shift idx forward
			idx = idx - min(idx)

		if max(idx) > rangeSize:                      # If i is les than 1/2 the filter size from the end of the array, shift idx backward
			idx = idx - (max(idx) - rangeSize) -1

		newRanges.append( median(oldRanges[idx]) )    # Add the new filtered mesaurment to the list


	lidar_msg.ranges = newRanges                       # Reassign the range measurmetns

	pub.publish(lidar_msg)



if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('laser_filter')

	rospy.loginfo("Starting 'laser_filter' Node")

	# Set up a subscriber and publisher.
	sub = rospy.Subscriber('median_filter/laser_scan_in', LaserScan, callback )
	pub = rospy.Publisher( 'median_filter/laser_scan_filterd', LaserScan, queue_size=1 )

	rospy.spin()
