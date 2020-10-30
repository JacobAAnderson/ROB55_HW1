#!/usr/bin/env python2
# Jake Anderson
# ROB 599 Software deve
# Homework 1
# Oct 27, 2020

# Publish Markers to RVIZ

import rospy

import numpy as np

from math                   import sin, cos
from sensor_msgs.msg        import LaserScan
from geometry_msgs.msg      import Quaternion
from tf.transformations     import quaternion_from_euler
from visualization_msgs.msg import Marker



def heading(yaw):
    """A helper function to getnerate quaternions from yaws."""
    q = quaternion_from_euler(0, 0, yaw)
    return Quaternion(*q)


def Arrow( frame_id, length, angle):

	marker = Marker()
	marker.header.frame_id = frame_id
	marker.type = marker.ARROW
	marker.action = marker.ADD
	marker.scale.x = length
	marker.scale.y = 0.1
	marker.scale.z = 0.1
	marker.color.a = 1.0
	marker.color.r = 1.0
	marker.color.g = 1.0
	marker.color.b = 0.0
	marker.pose.orientation = heading(angle)
	marker.pose.position.x = 0.0
	marker.pose.position.y = 0.0
	marker.pose.position.z = 0.25

	return marker


def Text( frame_id, length, angle):

	marker = Marker()
	marker.header.frame_id = frame_id
	marker.type = marker.TEXT_VIEW_FACING
	marker.action = marker.ADD
	marker.text = "Dist: {:.2f} [m]".format(length)
	marker.scale.x = 0.5
	marker.scale.y = 0.5
	marker.scale.z = 0.5
	marker.color.a = 1.0
	marker.color.r = 0.0
	marker.color.g = 0.1
	marker.color.b = 0.1
	marker.pose.orientation.w = 1.0
	marker.pose.position.x = (length -3) * cos(angle + 0.5)
	marker.pose.position.y = (length -3) * sin(angle + 0.5)
	marker.pose.position.z = 0.25

	return marker


def callback(lidar_msg):
	"""
	Callback function to create LaserScan markders.
	"""

	min_range = min(lidar_msg.ranges)
	idx   = lidar_msg.ranges.index(min_range)
	angle = lidar_msg.angle_min + idx * lidar_msg.angle_increment
	arrow = Arrow( lidar_msg.header.frame_id, min_range, angle )
	text  = Text( lidar_msg.header.frame_id, min_range, angle )

	pubA.publish(arrow)
	pubT.publish(text)


if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('lidar_marker')

	rospy.loginfo("Starting 'lidar_marker' Node")

	# Set up a subscriber and publisher.
	sub = rospy.Subscriber('base_scan_filterd', LaserScan, callback )

	pubA = rospy.Publisher( 'lidar_marker_arrow', Marker, queue_size=1 )
	pubT = rospy.Publisher( 'lidar_marker_text', Marker, queue_size=1 )

	rospy.spin()
