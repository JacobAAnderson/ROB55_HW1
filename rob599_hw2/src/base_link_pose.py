#!/usr/bin/env python

import rospy
import sys

import tf

from nav_msgs.msg      import Odometry
from geometry_msgs.msg import PoseStamped


def BaseLink_CallBack(msg):

	p = PoseStamped()
	p.header = msg.header
	p.pose = msg.pose.pose

	try:
		pose = listener.transformPose('map', p)
		publisher.publish(pose)

	except:
		pass


if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('base_link_pose', argv=sys.argv)

	# Set up the filter.
	subscriber = rospy.Subscriber('odom', Odometry, BaseLink_CallBack, queue_size=1)
	publisher  = rospy.Publisher('base_link_pose', PoseStamped, queue_size=1)

	listener = tf.TransformListener()

	print("Node initialized")
	# Give control over to ROS.
	rospy.spin()
