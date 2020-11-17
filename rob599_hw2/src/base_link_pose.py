#!/usr/bin/env python

import rospy
import sys

import tf

from std_msgs.msg 		import String
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg  import PoseStamped


def heading(q):
	"""A helper function to getnerate quaternions from yaws."""

	return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]



if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('base_link_pose', argv=sys.argv)

	# Set up the filter.
	subscriber = rospy.Subscriber('odom', Odometry, BaseLink_CallBack, queue_size=1)
	pub_pose   = rospy.Publisher('base_link_pose', PoseStamped, queue_size=1)
	pub_text   = rospy.Publisher('base_link_text', String, queue_size=1)

	listener = tf.TransformListener()

	print("Node initialized")


	p = PoseStamped()
	p.header.frame_id = 'base_link'
	p.pose.position.x = 0.0
	p.pose.position.y = 0.0
	p.pose.position.z = 0.0
	p.pose.orientation.x = 0.0
	p.pose.orientation.y = 0.0
	p.pose.orientation.z = 0.0
	p.pose.orientation.w = 0.0


	rate = rospy.Rate(10)

	while not rospy.is_shutdown():

		p.header.stamp = rospy.Time.now()

		try:
			pose = listener.transformPose('map', p)
			pub_pose.publish(pose)

			txt = "[X:{0:3.2f},  Y:{1:3.2f}, H:{2:3.2}]". format( msg.pose.pose.position.x,
			 													msg.pose.pose.position.y,
																heading(msg.pose.pose.orientation) )

		except:
			txt = "[X,  Y,  H]"

		pub_text.publish(txt)

		# Give control over to ROS.
		rospy.spin()
