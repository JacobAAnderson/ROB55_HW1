#!/usr/bin/env python2
# Jake Anderson
# ROB 599 Software Dev
# Homework 1
# Oct 19, 2020

# Find the plane of a wall
# Use laser scan data to do Total Least Squares
import rospy

import numpy as np

from sensor_msgs.msg        import LaserScan
from geometry_msgs.msg      import Pose, Point, PoseStamped, Quaternion
from tf.transformations     import quaternion_from_euler
from visualization_msgs.msg import Marker



def heading(yaw):
    """A helper function to getnerate quaternions from yaws."""
    q = quaternion_from_euler(0, 0, yaw)
    return Quaternion(*q)


def CheckCov_eigVectors(V):
    # Make sure that we have the eigen vecors right
	u = np.append(V[:,0], 0)
	v = np.append(V[:,1], 0)
	c = np.cross(u,v)
	c = np.linalg.norm(c)
	d = np.dot(u,v)

	theta = np.arctan2(c,d)

	if not theta == np.pi/2:
		rospy.logwarn("Covaraince Eigen Vectors are not Orthoganal: {0}".format(theta * 180 / np.pi))
		return False

	return True



def cross2d(a,b):

	n1 = np.sqrt(a[0]*a[0] + a[1]*a[1] )
	n2 = np.sqrt(b[0]*b[0] + b[1]*b[1] )

	ab = a[0] * b[0] + b[1] * a[1]

	a = ab/ (n1*n2)

	theta = np.arccos(a)

	if abs( theta ) > np.pi:
		theta = 2*pi - theta;

	return theta


def callback(lidar_msg):
	"""
	Callback function to create LaserScan markders.
	"""

	# Get X,Y Coordinates of the laser contacts
	x = []
	y = []

	angles = np.arange(lidar_msg.angle_min, lidar_msg.angle_max, lidar_msg.angle_increment)
	for r, a in zip(lidar_msg.ranges, angles):
		x.append(r * np.cos(a))
		y.append(r * np.sin(a))

	# Get the center of the wall and the covariance matrix
	x_  = np.mean(x)
	y_  = np.mean(y)
	cov = np.cov(x,y)

	# Use covariance matrix to find angle between the wall and robot
	W, V = np.linalg.eig(cov)

	print("\n\n\nV: {0}".format(V))

	if V[1,0] > 0:
		V[:,0] = -V[:,0]

	if not CheckCov_eigVectors(V):     # Make Sure the Eigen Vectors are correct
		return

	theta = cross2d(V[:,1], [0,-1])

	print("\nV: {0}".format(V))
	rospy.loginfo("Wall Angle: {0} [deg]".format(theta * 180 / np.pi))

	wall = PoseStamped()
	wall.header = lidar_msg.header
	wall.pose   = Pose(Point(x_, y_, 0), heading(theta))

	pub.publish(wall)


if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('wall_finder')

	rospy.loginfo("Starting 'wall_finder' Node")

	# Set up a subscriber and publisher.
	sub = rospy.Subscriber('base_scan_filterd', LaserScan, callback )
	pub = rospy.Publisher( 'wall_pose', PoseStamped, queue_size=1 )

	rospy.spin()
