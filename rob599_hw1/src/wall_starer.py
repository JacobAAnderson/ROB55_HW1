#!/usr/bin/env python3
# Jake Anderson
# ROB 599 Software Dev
# Homework 1
# Oct 27, 2020

# Move Fetch Robot to within a certain distance of the wall
import rospy
import actionlib

import numpy as np

from math              import tanh, sqrt, isnan, inf
from sensor_msgs.msg   import LaserScan
from geometry_msgs.msg import Twist
from rob599_hw1.srv    import Stopping_distance, Stopping_distanceResponse
from rob599_hw1.msg    import Approach_WallAction, Approach_WallGoal, Approach_WallFeedback, Approach_WallResult




def kalmanFilter(x, sig, u, z, A, B, Q, R):

	X_ = A * x  +  B * u
	P_ = A * sig * A + Q

	# K = Err in estimate / ( err in eatimate + err in measurment)
	K = P_/(P_ + R)

	x = X_ + K * (z - X_)

	# sigma = (1-K) * Err in estimate
	sig = (1 - K) * P_

	return x, sig



class wall_starer:

	def __init__(self, distance=1, max_speed=1, aggressiveness=1):

		"""
		Initialize the laser_subscriber Node
		"""

		# Initialize the node.
		rospy.init_node('staring_at_wall')

		# Set up a subscriber and publisher.

		laser_topic = 'base_scan_filterd'
#		laser_topic = 'base_scan'

		self.sub = rospy.Subscriber(laser_topic, LaserScan, self.lidar_callback)
		self.ser = rospy.Service('stopping_distance', Stopping_distance, self.SD_Callback)
		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		self.x = 0
		self.sig = 0
		self.speed = 0
		self.is_cal = False
		self.cal_set = []
		self.lidar_STD = 0
		self.distance = distance
		self.max_speed = max_speed
		self.aggressiveness = aggressiveness

		# Prealocate Twist message
		self.twist = Twist()
		self.twist.linear.x = 0.0
		self.twist.linear.y = 0.0
		self.twist.linear.z = 0.0
		self.twist.angular.x = 0.0
		self.twist.angular.y = 0.0
		self.twist.angular.z = 0.0

		rospy.loginfo("Starting 'laser_subscriber' Node")
		rospy.loginfo("Subsribing to: {0}".format(laser_topic))
		rospy.loginfo('Stopping Distance Service started')
		rospy.loginfo("Getting Laser Calibration")


	# Lidar Scan Subscriber Callback
	def lidar_callback(self, lidar_msg):
		"""
		Handel Laser scan data
		"""

		min_range = min(lidar_msg.ranges) 	# Shortest range measurment

		# Lidar Calibration --> Get standard deviation in the lidar measurments
		if not self.is_cal:
			self.cal_set.append(min_range) 	# Create Calibration set

			if len(self.cal_set) >= 50: 				# ___ Do calibration ____
				self.lidar_STD = np.std(self.cal_set) 	# Lidar Standared Deviation
				self.x = np.average(self.cal_set) 		# average Lidare range to initiate measurment state
				self.sig = self.lidar_STD 				# Initialize measurment uncertainty with Lidar uncertainty
				self.is_cal = True 						# Indicate that the lidar has been calibrated

				self.t_last = rospy.get_time() 			# Initalize time keeping

#				print("Done Calibarting")
#				print('lidar_STD: {0}\n'.format(self.lidar_STD) )

			else: return


		##---- Filter Lidar Range Measurments -----
		# Elapsed time between filter updates
		time_ = rospy.get_time()
		dt = time_ - self.t_last
		self.t_last = time_

		# Filter settings
		A =  1   				# Process Model  --> Lidar measurment
		B = -dt   				# Control Model  --> Robot movement
		Q =  0.05  				# Process STD    --> Error in robot Movement
		R =  self.lidar_STD 	# Measurment STD --> Lidar noise

		self.x, self.sig = kalmanFilter(self.x, self.sig, self.speed, min_range, A, B, Q, R)


		# --- State Error ---
		err = self.x - self.distance

		if abs(err) > self.lidar_STD * 3: 	# Check to see if we can really do better
			self.speed = tanh(err * self.aggressiveness) * self.max_speed

		else:
			self.speed = 0.0


		# --- Send Comand Message ----
		self.twist.linear.x = self.speed
		self.pub.publish(self.twist)

		# --- Debugging ---
		#rospy.loginfo( 'Min Range: {0} [m]'.format(min_range) )
		#rospy.loginfo( 'X: {0} {1} {2}'.format(self.x, u"\u00B1", self.sig) )
		#rospy.loginfo( 'Error: {0}'.format(err) )
#		rospy.loginfo( 'dt: {0}'.format(dt) )
		#rospy.loginfo( 'Speed: {0} [m/s]\n'.format(self.speed) )


	# Stopping Distance Service Call Back
	def SD_Callback(self, request):

		sd = request.stopping_distance

		rospy.loginfo('Stopping Distance Service Got: {0}'.format(sd))

		max_dist = 20

		# Check if the set point is a valid number
		if isnan(sd) or sd < 0.5 or max_dist < sd:
			rospy.loginfo("Stopping Distance Rejected")
			return Stopping_distanceResponse(False)

		else:
			rospy.loginfo("Stopping Distance Accepted")
			self.dist = sd;
			return Stopping_distanceResponse(True)


	# Approach Action server
	def Approach(self,goal):

		rospy.loginfo( "Goal Recived: {0}".format(goal.distance) )

		sd = goal.distance
		max_dist = 20
		if isnan(sd) or sd < 0.5 or max_dist < sd:
			rospy.loginfo("Stopping Distance Rejected")
			server.set_preempted(Approach_WallResult(arrived=False))
			return

		self.ser.shutdown('approach_wall action server is taking over')

		self.dist = goal.distance

		err = inf
		rate = rospy.Rate(2)

		while not rospy.is_shutdown() and abs(err) > self.lidar_STD * 3:

			# Publish feedback, so that the caller knows about progress, if it cares.
			err = self.x - self.dist
			server.publish_feedback(Approach_WallFeedback(err= err))

			# Did we receive a new goal?  If so, preempt the current one.
			if server.is_new_goal_available():
				server.set_preempted(Approach_WallResult(arrived=False))
				return

			rate.sleep()


		server.set_succeeded(Approach_WallResult(arrived=True))

		self.ser = rospy.Service('stopping_distance', Stopping_distance, self.SD_Callback)



if __name__ == '__main__':

	# Instanciate the mover and giv it an aggressiveness of 2
	ws = wall_starer(aggressiveness=2)

	# Create wall approach server
	server = actionlib.SimpleActionServer('approach_wall', Approach_WallAction, ws.Approach, False)
	server.start()

	rospy.loginfo( "Wall Approach Action Server Started")
	# Give control to ROS.
	rospy.spin()
