#!/usr/bin/env python3

import sys
import rospy

from sensor_msgs.msg   import LaserScan


class timer:

	def __init__(self, cycles=100):

		self.sub1 = rospy.Subscriber('base_scan',    LaserScan, self.callback1, queue_size=1   )
		self.sub2 = rospy.Subscriber('timer/clocked_scan', LaserScan, self.callback2,  queue_size=100 )
		self.pub  = rospy.Publisher( 'timer/queued_scan',  LaserScan, queue_size=100 )

		self.manifest = {}
		self.count = 0
		self.length = cycles
		self.times = []




	def callback1(self, lidar_msg):
		"""
		Callback function to filter LaserScan Data.
		"""
		if self.count == self.length:
			self.count = self.count+1

			ave = sum(self.times)/len(self.times)

			rospy.loginfo("Timming is complete, Ave: {}".format(ave))

			f = open("/home/ubuntu/rob599_homeWork/py_average_time.txt","w")
			f.write( "Average Time: {}".format(ave) )
			f.close()

			rospy.loginfo("Ave saved to file")

			return

		elif self.count > self.length:
			return

		self.count = self.count+1

		self.manifest[self.count] = rospy.get_time()

		self.pub.publish(lidar_msg)



	def callback2(self, lidar_msg):

		dt = rospy.get_time()  - self.manifest[lidar_msg.header.seq]

		self.times.append(dt)

		rospy.loginfo("Time: {0}".format(dt))



if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('laser_timmer')

	t = timer()

	rospy.loginfo("'laser_timmer' Node Started\n")

	# Set up a subscriber and publisher.

	rospy.spin()
