#!/usr/bin/env python3
# Jake Anderson
# ROB 599 Software deve
# Homework 1
# Oct 27, 2020


import rospy
import sys

# Import tha base service message type.
from rob599_hw1.srv import Stopping_distance
from math 			import inf, nan

if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('Change_Stopping_Distance')

	# This will wait for the service to become available.
	rospy.wait_for_service('stopping_distance')

	# Assuming that the service is up, setting up a service proxy gives us access to it.
	distance_Setter = rospy.ServiceProxy('stopping_distance', Stopping_distance)

	# To illustrate the use of a service, we're going to count up from zero, sending the numbers to the
	# service and logging the responses.
	for i in [inf, 5, 4, 0.5, 0, -1, -inf, nan]:
		# Once we have the service proxy set up, we can use it just like a function.  The arguments to this
		# function are copied into the fields of a DoublerRequest instance, and then sent over.  The functor
		# returns an instance of DoublerResponse, from which you can access as usual.  It's good practice to
		# wrap this in a try-except block in case something goes wrong.
		try:
			answer = distance_Setter(i)
		except rospy.ServiceException as e:
			rospy.logwarn('Service call failed for {0}: {1}'.format(i, e))

		rospy.loginfo('Sent {0} and got {1}'.format(i, answer.set))
