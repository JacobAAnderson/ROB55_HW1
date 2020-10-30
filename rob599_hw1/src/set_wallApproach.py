#!/usr/bin/env python3
# Jake Anderson
# ROB 599 Software Dev
# Homework 1
# Oct 27, 2020

# Wall Approach Service Client
import sys
import rospy
import actionlib

from rob599_hw1.msg    import Approach_WallAction, Approach_WallGoal,  Approach_WallResult

# This callback will be called when the action is complete.
def done_callback(status, result):
	# The status argument tells you if the action succeeded.  Sometimes actions that did not succeed can
	# return partial results.
	if status == actionlib.GoalStatus.SUCCEEDED:
		rospy.loginfo('Suceeded with result {0}'.format(result.arrived))
	else:
		rospy.loginfo('Failed with result {0}'.format(result.arrived))


# This callback will be called when the action becomes active on the server.  If the server is
# set up to only handle one action at a time, this will let you know when it's actively working
# on your action request.
def active_callback():
	rospy.loginfo('Action is active')


# This callback is called every time the server issues a feedback message.
def feedback_callback(feedback):
	rospy.loginfo('Feedback: {0}'.format(feedback.err))


if __name__ == '__main__':

	# Get the command line argument, if one exists.  Default to 2.5
	try:
		n = float(sys.argv[1])
	except:
		n = 2.5

	# Initialize the node
	rospy.init_node('set_wallApproach')

	# Create an action client.
	client = actionlib.SimpleActionClient('approach_wall', Approach_WallAction)
	client.wait_for_server()

	# Create a goal message to send to the server.
	goal = Approach_WallGoal(distance=n)

	# Send the action request, and register the callbacks.
	client.send_goal(goal, done_cb=done_callback, active_cb=active_callback, feedback_cb=feedback_callback)

	# This will cause the client to wait until there's some result from the server.
	client.wait_for_result()
