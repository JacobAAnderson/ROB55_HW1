#!/usr/bin/env python3

import sys
import rospy
import actionlib

from geometry_msgs.msg  import PoseStamped, Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from rob599_hw2.srv     import location, locationResponse
from rob599_hw2.msg     import GoToPlaceAction, GoToPlaceGoal, GoToPlaceFeedback, GoToPlaceResult
from rob599_hw2.msg     import PatrolAction, PatrolGoal, PatrolFeedback, PatrolResult

class driver:

	def __init__(self):

		self.sub = rospy.Subscriber('base_link_pose', PoseStamped, self.pose_callback)
		self.ser = rospy.Service('location', location, self.sevice_Callback)

		self.pose = Pose()
		self.locations = {}
		self.target = ''

		self.action = actionlib.SimpleActionServer('goto_place', GoToPlaceAction, self.moveToGoal, False)
		self.action.start()

		self.patrol = actionlib.SimpleActionServer('patrol', PatrolAction, self.Patroling, False)
		self.patrol.start()

		self.goal = MoveBaseGoal()
		self.goal.target_pose.header.frame_id = 'map'

		self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		self.move_base.wait_for_server()
		rospy.loginfo('{0}: Made contact with move_base server'.format(self.__class__.__name__))


	def pose_callback(self, msg):
#		print("Pose Callback")
		self.pose = msg.pose


	def sevice_Callback(self, request):
#		print("Service Callback")

		loc = request.location

		rospy.loginfo('Lacation Recived: {0}'.format(loc))

		self.locations[loc] = self.pose

#		print(self.locations)

		return locationResponse(True)


	def moveToGoal(self, goal, wait=False):

		rospy.loginfo( "Goal Recived: {0}".format(goal.location) )

		# Make Sure the destination exists
		if not goal.location in self.locations:
			self.action.publish_feedback(GoToPlaceFeedback(progress= "This Place Doesn't Exist"))
			self.action.set_succeeded(GoToPlaceResult(arrived=False))
			rospy.loginfo("This Place Doesn't Exist")
			return

		# Try to go there
		success, state = self.drive(goal.location)

		# Did everything work as expects?
		# We can check the status of the action, to make sure we are where we thought we would be.
		if success and state == actionlib.GoalStatus.SUCCEEDED:
			rospy.loginfo('Arrived at destination.')
			self.action.set_succeeded(GoToPlaceResult(arrived=True))
		else:
			rospy.loginfo('Something went wrong.')
			self.action.set_succeeded(GoToPlaceResult(arrived=False))


	def active_callback(self):
		"""
		A simple callback function for when the navigation action becomes active.
		"""
		rospy.loginfo('Move Action is active.')


	def feedback_callback(self, feedback):

<<<<<<< HEAD
=======
		print("Feedback Typp: {0}".format(type(feedback)))

>>>>>>> d6bc2eac228e253473f970c7650e0ce352990a6f
		try:
			self.action.publish_feedback(GoToPlaceFeedback(progress= "Moving to: {0}".format(self.target)))
		except:
			self.action.publish_feedback(PatrolFeedback(progress= "Moving to: {0}".format(self.target)))

<<<<<<< HEAD
=======

>>>>>>> d6bc2eac228e253473f970c7650e0ce352990a6f
	def Patroling(self, goal):

		print("Patroling {}".format(goal.patrol))

		if goal.patrol:
			for key in self.locations.keys():

				self.target = key
				success, state = self.drive(key)

			if success and state == actionlib.GoalStatus.SUCCEEDED:
				rospy.loginfo('Arrived at destination.')
				self.action.set_succeeded(GoToPlaceResult(arrived=True))
			else:
				rospy.loginfo('Something went wrong.')
				self.action.set_succeeded(GoToPlaceResult(arrived=False))



	def drive(self, loc):

		self.target = loc

		self.goal.target_pose.header.stamp = rospy.Time.now()
		self.goal.target_pose.pose = self.locations[loc]

		self.move_base.send_goal(self.goal,
								active_cb=self.active_callback,
								feedback_cb=self.feedback_callback)
#								done_cb=self.done_callback)


		success = self.move_base.wait_for_result()
		state = self.move_base.get_state()

		return success, state




# Execution of the node starts here, since it's called as an executable.
if __name__ == '__main__':

	rospy.init_node('pose_memorizer', argv=sys.argv)

	dr = driver()

	print("Spining")
	rospy.spin()
