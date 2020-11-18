#!/usr/bin/env python3

import sys
import rospy
import actionlib
import pickle

from geometry_msgs.msg  import PoseStamped, Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from rob599_hw2.srv     import location, locationResponse
from rob599_hw2.srv     import save_places, save_placesResponse
from rob599_hw2.msg     import GoToPlaceAction, GoToPlaceGoal, GoToPlaceFeedback, GoToPlaceResult
from rob599_hw2.msg     import PatrolAction, PatrolGoal, PatrolFeedback, PatrolResult

class driver:

	def __init__(self):

		self.sub  = rospy.Subscriber('base_link_pose', PoseStamped, self.pose_callback)
		self.ser  = rospy.Service('location', location, self.sevice_Callback)
		self.save = rospy.Service('save_places', save_places, self.savePlaces)
		self.open = rospy.Service('open_places', save_places, self.openPlaces)

		self.pose = Pose()
		self.locations = {}
		self.target = ''

		self.action = actionlib.SimpleActionServer('goto_place', GoToPlaceAction, self.moveToGoal, False)
		self.action.start()

		self.patrol = actionlib.SimpleActionServer('patrol', PatrolAction, self.Patroling, False)
		self.patrol.start()
		self.is_patroling = False

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


	def savePlaces(self, request):

		rospy.loginfo("Saving places to: {0}".format(request.fileName))

		try:
			# Serialize file so that its easy to reopen
			f = open("{0}.pkl".format(request.fileName),"wb")
			pickle.dump(self.locations,f)
			f.close()

			# Text output for human readability
			f = open("{0}.txt".format(request.fileName),"w")
			f.write( str(self.locations) )
			f.close()

			return save_placesResponse(True)

		except:
			return save_placesResponse(False)


	def openPlaces(self, request):

		rospy.loginfo("Opening File: {0}.pkl".format(request.fileName))

		try:

			file_to_read = open("{0}.pkl".format(request.fileName), "rb")

			self.locations = pickle.load(file_to_read)

			print(self.locations)

			return save_placesResponse(True)

		except:
			return save_placesResponse(False)



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

		if self.is_patroling:
			self.patrol.publish_feedback(PatrolFeedback(progress= "Moving to: {0}".format(self.target)))
#			print("send patrol feed back")

		else:
			self.action.publish_feedback(GoToPlaceFeedback(progress= "Moving to: {0}".format(self.target)))
#			print("Send move to feedback")


	def Patroling(self, goal):

		print("Patroling {}".format(goal.patrol))

		if goal.patrol:

			self.is_patroling = True

			for key in self.locations.keys():

				self.target = key
				success, state = self.drive(key)

			# If we make it to the last pose then its a success
			if success and state == actionlib.GoalStatus.SUCCEEDED:
				rospy.loginfo('Arrived at destination.')
				self.patrol.set_succeeded(PatrolResult(arrived=True))
			else:
				rospy.loginfo('Something went wrong.')
				self.patrol.set_succeeded(PatrolResult(arrived=False))


		self.is_patroling = False



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
