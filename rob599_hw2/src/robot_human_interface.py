#!/usr/bin/env python3

import sys
import rospy
import actionlib

import tkinter as tk

from std_msgs.msg   import String
from rob599_hw2.srv import location
from rob599_hw2.srv import save_places
from rob599_hw2.msg import GoToPlaceAction, GoToPlaceGoal, GoToPlaceResult
from rob599_hw2.msg import PatrolAction, PatrolGoal, PatrolResult

class Interface:

	def __init__(self):

		self.locations = {}

		self.sevice_client0 = rospy.ServiceProxy('location', location)
		self.sevice_client1 = rospy.ServiceProxy('save_places', save_places)
		self.sevice_client2 = rospy.ServiceProxy('open_places', save_places)

		self.action = actionlib.SimpleActionClient('goto_place', GoToPlaceAction)
		self.action.wait_for_server()

		self.patrol = actionlib.SimpleActionClient('patrol', PatrolAction)
		self.action.wait_for_server()

		window = tk.Tk()
		window.title("Robot Location Tracker")

		# --- Create Frames Frames ----------------------
		relief = [tk.FLAT, tk.RIDGE, tk.FLAT, tk.RIDGE, tk.FLAT]
		self.frames = [tk.Frame(master=window, relief=r, borderwidth=5) for r in relief]


		# --- Create Lables ----------------------------
		frameIdx = [0,1,1,2,2,3]
		text = ["\nDrive Robot with keyboard / RVIZ\n",
				"Robots Current Pose: ",
				"[X,  Y,  0]",
				"\n\nAdd location by type its name below\nThen press the Add button\n",
				"\nSaved Locations\nClick on button to have the robot return to that location",
				"",
				]

		self.labels = [tk.Label(master=self.frames[i], text=t) for i, t in zip(frameIdx, text)]

		self.labels[0].pack()
		self.labels[1].grid(row=0, column=0, sticky="nsew")
		self.labels[2].grid(row=0, column=1)
		self.labels[3].grid(row=0)
		self.labels[4].grid(row=2)
		self.labels[5].grid(row=0, column=1)


		# --- Create Entry Boxes --------------------------------
		frameIdx = [2, 4, 4]
		widths = [25, 25, 25]
		self.entrys = [tk.Entry( master=self.frames[i],  width=w) for i, w in zip(frameIdx, widths)]
		self.entrys[0].grid(row=1, column=0, sticky="nsew")
		self.entrys[1].grid(row=2, column=0, sticky="nsew")
		self.entrys[2].grid(row=3, column=0, sticky="nsew")


		# --- Create Buttons -------------------------------------
		frameIdx = [2,4,4]
		text = ["Add",
				"Save Places",
				"Open File"]
		funs = [self.Add_Location,
				self.Save_Locations,
				self.Open_Locations]

		self.buttons = [tk.Button(master=self.frames[i], text=t, command=c) for i,t,c in zip(frameIdx, text, funs)]
		self.buttons[0].grid(row=1, column=1, sticky="nsew")
		self.buttons[1].grid(row=2, column=1, sticky="nsew")
		self.buttons[2].grid(row=3, column=1, sticky="nsew")


		# --- Pack the frames --------------------------------------
		for f in self.frames: f.pack(fill=tk.X)


		# --- ROS Stuff --------------------------------------------
		self.sub = rospy.Subscriber('base_link_text', String, self.UpdateLocation, queue_size=1)
		print("Subscribing to base_link_pose")

		window.mainloop()


	def __del__(self):
		print('Destructor called, Window deleted.')


	def Add_Location(self):

		# --- Add the new location to the list ----------
		name = self.entrys[0].get() 			# Get name from the text entry
		self.entrys[0].delete(0, tk.END)		# Clear the text entry

		print("Added Location: {0}".format(name))

#		self.locations[name] = self.robotPose 	# Get Robot Pose
		try:
			tf = self.sevice_client0(name)

		except rospy.ServiceException as e:
			rospy.logwarn('Service call failed for {0}: {1}'.format(name, e))
			return

		if not tf: return

		self.buttons.append(tk.Button(	master=self.frames[3], 		# Add the new location to the GUI list
										text=name,
										command= lambda : self.Goto_Location(name))
										)

		idx = len(self.buttons) - 1

		print(idx)

		self.buttons[idx].grid(row=idx-3, column=0, sticky="nsew")

		self.frames[3].pack(fill=tk.BOTH)

		# --- Give the option to patrol once there is more than one locaiton
		if idx == 4:
			print("Adding Patrol Buttons")
			self.patrol_buttons()



	def Save_Locations(self):
		rospy.loginfo("Saving Places")

		# --- Add the new location to the list ----------
		name = self.entrys[1].get() 			# Get name from the text entry
		self.entrys[1].delete(0, tk.END)		# Clear the text entry

		print(" Saving Locations as: {0}".format(name))

		#		self.locations[name] = self.robotPose 	# Get Robot Pose
		try:
			tf = self.sevice_client1(name)

		except rospy.ServiceException as e:
				rospy.logwarn('Service call failed for {0}: {1}'.format(name, e))
				return



	def Open_Locations(self):

		# --- Add the new location to the list ----------
		name = self.entrys[2].get() 			# Get name from the text entry
		self.entrys[2].delete(0, tk.END)		# Clear the text entry

		print("Opening Location: {0}".format(name))

#		self.locations[name] = self.robotPose 	# Get Robot Pose
		try:
			tf = self.sevice_client2(name)

			print(tf.saved)

		except rospy.ServiceException as e:
			rospy.logwarn('Service call failed for {0}: {1}'.format(name, e))
			return

		if tf.saved:
			self.patrol_buttons()


	def Goto_Location(self, loc):
		print("Go to Location: {0}".format(loc))

		goal = GoToPlaceGoal(location=loc)
		self.action.send_goal(goal, done_cb=self.done_callback,
									active_cb=self.active_callback,
									feedback_cb=self.action_feedback)


	def Patrol(self, start):
		print("Patroling? {0}".format(start))

		goal = PatrolGoal(patrol=True)
		self.patrol.send_goal(goal, done_cb=self.done_callback,
									active_cb=self.active_callback,
									feedback_cb=self.action_feedback)


	def patrol_buttons(self):

		print("Adding Patrol Buttons")
		self.labels.append(tk.Label(master=self.frames[4],
									text="\nStart or stop robot patroling with the buttons below"),
									)
		self.labels[-1].grid(row=0, column=0, sticky="nsew")

		p_btn = tk.Button(	master=self.frames[4],
							text="Patrol",
							command= lambda : self.Patrol(True),
							)

		p_btn.grid(row=1, column=0, sticky="nsew")

		s_btn = tk.Button(	master=self.frames[4],
							text="Stop Patrol",
							command=lambda : self.Patrol(False),
							)

		s_btn.grid(row=1, column=1, sticky="nsew")

		self.frames[4].pack(fill=tk.BOTH)


	# This callback will be called when the action is complete.
	def done_callback(self, status, result):
		# The status argument tells you if the action succeeded.  Sometimes actions that did not succeed can
		# return partial results.
		if status and result == actionlib.GoalStatus.SUCCEEDED:
			rospy.loginfo('Suceeded with result {0}'.format(result.arrived))
			self.labels[5]['text'] = "Arrived"
		else:
			rospy.loginfo('Failed with result {0}'.format(result.arrived))
			self.labels[5]['text'] = "Failed"


	# This callback will be called when the action becomes active on the server.  If the server is
	# set up to only handle one action at a time, this will let you know when it's actively working
	# on your action request.
	def active_callback(self):
		rospy.loginfo('Action is active')


	def action_feedback(self, feedback):
#		print("Action Feedback")
#		print(feedback.progress)
		self.labels[5]['text'] = "{0}".format(feedback.progress)


	# Get Updated Pose Information
	def UpdateLocation(self, pose_msg):

		self.labels[2]['text'] = "{0}". format( pose_msg.data)



# Execution of the node starts here, since it's called as an executable.
if __name__ == '__main__':

	rospy.init_node('human_Interface_node', argv=sys.argv)

	rospy.wait_for_service('location')

	gui = Interface()
