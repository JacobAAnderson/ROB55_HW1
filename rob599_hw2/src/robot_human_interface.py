#!/usr/bin/env python3

import sys
#import rospy

import tkinter as tk

from std_msgs.msg import String

class Interface:

	def __init__(self):

		self.locations = {}

		window = tk.Tk()
		window.title("Robot Location Tracker")

		# --- Create Frames Frames ----------------------
		relief = [tk.FLAT, tk.RIDGE, tk.FLAT, tk.RIDGE, tk.FLAT]
		self.frames = [tk.Frame(master=window, relief=r, borderwidth=5) for r in relief]


		# --- Create Lables ----------------------------
		frameIdx = [0,1,1,2,2]
		text = ["\nDrive Robot with keyboard / RVIZ\n",
				"Robots Current Pose: ",
				"[X,  Y,  0]",
				"\n\nAdd location by type its name below\nThen press the Add button\n",
				"\nSaved Locations\nClick on button to have the robot return to that location"
				]

		self.labels = [tk.Label(master=self.frames[i], text=t) for i, t in zip(frameIdx, text)]

		self.labels[0].pack()
		self.labels[1].grid(row=0, column=0, sticky="nsew")
		self.labels[2].grid(row=0, column=1)
		self.labels[3].grid(row=0)
		self.labels[4].grid(row=2)


		# --- Create Entry Boxes --------------------------------
		frameIdx = [2]
		widths = [25]
		self.entrys = [tk.Entry( master=self.frames[i],  width=w) for i, w in zip(frameIdx, widths)]
		self.entrys[0].grid(row=1, column=0, sticky="nsew")


		# --- Create Buttons -------------------------------------
		frameIdx = [2]
		text = ["Add"]
		funs = [self.Add_Location]

		self.buttons = [tk.Button(master=self.frames[i], text=t, command=c) for i,t,c in zip(frameIdx, text, funs)]
		self.buttons[0].grid(row=1, column=1, sticky="nsew")


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

		self.locations[name] = self.robotPose 	# Get Robot Pose

		self.buttons.append(tk.Button(	master=self.frames[3], 		# Add the new location to the GUI list
										text=name,
										command= lambda : self.Goto_Location(name))
										)

		idx = len(self.buttons) - 1
		self.buttons[idx].grid(row=idx-1, column=0, sticky="nsew")

		self.frames[3].pack(fill=tk.BOTH)

		# --- Give the option to patrol once there is more than one locaiton
		if idx == 2:

			self.labels.append(tk.Label(master=self.frames[4],
										text="\nStart or stop robot patroling with the buttons below"),
										)
			self.labels[-1].grid(row=0, column=1, sticky="nsew")

			p_btn = tk.Button(	master=self.frames[4],
								text="Patrol",
								command= lambda : self.Patrol(True),
								)

			p_btn.grid(row=1, column=1, sticky="nsew")

			s_btn = tk.Button(	master=self.frames[4],
								text="Stop Patrol",
								command=lambda : self.Patrol(False),
								)

			s_btn.grid(row=1, column=2, sticky="nsew")

			self.frames[4].pack(fill=tk.BOTH)


	def Goto_Location(self, loc):

		print("Go to Location")
		print(loc)
		print(self.locations[loc])


	def Patrol(self, start):
		print("Patroling? {0}".format(start))


	def UpdateLocation(self, pose_msg):

		self.labels[2]['text'] = "{0}". format( pose_msg.data)



# Execution of the node starts here, since it's called as an executable.
if __name__ == '__main__':

	rospy.init_node('human_Interface_node', argv=sys.argv)

	gui = Interface()
