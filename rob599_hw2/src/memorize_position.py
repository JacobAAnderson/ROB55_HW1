#!/usr/bin/env python3

import tkinter as tk




class GetLocations:

	def __init__(self):

		self.locations = {}
		self.count = 0
		self.robotPose = "[x,y,0]"

		window = tk.Tk()
		window.title("Robot Location Tracker")

		# --- Frame A: Title----------------------
		frame_a = tk.Frame(	master=window,
							relief=tk.FLAT,
							borderwidth=5,
							height=200,
							)

		label1 = tk.Label(master=frame_a, text="\nDrive Robot with keyboard\n")
		label1.pack()

		frame_a.pack(fill=tk.X)


		# --- Frame B: Robot Position ---------------
		frame_b = tk.Frame(	master=window,
							relief=tk.RIDGE,
							borderwidth=5,
							height=300,
							)


		label2 = tk.Label(master=frame_b, text="Robots Current Pose: ")
		label2.grid(row=0, column=0, sticky="nsew")

		self.xyt_label = tk.Label(master=frame_b, text=self.robotPose)
		self.xyt_label.grid(row=0, column=1)

		frame_b.pack(fill=tk.X)


		# --- Frame C: Location Naming --------------------
		frame_c = tk.Frame(	master=window,
							relief=tk.FLAT,
							borderwidth=5,
							height=300,
					)

		label3 = tk.Label(master=frame_c, text="\n\nAdd location by type itsname below\nThen press the Add button\n")
		label3.grid(row=0)
	#	label2.pack()

		self.entry = tk.Entry( master=frame_c,  width=25)
		self.entry.grid(row=1, column=0, sticky="nsew")

		btn_add = tk.Button(master=frame_c, text="Add", command=self.Add_Location)
		btn_add.grid(row=1, column=1, sticky="nsew")

		label4 = tk.Label(master=frame_c, text="\n\nAdd location by type itsname below\nThen press the Add button\n")
		label4.grid(row=0)

		frame_c.pack(fill=tk.X)


		# --- Frame D: Saved Locations --------------------
		self.frame_d = tk.Frame(	master=window,
							relief=tk.RIDGE,
							borderwidth=5,
							height=300,
				)

		label5 = tk.Label(master=self.frame_d, text="\nSaved Locations\n")
		label5.pack()
#		label2.grid(row=0, column=0, sticky="nsew")

		self.frame_d.pack(fill=tk.X)


		# --- Spin the window
		window.mainloop()


	def __del__(self):
		print('Destructor called, Window deleted.')


	def Add_Location(self):
		print("Added Location: {0} - {1}".format(self.entry.get(), self.robotPose ))

		name = self.entry.get()

		self.locations[name] = self.robotPose

		self.entry.delete(0, tk.END)

		print(self.locations)

		self.count = self.count +1

		btn_add = tk.Button(master=self.frame_d,
							text=name,
							command= lambda : self.Goto_Location(name))

		btn_add.pack()
#		btn_add.grid(row=self.count, column=0, sticky="nsew")

		self.frame_d.pack(fill=tk.X)


	def Goto_Location(self, loc):

		print("Go to Location")
		print(loc)
		print(self.locations[loc])


# Execution of the node starts here, since it's called as an executable.
if __name__ == '__main__':


	GetLocations()
