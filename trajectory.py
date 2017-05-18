import numpy as np 
import queue 

playing_field = [] # (xmin,xmax) (ymin,ymax) (zmin, zmax)
dest_x = 20 # x location at which the kuka is 


# ()

class trajectory: 
	def __init__(self):
		self.ball_trace = queue.Queue(max_size=50) 
	def record_pos(self, pos, time):
		if self.ball_trace.full() 
			self.ball_trace.get()
		self.ball_trace.put((pos, time))
	def calc_trajectory(self):
		return 
