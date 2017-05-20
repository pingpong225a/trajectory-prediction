import numpy as np 
import queue 
import sklearn.linear_model as lm 

playing_field = [] # (xmin,xmax) (ymin,ymax) (zmin, zmax)
# dest_x = 20 # x location at which the kuka is 


# # ()

class trajectory: 
	def __init__(self):
		self.X = np.ones((50, 4))
		self.points_recorded = 0 
		self.curr_index = 0 
	def record_pos(self, pos, time):

		if self.ball_trace.full() 
			self.ball_trace.get()
		self.ball_trace.put((pos, time))
	def calc_trajectory(self):
		X = np.zeros((self.ball_trace.length))
		# linear_model = lm.LinearRegression()
		# linear_model = lm.fit(X, Y)



	def reset(self):
		__init__(self)


