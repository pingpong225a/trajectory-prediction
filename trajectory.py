import numpy as np 
#import queue 
import redis 

playing_field = [] # (xmin,xmax) (ymin,ymax) (zmin, zmax)
dest_x = 20 # x location at which the kuka is 


# ()
MAX_POSITIONS = 20 
KUKA_POS_Y = 0 # in optitrack coordinates 
KUKA_POS_Z = 2.85

class Trajectory: 
    def __init__(self):
        global MAX_POSITIONS
        #self.ball_trace = queue.Queue(max_size=50) 
        #self.redis_conn = redis.StrictRedis(host='localhost', port=6379, db=0)
        self.positions = np.zeros((MAX_POSITIONS, 2))
        self.time = np.ones((MAX_POSITIONS, 2))
        self.index = 0
        self.num_points = 0 

    # def lin_reg(X, Y):

        

    def record_pos(self, pos, time):
        global MAX_POSITIONS
        self.positions[self.index, :] = pos[0:2] 
        self.time[self.index, 0] = time
        self.index = (self.index + 1) % MAX_POSITIONS  
        if self.num_points < MAX_POSITIONS
            self.num_points += 1 

    def calc_trajectory(self):
        global MAX_POSITIONS
        if self.num_points >= MAX_POSITIONS:
            tx = np.linalg.lstsq(self.time, self.positions[:, 0])
            ty = np.linalg.lstsq(self.time, self.positions[:, 1])
            cx = tx[0]
            mx = tx[1]
            cy = ty[0]
            my = ty[1]
            t = -1.0 * cy / my
            x_pred = mx * t + cx
            y_pred = my * t + cy
            return [x_pred, y_pred]
        return None 

    # Record position in redis 
    # TODO ensure that the marker getting recorded is the correct one 
    def push_ball(self, pos, frameNumber):
    #    #print('setting ball: (', pos[0], pos[1], pos[2], ')', frameNumber)
    #    self.redis_conn.set('ball_pos', pos)
    #    self.redis_conn.set('ball_time', frameNumber)
        pass

    def push_kuka(self, pos, rot, frameNumber):
       # print('setting kuka: (', pos[0], pos[1], pos[2], ') (', rot[0], rot[1], rot[2], ')', frameNumber)
        #self.redis_conn.set('kuka_pos', pos)
        #self.redis_conn.set('kuka_rot', rot)
        #self.redis_conn.set('kuka_time', frameNumber)
        pass
