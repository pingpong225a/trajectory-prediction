import numpy as np 
#import queue 
import redis 

playing_field = [] # (xmin,xmax) (ymin,ymax) (zmin, zmax)
dest_x = 20 # x location at which the kuka is 


# ()
MAX_POSITIONS = 30
KUKA_POS_Y = 0 # in optitrack coordinates 
KUKA_POS_Z = 2.85

class Trajectory: 
    def __init__(self):
        global MAX_POSITIONS
        #self.ball_trace = queue.Queue(max_size=50) 
        #self.redis_conn = redis.StrictRedis(host='localhost', port=6379, db=0)
        self.positions = np.zeros((MAX_POSITIONS, 3))
        self.time = np.ones((MAX_POSITIONS, 2))
        self.index = 0
        self.num_points = 0
        self.pred_hist = np.zeros((MAX_POSITIONS, 3))
        self.pred_weight = np.exp(np.linspace(-MAX_POSITIONS,-1,MAX_POSITIONS))
        self.pred_weight = self.pred_weight / np.sum(self.pred_weight)
    # def lin_reg(X, Y):

    
    def record_pos(self, pos, time):
        global MAX_POSITIONS
        self.positions[self.index, :] = pos[0:3] 
        self.time[self.index, 0] = time / 200.0
        self.index = (self.index + 1) % MAX_POSITIONS
        if self.num_points < MAX_POSITIONS:
            self.num_points += 1 

    def filter_ball_des(self, a):
        x, y, z = a
        #x = 0.2
        if y < -0.3:
            y = -0.3
        if y > 0.2:
            y = 0.2
        if z > 0.7:
            z = 0.7
        if z < 0.2:
            z = 0.2
        return [x, y, z]

    def calc_trajectory(self):
        global MAX_POSITIONS
        if self.num_points >= MAX_POSITIONS:
            tx = np.linalg.lstsq(self.time, self.positions[:, 0])
            ty = np.linalg.lstsq(self.time, self.positions[:, 1])
            tz = np.linalg.lstsq(self.time, self.positions[:, 2])
            #tz = np.polyfit(self.time[0:MAX_POSITIONS,0], self.positions[0:MAX_POSITIONS, 2],2)
            cx = tx[0][1]
            mx = tx[0][0]
            cy = ty[0][1]
            my = ty[0][0]
            cz = tz[0][1]
            mz = tz[0][0]
            #pz2 = tz[0]
            #pz1 = tz[1]
            #pz0 = tz[2]
            t = (0.526 - 1.0 * cx) / mx
            dt = (t - self.time[self.index,0])
            x_pred = mx * t + cx
            y_pred = my * t + cy
            z_pred = mz * t + cz  - 9.8/2 * dt * dt 
            print("time needed to reach to the prediction pos")
            print(dt)
            #z_pred = pz2 * t * t + pz1 * t + pz0
            #print (x_pred, y_pred, z_pred)
            self.pred_hist[self.index][0] = x_pred
            self.pred_hist[self.index][1] = y_pred
            self.pred_hist[self.index][2] = z_pred
            x_pred = np.mean( self.pred_hist[:,0] ) * 0.3 + 0.7 * x_pred
            y_pred = np.mean( self.pred_hist[:,1] ) * 0.3 + 0.7 * y_pred
            z_pred = np.mean( self.pred_hist[:,2] ) * 0.3 + 0.7 * z_pred
             
            return self.filter_ball_des([x_pred, y_pred, z_pred]) #if y_pred > -0.5 and y_pred < 0.5 else None        
            return [x_pred, y_pred, z_pred] #if y_pred > -0.5 and y_pred < 0.5 else None        
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
