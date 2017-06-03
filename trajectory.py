import numpy as np 
import redis 
import time as pythontime

WORKSPACE_MIN_X = 0.1 # 0.1
WORKSPACE_MAX_X = 0.9  # 0.7
WORKSPACE_MIN_Y = -0.75 #  -0.61
WORKSPACE_MAX_Y =  -0.2 # -0.25

STAGE_MIN_X = 0
STAGE_MAX_X = 2.0
STAGE_MIN_Y = -0.7
STAGE_MAX_Y = 0.3 
STAGE_MIN_Z = -0.05

MAX_POSITIONS = 10 # 30
KUKA_POS_Y = 0 # in optitrack coordinates 
KUKA_POS_Z = 0.4 # 0.6  # 0.3
TABLE_POS_Z = 0.07 # 0.33 # 0

TRACK_BALL = True
HOME_POS = [0.05, -0.62, KUKA_POS_Z]

wait_time = 0 

class Trajectory: 
    def __init__(self):
        global MAX_POSITIONS
        self.positions = np.zeros((MAX_POSITIONS, 3))
        self.time = np.ones((MAX_POSITIONS, 2))
        self.index = 0
        self.num_points = 0

    def record_pos(self, pos, time):
        self.positions[self.index, :] = pos[0:3] 
        self.time[self.index, 0] = time / 200.0
        self.index = (self.index + 1) % MAX_POSITIONS
        if self.num_points < MAX_POSITIONS:
            self.num_points += 1 

    def clear_history(self):
        self.positions = np.zeros((MAX_POSITIONS, 3))
        self.time = np.ones((MAX_POSITIONS, 2))
        self.index = 0
        self.num_points = 0

    def going_away_from_kuka(self, mx):
        return mx >= 0

    def location_within_workspace(self, x, y, z):
        global WORKSPACE_MIN_X
        global WORKSPACE_MAX_X
        global WORKSPACE_MIN_Y
        global WORKSPACE_MAX_Y
        return x > WORKSPACE_MIN_X and x < WORKSPACE_MAX_X \
            and y > WORKSPACE_MIN_Y and y < WORKSPACE_MAX_Y

    def location_on_stage(self, pos):
        x, y, z = pos[0:3]
        global STAGE_MIN_X
        global STAGE_MAX_X
        global STAGE_MIN_Y
        global STAGE_MAX_Y
        global STAGE_MIN_Z
        return x > STAGE_MIN_X and x < STAGE_MAX_X \
             and y > STAGE_MIN_Y and y < STAGE_MAX_Y \
             and z > STAGE_MIN_Z

    def ball_hit_table(self, pos):
        global TABLE_POS_Z
        x, y, z = pos[0:3]
        return z < TABLE_POS_Z + 0.03

    def calc_trajectory(self, ball_pos, time):
        global MAX_POSITIONS
        global TABLE_POS_Z
        global KUKA_POS_Z
        global TRACK_BALL
        global HOME_POS

        global wait_time 
        curr_time = pythontime.time() 
        if curr_time < wait_time:
            return None    

        # Record/predict ball if on stage
        if self.location_on_stage(ball_pos): 

            if self.ball_hit_table(ball_pos):
                wait_time = pythontime.time() + 1 
                self.clear_history()
                print("returning home")
                return HOME_POS  

            self.record_pos(ball_pos, time)

            # Fit trajectory on ball path  
            if self.num_points >= MAX_POSITIONS:
                tx = np.linalg.lstsq(self.time, self.positions[:, 0])
                ty = np.linalg.lstsq(self.time, self.positions[:, 1])
                tz = np.polyfit(self.time[0:MAX_POSITIONS,0], self.positions[0:MAX_POSITIONS, 2],2)
                cx = tx[0][1]
                mx = tx[0][0]
                cy = ty[0][1]
                my = ty[0][0]
                pz2 = tz[0]
                pz1 = tz[1]
                pz0 = tz[2]
                pz0 -= TABLE_POS_Z
                coeffi = [pz2, pz1, pz0]
                roots = np.roots(coeffi) 
                t = np.max(np.real(roots))
                x_pred = mx * t + cx
                y_pred = my * t + cy
                z_pred = KUKA_POS_Z

                if self.going_away_from_kuka(mx) or (not self.location_within_workspace(x_pred, y_pred, z_pred)):
                    if self.going_away_from_kuka(mx) and (not self.location_within_workspace(x_pred, y_pred, z_pred)):
                        print("both")
                    elif self.going_away_from_kuka(mx):
                        print("going away from kuka")
                    elif not self.location_within_workspace(x_pred, y_pred, z_pred):
                        print("location outside workspace")
                        print([x_pred, y_pred, z_pred])

                    print("returning home 2")
                    self.clear_history() 
                    return HOME_POS
                else: 
                    print("sending pred")
                    print([x_pred, y_pred, z_pred])
                    return [x_pred, y_pred, z_pred]
       # else: 
           # print("location outside stage")

        return None

