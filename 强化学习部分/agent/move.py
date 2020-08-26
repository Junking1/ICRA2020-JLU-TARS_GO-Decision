import numpy as np
import math

class NaiveMove:
    def __init__(self):
        self.done = False

    def moveTo(self, pos, vel, angle, goal):
        if ((pos[0]-goal[0])**2 + (pos[1]-goal[1])**2) < 0.01:
            return [0, 0], 0
            self.done = True
        else:
            self.done = False
        MAX_ACC = 1.5 * (1/30.0)
        pos_delta = np.array([goal[0]-pos[0], goal[1]-pos[1]])
        target_angle = math.atan2(pos_delta[1], pos_delta[0])
        pos_delta = pos_delta.reshape([2, 1])
        mat_angle = np.array([
            [math.cos(angle), math.sin(angle)],
            [math.sin(angle), -math.cos(angle)]])
        v_target = np.matmul(np.linalg.inv(mat_angle), pos_delta).reshape([2])
        v_target /= np.abs(v_target).max()
        #v_now = np.array(vel)
        #v_delta = v_target - v_now
        #print(v_target, v_now, v_delta)
        #v_delta /= np.abs(v_delta).max()

        #MAX_ACC = min(MAX_ACC, np.abs(0.5 - v_now).max())
        #v_real = v_delta*MAX_ACC + v_now
        #print(goal, v_real)
        return v_target, target_angle - angle
