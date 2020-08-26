import math
import sys
import time

import matplotlib.pyplot as plt
import numpy as np

class Config():
    # simulation parameters

    def __init__(self):
        # robot parameter
        self.max_speed = 1.0  # [m/s]
        self.min_speed = -1.0  # [m/s]
        self.max_yawrate = 180 * math.pi / 180.0  # [rad/s]  #40
        self.max_accel = 5.0  # [m/ss]
        self.max_dyawrate = 400 * math.pi / 180.0  # [rad/ss]
        self.v_reso = 0.1  # [m/s]
        self.yawrate_reso = 1 * math.pi / 180.0  # [rad/s]
        self.dt = 1/30.0  # [s]
        self.predict_time = 3.0  # [s]
        self.to_goal_cost_gain = 5.0
        self.speed_cost_gain = 3.0
        self.robot_radius = 0.3  # [m]

def motion(x, u, dt):
    # motion model

    # u:velocity.u1: angle=>chuizhi velocity  u0: linar, x:position x0:x,x1:y,x2:angle
    x[2] += u[1] * dt
    x[0] += u[0] * math.cos(x[2]) * dt + u[2] * math.sin(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt - u[2] * math.cos(x[2]) * dt
    x[3] = u[0]
    x[4] = u[1]
    x[5] = u[2]

    return x

def calc_trajectory(xinit, v, y, vv, config):

    x = np.array(xinit)
    traj = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, y, vv], config.dt)
        traj = np.vstack((traj, x))
        time += config.dt

    return traj

config = Config()
# initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s), v_n(m/s)]
xinit = np.array([0.5, 0.5, math.pi / 2.0, 0.0, 0.0, 0.0])
traj = calc_trajectory(xinit, 0.1, -0.1, 1.0, config)

plt.cla()
plt.plot(traj[:, 0], traj[:, 1], "-g")
plt.xlim([0,5])
plt.ylim([0,5])
plt.show()