"""
Mobile robot motion planning sample with Dynamic Window Approach
author:
"""
import math
import sys
import time

import matplotlib.pyplot as plt
import numpy as np

sys.path.append(".")
from Referee.ICRAMap import BORDER_BOX, BORDER_POS

show_animation = True


class Config():
    # simulation parameters

    def __init__(self):
        # robot parameter
        self.max_speed = 2.0  # [m/s]
        self.min_speed = 0.05  # [m/s]
        self.max_yawrate = 180 * math.pi / 180.0  # [rad/s]  #40
        self.max_accel = 3.0  # [m/ss]
        self.max_dyawrate = 400 * math.pi / 180.0  # [rad/ss]
        self.v_reso = 0.05  # [m/s]
        self.yawrate_reso = 1 * math.pi / 180.0  # [rad/s]
        self.dt = 1/30.0  # [s]
        self.predict_time = 2.0  # [s]
        self.to_goal_cost_gain = 100.0
        self.speed_cost_gain = 10.0
        self.robot_radius = 0.4  # [m]


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


def calc_dynamic_window(x, config):

    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,
          -config.max_yawrate, config.max_yawrate]

    # Dynamic window from motion model
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_dyawrate * config.dt,
          x[4] + config.max_dyawrate * config.dt]

    #  [vmin,vmax, yawrate min, yawrate max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw


def calc_trajectory(xinit, v, y, vv, config):

    x = np.array(xinit)
    traj = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, y, vv], config.dt)
        traj = np.vstack((traj, x))
        time += config.dt

    return traj


def calc_final_input(x, u, dw, config, goal, ob, goal_angle):

    xinit = x[:]
    min_cost = 1e11
    min_u = u
    min_u[0] = 0.0
    best_traj = np.array([x])

    #delta = np.array([goal[0]-x[0], goal[1]-x[1]])
    #target_angle = math.atan2(delta[1], delta[0])
    omega = min([(goal_angle - x[2]) * 3, config.max_yawrate])
    #omega = 0

    # evalucate all trajectory with sampled input in dynamic window
    for theta in np.arange(0, math.pi/4, 0.1):
        #omega = min([(goal_angle - theta)*5, config.max_yawrate])
        for v in np.arange(dw[0], dw[1], config.v_reso):
            v_t = v * math.cos(theta)
            v_n = -v * math.sin(theta)
            traj = calc_trajectory(xinit, v_t, omega, v_n, config)
            to_goal_cost = calc_to_goal_cost(traj, goal, config)
            speed_cost = config.speed_cost_gain * (config.max_speed - traj[-1, 3])
            ob_cost = calc_obstacle_cost(traj, ob, config)

            final_cost = to_goal_cost + speed_cost + ob_cost

            if min_cost >= final_cost:
                min_cost = final_cost
                min_u = [v_t, omega, v_n]
                best_traj = traj

        theta = -theta
        #omega = omega_base - theta*0.1
        #omega = min([(goal_angle - theta)*5, config.max_yawrate])
        for v in np.arange(dw[0], dw[1], config.v_reso):
            v_t = v * math.cos(theta)
            v_n = -v * math.sin(theta)
            traj = calc_trajectory(xinit, v_t, omega, v_n, config)
            to_goal_cost = calc_to_goal_cost(traj, goal, config)
            speed_cost = config.speed_cost_gain * (config.max_speed - traj[-1, 3])
            speed_cost += config.speed_cost_gain/2 * (config.max_speed - traj[-1, 5])
            ob_cost = calc_obstacle_cost(traj, ob, config)

            final_cost = to_goal_cost + speed_cost + ob_cost

            if min_cost >= final_cost:
                min_cost = final_cost
                min_u = [v_t, omega, v_n]
                best_traj = traj
            
    '''
    for v in np.arange(dw[0], dw[1], config.v_reso):
        for y in [(target_angle-x[2])*10]:
            for vv in np.arange(-dw[1], dw[1], config.v_reso):
                traj = calc_trajectory(xinit, v, y, vv, config)
                # calc cost
                to_goal_cost = calc_to_goal_cost(traj, goal, config)
                speed_cost = config.speed_cost_gain * \
                    (config.max_speed - traj[-1, 3])
                ob_cost = calc_obstacle_cost(traj, ob, config)
                #print("ob cost: {}".format(ob_cost))

                final_cost = to_goal_cost + speed_cost + ob_cost

                #print (final_cost)

                # search minimum trajectory
                if min_cost >= final_cost:
                    min_cost = final_cost
                    min_u = [v, y, vv]
                    best_traj = traj
    '''

    return min_u, best_traj


def calc_obstacle_cost(traj, ob, config):
    # calc obstacle cost inf: collistion, 0:free

    skip_n = 2
    minr = float("inf")

    for ii in range(0, len(traj[:, 1]), skip_n):
        x = min(int(traj[ii, 0]*10), 79)
        y = min(int(traj[ii, 1]*10), 49)
        ox, oy = ob[y, x]
        dx = (traj[ii, 0]) - ox
        dy = (traj[ii, 1]) - oy
        #r = np.hypot(dx, dy)
        r = math.sqrt(dx**2 + dy**2)
        if r <= config.robot_radius:
            #print("r: {}".format(r))
            return float("Inf")  # collision
        if minr >= r:
            minr = r
        '''
        for i in range(len(ob[:, 0])):
            ox = ob[i, 0]
            oy = ob[i, 1]
            dx = traj[ii, 0] - ox
            dy = traj[ii, 1] - oy

            r = math.sqrt(dx**2 + dy**2)
            if r <= config.robot_radius:
                print("r: {}".format(r))
                return float("Inf")  # collision

            if minr >= r:
                minr = r
        '''

    #print("minr: {}".format(minr))
    return 1.0 / minr  # OK


def calc_to_goal_cost(traj, goal, config):
    # calc to goal cost. It is 2D norm.
    '''
    goal_magnitude = math.sqrt(goal[0]**2 + goal[1]**2)
    traj_magnitude = math.sqrt(traj[-1, 0]**2 + traj[-1, 1]**2)
    dot_product = (goal[0] * traj[-1, 0]) + (goal[1] * traj[-1, 1])
    error = dot_product / (goal_magnitude * traj_magnitude)
    error_angle = math.acos(error)
    cost = config.to_goal_cost_gain * error_angle
    angle_goal = math.atan2(goal[1] - traj[0,1], goal[0] - traj[0,0])
    angle_now = math.atan2(traj[-1,1] - traj[0,1], traj[-1,0] - traj[0,0])
    error_angle = (angle_now - angle_goal)**2
    cost = config.to_goal_cost_gain * error_angle
    '''
    cost = config.to_goal_cost_gain * (1 + math.sqrt(
        (goal[0]-traj[-1, 0])**2 + (goal[1]-traj[-1, 1])**2))

    return cost


def dwa_control(x, u, config, goal, ob, goal_angle):
    # Dynamic Window control

    dw = calc_dynamic_window(x, config)

    u, traj = calc_final_input(x, u, dw, config, goal, ob, goal_angle)

    return u, traj


def plot_arrow(x, y, yaw, length=0.5, width=0.1):  # pragma: no cover
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)


class DynamicWindow():
    def __init__(self):
        #ob = [[-1, -1]]
        self.ob = np.load("ob.npy")
        self.config = Config()

    def moveTo(self, action, pos, vel, angle, angular, goal, goal_angle=0):
        # vel = math.sqrt(vel[0]**2+vel[1]**2)
        x = np.array([pos[0], pos[1], angle, 0.0, 0.0, 0.0])
        u = np.array([vel[0]*math.cos(angle) + vel[1]*math.sin(angle), angular,
                      vel[0]*math.sin(angle) - vel[1]*math.cos(angle)])
        x[-3:] = u[-3:]
        u, ltraj = dwa_control(x, u, self.config, goal, self.ob, goal_angle)
        #print("Real angle: {}, angular :{}".format(angle, angular))
        #x = motion(x, u, self.config.dt)
        #print("Simulated angle: {}, angular: {}".format(x[2], x[4]))
        #print(pos, goal, u)
        #print(x, u)

        action[0] = u[0]
        action[1] = u[1]
        action[2] = u[2]
        return action


def main(gx, gy, ob):
    print(__file__ + " start!!")
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s), v_n(m/s)]
    x = np.array([0.5, 0.5, math.pi / 2.0, 0.0, 0.0, 0.0])
    # goal position [x(m), y(m)]
    goal = np.array([gx, gy])
    # obstacles [x(m) y(m), ....]
    ox, oy = [], []
    for (c_x, c_y), (w, h) in zip(BORDER_POS, BORDER_BOX):
        for i in np.arange(c_x-w, c_x+w, 1):
            for j in np.arange(c_y-h, c_y+h, 1):
                ox.append(i)
                oy.append(j)
    #ob = np.array(ob)
    '''
    ob = np.array([[-1, -1],
                   [0, 2],
                   [4.0, 2.0],
                   [5.0, 4.0],
                   [5.0, 5.0],
                   [5.0, 6.0],
                   [5.0, 9.0],
                   [8.0, 9.0],
                   [7.0, 9.0],
                   [12.0, 12.0]
                   ])
    '''

    u = np.array([0.0, 0.0, 0.0])
    config = Config()
    traj = np.array(x)

    for i in range(1000):
        tic = time.time()
        u, ltraj = dwa_control(x, u, config, goal, ob, math.pi/4)
        # print(time.time()-tic)

        x = motion(x, u, config.dt)
        print(x, u)
        traj = np.vstack((traj, x))  # store state history

        # print(traj)
        #print(x, goal, u)

        if show_animation:
            plt.cla()
            plt.plot(ltraj[:, 0], ltraj[:, 1], "-g")
            plt.plot(x[0], x[1], "xr")
            plt.plot(goal[0], goal[1], "xb")
            # plt.imshow(ob[:,:])
            plt.plot(ox, oy, "ok")
            plot_arrow(x[0], x[1], x[2])
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.0001)

        # check goal
        if math.sqrt((x[0] - goal[0])**2 + (x[1] - goal[1])**2) <= config.robot_radius:
            print("Goal!!")
            break

    print("Done")
    plt.close()
    if show_animation:
        plt.plot(traj[:, 0], traj[:, 1], "-r")
        plt.pause(0.0001)

    plt.show()


if __name__ == '__main__':
    dy = DynamicWindow()
    ob = dy.ob
    main(3.5, 4.5, ob)
