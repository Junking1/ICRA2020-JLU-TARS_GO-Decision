
import math
import random
import sys

import Box2D
import gym
import numpy as np
import pyglet
from gym import spaces
from gym.utils import EzPickle, colorize, seeding
from pyglet import gl

from battlefield.body.obstacle import ICRALayout
from battlefield.body.robot import Robot
from battlefield.body.projectile import Projectile
from battlefield.referee.contact import ContactListener
from battlefield.referee.buff import AreaBuff
from battlefield.referee.supply import AreaSupply
from battlefield.sensor.capture import callback_capture
from utils import *
from pyglet.window import key, mouse

WINDOW_W = 1200
WINDOW_H = 1000

SCALE = 40.0        # Track scale
PLAYFIELD = 400/SCALE  # Game over boundary
FPS = 30
ZOOM = 2.7        # Camera zoom

SCAN_RANGE = 8  # m


class ICRABattleField(gym.Env, EzPickle):

    __pos_safe = [
        [0.5, 0.5], [0.5, 2.0], [0.5, 3.0], [0.5, 4.5],  # 0 1 2 3
        [1.5, 0.5], [1.5, 3.0], [1.5, 4.5],             # 4 5 6
        [2.75, 0.5], [2.75, 2.0], [2.75, 3.0], [2.75, 4.5],  # 7 8 9 10
        [4.0, 1.75], [4.0, 3.25],                         # 11 12
        [5.25, 0.5], [5.25, 2.0], [5.25, 3.0], [5.25, 4.5],  # 13 14 15 16
        [6.5, 0.5], [6.5, 2.0], [6.4, 4.3],             # 17 18 19
        [7.5, 0.5], [7.5, 2.0], [7.5, 3.0], [7.5, 4.5]  # 20 21 22 23
    ]
    __id_pos_linked = [
        [1, 2, 3, 4], [0, 2, 3], [0, 1, 3, 5], [0, 1, 2, 6],
        [0, 7], [2, 9], [3, 10],
        [8, 9, 10, 4], [7, 9, 10, 11], [7, 8, 10, 5, 12], [7, 8, 9],
        [8, 14], [9, 15],
        [14, 15, 16, 17], [13, 15, 16, 18, 11, 11, 11, 11, 11], [
            13, 14, 16, 12, 12, 12, 12, 12], [13, 14, 15, 19],
        [13, 20], [14, 21], [16, 23],
        [21, 22, 23, 17], [20, 22, 23, 18], [20, 21, 23], [20, 21, 22, 19]
    ]

    def __init__(self):
        EzPickle.__init__(self)
        self.seed()
        self.__contactListener_keepref = ContactListener(self)
        self.__world = Box2D.b2World(
            (0, 0), contactListener=self.__contactListener_keepref)
        self.viewer = None
        self.__robots = []
        self.__robot_name = [ID_R1, ID_B1, ID_R2, ID_B2]
        self.__obstacle = None
        self.__area_buff = None
        self.__projectile = None
        #self.__area_supply = None
        self.__callback_autoaim = callback_capture()

        self.reward = 0.0
        self.prev_reward = 0.0
        self.actions = None
        self.action = [Action(), Action()]
        self.state = None
        self.step_reward = 0
        self.conflict = False

        self.o_ang = 0
        self.o_dis = 8.0
        self.control = False

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _destroy(self):
        for r in self.__robots:
            if r:
                r.destroy()
            r = None
        if self.__obstacle:
            self.__obstacle.destroy()
        self.__obstacle = None
        if self.__projectile:
            self.__projectile.destroy()
        self.__projectile = None

    def reset(self):
        self._destroy()
        self.reward = 0.0
        self.prev_reward = 0.0
        self.t = 0.0

        random_index = random.randint(0, 23)
        #random_index = 5
        init_pos_0 = self.__pos_safe[random_index]
        #init_pos_1 = self.__pos_safe[9]
        init_pos_1 = self.__pos_safe[random.choice(
            self.__id_pos_linked[random_index])]
        while(init_pos_1[0] < 1.0):
            init_pos_1 = self.__pos_safe[random.randint(0, 23)]

        # 新添加的位置
        init_pos_2 = self.__pos_safe[random.randint(0, 23)]
        init_pos_3 = self.__pos_safe[random.randint(0, 23)]
        while(init_pos_3[0] < 1.0):
            init_pos_3 = self.__pos_safe[random.randint(0, 23)]
        # print(init_pos_0, init_pos_1)

        self.__R1 = Robot(self.__world, 0, [0.5, 4.0], ID_R1, color_=(0.6, 0.2, 0.6))
        self.__B1 = Robot(self.__world, 0, init_pos_1, ID_B1)
        # 增加两个车
        self.__R2 = Robot(self.__world, 0, [0.5, 0.5], ID_R2)
        self.__B2 = Robot(self.__world, 0, init_pos_3, ID_B2)
        print([0.5, 4.5], init_pos_1, [0.5, 0.5], init_pos_3)
        self.__robots = [self.__R1, self.__B1, self.__R2, self.__B2]

        self.__obstacle = ICRALayout(self.__world)
        self.__projectile = Projectile(self.__world)
        self.__area_buff = AreaBuff()
        #self.__area_supply = AreaSupply()

        self.state = [RobotState([0.5, 4.4]), RobotState(init_pos_1), RobotState([0.5, 0.5]), RobotState(init_pos_3)]
        self.actions = [Action(), Action(), Action(), Action()]

        self.reward = 0

        return init_pos_1, init_pos_3
        # return self.step(None)[0]

    def key_press(self, k, mod):
        if k == key.ESCAPE:
            self.done = True
        if k == key.W:
            self.action[0].v_t = +1.0
        if k == key.S:
            self.action[0].v_t = -1.0
        if k == key.Q:
            self.action[0].angular = +1.0
        if k == key.E:
            self.action[0].angular = -1.0
        if k == key.D:
            self.action[0].v_n = +1.0
        if k == key.A:
            self.action[0].v_n = -1.0
        if k == key.SPACE:
            self.action[0].shoot = +1.0
        # if k == key.R:
        #     self.action[0].supply = +1.0

        if k == key.UP:
            self.action[1].v_t = +1
        if k == key.DOWN:
            self.action[1].v_t = -1
        if k == key.RIGHT:
            self.action[1].angular = -1
        if k == key.LEFT:
            self.action[1].angular = +1
        if k == key.C:
            self.action[1].shoot = +1

    def key_release(self, k, mod):
        if k == key.W:
            self.action[0].v_t = +0.0
        if k == key.S:
            self.action[0].v_t = -0.0
        if k == key.Q:
            self.action[0].angular = +0.0
        if k == key.E:
            self.action[0].angular = -0.0
        if k == key.D:
            self.action[0].v_n = +0.0
        if k == key.A:
            self.action[0].v_n = -0.0
        if k == key.SPACE:
            self.action[0].shoot = +0.0

        if k == key.UP:
            self.action[1].v_t = +0
        if k == key.DOWN:
            self.action[1].v_t = -0
        if k == key.RIGHT:
            self.action[1].angular = +0
        if k == key.LEFT:
            self.action[1].angular = -0
        if k == key.C:
            self.action[1].shoot = +0

    def __step_contact(self):
        contact_bullet_robot = self.__contactListener_keepref.collision_bullet_robot
        contact_bullet_wall = self.__contactListener_keepref.collision_bullet_wall
        contact_robot_wall = self.__contactListener_keepref.collision_robot_wall
        contact_robot_robot = self.__contactListener_keepref.collision_robot_robot
        for bullet, robot in contact_bullet_robot:
            self.__projectile.destroyById(bullet.id)
            if(self.__robots[robot.id].buff_left_time) > 0:
                self.__robots[robot.id].lose_health(25)
            else:
                self.__robots[robot.id].lose_health(50)
            if bullet.robort_id == ID_R1 and robot.id%2 == ID_B1:
                # FIXME 红车击中蓝车奖励
                if min(self.dist_record(bullet.robort_id)) >= 1.2:
                    self.step_reward += 0.01
        for bullet in contact_bullet_wall:
            self.__projectile.destroyById(bullet.id)

        # FIXME 冲撞惩罚
        for robot in contact_robot_wall:
            if robot.id % 2 == 0:
                if self.__robots[robot.id].get_health() > 0:
                    self.__robots[robot.id].lose_health(80)
                    if robot.id == ID_R1:
                        #print(f"hit wall, HP left: {self.__robots[robot.id].get_health()}")
                        if self.__robots[ID_R1].get_health() <= 0:
                            self.step_reward = -1.0
                        else:
                            self.step_reward -= 0.3
                        self.conflict = True
            # self.__robots[robot.id].lose_health(100)
        for robot in contact_robot_robot:
            if robot.id % 2 == 0:
                if self.__robots[robot.id].get_health() > 0:
                    self.__robots[robot.id].lose_health(80)
                    if robot.id == ID_R1:
                        #print(f"hit robot, HP left: {self.__robots[robot.id].get_health()}")
                        if self.__robots[ID_R1].get_health() <= 0:
                            self.step_reward = -1.0
                        else:
                            self.step_reward -= 0.3
                        self.conflict = True

        self.__contactListener_keepref.clean()

    def _step_action(self, robot: Robot, action: Action):
        # gas, rotate, transverse, rotate cloud terrance, shoot
        robot.move_ahead_back(action.v_t)
        robot.turn_left_right(action.angular)
        robot.move_left_right(action.v_n)
        # if int(self.t * FPS) % (60 * FPS) == 0:
        #     robot.refresh_supply_oppotunity()
        # if action.supply > 0.99:
        #     action.supply = 0.0
        #     if robot.if_supply_available():
        #         robot.use_supply_oppotunity()
        #         if self.__area_supply.if_in_area(robot):
        #             robot.supply()
        if action.shoot > 0.99 and int(self.t*FPS) % (FPS/5) == 1:
            if (robot.if_left_projectile() > 0) and robot.get_health() > 0:
                # print("shoot")
                # FIXME 下面这行条件只允许红方发射子弹
                if robot.robot_id % 2 == ID_R1:
                    angle, pos = robot.get_gun_angle_pos()
                    robot.shoot()
                    self.__projectile.shoot(angle, pos, robot.robot_id)

    def _autoaim(self, robot: Robot, state: RobotState):
        #detected = {}
        if robot.robot_id == ID_R1:
            self.o_dis = 8.0
        scan_distance, scan_type = [], []
        state.detect = False
        type_ = None
        for i in range(-135, 135, 2):
            angle, pos = robot.get_angle_pos()
            angle += i/180*math.pi
            p1 = (pos[0] + 0.374*math.cos(angle), pos[1] + 0.374*math.sin(angle))
            p2 = (pos[0] + SCAN_RANGE*math.cos(angle),
                  pos[1] + SCAN_RANGE*math.sin(angle))
            self.__world.RayCast(self.__callback_autoaim, p1, p2)
            scan_distance.append(self.__callback_autoaim.fraction)
            #print(f"ang {i}\tfraction:{self.__callback_autoaim.fraction}\t")
            u = self.__callback_autoaim.userData
            # print(f"u type {u.type}")
            # if abs(i) == 45 and robot.robot_id == ID_R1:
            #     print(f"angle:{i}, frac:{self.__callback_autoaim.fraction}")
            # FIXME 射线扫描距离检测
            if robot.robot_id==ID_R1 and u is not None:
                if u.type == 'wall':
                    if self.o_dis > self.__callback_autoaim.fraction * (SCAN_RANGE-0.374):
                        self.o_dis = self.__callback_autoaim.fraction * (SCAN_RANGE-0.374)
                        self.o_ang = i
                        #print("wall", self.o_dis)
                if u.type == 'robot':
                    d = self.__callback_autoaim.fraction * (SCAN_RANGE-0.374) - 0.10
                    if self.o_dis > d:
                        self.o_dis = d
                        #print("robot", self.o_dis)
                        self.o_ang = i

            if u is not None and u.type == "robot":
                if u.id%2 == robot.robot_id%2:
                    scan_type.append(0)
                    continue
                else:
                    scan_type.append(1)
                if -45 <= i <= 45:
                    #if u.id not in [ID_R1, ID_R2]:
                    if u.id%2 != robot.robot_id%2:
                        if not state.detect:
                            robot.set_gimbal(angle)
                            state.detect = True
            else:
                scan_type.append(0)
        # FIXME 距离障碍物太近的时候惩罚, 但是由于扫面半径起点不是刚好是车的外表, 可能这个不加更好
        # if min(scan_distance) * (SCAN_RANGE-0.374) <= 0.005:
        #     self.conflict = True
        
        
        state.scan = [scan_distance, scan_type]
        # print(state.scan)
        # exit(0)

    def _update_robot_state(self, robot: Robot, state: RobotState):
        state.pos = robot.get_pos()
        state.health = robot.get_health()
        state.angle = robot.get_angle()
        state.velocity = robot.get_velocity()
        state.angular = robot.get_angular()

    def set_robot_action(self, robot_id, action: Action):
        self.actions[robot_id] = action if self.__robots[robot_id].get_health() > 0 else None

    def step(self, action: list):
        ###### observe ######
        for robot, state in zip(self.__robots, self.state):
            self._autoaim(robot, state)
            self._update_robot_state(robot, state)

        ###### action ######
        self.control = False
        action_= self._auto_aviod(self.o_ang, self.o_dis)
        # print(d)
        # if d < 0.1:
        #     print("action modified")
        #     action[0].angular = r
        #     if m == 0:
        #         action[0].v_t = 1
        #     else:
        #         action[0].v_n = m
        # print(action[0])
        if not self.control:
            #print("by agent")
            self.set_robot_action(ID_R1, action[0])
        else:
            #print("by hand")
            action_.shoot = action[0].shoot
            self.set_robot_action(ID_R1, action_)
        self.set_robot_action(ID_R2, action[1])

        for i in range(len(self.__robots)):
            if self.__robots[i].get_health() <= 0:
                self.actions[i] = None

        for robot, action in zip(self.__robots, self.actions):
            if action is not None:
                self._step_action(robot, action)
            robot.step(1.0/FPS)
        self.__world.Step(1.0/FPS, 6*30, 2*30)
        self.t += 1.0/FPS

        # 碰撞检测预处理
        self.step_reward = 0.0
        self.conflict = False
        ###### Referee ######
        self.__step_contact()
        if not self.conflict:
            x, y = self.__robots[ID_R1].get_pos()
            if 7.0>x>1.0 and 0.2<y<4.8:
                self.step_reward += 0.01
            else:
                self.step_reward += 0.005

        for robot in self.__robots:
            self.__area_buff.detect(robot, self.t)
        # TODO 距离太近惩罚
        if min(self.dist_record(ID_R1)) <= 1.20:
            self.step_reward -= 0.1
        ###### reward ######
        done = False
        # First step without action, called from reset()
        # if self.actions[ID_R1] is not None:
        #     self.reward = (self.__robots[ID_R1].get_health() -
        #                    self.__robots[ID_B1].get_health()) / 4000.0

        #self.reward += 10 * self.t * FPS
        # self.step_reward = self.reward - self.prev_reward
        # if self.state[ID_R1].detect:
        #     self.step_reward += 1/3000

        if self.__robots[ID_R1].get_health() <= 0:
            done = True
            #step_reward -= 1
        if self.__robots[ID_B1].get_health() <= 0 and self.__robots[ID_B2].get_health() <= 0.0:
            done = True
            #step_reward += 1
        self.reward += self.step_reward
        self.prev_reward = self.reward

        return self.state, self.step_reward, done, {}

    @staticmethod
    def get_gl_text(x, y):
        return pyglet.text.Label('0000', font_size=16, x=x, y=y,
                                 anchor_x='left', anchor_y='center',
                                 color=(255, 255, 255, 255))

    def render(self, mode='god'):
        if self.viewer is None:
            from gym.envs.classic_control import rendering
            self.viewer = rendering.Viewer(WINDOW_W, WINDOW_H)
            self.time_label = self.get_gl_text(20, WINDOW_H * 5.0 / 40.0)
            self.score_label = self.get_gl_text(520, WINDOW_H * 2.5 / 40.0)
            self.health_label = self.get_gl_text(520, WINDOW_H * 3.5 / 40.0)
            self.projectile_label = self.get_gl_text(
                520, WINDOW_H * 4.5 / 40.0)
            self.buff_left_time_label = self.get_gl_text(
                520, WINDOW_H * 5.5 / 40.0)
            self.buff_stay_time = self.get_gl_text(520, WINDOW_H * 6.5 / 40.0)
            self.transform = rendering.Transform()

        if "t" not in self.__dict__:
            return  # reset() not called yet

        zoom = ZOOM*SCALE
        scroll_x = 4.0
        scroll_y = 0.0
        angle = 0
        self.transform.set_scale(zoom, zoom)
        self.transform.set_translation(
            WINDOW_W/2 - (scroll_x*zoom*math.cos(angle) -
                          scroll_y*zoom*math.sin(angle)),
            WINDOW_H/4 - (scroll_x*zoom*math.sin(angle) + scroll_y*zoom*math.cos(angle)))

        self.__obstacle.draw(self.viewer)
        if mode == 'god':
            for robot in self.__robots:
                robot.draw(self.viewer)
        elif mode == "fps":
            self.__robots[ID_R1].draw(self.viewer)
            self.__robots[ID_B1].draw(self.viewer)
        self.__projectile.draw(self.viewer)

        arr = None
        win = self.viewer.window
        if mode != 'state_pixels':
            win.switch_to()
            win.dispatch_events()

        win.clear()
        t = self.transform
        gl.glViewport(0, 0, WINDOW_W, WINDOW_H)
        t.enable()
        self._render_background()
        for geom in self.viewer.onetime_geoms:
            geom.render()
        t.disable()
        self._render_indicators(WINDOW_W, WINDOW_H)
        win.flip()

        self.viewer.onetime_geoms = []
        return arr

    def close(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None

    def _render_background(self):
        gl.glBegin(gl.GL_QUADS)
        gl.glColor4f(0.4, 0.8, 0.4, 1.0)
        gl.glVertex3f(-PLAYFIELD, +PLAYFIELD, 0)
        gl.glVertex3f(+PLAYFIELD, +PLAYFIELD, 0)
        gl.glVertex3f(+PLAYFIELD, -PLAYFIELD, 0)
        gl.glVertex3f(-PLAYFIELD, -PLAYFIELD, 0)
        gl.glColor4f(0.4, 0.9, 0.4, 1.0)
        k = PLAYFIELD/20.0
        for x in range(-20, 20, 2):
            for y in range(-20, 20, 2):
                gl.glVertex3f(k*x + k, k*y + 0, 0)
                gl.glVertex3f(k*x + 0, k*y + 0, 0)
                gl.glVertex3f(k*x + 0, k*y + k, 0)
                gl.glVertex3f(k*x + k, k*y + k, 0)
        gl.glEnd()
        self.__area_buff.render(gl)
        # self.__area_supply.render(gl)

    def _render_indicators(self, W, H):
        self.time_label.text = "Time: {} s".format(int(self.t))
        self.score_label.text = "Score: %04i" % self.reward
        self.health_label.text = "health left Car0 : {} Car1: {} ".format(
            self.__robots[ID_R1].get_health(), self.__robots[ID_B1].get_health())
        self.projectile_label.text = "Car0 bullets : {}, oppotunity to add : {}  ".format(
            self.__robots[ID_R1].get_left_projectile(
            ), self.__robots[ID_R1].supply_opportunity_left
        )
        self.buff_stay_time.text = 'Buff Stay Time: Red {}s, Blue {}s'.format(
            int(self.__area_buff.get_single_buff(GROUP_RED).get_stay_time()),
            int(self.__area_buff.get_single_buff(GROUP_BLUE).get_stay_time()))
        self.buff_left_time_label.text = 'Buff Left Time: Red {}s, Blue {}s'.format(
            int(self.__robots[ID_R1].buff_left_time),
            int(self.__robots[ID_B1].buff_left_time))
        self.time_label.draw()
        self.score_label.draw()
        self.health_label.draw()
        self.projectile_label.draw()
        self.buff_stay_time.draw()
        self.buff_left_time_label.draw()

    def get_roborts(self):
        return self.__robots

    def _auto_aviod(self, ang:int, dis:float):
        thet = (ang+135) % 90
        thet = min(thet, 90-thet)
        min_dis = math.cos(thet) * 0.374
        # print("auto avoid")
        # print(self.control, dis, sep='\t')
        # FIXME 墙的安全距离
        self.control = False
        action = Action()
        # print(dis)
        escap, action.v_n, action.v_t = self._out_clip()
        if escap:
            print("clip")
            return action
        if dis <= 0.10:
            self.control = True
            if self.conflict:
                print("avoid")
            ang = (ang + 45) * math.pi / 180.0
            action.v_t = -math.sin(ang)
            action.v_n = -math.cos(ang)
            # if dis >= 0.009:
            #     print(ang)
            #     # action.angular = 1
            #     action.v_t = -1
            #     print("jun")
            # if -135 <= ang < -90:
            #     action.v_n = -1
            #     # if dis >= 0.009:
            #     #     action.angular = -1
            #     #action.v_t = 1
            # elif -90 <= ang < -45:
            #     action.v_n = -1
            #     # if dis >= 0.009:
            #     #     action.angular = -1
            # elif -45 <= ang < 0:
            #     action.v_n = -1
            #     # if dis >= 0.009:
            #     #     action.angular = -1
            # elif 0 <= ang < 45:
            #     action.v_n = 1
            #     # if dis >= 0.009:
            #     #     action.angular = 1
            # elif 45 <= ang < 90:
            #     action.v_n = 1
            #     # if dis >= 0.009:
            #     #     action.angular = 1
            # else:
            #     # action.v_n = -1
            #     action.v_t = 1
            #     print("else")
                # if dis >= 0.009:
                #     action.angular = 1
                #action.v_t = 1

            # # action.v_t = 1
            # if -90 < ang < 90:
            #     if -45 < ang < 45:
            #         if ang < 0:
            #             action.v_n = -1
            #         else:
            #             action.v_n = 1
            #     elif ang < 0:
            #         action.angular = 1 # 左转
            #         action.v_n = -1
            #     else:
            #         action.angular = -1 # 右转
            #         action.v_n = 1
            # else:
            #     action.v_t = 1    # 向前
            #     if ang < 0:
            #         action.angular = 1
            #     else:
            #         action.angular = -1
            return action
        else:
            self.control = False
            return None

    def dist_record(self, robot_id:int)->list:
        pos = self.__robots[robot_id].get_pos()
        dis = []
        for r in range(4):
            if r != robot_id:
                dis.append(math.sqrt((pos.x - self.__robots[r].get_pos().x)**2 + (pos.y - self.__robots[r].get_pos().y)**2))
        return dis

    def _out_clip(self):
        # TODO 脱离夹击
        frac = self.state[ID_R1].scan[0]
        # type = self.state[ID_R1].scan[1]
        dis = [x*(SCAN_RANGE-0.374) for x in frac]

        for i in range(0, 80):
            st = min(dis[i:i+10])
            en = min(dis[i+45:i+55])
            if st + en <= 0.40:
                ahead = min(dis[i+20:i+25])
                ang = 0
                if i > 40:
                    if ahead < 0.5:
                        ang = self._index2dec((i+90)%180)
                    else:
                        ang = self._index2dec(i)
                else:
                    back = min(dis[(i+113)%135:(i+113)%135+5])
                    if back <= ahead:
                        ang = self._index2dec(i)
                    else:
                        ang = self._index2dec((i+90)%180)
                v_n = math.cos(ang)
                v_t = math.sin(ang)
                return True, v_n, v_t
            else:
                return False, None, None

    def _index2dec(self, index:int):
        # FIXME 扫描顺逆时针待确认
        ang = index * 2 + 45
        rad = ang / 180.0 * math.pi
        return rad


if __name__ == "__main__":
    from pyglet.window import key, mouse

    # gas, rotate, transverse, rotate cloud terrance, shoot, reload
    #a = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    #target = [0, 0]
    a = Action()

    def on_mouse_release(x, y, button, modifiers):
        x_low, x_high, y_low, y_high = 168, 1033, 249, 789
        width = x_high - x_low
        height = y_high - y_low
        x = (x - x_low) / width * 8.0
        y = (y - y_low) / height * 5.0
        target[0] = x
        target[1] = y

    def key_press(k, mod):
        global restart
        if k == key.ESCAPE:
            restart = True
        if k == key.W:
            a.v_t = +1.0
        if k == key.S:
            a.v_t = -1.0
        if k == key.Q:
            a.angular = +1.0
        if k == key.E:
            a.angular = -1.0
        if k == key.D:
            a.v_n = +1.0
        if k == key.A:
            a.v_n = -1.0
        if k == key.SPACE:
            a.shoot = +1.0
        if k == key.R:
            a.supply = +1.0

    def key_release(k, mod):
        if k == key.W:
            a.v_t = +0.0
        if k == key.S:
            a.v_t = -0.0
        if k == key.Q:
            a.angular = +0.0
        if k == key.E:
            a.angular = -0.0
        if k == key.D:
            a.v_n = +0.0
        if k == key.A:
            a.v_n = -0.0
        if k == key.SPACE:
            a.shoot = +0.0

    env = ICRABattleField()
    env.render()
    record_video = False
    if record_video:
        env.monitor.start('/tmp/video-test', force=True)
    env.viewer.window.on_key_press = key_press
    env.viewer.window.on_key_release = key_release
    #env.viewer.window.on_mouse_release = on_mouse_release
    #move = NaiveMove()
    while True:
        env.reset()
        total_reward = 0.0
        steps = 0
        restart = False
        s, r, done, info = env.step(a)
        while True:
            s, r, done, info = env.step(a)
            total_reward += r

            if steps % 200 == 0 or done:
                print("step {} total_reward {:+0.2f}".format(steps, total_reward))
            steps += 1

            # Faster, but you can as well call env.render() every time to play full window.
            if not record_video:
                env.render()
            if done or restart:
                break
    env.close()
