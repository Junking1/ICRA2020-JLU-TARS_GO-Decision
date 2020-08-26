#from Referee.SupplyArea import SUPPLYAREABOX_BLUE
#from Referee.SupplyArea import SUPPLYAREABOX_RED
import numpy as np
import math
import Box2D
from Box2D.b2 import (fixtureDef, polygonShape, revoluteJointDef, )
from utils import UserData, COLOR_RED, COLOR_BLUE, COLOR_BLACK

# ICRA 2020 Robot Simulation
#JIlIN UNIVERSITY ZHAOZENGJUN


SIZE = 0.001
ROBOT_SIZE = 400 // 2
WHEEL_R = 50
WHEEL_W = 30
ROBOT_WIDTH = ROBOT_SIZE - WHEEL_W*2
ROBOT_LENGTH = ROBOT_SIZE
HULL_POLY = [
    (-ROBOT_LENGTH, +ROBOT_WIDTH), (+ROBOT_LENGTH, +ROBOT_WIDTH),
    (+ROBOT_LENGTH, -ROBOT_WIDTH), (-ROBOT_LENGTH, -ROBOT_WIDTH)
]
WHEEL_POS_X = ROBOT_LENGTH - WHEEL_R
WHEEL_POS_Y = ROBOT_WIDTH + WHEEL_W
WHEEL_POS = [
    (-WHEEL_POS_X, +WHEEL_POS_Y), (+WHEEL_POS_X, +WHEEL_POS_Y),
    (-WHEEL_POS_X, -WHEEL_POS_Y), (+WHEEL_POS_X, -WHEEL_POS_Y)
]
WHEEL_POLY = [
    (-WHEEL_R, +WHEEL_W), (+WHEEL_R, +WHEEL_W),
    (+WHEEL_R, -WHEEL_W), (-WHEEL_R, -WHEEL_W)
]
WHEEL_COLOR = (0.0, 0.0, 0.0)

GUN_POLY = [
    (-0, +20), (+260, +20),
    (+260, -20), (-0, -20)
]

BULLETS_ADDED_ONE_TIME = 50

#SUPPLY_AREAS = [
    #SUPPLYAREABOX_RED,  # (x, y, w, h)
    #SUPPLYAREABOX_BLUE
#]

ROBOT_COLOR = [COLOR_RED, COLOR_BLUE]


class Robot:

    def _create_dynamic_body(self, x, y, poly, userData, density=1.0, ang=0):
        return self.__world.CreateDynamicBody(
            position=(x, y), angle=ang,
            fixtures=[
                fixtureDef(
                    shape=polygonShape(vertices=[
                        (x*SIZE, y*SIZE) for x, y in poly
                    ]),
                    density=density, restitution=1, friction=1, userData=userData
                ),
            ]
        )

    def __init__(self, world, init_angle, init_pos, robot_id:int, color_=None):
        init_x, init_y = init_pos
        if color_ is not None:
            color = color_
        else:
            color = ROBOT_COLOR[robot_id % 2]
        userData = UserData("robot", robot_id, None)
        self.__world = world
        self.__hull = self._create_dynamic_body(init_x, init_y, HULL_POLY, userData)
        self.__hull.color = color
        # 记录添加伤害来源
        self.damage_source = None
        #self.__hull.userData = userData
        # 安装轮子
        self.__wheels = []
        for wx, wy in WHEEL_POS:
            front_k = 1.0
            w = self._create_dynamic_body(
                init_x+wx*SIZE, init_y+wy*SIZE, WHEEL_POLY, userData)
            rjd = revoluteJointDef(
                bodyA=self.__hull,
                bodyB=w,
                localAnchorA=(wx*SIZE, wy*SIZE),
                localAnchorB=(0, 0),
                enableMotor=False,
                enableLimit=True,
                lowerAngle=-0.0,
                upperAngle=+0.0,
            )
            w.joint = self.__world.CreateJoint(rjd)
            w.color = WHEEL_COLOR
            #w.userData = userData
            self.__wheels.append(w)

            # INkY added, used for punishment
            self.buff_type = 0
        # 安装枪管
        self.__gun = self._create_dynamic_body(init_x, init_y, GUN_POLY, userData, density=1e-4)
        self.gun_joint = self.__world.CreateJoint(revoluteJointDef(
            bodyA=self.__hull,
            bodyB=self.__gun,
            localAnchorA=(0, 0),
            localAnchorB=(0, 0),
            enableMotor=True,
            enableLimit=True,
            maxMotorTorque=180*900*SIZE*SIZE,
            motorSpeed=0.0,
            lowerAngle=-math.pi/1,
            upperAngle=+math.pi/1,
        ))
        self.__gun.color = COLOR_BLACK
        #self.__gun.userData = userData

        self.__hull.angle = init_angle
        self.__gun.angle = init_angle
        self.__drawlist = self.__wheels + [self.__hull, self.__gun]
        self.group = ["red", "blue"][robot_id%2]
        self.robot_id = robot_id
        self.__health = 2000.0
        self.buff_left_time = 0
        self.command = {"ahead": 0, "rotate": 0, "transverse": 0}

        self.__n_projectile = 200  # 40
        self.supply_opportunity_left = 2
        # TODO 按比例控制小车运动速度
        self.scale = 0.5

    def refresh_supply_oppotunity(self):
        self.supply_opportunity_left = 2

    def supply(self):
        self.__n_projectile += BULLETS_ADDED_ONE_TIME

    def if_supply_available(self):
        return self.supply_opportunity_left > 0

    def use_supply_oppotunity(self):
        self.supply_opportunity_left -= 1

    def get_pos(self):
        return self.__hull.position

    def get_angle(self):
        return self.__hull.angle

    def get_velocity(self):
        return self.__hull.linearVelocity

    def get_angular(self):
        return self.__hull.angularVelocity

    def get_gun_angle_pos(self):
        return self.__gun.angle, self.__gun.position

    def get_angle_pos(self):
        return self.__hull.angle, self.__hull.position

    def get_world_vector(self):
        return self.__hull.GetWorldVector

    def lose_health(self, damage):
        if self.__health > 0:
            self.__health = max(0, self.__health - damage)

    def get_health(self):
        return self.__health

    def get_left_projectile(self):
        return self.__n_projectile

    def if_left_projectile(self):
        return self.__n_projectile > 0

    def shoot(self):
        if self.buff_type != 3 and self.buff_left_time <= 0:
            self.__n_projectile -= 1

    def rotate_gimbal(self, angular_vel):
        if self.buff_type != 3 and self.buff_left_time <= 0:
            self.gun_joint.motorSpeed = angular_vel

    def set_gimbal(self, angle):
        if self.buff_type != 3 and self.buff_left_time <= 0:
            self.__gun.angle = angle

    def move_ahead_back(self, gas):
        if self.buff_type != 2 and self.buff_left_time <= 0:
            self.command["ahead"] = gas

    def move_left_right(self, transverse):
        if self.buff_type != 2 and self.buff_left_time <= 0:
            self.command["transverse"] = transverse

    def turn_left_right(self, r):
        if self.buff_type != 2 and self.buff_left_time <= 0:
            self.command["rotate"] = r

    def step(self, dt):
        # 小车运动设置, 速度角速度等
        forw = self.__hull.GetWorldVector((1, 0))  # forward
        side = self.__hull.GetWorldVector((0, -1))
        v = self.__hull.linearVelocity
        vf = forw[0]*v[0] + forw[1]*v[1]  # forward speed???
        vs = side[0]*v[0] + side[1]*v[1]  # side speed
        #f_a = (-vf + self.command["ahead"]) * 5
        #p_a = (-vs + self.command["transverse"]) * 5

        #f_force = self.hull.mass * f_a
        #p_force = self.hull.mass * p_a
        f_force = self.command["ahead"] * self.scale
        p_force = self.command["transverse"] * self.scale

        # self.hull.ApplyForceToCenter((
        #(p_force)*side[0] + f_force*forw[0],
        # (p_force)*side[1] + f_force*forw[1]), True)
        self.__hull.linearVelocity = (
            (p_force)*side[0] + f_force*forw[0],
            (p_force)*side[1] + f_force*forw[1])

        # omega = - self.hull.angularVelocity * \
        #0.5 + self.command["rotate"] * 2
        #torque = self.hull.mass * omega
        #self.hull.ApplyTorque(torque, True)
        self.__hull.angularVelocity = self.command["rotate"] * 3 * self.scale

    def draw(self, viewer):
        for obj in self.__drawlist:
            for f in obj.fixtures:
                trans = f.body.transform
                path = [trans*v for v in f.shape.vertices]
                viewer.draw_polygon(path, color=obj.color)

    def destroy(self):
        self.__world.DestroyBody(self.__hull)
        self.__hull = None
        for w in self.__wheels:
            self.__world.DestroyBody(w)
        self.__wheels = []
        if self.__gun:
            self.__world.DestroyBody(self.__gun)
        self.__gun = None
