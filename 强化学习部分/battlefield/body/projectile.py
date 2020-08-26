import numpy as np
import math
import Box2D
from Box2D.b2 import (fixtureDef, polygonShape, )
from utils import UserData, COLOR_BLACK

SIZE = 0.001
BULLET_BOX = (40*SIZE, 10*SIZE)
RADIUS_START = 0.9


class Projectile:
    def __init__(self, world):
        self.__world = world
        self.__projectile = {}
        self.__ctr = 1
        self.__fixture_bullet = [fixtureDef(
            shape=polygonShape(box=BULLET_BOX),
            categoryBits=0x02,
            maskBits=0xFD,
            density=1e-6
        )]
        # self._id = id

        # 参数添加一个机器人id, 用于设置子弹的来源, 添加子弹来源的标记
    def shoot(self, init_angle, init_pos, robort_id):
        angle = init_angle
        x, y = init_pos
        # TODO 修改了子弹的起始位置, 等待调整
        x += math.cos(angle) * RADIUS_START * 0.5
        y += math.sin(angle) * RADIUS_START * 0.5
        userData = UserData("bullet", self.__ctr, robort_id)
        self.__fixture_bullet[0].userData = userData
        projectile = self.__world.CreateDynamicBody(
            position=(x, y),
            angle=angle,
            fixtures=self.__fixture_bullet,
        )
        #bullet.bullet = True
        projectile.color = COLOR_BLACK
        #bullet.userData = userData
        projectile.linearVelocity = (math.cos(angle)*5, math.sin(angle)*5)
        self.__projectile[self.__ctr] = projectile
        self.__ctr += 1

    def draw(self, viewer):
        for obj in self.__projectile.values():
            for f in obj.fixtures:
                trans = f.body.transform
                path = [trans*v for v in f.shape.vertices]
                viewer.draw_polygon(path, color=obj.color)

    def destroyById(self, bullet_id):
        body = self.__projectile.pop(bullet_id, None)
        if body is not None:
            self.__world.DestroyBody(body)

    def destroy(self):
        for bullet in self.__projectile.values():
            self.__world.DestroyBody(bullet)
        self.__projectile = {}
