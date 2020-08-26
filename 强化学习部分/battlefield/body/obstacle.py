import numpy as np
import math
import Box2D
from Box2D.b2 import (edgeShape, circleShape, fixtureDef, polygonShape,
                      revoluteJointDef, contactListener, shape, fixtureDef)
from utils import *
#import matplotlib.pyplot as plt
import os

SIZE = 1

#center_x, center_y
BORDER_POS = [(-0.1, 2.24), (4.04, -0.1), (4.04, 4.58), (8.18, 2.24)]
#width/2, height/2
BORDER_BOX = [(0.1, 2.44), (4.24, 0.1), (4.24, 0.1), (0.1, 2.44)]

# OBSTACLE_POS = [(1.525, 1.9), (3.375, 0.5), (6.475, 3.1),
#                 (4.625, 4.5), (1.7, 3.875), (4, 2.5), (6.3, 1.125)]
# OBSTACLE_BOX = [(0.125, 0.5), (0.125, 0.5), (0.125, 0.5), (0.125, 0.5),
                # (0.5, 0.125), (0.5, 0.125), (0.5, 0.125)]  # Half of the width and height
OBSTACLE_POS = [
    (0.5, 3.38), (1.9, 2.24), (1.6, 0.5), (4.04, 3.445),
    (4.04, 1.035), (6.480, 3.98), (6.18, 2.24), (7.58, 1.1)
]
OBSTACLE_BOX = [
    (0.5, 0.1), (0.4, 0.1), (0.1, 0.5), (0.5, 0.1),
    (0.5, 0.1), (0.1, 0.5), (0.4, 0.1), (0.5, 0.1)
]
DIAMOND_POS = [
    (4.040, 2.063), (4.040, 2.417), (4.04-0.177, 2.24), (4.04+0.177, 2.24)
]
class ICRALayout:
    def __init__(self, world=None):
        if(world):
            userData = UserData("wall", None, None)
            self.__world = world
            self.__borders = [world.CreateStaticBody(
                position=p,
                # shapes=polygonShape(box=b),
                #userData = "wall"
                fixtures=[
                    fixtureDef(
                        shape=polygonShape(box=b),
                        density=0.01, userData=userData, friction=1)
                ]
            ) for p, b in zip(BORDER_POS + OBSTACLE_POS, BORDER_BOX + OBSTACLE_BOX)]
            self.__borders.append(
                world.CreateStaticBody(
                    position=(4.04, 2.24),
                    fixtures=[
                        fixtureDef(
                            shape=polygonShape(box=(0.125, 0.125)),
                            density=0.01, userData=userData, friction=1
                        )
                    ],
                    angle=40
                )
            )
            # self.__borders[-1].SetTransform(self.__borders[-1].GetPosition(), 45)
            for i in range(len(self.__borders)):
                self.__borders[i].color = COLOR_WHITE
                self.__borders[i].userData = userData

            self.__drawlist = self.__borders

        #self.image_file = 'map.npy'
        #self.image = self.imread_map()

    def step(self, dt):
        pass

    def draw(self, viewer, draw_particles=True):
        for obj in self.__drawlist:
            for f in obj.fixtures:
                trans = f.body.transform
                path = [trans*v for v in f.shape.vertices]
                viewer.draw_polygon(path, color=obj.color)

    def destroy(self):
        for border in self.__borders:
            self.__world.DestroyBody(border)

    def imread_map(self):
        if(not os.path.isfile(self.image_file)):
            return self.imwrite_map(self.image_file)

    def imwrite_map(self, image_file):
        width = 8080
        height = 4480
        times = 1000
        margin = 3
        img = np.ones((height, width))
        pos = np.array(OBSTACLE_POS + BORDER_POS) * times
        box = np.array(OBSTACLE_BOX + BORDER_BOX) * times
        pos = pos.astype("int")
        box = box.astype("int")
        #obstacle_pos = [(int(x * times), int(y * times)) for x, y in OBSTACLE_POS]
        #obstacle_box = [(int(x * times), int(y * times)) for x, y in OBSTACLE_BOX]

        for p, b in zip(pos, box):
            left = p[0] - b[0] - margin
            right = p[0] + b[0] + margin
            top = p[1] + b[1] + margin
            bottom = p[1] - b[1] - margin
            left = max(0, left)
            right = min(80, right)
            bottom = max(0, bottom)
            top = min(50, top)
            #top = height - p[1] + b[1]
            #bottom = height - p[1] - b[1]

            img[bottom:top, left:right] = np.zeros(
                (top - bottom, right - left))
        up = (np.array(DIAMOND_POS[0])*times).astype('int')
        down = (np.array(DIAMOND_POS[1])*times).astype('int')
        print("******************", up)
        # h = up[1] - down[1]
        right = np.array(DIAMOND_POS[2]).astype('int')
        left = np.array(DIAMOND_POS[3]).astype('int')
        for m in range(0, up[1] - right[1] + 1):
            img[right[1]+m, right[0]+m:left[0]-m] = np.zeros((1,left[0]-right[0]-2*m))
            img[right[1]-m, right[0]+m:left[0]-m] = np.zeros((1,left[0]-right[0]-2*m))
        np.save(image_file, img)
        return img


if __name__ == '__main__':
    m = ICRALayout()
    #plt.ylim(0, 49)
    #plt.xlim(0, 79)
    #plt.imshow(m.image)
    #plt.show()
