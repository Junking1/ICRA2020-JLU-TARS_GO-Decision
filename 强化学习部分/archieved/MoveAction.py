import sys
import math
import time
import numpy as np

import sys
import Box2D

sys.path.append("..")
from SupportAlgorithm.Astar import astar, pathprocess
from util.Grid import map2grid, view_path, grid2world, world2grid

WIDTH = 200
HEIGHT = 120
MINBIAS = 0.6

MAXVELOCITY =1000
ACC = 1


class MoveAction:
    def __init__(self, target, pos, vel, ang):
        target = Box2D.b2Vec2(target[0], target[1])
        pos = Box2D.b2Vec2(pos[0], pos[1])
        vel = Box2D.b2Vec2(vel[0], vel[1])
        self.index = 0
        DUNGEON = map2grid(WIDTH, HEIGHT)
        self.target_grid = world2grid(target)
        self.selfpos_gird = world2grid(pos)
        tic = time.time()
        #print(DUNGEON)
        path = astar(DUNGEON, WIDTH, HEIGHT, self.selfpos_gird, 0, self.target_grid)
        #print(path)
        #print("Astar: {}".format(time.time()-tic))
        self.path = pathprocess(path)
        self.tonext = 1000
        self.velocity = vel

    def MoveTo(self, pos, vel, ang, action):
        pos = Box2D.b2Vec2(pos[0], pos[1])
        vel = Box2D.b2Vec2(vel[0], vel[1])
        selfpos = pos
        self.velocity = vel
        if self.index  < self.path.__len__():
            nexttarget = grid2world(self.path[self.index])
            self.tonext = self.dist(selfpos, nexttarget)
            #print(self.tonext)
            if self.tonext < 1.414 * MINBIAS:
                self.index += 1
                action[0] = +0.0
                action[2] = +0.0
            else:
                action = self.MoveSubTo(nexttarget, selfpos, self.velocity, ang, action)

        return action

    def MoveSubTo(self, target, selfpos, velocity, ang, action):
        distance = np.sqrt(np.square(target.x - selfpos.x) + np.square(target.y - selfpos.y))
        delta = target - selfpos
        ang = np.pi/2 + ang
        vx = velocity.x * np.cos(ang) - velocity.y * np.sin(ang)
        vy = velocity.x * np.sin(ang) + velocity.y * np.cos(ang)
        #print('vy', vy)
        dx = delta.x
        dy = delta.y
        #print(delta)
        decelerate = 0.5 * np.square(MAXVELOCITY) / ACC
        if distance < 1.414 * MINBIAS:
            action[0] = +0.0
            action[2] = +0.0
        else:
            if dx > MINBIAS:
                action[0] = +ACC
            elif dx < -MINBIAS:
                action[0] = -ACC
            else:
                action[0] = 0
            if dy > MINBIAS:
                action[2] = -ACC
            elif dy < -MINBIAS:
                action[2] = +ACC
            else:
                action[2] = 0
        if vx > MAXVELOCITY:
             action[0] = 0
        if vy > MAXVELOCITY:
             action[2] = 0
        ax = action[0]
        ay = action[2]

        action[0] = ax * np.cos(ang) - ay * np.sin(ang)
        action[2] = ax * np.sin(ang) + ay * np.cos(ang)
        #print('ay', action[2])
        return action

    def dist(self, selfpos, target):
        distance = np.sqrt(np.square(target.x - selfpos.x) + np.square(target.y - selfpos.y))
        return distance


'''
The cood of Box2Dworld:
 /\ y     
 ||
 ||
 ||
 ||
 ||
 ====================>
                   x


 The cood of Grid map:
 ====================>x
||
||
||
||
||
\/ y
So it is up-down flipped

 #####################################################################################              
 #####################################################################################              
########################################################################################            
########################################################################################            
########################################################################################            
########################################################################################            
########################################################################################            
######                            #######                                        #######            
######                            #######                                        #######            
######                            #######                                        #######            
######                            #######                                        #######            
######                            #######                                        #######            
######                            #######                  ###############       #######            
######                            #######                  ###############       #######            
######                            #######                  ###############       #######            
######                            #######                  ###############       #######            
######         ########           #######                  ###############       #######            
######         ########                                    ###############       #######            
######         ########                                    ###############       #######            
######         ########                                    ###############       #######            
######         ########                                                          #######            
######       : ########                                                          #######            
######       : ########                                                          #######            
######       : ########                                                          #######            
######       : ########                                                          #######            
######       : ########                                                          #######            
######       : ########             ###############                              #######            
######       : ########             ###############                              #######            
######       : ########             ###############              #######         #######            
######       : ########             ###############              #######         #######            
######       : ########             ###############              #######         #######            
######        :                     ###############              #######         #######            
######         :                    ###############              #######         #######            
######          :                                                #######         #######            
######           :                                               #######         #######            
######            :                                              #######         #######            
######             :                                             #######         #######            
######              :                                            #######         #######            
######               :                                           #######         #######            
######                :::::                                      #######         #######            
######       ##############:                                     #######         #######            
######       ###############:::::::::::::::::::::::::            #######         #######            
######       ###############                  #######:           #######         #######            
######       ###############                  ########:                          #######            
######       ###############                  ######## :                         #######            
######       ###############                  ########  :                        #######            
######       ###############                  ########   :                       #######            
######                                        ########    :                      #######            
######                                        ########     :                     #######            
######                                        ########     :                     #######            
######                                        ########     :                     #######            
######                                        ########                           #######            
########################################################################################            
########################################################################################            
########################################################################################            
########################################################################################            
########################################################################################            
 #####################################################################################              
 #####################################################################################     
'''
