import math
import sys
import time

import matplotlib.pyplot as plt
import numpy as np
from extremitypathfinder.extremitypathfinder import \
    PolygonEnvironment as Environment
from matplotlib.patches import Polygon

sys.path.append(".")
from SupportAlgorithm.DynamicWindow import DynamicWindow

BORDER_POS = [(1.525, 1.9), (6.475, 3.1),
              (1.7, 3.875), (4, 2.5), (6.3, 1.125)]
BORDER_BOX = [(0.125, 0.5), (0.125, 0.5),
              (0.5, 0.125), (0.5, 0.125), (0.5, 0.125)]  # Half of the weight and height

ROBOT_SIZE = 0.01

POLYGON_SETTINGS = {
    'edgecolor': 'black',
    'fill': False,
    'linewidth': 1.0,
}

SHOW_PLOTS = True
# parameter
MAX_T = 100.0  # maximum time to the goal [s]
MIN_T = 5.0  # minimum time to the goal[s]

show_animation = True

max_accel = 1.0  # max accel [m/ss]
max_jerk = 0.5  # max jerk [m/sss]

def mark_points(vertex_iter, **kwargs):
    xs = []
    ys = []
    if type(vertex_iter) == set:
        for v in vertex_iter:
            xs.append(v.coordinates[0])
            ys.append(v.coordinates[1])
    elif type(vertex_iter[0]) == tuple:
        for x, y in vertex_iter:
            xs.append(x)
            ys.append(y)
    else:
        for v in vertex_iter:
            xs.append(v.coordinates[0])
            ys.append(v.coordinates[1])

    plt.scatter(xs, ys, **kwargs)


def draw_edge(v1, v2, c, alpha, **kwargs):
    if type(v1) == tuple:
        x1, y1 = v1
        x2, y2 = v2
    else:
        x1, y1 = v1.coordinates
        x2, y2 = v2.coordinates
    plt.plot([x1, x2], [y1, y2], color=c, alpha=alpha, **kwargs)


def draw_polygon(ax, coords, **kwargs):
    kwargs.update(POLYGON_SETTINGS)
    polygon = Polygon(coords, **kwargs)
    ax.add_patch(polygon)


def draw_boundaries(map, ax):
    # TODO outside light grey
    # TODO fill holes light grey
    draw_polygon(ax, map.boundary_polygon.coordinates)
    for h in map.holes:
        draw_polygon(ax, h.coordinates, facecolor='grey', fill=True)

    mark_points(map.all_vertices, c='black', s=15)
    mark_points(map.all_extremities, c='red', s=50)


def draw_internal_graph(map, ax):
    for start, all_goals in map.graph.get_neighbours():
        for goal in all_goals:
            draw_edge(start, goal, c='red', alpha=0.2, linewidth=2)


def set_limits(map, ax):
    ax.set_xlim((min(map.boundary_polygon.coordinates[:, 0]) - 1, max(
        map.boundary_polygon.coordinates[:, 0]) + 1))
    ax.set_ylim((min(map.boundary_polygon.coordinates[:, 1]) - 1, max(
        map.boundary_polygon.coordinates[:, 1]) + 1))


def draw_path(vertex_path):
    # start, path and goal in green
    if vertex_path:
        mark_points(vertex_path, c='g', alpha=0.9, s=50)
        mark_points([vertex_path[0], vertex_path[-1]], c='g', s=100)
        v1 = vertex_path[0]
        for v2 in vertex_path[1:]:
            draw_edge(v1, v2, c='g', alpha=1.0)
            v1 = v2


def draw_prepared_map(map):
    fig, ax = plt.subplots()

    draw_boundaries(map, ax)
    #draw_internal_graph(map, ax)
    set_limits(map, ax)
    if SHOW_PLOTS:
        plt.show()


class GlobalLocalPlanner():
    def __init__(self):
        self.environment = Environment()

        # counter clockwise vertex numbering!
        boundary_coordinates = [
            (0+ROBOT_SIZE, 0+ROBOT_SIZE),
            (3.25-ROBOT_SIZE, 0+ROBOT_SIZE), (3.25-ROBOT_SIZE, 1.0+ROBOT_SIZE),
            (3.5 + ROBOT_SIZE, 1.0+ROBOT_SIZE), (3.5+ROBOT_SIZE, 0+ROBOT_SIZE),
            (8.0-ROBOT_SIZE, 0+ROBOT_SIZE),
            (8.0-ROBOT_SIZE, 5.0-ROBOT_SIZE),
            (8.0-3.25+ROBOT_SIZE, 5.0-ROBOT_SIZE),
            (8.0 - 3.25+ROBOT_SIZE, 4.0-ROBOT_SIZE),
            (8.0 - 3.5-ROBOT_SIZE, 4.0-ROBOT_SIZE),
            (8.0 - 3.5-ROBOT_SIZE, 5.0-ROBOT_SIZE),
            (0+ROBOT_SIZE, 5.0-ROBOT_SIZE)
        ]

        # clockwise numbering!
        list_of_holes = []
        for (x, y), (w, h) in zip(BORDER_POS[:], BORDER_BOX[:]):
            #x, y, w, h = x*10, y*10, w*10, h*10
            list_of_holes.append([
                ((x-w-ROBOT_SIZE), (y-h-ROBOT_SIZE)),
                ((x-w-ROBOT_SIZE), (y+h+ROBOT_SIZE)),
                ((x+w+ROBOT_SIZE), (y+h+ROBOT_SIZE)),
                ((x+w+ROBOT_SIZE), (y-h-ROBOT_SIZE)),
            ])

        self.environment.store(boundary_coordinates,
                               list_of_holes, validate=False)
        self.environment.prepare()
        self.done = True
        self.dynamic = DynamicWindow()

    def plot(self):
        draw_prepared_map(self.environment)

    def findPath(self, start, goal):
        path, length = self.environment.find_shortest_path(start, goal)
        return path

    def setGoal(self, start, goal, angle=0):
        #print(start, goal)
        self.path = self.findPath(start, goal)
        self.index = 1
        self.angle_path = []
        if len(self.path) > 1:
            self.next_target = self.path[1]
            #print("new target: {}".format(self.next_target))
            for i, j in zip(self.path[:-1], self.path[1:]):
                self.angle_path.append(math.atan2(j[1]-i[1], j[0]-i[0]))
            self.angle_path.append(angle)
            self.next_angle = self.angle_path[0]
            self.done = False
        else:
            self.done = True

    def moveTo(self, pos, vel, angle, angular, action):
        if self.done:
            return action
        if self.distance(pos, self.next_target) < 0.4:
            self.index += 1
            if self.index < len(self.path):
                self.next_target = self.path[self.index]
                self.next_angle = self.angle_path[self.index-1]
                print("new target: {}".format(self.next_target))
            else:
                self.done = True
                action[0], action[1], action[2] = 0, 0, 0
                return action
        #tic = time.time()
        tmp_angle = math.atan2(self.next_target[1]-pos[1], self.next_target[0]-pos[0])
        tmp_length = (self.next_target[0]**2 + self.next_target[1]**2)
        tmp_target = np.array([math.cos(tmp_angle), math.sin(tmp_angle)])*min(5, tmp_length)
        action = self.dynamic.moveTo(action, pos, vel, angle, angular, tmp_target, self.next_angle)
        #print(time.time() - tic)
        '''
        u = np.array([
            self.next_target[0]-pos[0],
            self.next_target[1]-pos[1]])
        target_angle = math.atan2(u[1], u[0])
        u = u.reshape([2, 1])
        mat_angle = np.array([
            [math.cos(angle), math.sin(angle)],
            [math.sin(angle), -math.cos(angle)]])
        v = np.matmul(np.linalg.inv(mat_angle), u) / 10
        #print(u, v, angle)
        v /= v.max()
        action[0] = v[0][0] 
        action[1] = target_angle - angle
        action[2] = v[1][0]
        '''
        return action

    def distance(self, p1, p2):
        return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**0.5


if __name__ == "__main__":
    move = GlobalLocalPlanner()
    move.plot()
    start, goal = (0.5, 0.5), (1.5, 4.5)
    print(move.findPath(start, goal))
