from collections import namedtuple

ID_R1 = 0
ID_B1 = 1
ID_R2 = 2
ID_B2 = 3

COLOR_BLACK = (0.0, 0.0, 0.0)
COLOR_RED = (0.8, 0.0, 0.0)
COLOR_BLUE = (0.0, 0.0, 0.8)
COLOR_WHITE = (1.0, 1.0, 1.0)
COLOR_LIGHT_RED = (0.9, 0.4, 0.4, 1.0)
COLOR_LIGHT_BLUE = (0.4, 0.4, 9.0, 1.0)

GROUP_RED = "red"
GROUP_BLUE = "blue"

UserData = namedtuple("UserData", ["type", "id", 'robort_id'])

class RobotState():
    def __init__(self, pos):
        self.health = 2000
        self.pos = pos
        self.angle = 0
        self.velocity = [0, 0]
        self.angular = 0
        self.detect = False
        self.scan = []

class Action():
    def __init__(self):
        self.v_t = 0.0
        self.v_n = 0.0
        self.angular = 0.0
        self.shoot = 0.0
        self.supply = 0.0
