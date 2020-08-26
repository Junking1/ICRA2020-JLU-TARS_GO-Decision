from battlefield.body.robot import Robot
from utils import *
SUPPLYAREABOX_RED = (3.5, 4.0, 1.0, 1.0) #(x, y, w, h)
SUPPLYAREABOX_BLUE = (3.5, 0, 1.0, 1.0)

class AreaSupply(object):
    def __init__(self):
        self.supply_area_red = SUPPLYAREABOX_RED
        self.supply_area_blue = SUPPLYAREABOX_BLUE

    def render(self, gl):
        self._render(gl, self.supply_area_red, COLOR_LIGHT_RED)
        self._render(gl, self.supply_area_blue, COLOR_LIGHT_BLUE)

    def _render(self, gl, box, color):
        gl.glBegin(gl.GL_QUADS)
        gl.glColor4f(color[0], color[1], color[2], color[3])
        x, y, w, h = box
        gl.glVertex3f(x, y, 0)
        gl.glVertex3f(x + w, y, 0)
        gl.glVertex3f(x + w, y + h, 0)
        gl.glVertex3f(x, y + h, 0)
        gl.glEnd()

    def if_in_area(self, robot: Robot):
        if robot.group == "red":
            supply_area = self.supply_area_red
        elif robot.group == "blue":
            supply_area = self.supply_area_blue
        else:
            print("Wrong Input Object in supply area!")
            return False

        pos = robot.get_pos()
        x, y = pos.x, pos.y
        bx, by, w, h = supply_area
        return (bx <= x <= bx + w) and (by <= y <= by + h)
