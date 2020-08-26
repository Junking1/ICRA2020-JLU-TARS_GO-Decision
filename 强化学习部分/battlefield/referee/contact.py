from Box2D.b2 import contactListener


class ContactListener(contactListener):

    def __init__(self, env):
        contactListener.__init__(self)
        self.__env = env
        self.collision_bullet_robot = []
        self.collision_bullet_wall = []
        self.collision_robot_wall = []
        self.collision_robot_robot = []

    def BeginContact(self, contact):
        pass

    def EndContact(self, contact):
        pass

    def PreSolve(self, contact, oldManifold):
        u1 = contact.fixtureA.userData
        u2 = contact.fixtureB.userData
        if u1 is None or u2 is None:
            return
        #print(u1, u2)
        u1_type, u1_id = u1.type, u1.id
        u2_type, u2_id = u2.type, u2.id
        if u1_type == "bullet" and u2_type == "robot":
            self.collision_bullet_robot.append((u1, u2))
        elif u2_type == "bullet" and u1_type == "robot":
            self.collision_bullet_robot.append((u2, u1))
        elif u1_type == "bullet" and u2_type == "wall":
            self.collision_bullet_wall.append(u1)
        elif u2_type == "bullet" and u1_type == "wall":
            self.collision_bullet_wall.append(u2)
        elif u1_type == "robot" and u2_type == "wall":
            self.collision_robot_wall.append(u1)
        elif u2_type == "robot" and u1_type == "wall":
            self.collision_robot_wall.append(u2)
        elif u1_type == "robot" and u2_type == "robot" and u1_id != u2_id:
            self.collision_robot_robot.append(u1)
            self.collision_robot_robot.append(u2)
        
    def PostSolve(self, contact, impulse):
        pass

    def clean(self):
        self.collision_bullet_robot = []
        self.collision_bullet_wall = []
        self.collision_robot_wall = []
        self.collision_robot_robot = []
