
import math
import point


class Quadcopter(object):

    def __init__(self, x, y, z, problem):
        self.x = x
        self.y = y
        self.z = z
        self.problem = problem

        # in degrees
        self.viewing_angle = problem.viewing_angle
        self.heading_x = 0
        self.heading_y = 0
        self.learning_rate = 0.6

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def get_z(self):
        return self.z

    def get_pos_2d(self):
        return (self.x, self.y)

    def within_range(self, point_like):
        return self.get_point_2d().dist_to(point_like) <\
            self.get_sensor_radius()

    def get_point_2d(self):
        return point.Point(self.x, self.y)

    def get_viewing_angle(self):
        return self.viewing_angle

    def set_position(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        return self

    def get_dict(self):
        return {
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "coverage": self.get_sensor_radius()
        }

    def set_z(self, z):
        self.z = z

    def move_2d(self, unit_heading):

        h_x = int(unit_heading.get_x() * self.problem.step_size)
        h_y = int(unit_heading.get_y() * self.problem.step_size)

        self.x = self.x + (1 - self.learning_rate) * self.heading_x +\
            self.learning_rate * h_x
        self.y = self.y + (1 - self.learning_rate) * self.heading_y +\
            self.learning_rate * h_y

        self.intify()

        self.heading_x = h_x
        self.heading_y = h_y
        return self.x, self.y

    def intify(self):
        self.x = int(self.x)
        self.y = int(self.y)
        self.z = int(self.z)

    def get_sensor_radius(self):
        return self.z * math.tan(math.radians(self.viewing_angle))
