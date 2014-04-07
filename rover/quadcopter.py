
import math

class Quadcopter(object):

    def __init__(self, x, y, z, viewing_angle):
        self.x = x
        self.y = y
        self.z = z

        # in degrees
        self.viewing_angle = viewing_angle

    def get_x(self):
        return self.x


    def get_y(self):
        return self.y


    def get_z(self):
        return self.z


    def get_viewing_angle(self):
        return self.viewing_angle


    def set_position(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        return self


    def get_sensor_radius(self):
        return self.z * math.tan(math.radians(self.viewing_angle))
