
import time
import point
import math
import numpy as np

class TimeGrid(object):

    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.grid = np.zeros((480, 640))
        self.init_grid(self.grid)
        self.num_samples = 100
        self.radius_ext = 2


    def init_grid(self, grid):
        current_time = time.time()
        for h in xrange(self.height):
            for w in xrange(self.width):
                self.grid[h, w] = current_time


    def update_grid(self, quad):
        x = quad.x
        y = quad.y
        radius = int(quad.get_sensor_radius())

        top_left_x = x - radius if x - radius > 0 else 0
        top_left_y = y - radius if y - radius > 0 else 0

        bottom_right_x = bottom_right_y = None

        if x + radius >= self.width:
            bottom_right_x = self.width - 1
        else:
            bottom_right_x = x + radius

        if y + radius >= self.height:
            bottom_right_y = self.height - 1
        else:
            bottom_right_y = y + radius

        current_time = time.time()

        for x_i in xrange(top_left_x, bottom_right_x + 1):
            for y_i in xrange(top_left_y, bottom_right_y + 1):
                dist = math.sqrt(pow(x - x_i, 2) + pow(y - y_i, 2))
                if dist <= radius:
                    self[x_i, y_i] = current_time


    def constrain(self, x, y):
        ret_x = x
        ret_y = y

        b_x = False
        b_y = False

        if x < 0:
            ret_x = 0
            b_x = True
        elif x >= self.width:
            ret_x = self.width - 1
            b_x = True

        if y < 0:
            ret_y = 0
            b_y = True
        elif ret_y >= self.height:
            ret_y = self.height - 1
            b_y = True

        return ret_x, ret_y, b_x and b_y


    def get_new_direction(self, quad):
        """
        Returns the unit vector of the direction the quad should go
        """

        angle_step = 2 * math.pi / self.num_samples
        angle = float(0)
        sample_radius = quad.get_sensor_radius() + self.radius_ext
        min_time = None
        min_x_y = None

        for _ in xrange(self.num_samples):
            x = int(quad.get_x() + sample_radius * math.cos(angle))
            y = int(quad.get_y() + sample_radius * math.sin(angle))
            x, y, out = self.constrain(x, y)

            angle += angle_step
            if out:
                continue

            if min_time == None or min_time > self[x, y]:
                min_time = self[x, y]
                min_x_y = point.Point(x - quad.get_x(), y - quad.get_y())

        return min_x_y.to_unit_vector()


    def __getitem__(self, index):
        return self.grid[index[1], index[0]]


    def __setitem__(self, index, value):
        self.grid[index[1], index[0]] = value

