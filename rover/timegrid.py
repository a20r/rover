
import time
import point
import math
import numpy as np

class TimeGrid(object):

    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.grid = np.zeros((height, width))
        self.init_grid(self.grid)
        self.num_samples = 100
        self.radius_ext = 2
        self.angle_range = math.pi / 8
        self.angle_step = 2 * math.pi / self.num_samples


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

        return ret_x, ret_y, b_x or b_y


    def get_sample_direction(self, angle, quad):

        sample_radius = quad.get_sensor_radius() + self.radius_ext
        inner_angle = angle - self.angle_range

        max_time = 0.0
        min_time = None
        time_dict = dict()
        total_time = 0.0
        counter = 0

        while inner_angle < angle + self.angle_range:
            x = int(quad.get_x() + sample_radius * math.cos(inner_angle))
            y = int(quad.get_y() + sample_radius * math.sin(inner_angle))
            x, y, out = self.constrain(x, y)

            inner_angle += self.angle_step

            if out:
                raise ValueError("Exists an unvaible point")

            time_dict[(x, y)] = self[x, y]
            total_time += self[x, y]
            counter += 1

            if self[x, y] > max_time:
                max_time = self[x, y]
            elif min_time == None or self[x, y] < min_time:
                min_time = self[x, y]

        avg_x = 0.0
        avg_y = 0.0
        for (x, y), t in time_dict.iteritems():
            weight = t / total_time
            avg_x += x * weight
            avg_y += y * weight

        return avg_x, avg_y, total_time / counter


    def get_new_direction(self, quad):
        """
        Returns the unit vector of the direction the quad should go
        """

        angle = float(0)
        min_time = None
        min_x_y = None

        while angle < 2 * math.pi:

            try:
                x, y, avg_time = self.get_sample_direction(angle, quad)
            except ValueError:
                continue
            finally:
                angle += self.angle_range

            if min_time == None or min_time > avg_time:
                min_time = avg_time
                min_x_y = point.Point(x - quad.get_x(), y - quad.get_y())

        return min_x_y.to_unit_vector()


    def __getitem__(self, index):
        return self.grid[index[1], index[0]]


    def __setitem__(self, index, value):
        self.grid[index[1], index[0]] = value

