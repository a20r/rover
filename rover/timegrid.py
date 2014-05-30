
import time
import math
import numpy as np


class TimeGrid(object):

    def __init__(self, width, height):
        self.scaling_factor = 8
        self.width = width // self.scaling_factor
        self.height = height // self.scaling_factor
        self.grid = np.zeros((self.height, self.width))
        self.init_grid(self.grid)

    def init_grid(self, grid):
        current_time = time.time()
        for h in xrange(self.height):
            for w in xrange(self.width):
                self.grid[h, w] = current_time

    def update_grid(self, quad):
        x = quad.x // self.scaling_factor
        y = quad.y // self.scaling_factor
        radius = int(quad.get_sensor_radius()) // self.scaling_factor

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
                    self.raw_set_item(x_i, y_i, current_time)

    def raw_set_item(self, x, y, value):
        self.grid[y, x] = value

    def __getitem__(self, index):
        return self.grid[
            index[1] // self.scaling_factor,
            index[0] // self.scaling_factor
        ]

    def __setitem__(self, index, value):
        self.grid[
            index[1] // self.scaling_factor,
            index[0] // self.scaling_factor
        ] = value
