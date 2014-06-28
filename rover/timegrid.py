
import math
import numpy as np


class TimeGrid(object):

    def __init__(self, width, height, problem):
        self.scaling_factor = 4
        self.width = width / self.scaling_factor
        self.height = height / self.scaling_factor
        self.grid = np.zeros((self.height, self.width))
        self.init_grid(self.grid)

    def init_grid(self, grid):
        for h in xrange(self.height):
            for w in xrange(self.width):
                self.grid[h, w] = 1

    def update_grid(self, quad, ct):
        x = quad.x // self.scaling_factor
        y = quad.y // self.scaling_factor
        r_ma = int(quad.get_ellipse_major() // self.scaling_factor)
        r_mi = int(quad.get_ellipse_minor() // self.scaling_factor)
        h = (quad.get_ellipse_center_dist() + quad.x) // self.scaling_factor
        k = quad.y // self.scaling_factor

        top_left_x = int(h - r_ma)
        top_left_y = int(k - r_mi)
        bottom_right_x = int(h + r_ma)
        bottom_right_y = int(k + r_mi)

        for x_i in xrange(top_left_x, bottom_right_x + 1):
            for y_i in xrange(top_left_y, bottom_right_y + 1):

                el_eval_x = pow(x_i - h, 2) / float(pow(r_ma, 2))
                el_eval_y = pow(y_i - k, 2) / float(pow(r_mi, 2))
                beta = math.radians(quad.beta)

                if el_eval_x + el_eval_y <= 1:
                    x_o = x_i - x
                    y_o = y_i - y

                    new_x = x_o * math.cos(beta) - y_o * math.sin(beta) + x
                    new_y = y_o * math.cos(beta) + x_o * math.sin(beta) + y

                    self.raw_set_item(int(new_x), int(new_y), ct)
                    self.raw_set_item(
                        int(math.ceil(new_x)), int(math.ceil(new_y)), ct
                    )

    def raw_set_item(self, x, y, value):
        if x >= self.width:
            x = self.width - 1
        elif x < 0:
            x = 0

        if y >= self.height:
            y = self.height - 1
        elif y < 0:
            y = 0

        self.grid[y, x] = value

    def get_raw(self, x, y):
        return self.grid[y, x]

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
