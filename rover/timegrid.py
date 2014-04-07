
import time
import point
import math

class TimeGrid(object):

    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.grid = list()
        self.init_grid(self.grid)
        self.num_samples = 100


    def init_grid(self, grid):
        current_time = time.time()
        for _ in xrange(self.height):
            grid.append([current_time for _ in xrange(self.width)])


    def update_grid(self, x, y, radius):
        # approximate with square for now
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
                self[x_i, y_i] = current_time


    def sample_new_point(self, quad):
        self.update_grid(
            quad.get_x(), quad.get_y(), quad.get_sensor_radius()
        )

        # WORK ON THIS SHIT MOFO


    def __getitem__(self, index):
        return self.grid[index[0]][index[1]]


    def __setitem__(self, index, value):
        self.grid[index[0]][index[1]] = value

