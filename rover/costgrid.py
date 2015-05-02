
import math
import random


class CostGrid(object):

    RISK_CONST = 20
    RISK_STD = 0.05

    def __init__(self, time_grid, risk_grid):
        self.time_grid = time_grid
        self.risk_grid = risk_grid

    def get(self, x, y, i):
        r_val = self.risk_grid[x, y] + random.uniform(0, self.RISK_STD)
        try:
            u_val = math.exp(i - self.time_grid[x, y])
        except OverflowError:
            u_val = 0

        if r_val > 1:
            r_val = 1
        elif r_val < 0:
            r_val = 0

        r_val = self.RISK_CONST * r_val

        return r_val - u_val

    def __getitem__(self, index):
        x, y = index[0], index[1]
        return 100 * self.risk_grid[x, y] + self.time_grid[x, y]
        # return self.time_grid[x, y]
