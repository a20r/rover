

class CostGrid(object):

    def __init__(self, time_grid, risk_grid):
        self.time_grid = time_grid
        self.risk_grid = risk_grid

    def __getitem__(self, index):
        x, y = index[0], index[1]
        return 100 * self.risk_grid[x, y] + self.time_grid[x, y]
