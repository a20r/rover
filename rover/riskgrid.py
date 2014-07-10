
import numpy as np


class RiskGrid(object):

    def __init__(self, problem):
        self.problem = problem
        self.space = np.loadtxt(problem.scene)

    def get_risk(self, x, y):
        if x >= self.problem.width:
            x = self.problem.width - 1
        if y >= self.problem.height:
            y = self.problem.height - 1
        if x < 0:
            x = 0
        if y < 0:
            y = 0

        return self.space[y, x]

    def __getitem__(self, index):
        return self.get_risk(index[0], index[1])
