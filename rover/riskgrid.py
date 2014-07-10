
import numpy as np


class RiskGrid(object):

    def __init__(self, problem):
        self.problem = problem
        self.space = np.loadtxt(problem.scene)

    def get_risk(self, x, y):
        return self.space[y, x]

    def __getitem__(self, index):
        return self.get_risk(index[0], index[1])
