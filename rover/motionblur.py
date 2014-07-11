
import math
import random


class MotionBlur(object):

    def __init__(self, problem, practical):
        self.practical = practical
        self.problem = problem

    def get_blur(self, quad):
        if self.practical:
            return self.get_blur_practical(quad)
        else:
            return self.get_blur_simulation(quad)

    def get_blur_practical(self, quad):
        return 0

    def get_blur_simulation(self, quad):
        x = -quad.get_speed() + self.problem.step_size / 2
        return 1 / (1 + math.exp(x)) + random.gauss(0, 0.01)
