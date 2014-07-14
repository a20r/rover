
import math
import random
import stats


class MotionBlur(object):

    def __init__(self, problem):
        self.practical = problem.practical
        self.problem = problem
        self.amb = stats.AverageMotionBlur()

    def get_blur(self, quad):
        if self.practical:
            blur = self.get_blur_practical(quad)
        else:
            blur = self.get_blur_simulation(quad)

        self.amb.update(blur)
        return blur

    def get_average_blur(self):
        return self.amb.get_average()

    def get_blur_practical(self, quad):
        return 0

    def get_blur_simulation(self, quad):
        x = -quad.get_speed() + self.problem.step_size / 2
        return 1 / (1 + math.exp(x)) + random.gauss(0, 0.01)
