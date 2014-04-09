
import random
import point
import math

class MonteCarloArea(object):

    def __init__(self, problem, num_sample_points):
        self.num_sample_points = num_sample_points
        self.problem = problem
        self.total_efficiency = 0.0
        self.number_of_updates = 0
        self.moving_average = 0.0
        self.learning_rate = 0.9


    def update_average_efficiency(self, quads):
        num_in = 0
        total = 0
        for _ in xrange(self.num_sample_points):
            r_p = point.get_random_point(
                self.problem.width, self.problem.height
            )

            for quad in quads:
                if r_p.dist_to(quad) <= quad.get_sensor_radius():
                    num_in += 1
                    break

            total += 1

        self.total_efficiency += float(num_in) / total
        self.number_of_updates += 1

        self.moving_average = (1 - self.learning_rate) * self.moving_average +\
                self.learning_rate * float(num_in) / total

        return self


    def get_moving_average_efficiency(self):
        return self.moving_average


    def get_average_efficiency(self):
        return self.total_efficiency / float(self.number_of_updates)


class SensorQualityAverage(object):

    def __init__(self, problem):
        self.moving_average = 0.0
        self.learning_rate = 0.9
        self.problem = problem


    def get_sensor_quality(self, quad):
        return math.pow(self.problem.min_height / float(quad.get_z()), 2)


    def update_average_sq(self, quads):
        sub_total = 0.0
        for quad in quads:
            sub_total += self.get_sensor_quality(quad)

        self.moving_average = (1 - self.learning_rate) *\
                self.moving_average + self.learning_rate *\
                sub_total / len(quads)

        return self


    def get_moving_average(self):
        return self.moving_average


class RiskAverage(object):

    def __init__(self, problem, risk_grid):
        self.problem = problem
        self.risk_grid = risk_grid
        self.moving_average = 0.0
        self.learning_rate = 0.9


    def update_average_risk(self, quads):
        sub_total = 0.0
        for quad in quads:
            sub_total += self.risk_grid.get_risk_3d(quad.x, quad.y, quad.z)

        self.moving_average = (1 - self.learning_rate) *\
                self.moving_average + self.learning_rate *\
                sub_total / len(quads)

        return self


    def get_moving_average(self):
        return self.moving_average
