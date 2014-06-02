
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
                old_x = r_p.x - quad.x
                old_y = r_p.y - quad.y

                beta_r = math.radians(quad.beta)
                X = old_x * math.cos(-beta_r) - old_y * math.sin(-beta_r)
                Y = old_y * math.cos(-beta_r) + old_x * math.sin(-beta_r)

                r_ma = quad.get_ellipse_major()
                r_mi = quad.get_ellipse_minor()
                h = quad.get_ellipse_center_dist()
                k = 0

                el_eval_x = pow(X - h, 2) / float(pow(r_ma, 2))
                el_eval_y = pow(Y - k, 2) / float(pow(r_mi, 2))


                if el_eval_x + el_eval_y <= 1:
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

    def __init__(self, planner):
        self.moving_average = 0.0
        self.learning_rate = 0.9
        self.planner = planner

    def update_average_sq(self, quads):
        sub_total = 0.0
        for quad in quads:
            sub_total += self.planner.sq(quad.x, quad.y, quad.z)

        self.moving_average = (1 - self.learning_rate) *\
            self.moving_average + self.learning_rate *\
            sub_total / len(quads)

        return self

    def get_moving_average(self):
        return self.moving_average


class RiskAverage(object):

    def __init__(self, planner):
        self.moving_average = 0.0
        self.learning_rate = 0.9
        self.planner = planner

    def update_average_risk(self, quads):
        sub_total = 0.0
        for quad in quads:
            sub_total += self.planner.risk(quad.x, quad.y, quad.z)

        self.moving_average = (1 - self.learning_rate) *\
            self.moving_average + self.learning_rate *\
            sub_total / len(quads)

        return self

    def get_moving_average(self):
        return self.moving_average
