
import point
import math


class Evader(object):

    def __init__(self, x, y, g_x, g_y, problem, risk_grid):
        self.x = x
        self.y = y
        self.goal_point = point.Point(g_x, g_y)
        self.problem = problem
        self.risk_grid = risk_grid
        self.sensing_radius = 100
        self.num_samples = 100

    def constrain(self, x, y):
        ret_x = x
        ret_y = y

        b_x = False
        b_y = False

        if x < 0:
            ret_x = 0
            b_x = True
        elif x >= self.problem.width:
            ret_x = self.problem.width - 1
            b_x = True

        if y < 0:
            ret_y = 0
            b_y = True
        elif ret_y >= self.problem.height:
            ret_y = self.problem.height - 1
            b_y = True

        return ret_x, ret_y, b_x or b_y

    def get_potential(self, x, y, quads):
        sensor_quality = 0.0
        for quad in quads:
            if quad.within_range(point.Point(x, y)):
                sensor_quality += pow(
                    self.problem.min_height / float(quad.get_z()), 2
                )
        return sensor_quality

    def move_2d(self, unit_heading):
        h_x = int(unit_heading.get_x() * 1.5 * self.problem.step_size)
        h_y = int(unit_heading.get_y() * 1.5 * self.problem.step_size)
        self.x = self.x + h_x
        self.y = self.y + h_y

        return self.x, self.y

    def sample_random_point(self):
        r_point = point.get_random_point(
            self.problem.width, self.problem.height
        )

        if r_point.dist_to(self) < self.sensing_radius:
            x_s = r_point.x
            y_s = r_point.y
        else:
            angle = math.atan2(r_point.y - self.y, r_point.x - self.x)
            x_s = int(self.x + self.sensing_radius * math.cos(angle))
            y_s = int(self.y + self.sensing_radius * math.sin(angle))

        return x_s, y_s

    def step(self, quads):
        min_potential = None
        min_x = min_y = None

        for _ in xrange(self.num_samples):

            x_s, y_s = self.sample_random_point()
            x_s, y_s, out = self.constrain(x_s, y_s)

            if out:
                continue

            potential = self.get_potential(x_s, y_s, quads)

            if min_potential is None or potential < min_potential:
                min_potential = potential
                min_x = x_s
                min_y = y_s

        unit_heading = point.Point(
            min_x - self.x, min_y - self.y
        ).to_unit_vector()

        self.move_2d(unit_heading)

        return unit_heading

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y
