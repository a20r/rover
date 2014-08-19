
import math
import point
import numpy as np
import time


class TimeGrid(object):

    def __init__(self, width, height, problem):
        self.problem = problem
        self.scaling_factor = 8
        self.width = width / self.scaling_factor
        self.height = height / self.scaling_factor
        self.grid = np.zeros((self.height, self.width))
        self.cov_grid = np.zeros((self.height, self.width))
        self.inter_grid = np.zeros((self.height, self.width))
        self.ass_risk_grid = np.zeros((self.height, self.width))
        self.sq_grid = np.zeros((self.height, self.width))
        self.copy_grid = np.zeros((self.height, self.width))
        self.init_grid(self.grid)
        self.cum_cov = 0
        self.inter_cum_cov = 0
        self.iter_between = 0
        self.time_between = 0
        self.time_start = time.time()
        self.iter_start = 2
        self.cov_to_reach = 0.9
        self.risk_grid = None

    def set_risk_grid(self, risk_grid):
        # SUCH A STUPID FUCKING METHOD
        self.risk_grid = risk_grid

    def init_grid(self, grid):
        for h in xrange(self.height):
            for w in xrange(self.width):
                self.grid[h, w] = 1

    def update_grid(self, quad, ct):
        x = quad.x // self.scaling_factor
        y = quad.y // self.scaling_factor
        r_ma = int(quad.get_ellipse_major() // self.scaling_factor) + 0.1
        r_mi = int(quad.get_ellipse_minor() // self.scaling_factor) + 0.1
        h = (quad.get_ellipse_center_dist() + quad.x) // self.scaling_factor
        k = quad.y // self.scaling_factor

        top_left_x = int(h - r_ma)
        top_left_y = int(k - r_mi)
        bottom_right_x = int(h + r_ma)
        bottom_right_y = int(k + r_mi)

        for x_i in xrange(top_left_x, bottom_right_x + 1):
            for y_i in xrange(top_left_y, bottom_right_y + 1):

                el_eval_x = pow(x_i - h, 2) / float(pow(r_ma, 2))
                el_eval_y = pow(y_i - k, 2) / float(pow(r_mi, 2))
                beta = math.radians(quad.beta)

                if el_eval_x + el_eval_y <= 1:
                    x_o = x_i - x
                    y_o = y_i - y

                    new_x = x_o * math.cos(beta) - y_o * math.sin(beta) + x
                    new_y = y_o * math.cos(beta) + x_o * math.sin(beta) + y

                    self.update_performance_grid(int(new_x), int(new_y), quad)
                    self.raw_set_item(int(new_x), int(new_y), ct)
                    self.raw_set_item(
                        int(math.ceil(new_x)), int(math.ceil(new_y)), ct
                    )

    def raw_set_item(self, x, y, value):
        if x >= self.width:
            x = self.width - 1
        elif x < 0:
            x = 0

        if y >= self.height:
            y = self.height - 1
        elif y < 0:
            y = 0

        self.update_stats(x, y, value)
        self.grid[y, x] = value

    def get_sq(self, distance):
        X = (distance - self.problem.sq_height) / self.problem.sq_std
        return math.exp(-0.5 * math.pow(X, 2))

    def update_performance_grid(self, x_scaled, y_scaled, quad):
        if not self.inside_workspace(x_scaled, y_scaled):
            return

        x = self.scaling_factor * x_scaled
        y = self.scaling_factor * y_scaled
        distance = quad.get_position().dist_to(point.Point(x, y, 0))

        sq = self.get_sq(distance)
        if sq > self.sq_grid[y_scaled, x_scaled]:
            self.sq_grid[y_scaled, x_scaled] = sq

        ass_risk = self.risk_grid[quad.x, quad.y]
        if ass_risk > self.ass_risk_grid[y_scaled, x_scaled]:
            self.ass_risk_grid[y_scaled, x_scaled] = ass_risk

    def inside_workspace(self, x_scaled, y_scaled):
        if not x_scaled > 0 or not x_scaled < self.width:
            return False

        if not y_scaled > 0 or not y_scaled < self.height:
            return False

        return True

    def update_stats(self, x, y, ct):
        self.update_cumulative_coverage(x, y)
        self.update_inter_cum_coverage(x, y, ct)

    def update_cumulative_coverage(self, x, y):
        if self.cov_grid[y, x] == 0:
            self.cov_grid[y, x] = 1
            self.cum_cov = self.cov_grid.sum() / (self.height * self.width)

    def update_inter_cum_coverage(self, x, y, ct):
        if self.inter_grid[y, x] == 0:
            self.inter_grid[y, x] = 1
            self.inter_cum_cov = self.inter_grid.sum()\
                / (self.height * self.width)

            if self.inter_cum_cov >= self.cov_to_reach:
                iter_diff = ct - self.iter_start
                time_diff = time.time() - self.time_start

                if self.iter_between == 0:
                    self.iter_between = iter_diff
                else:
                    self.iter_between = (iter_diff + self.iter_between) / 2.0

                if self.time_between == 0:
                    self.time_between = time_diff
                else:
                    self.time_between = (time_diff + self.time_between) / 2.0

                self.iter_start = ct
                self.time_start = time.time()
                self.inter_grid = np.zeros((self.height, self.width))

    def get_cumulative_coverage(self):
        return self.cum_cov

    def get_inter_cum_coverage(self):
        return self.inter_cum_cov

    def get_average_iter_between(self):
        return self.iter_between

    def get_average_time_between(self):
        return self.time_between

    def get_raw(self, x, y):
        return self.grid[y, x]

    def get_performance(self, ct):
        uncertainty_grid = ct - self.copy_grid
        metrics_grid = np.multiply((1 - self.ass_risk_grid), self.sq_grid)
        perf_grid = np.multiply(metrics_grid, uncertainty_grid)
        self.ass_risk_grid = np.zeros((self.height, self.width))
        self.sq_grid = np.zeros((self.height, self.width))
        return perf_grid.mean()

    def set_start(self):
        self.copy_grid = self.grid.copy()

    def __getitem__(self, index):
        return self.grid[
            index[1] // self.scaling_factor,
            index[0] // self.scaling_factor
        ]

    def __setitem__(self, index, value):
        self.grid[
            index[1] // self.scaling_factor,
            index[0] // self.scaling_factor
        ] = value
