
import quadcopter
import time
import math
import numpy as np

class Planner(object):

    def __init__(self, problem, risk_grid):
        self.problem = problem
        self.quad_list = self.init_quads()
        self.risk_grid = risk_grid


        for quad in self.quad_list:
            self.problem.grid.update_grid(quad)


    def determine_height(self, x, y):
        # TODO HIGHLY EXPERMIMENTAL
        init_risk = float(self.risk_grid.get_risk(x, y))
        c3 = init_risk / (self.problem.max_height - self.problem.min_height)
        c2 = 1 - init_risk
        c1 = 0
        c0 = -1 * math.pow(self.problem.min_height, 2)
        root = max(np.roots([c3, c2, c1, c0]))
        return root.real


    def init_quads(self):
        quad_list = list()
        init_length = math.ceil(math.sqrt(self.problem.num_quads))
        for i in xrange(self.problem.num_quads):
            down = int(i // init_length)
            accross = int(i % init_length)

            s_x, s_y = (
                3 * self.problem.quad_size * accross,
                3 * self.problem.quad_size * down
            )

            quad_list.append(quadcopter.Quadcopter(
                s_x, s_y, self.problem.min_height,
                self.problem
            ))

        return quad_list


    def step(self):
        for quad in self.quad_list:
            uv = self.problem.grid.get_new_direction(quad)
            quad.move_2d(uv)
            self.problem.grid.update_grid(quad)
            risk = self.risk_grid.get_risk(quad.x, quad.y)
            quad.set_z(
                self.problem.min_height + risk *\
                (self.problem.max_height - self.problem.min_height)
            )

            # quad.z = self.determine_height(quad.x, quad.y)

        return self.quad_list

