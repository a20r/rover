
import quadcopter
import time
import math

class Planner(object):

    def __init__(self, problem, risk_grid):
        self.problem = problem
        self.quad_list = self.init_quads()
        self.risk_grid = risk_grid

        for quad in self.quad_list:
            self.problem.grid.update_grid(quad)


    def init_quads(self):
        quad_list = list()
        init_length = math.ceil(math.sqrt(self.problem.num_quads))
        for i in xrange(self.problem.num_quads):
            down = int(i // init_length)
            accross = int(i % init_length)

            s_x, s_y = (
                # self.problem.width / 2 +
                3 * self.problem.quad_size * accross,
                # self.problem.height / 2 +
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
            quad.z = self.problem.min_height + risk *\
                (self.problem.max_height - self.problem.min_height)

        return self.quad_list

