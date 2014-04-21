
import quadcopter
import math


class LawnMower(object):

    def __init__(self, problem):
        self.problem = problem
        self.quad_list = self.init_quads()
        self.current_heading = [0, 1]
        self.right_step = [1, 0]
        self.left_step = [-1, 0]
        self.up = [0, -1]
        self.down = [0, 1]

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

    def get_new_direction(self, quad):
        pass

    def update_quad(self, quad):
        uv = self.get_new_direction(quad)
        quad.move_2d(uv)

    def step(self):
        for quad in self.quad_list:
            self.update_quad(quad)

        return self.quad_list
