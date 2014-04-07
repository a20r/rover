
import quadcopter
import time
import math

class Simulation(object):

    def __init__(self, problem, drawer):
        self.problem = problem
        self.quad_list = self.init_quads()
        self.drawer = drawer
        map(self.problem.grid.update_grid, self.quad_list)


    def init_quads(self):
        quad_list = list()
        init_length = math.ceil(math.sqrt(self.problem.num_quads))
        for i in xrange(self.problem.num_quads):
            down = int(i // init_length)
            accross = int(i % init_length)

            s_x, s_y = (
                self.problem.width / 2 +
                3 * self.problem.quad_size * accross,
                self.problem.height / 2 +
                3 * self.problem.quad_size * down
            )

            quad_list.append(quadcopter.Quadcopter(
                s_x, s_y, self.problem.min_height,
                self.problem
            ))

        return quad_list


    def step(self):
        for quad in self.quad_list:
            self.drawer.draw_quad(quad)
            uv = self.problem.grid.get_new_direction(quad)
            quad.move_2d(uv)
            self.problem.grid.update_grid(quad)


    def run(self):
        for _ in xrange(self.problem.num_steps):
            # self.drawer.clear_all()
            self.step()
            self.drawer.update()
            time.sleep(self.problem.delay)
