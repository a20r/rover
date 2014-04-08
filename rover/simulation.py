
import planner
import time

class Simulation(object):

    def __init__(self, problem, drawer, risk_grid):
        self.problem = problem
        self.planner = planner.Planner(problem, risk_grid)
        self.drawer = drawer
        self.risk_grid = risk_grid
        self.surface_list = list()


    def run(self):
        self.render()
        self.play()


    def render(self):
        for _ in xrange(self.problem.num_steps):
            quads = self.planner.step()
            for quad in quads:
                self.drawer.add_coverage(quad)
            self.drawer.clear_all()
            self.drawer.draw_coverage()
            self.drawer.draw_risk_grid(self.risk_grid)
            for quad in quads:
                self.drawer.draw_quad(quad)
            frame = self.drawer.update()
            self.surface_list.append(frame)


    def play(self):
        done = False
        counter = 0
        while not done:

            self.drawer.update_with_frame(self.surface_list[counter])

            key = self.drawer.get_key_pressed()
            if key[self.drawer.constants.K_LEFT]:
                counter -= 1
                time.sleep(0.04)
            elif key[self.drawer.constants.K_RIGHT]:
                counter += 1
                time.sleep(0.04)
            elif key[self.drawer.constants.K_ESCAPE]:
                done = True

            if counter >= len(self.surface_list):
                counter = len(self.surface_list) - 1
            elif counter < 0:
                counter = 0
