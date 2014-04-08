
import planner

class Simulation(object):

    def __init__(self, problem, drawer):
        self.problem = problem
        self.planner = planner.Planner(problem)
        self.drawer = drawer


    def run(self):
        for _ in xrange(self.problem.num_steps):
            self.drawer.clear_all()
            quads = self.planner.step()
            for quad in quads:
                self.drawer.draw_quad(quad)
            self.drawer.update()

