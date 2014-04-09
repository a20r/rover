
import planner
import time
import stats

class Simulation(object):

    def __init__(self, problem, risk_grid, drawer=None, out_file=None):
        self.problem = problem
        self.planner = planner.Planner(problem, risk_grid)
        self.drawer = drawer
        self.risk_grid = risk_grid
        self.mca = stats.MonteCarloArea(problem, 1000)
        self.sqa = stats.SensorQualityAverage(problem)
        self.ra = stats.RiskAverage(problem, risk_grid)
        self.surface_list = list()
        if not out_file == None:
            self.out_file = open(out_file, "w")


    def run(self):
        self.render()

        if not self.drawer == None:
            self.play()


    def render(self):
        for _ in xrange(self.problem.num_steps):
            quads = self.planner.step()

            if not self.drawer == None:
                for quad in quads:
                    self.drawer.add_coverage(quad)

                self.drawer.clear_all()
                self.drawer.draw_coverage()
                self.drawer.draw_risk_grid(self.risk_grid)

                for quad in quads:
                    self.drawer.draw_quad(quad)

                frame = self.drawer.update()
                self.surface_list.append(frame)

            self.mca.update_average_efficiency(quads)
            self.sqa.update_average_sq(quads)
            self.ra.update_average_risk(quads)

            if not self.out_file == None:
                self.out_file.write(
                    str(self.mca.get_moving_average_efficiency()) + " " +
                    str(self.sqa.get_moving_average()) + " " +
                    str(self.ra.get_moving_average()) + "\n"
                )


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
