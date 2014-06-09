
import planner
import time
import stats
import evader
import json
import plot


class Simulation(object):

    def __init__(self, problem, risk_grid, **kwargs):
        self.problem = problem
        self.planner_obj = kwargs.get("algorithm", planner.PlannerMonotonic)
        self.pl = self.planner_obj(problem, risk_grid)
        self.drawer = kwargs.get("drawer", None)
        self.show_time_grid = kwargs.get("show_time_grid", True)
        self.risk_grid = risk_grid
        self.droid_list = self.init_droids()
        self.mca = stats.MonteCarloArea(problem, 1000)
        self.sqa = stats.SensorQualityAverage(self.pl)
        self.ra = stats.RiskAverage(self.pl)
        self.surface_list = list()

        if self.show_time_grid:
            self.t_plotter = plot.TimeGridPlotter(self.problem.grid)

        if not kwargs.get("out_file", None) is None:
            self.out_file = open(kwargs.get("out_file", None), "w")
        else:
            self.out_file = None

        if not kwargs.get("position_file", None) is None:
            self.position_file = open(kwargs.get("position_file", None), "w")
        else:
            self.position_file = None

    def init_droids(self):
        droid_pos_list = [
            (0, 0), (self.problem.width, 0), (0, self.problem.height),
            (self.problem.width, self.problem.height)
        ]

        droid_list = [
            evader.Evader(
                x, y, 0, 0, self.problem, self.risk_grid
            ) for x, y in droid_pos_list
        ]

        return droid_list

    def run(self):
        self.render()

        if not self.drawer is None:
            self.play()

    def render(self):
        for i in xrange(self.problem.num_steps):
            quads = self.pl.step()
            if self.show_time_grid:
                self.t_plotter.update()

            for droid in self.droid_list:
                droid.step(quads)

            if not self.drawer is None:
                for quad in quads:
                    self.drawer.add_coverage(quad)

                self.drawer.clear_all()
                self.drawer.draw_coverage()

                for droid in self.droid_list:
                    self.drawer.draw_evader(droid.x, droid.y)

                self.drawer.draw_risk_grid(self.risk_grid)

                for quad in quads:
                    self.drawer.draw_quad(quad)

                frame = self.drawer.update()
                self.surface_list.append(frame)

            self.mca.update_average_efficiency(quads)
            self.sqa.update_average_sq(quads)
            self.ra.update_average_risk(quads)
            self.write_results(quads, i)

    def write_results(self, quads, i):
        if not self.out_file is None:
            self.out_file.write(
                str(self.mca.get_moving_average_efficiency()) + " " +
                str(self.sqa.get_moving_average()) + " " +
                str(self.ra.get_moving_average()) + "\n"
            )

        if not self.position_file is None:
            for j, quad in enumerate(quads):
                self.position_file.write(
                    "{}, {}, {}, {}, {}, {}\n".format(
                        i, j, quad.x, quad.y, quad.z,
                        quad.get_sensor_radius()
                    )
                )

    def play(self):
        if not self.drawer.can_play():
            return

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
