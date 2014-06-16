
import planner
import time
import stats
import evader
import json
import plot
import random


class Simulation(object):

    def __init__(self, problem, risk_grid, **kwargs):
        self.problem = problem
        self.planner_obj = kwargs.get("algorithm", planner.PlannerGaussian)
        self.pl = self.planner_obj(problem, risk_grid)
        self.drawer = kwargs.get("drawer", None)
        self.show_time_grid = kwargs.get("show_time_grid", True)
        self.risk_grid = risk_grid
        self.mca = stats.MonteCarloArea(problem, 1000)
        self.sqa = stats.SensorQualityAverage(self.pl)
        self.ra = stats.RiskAverage(self.pl)

        if not kwargs.get("out_file", None) is None:
            self.out_file = open(kwargs.get("out_file", None), "w")
        else:
            self.out_file = None

        if not kwargs.get("position_file", None) is None:
            self.position_file = open(kwargs.get("position_file", None), "w")
        else:
            self.position_file = None

        if self.show_time_grid:
            self.t_plotter = plot.TimeGridPlotter(self.problem.grid)

    def publish_position(self, quad):
        return self

    def get_actual_position(self, quad):
        return (
            quad.x + random.gauss(0, 10),
            quad.y + random.gauss(0, 10),
            quad.z + random.gauss(0, 10)
        )

    def run(self):
        for i in xrange(self.problem.num_steps):
            quads = self.pl.get_quads()

            for quad in quads:
                rpx, rpy, rpz = self.get_actual_position(quad)
                quad.set_position(rpx, rpy, rpz)
                self.problem.grid.update_grid(quad)
                self.pl.update_quad(quad)
                self.publish_position(quad)

            if self.show_time_grid:
                self.t_plotter.update()

            if not self.drawer is None:
                self.drawer.clear_all()

                self.drawer.draw_risk_grid(self.risk_grid)

                for quad in quads:
                    self.drawer.draw_quad(quad)

                frame = self.drawer.update()

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
