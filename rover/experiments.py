
import config
import riskgrid
import plot
import random
import riskgrid
import simulation
import time
import os
import json


class Experiments(object):

    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.seed = kwargs.get("seed", 0)
        random.seed(self.seed)

        self.min_risk_points = kwargs.get("min_risk_points", 1)
        self.max_risk_points = kwargs.get("max_risk_points", 20)
        self.risk_points_step = kwargs.get("risk_points_step", 5)
        self.min_quads = kwargs.get("min_quads", 1)
        self.max_quads = kwargs.get("max_quads", 20)
        self.quads_step = kwargs.get("quads_step", 5)
        self.experiments_folder = "experiments/{}/".format(
            time.asctime(time.gmtime()).replace(" ", "_").replace(":", "_")
        )

        self.init_experiments_folder()

        self.img_file_output = self.experiments_folder +\
            "imgs/heatmap_{0}_{1}.png"

        self.data_file_output = self.experiments_folder +\
            "data/out_{0}_{1}.txt"

        self.status_output = "Num Risk Points -> {} :: Num Quads -> {} ==="

    def init_experiments_folder(self):
        os.mkdir(self.experiments_folder)
        os.mkdir(self.experiments_folder + "imgs")
        os.mkdir(self.experiments_folder + "data")
        with open(self.experiments_folder + "config.json", "w") as f:
            config_str = json.dumps(self.kwargs)
            f.write(config_str)

    def print_status(self, num_risk_points, num_quads):
        print "===", time.asctime(time.gmtime()), "::",
        print self.status_output.format(num_risk_points, num_quads)

    def run(self):
        for nrp in range(
            self.min_risk_points,
            self.max_risk_points + self.risk_points_step,
            self.risk_points_step
        ):
            for nq in range(
                self.min_quads,
                self.max_quads + self.quads_step,
                self.quads_step
            ):
                self.print_status(nrp, nq)
                problem = config.Problem(num_quads=nq, **self.kwargs)
                risk_grid = riskgrid.RiskGrid(problem)
                risk_grid.add_random_points(nrp)
                plot.plot_risk_grid(
                    risk_grid,
                    self.img_file_output.format(nrp, nq)
                )

                sim = simulation.Simulation(
                    problem, risk_grid,
                    out_file=self.data_file_output.format(nrp, nq)
                )

                sim.run()
