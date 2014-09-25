
import config
import timegrid
import random
import riskgrid
import simulation
import time
import os
import json
import planner


class Experiments(object):

    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.seed = kwargs.get("seed", 0)
        random.seed(self.seed)

        self.num_quads = kwargs.get("num_quads")
        self.sq_heights = kwargs.get("sq_heights")
        self.control_noises = kwargs.get("control_noises")
        self.scenes = kwargs.get("scenes")
        self.dims = kwargs.get("dims")

        self.planner_name = kwargs.get("planner")
        self.show_time_grid = kwargs.get("show_time_grid", False)
        self.folder_name = kwargs.get("folder_name")

        self.pl = planner.planners.get(self.planner_name)
        self.init_experiments_folder()

        self.data_file_output = self.experiments_folder\
            + "data/out_{0}_{1}_{2}_{3}.txt"
        self.status_output = "Scene -> {} :: SQ Height -> {} :: "\
            + "Control Noise -> {} :: Num Quads -> {} ==="

    def init_experiments_folder(self):
        self.experiments_folder = "experiments/{}-{}/".format(
            self.folder_name,
            time.asctime(
                time.localtime()
            ).replace(" ", "_").replace(":", "_")
        )
        os.mkdir(self.experiments_folder)
        os.mkdir(self.experiments_folder + "data")
        with open(self.experiments_folder + "config.json", "w") as f:
            config_str = json.dumps(self.kwargs)
            f.write(config_str)

    def print_status(self, scene, sq_height, ctrl_noise, nq):
        print "===", time.asctime(time.localtime()), "::",
        print self.status_output.format(
            scene, sq_height, ctrl_noise, nq
        )

    def run(self):
        for dim, scene in zip(self.dims, self.scenes):
            default_problem = config.Problem(
                width=dim, height=dim, **self.kwargs
            )
            default_problem.scene = scene
            risk_grid = riskgrid.RiskGrid(default_problem)
            for sq_height in self.sq_heights:
                default_problem.sq_height = sq_height
                for control_noise in self.control_noises:
                    for nq in self.num_quads:
                        self.print_status(scene, sq_height, control_noise, nq)
                        default_problem.num_quads = nq
                        default_problem.grid = timegrid.TimeGrid(
                            dim, dim, default_problem
                        )
                        scene_name = scene.split("/")[1].split(".out")[0]
                        scene_name = scene_name.replace("_", "-")
                        out_file = self.data_file_output.format(
                            scene_name, sq_height, control_noise, nq
                        )

                        sim = simulation.Simulation(
                            default_problem, risk_grid,
                            out_file=out_file, algorithm=self.pl,
                            show_time_grid=self.show_time_grid
                        )

                        sim.run()
