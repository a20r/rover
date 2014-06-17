
import config
import simulation
import rosdrawer
import riskgrid
import plot
import experiments
import planner
import drawer
import rospy


def run(**kwargs):
    experimental = kwargs.get("experimental", False)

    if not experimental:
        problem = config.Problem(**kwargs)

        dr = rosdrawer.Drawer(problem)

        risk_grid = riskgrid.RiskGrid(problem)
        risk_grid.add_random_points(kwargs.get("num_risk_points", 4))

        plot.plot_risk_grid(
            risk_grid,
            kwargs.get("img_file", "imgs/test_heatmap.png")
        )

        pl = planner.planners.get(
            kwargs.get("planner", "rover_gaussian"),
            planner.PlannerGaussian
        )

        sim = simulation.Simulation(
            problem, risk_grid, drawer=dr,
            out_file=kwargs.get("out_file", "data/all.txt"),
            algorithm=pl, practical=kwargs.get("practical", False),
            show_time_grid=kwargs.get("show_time_grid", False),
            names=kwargs.get("names", dict())
        )

        try:
            sim.run()
        except rospy.ROSInterruptException:
            exit()
    else:
        if "num_quads" in kwargs.keys():
            del kwargs["num_quads"]

        exps = experiments.Experiments(**kwargs)
        exps.run()

