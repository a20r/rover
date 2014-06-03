
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
        colors = config.Colors(**kwargs)
        problem = config.Problem(**kwargs)

        dr_type = kwargs.get("drawer", "ros")

        if dr_type == "pygame":
            dr = drawer.Drawer(problem, colors)
        elif dr_type == "ros":
            dr = rosdrawer.Drawer(problem, colors)

        risk_grid = riskgrid.RiskGrid(problem)
        risk_grid.add_random_points(kwargs.get("num_risk_points", 4))

        plot.plot_risk_grid(
            risk_grid, kwargs.get("img_file", "imgs/test_heatmap.png")
        )

        pl = planner.planners.get(
            kwargs.get("planner", "rover_monotonic"), planner.PlannerMonotonic
        )

        sim = simulation.Simulation(
            problem, risk_grid, drawer=dr,
            out_file=kwargs.get("out_file", "data/all.txt"),
            algorithm=pl, show_time_grid=kwargs.get("show_time_grid", True)
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

