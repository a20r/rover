
import config
import simulation
import rosdrawer
import riskgrid
import plot
import experiments
import planner
import rospy
import raplanner


def run(**kwargs):
    experimental = kwargs.get("experimental", False)

    if not experimental:
        problem = config.Problem(**kwargs)

        dr = rosdrawer.Drawer(problem)

        risk_grid = riskgrid.RiskGrid(problem)

        plot.plot_risk_grid(
            risk_grid,
            kwargs.get("img_file", "imgs/test_heatmap.png")
        )

        pl = planner.planners.get(
            kwargs.get("planner"),
            raplanner.RiskAltitudePlanner
        )

        sim = simulation.Simulation(
            problem, risk_grid, drawer=dr, algorithm=pl,
            **kwargs
        )

        try:
            sim.run()
        except rospy.ROSInterruptException:
            exit()
    else:
        exps = experiments.Experiments(**kwargs)
        exps.run()
