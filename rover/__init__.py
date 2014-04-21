import config
import simulation
import drawer
import riskgrid
import plot
import experiments


def run(**kwargs):
    experimental = kwargs.get("experimental", False)

    if not experimental:
        colors = config.Colors(**kwargs)
        problem = config.Problem(**kwargs)
        dr = drawer.Drawer(problem, colors)

        risk_grid = riskgrid.RiskGrid(problem)
        risk_grid.add_random_points(kwargs.get("num_risk_points", 4))

        plot.plot_risk_grid(
            risk_grid, kwargs.get("img_file", "imgs/test_heatmap.png")
        )

        sim = simulation.Simulation(
            problem, risk_grid, drawer=dr,
            out_file=kwargs.get("out_file", "data/all.txt")
        )
        sim.run()
    else:
        if "num_quads" in kwargs.keys():
            del kwargs["num_quads"]

        exps = experiments.Experiments(**kwargs)
        exps.run()

