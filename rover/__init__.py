import config
import simulation
import drawer
import riskgrid
import plot

def run(**kwargs):
    colors = config.Colors(**kwargs)
    problem = config.Problem(**kwargs)
    # dr = drawer.Drawer(problem, colors)

    risk_grid = riskgrid.RiskGrid(problem)
    risk_grid.add_random_points(kwargs.get("num_risk_points", 4))

    plot.plot_risk_grid(
        risk_grid, kwargs.get("img_file", "imgs/test_heatmap.png")
    )

    sim = simulation.Simulation(
        problem, risk_grid, out_file=kwargs.get("out_file", "data/all.txt")
    )
    sim.run()
