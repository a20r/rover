
import config
import simulation
import drawer
import rgrids

def run(**kwargs):
    colors = config.Colors(**kwargs)
    problem = config.Problem(**kwargs)
    dr = drawer.Drawer(problem, colors)
    risk_grid_name = kwargs.get("risk_grid", "square")
    risk_grid = rgrids.get(risk_grid_name, rgrids.square)
    sim = simulation.Simulation(problem, dr, risk_grid)
    sim.run()
