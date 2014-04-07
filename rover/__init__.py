
import config
import simulation
import drawer

def run(**kwargs):
    colors = config.Colors(**kwargs)
    problem = config.Problem(**kwargs)
    dr = drawer.Drawer(problem, colors)
    sim = simulation.Simulation(problem, dr)
    sim.run()
