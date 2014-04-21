
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm


def plot_risk_grid(risk_grid, filename):
    fig = plt.figure("Risk Map")
    ax = fig.add_subplot(111)
    ax.set_xlabel("X Location")
    ax.set_ylabel("Y Location")

    x_step = 2
    y_step = 2
    x_min = 0
    y_min = 0
    x_max = risk_grid.problem.width
    y_max = risk_grid.problem.height

    x = np.arange(x_min, x_max, x_step)
    y = np.arange(y_min, y_max, y_step)
    X, Y = np.meshgrid(x, y)

    zs = np.array(
        [
            risk_grid.get_risk(x_i, y_max - y_i)
            for x_i, y_i in zip(np.ravel(X), np.ravel(Y))
        ]
    )

    Z = zs.reshape(X.shape)
    ax.pcolormesh(X, Y, Z, cmap=cm.jet)

    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)

    plt.savefig(filename)
    return plt
