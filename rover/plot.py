
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm


def plot_risk_grid(risk_grid, filename):
    fig = plt.figure("Risk Map")
    ax = fig.add_subplot(111)
    ax.set_xlabel("X Location")
    ax.set_ylabel("Y Location")

    x_step = 1
    y_step = 1
    x_min = 0
    y_min = 0
    x_max = risk_grid.problem.width
    y_max = risk_grid.problem.height

    x = np.arange(x_min, x_max, x_step)
    y = np.arange(y_min, y_max, y_step)
    X, Y = np.meshgrid(x, y)

    zs = np.array(
        [
            risk_grid.get_risk(x_i, y_i)
            for x_i, y_i in zip(np.ravel(X), np.ravel(Y))
        ]
    )

    np.savetxt("sandbox/risk.out", zs)
    Z = zs.reshape(X.shape)
    ax.pcolormesh(X, Y, Z, cmap=cm.jet)

    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)

    plt.savefig(filename)
    return plt


def plot_time_grid(time_grid, filename):
    fig = plt.figure("TimeGrid Map")
    ax = fig.add_subplot(111)
    ax.set_xlabel("X Location")
    ax.set_ylabel("Y Location")

    x_step = 1
    y_step = 1
    x_min = 0
    y_min = 0
    x_max = time_grid.width - 1
    y_max = time_grid.height - 1

    x = np.arange(x_min, x_max, x_step)
    y = np.arange(y_min, y_max, y_step)
    X, Y = np.meshgrid(x, y)

    zs = np.array(
        [
            time_grid.get_raw(x_i, y_max - y_i)
            for x_i, y_i in zip(np.ravel(X), np.ravel(Y))
        ]
    )

    Z = zs.reshape(X.shape)
    ax.pcolormesh(X, Y, Z, cmap=cm.jet)

    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)

    plt.savefig(filename)
    return plt


class TimeGridPlotter(object):

    def __init__(self, time_grid):
        self.time_grid = time_grid
        self.fig = plt.figure("TimeGrid Map")
        self.ax = self.fig.add_subplot(111)
        self.ax.set_xlabel("X Location")
        self.ax.set_ylabel("Y Location")
        self.x_step = 2
        self.y_step = 2
        self.x_min = 0
        self.y_min = 0
        self.x_max = time_grid.width - 1
        self.y_max = time_grid.height - 1
        self.x = np.arange(self.x_min, self.x_max, self.x_step)
        self.y = np.arange(self.y_min, self.y_max, self.y_step)
        self.X, self.Y = np.meshgrid(self.x, self.y)
        plt.ion()
        self.get_zs()
        self.ax.set_xlim(self.x_min, self.x_max)
        self.ax.set_ylim(self.y_min, self.y_max)
        self.iteration = 0

    def get_zs(self):
        zs = np.array(
            [
                self.time_grid.get_raw(x_i, y_i)
                for x_i, y_i in zip(np.ravel(self.X), np.ravel(self.Y))
            ]
        )
        return zs

    def update(self):
        try:
            self.graph.remove()
        except:
            pass

        zs = self.get_zs()
        Z = zs.reshape(self.X.shape)
        self.graph = self.ax.pcolormesh(self.X, self.Y, Z, cmap=cm.jet)
        plt.draw()
        plt.pause(0.0001)
        filename = "sandbox/grids/{}.out".format(self.iteration)
        self.iteration += 1
        np.savetxt(filename, self.time_grid.grid)
