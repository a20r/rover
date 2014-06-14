
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
import time
import random


class TargetGrid(object):

    def __init__(self, width, height, problem):
        self.width = width
        self.height = height
        self.problem = problem
        self.t_max = 2
        self.start_time = None
        self.lx = 0
        self.ly = 0

    def x(self, t):
        return 100 * math.cos(t) + 500 + random.gauss(0, 1)

    def y(self, t):
        return 500 + 100 * math.sin(2 * t) + random.gauss(0, 1)

    def f(self, t):
        return self.x(t), self.y(t)

    def phi(self, mu, sig, X):
        coef = 1.0 / math.sqrt(2 * math.pi * pow(sig, 2))
        expo = math.exp(-0.5 * pow((X - mu) / sig, 2))
        return coef * expo

    def phi_0(self, sig, X):
        return self.phi(0, sig, X)

    def lam(self, X):
        return 1 * (X + 1) ** 2

    def P(self, x, y):

        if self.start_time is None:
            self.start_time = time.time()

        t_0 = time.time() - self.start_time
        self.lx, self.ly = self.f(t_0)

        delta_t = 0.1
        s = 0
        count = 0

        for t_ in xrange(int(t_0 / delta_t), int((t_0 + self.t_max) / delta_t) + 1):
            t = t_ * delta_t
            sim_x, sim_y = self.f(t)
            sig = self.lam(abs(t - t_0))
            X = math.sqrt(pow(x - sim_x, 2) + pow(y - sim_y, 2))
            a = self.phi_0(sig, X)
            if a > s:
                s = a
            #s += a
            count += 1

        return s / float(count) + 1

    def update_grid(self, *args):
        return self

    def __getitem__(self, index):
        return -self.P(index[0], index[1])

    def raw_set_item(self, *args):
        return self

    def get_raw(self, x, y):
        return self[x, y]

    def plot(self):
        fig = plt.figure("Position Probability")
        ax = fig.add_subplot(111)
        ax.set_xlabel("X Location")
        ax.set_ylabel("Y Location")

        x_step = 0.1
        y_step = 0.2
        x_min = 0
        x_max = 10
        y_min = -3
        y_max = 3

        x = np.arange(x_min, x_max, x_step)
        y = np.arange(y_min, y_max, y_step)
        X, Y = np.meshgrid(x, y)
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        plt.ion()

        start_time = time.time()

        while True:
            time_off = time.time() - start_time

            zs = np.array(
                [
                    -self.P(x_i, y_i, time_off)
                    for x_i, y_i in zip(np.ravel(X), np.ravel(Y))
                ]
            )

            Z = zs.reshape(X.shape)

            try:
                graph.remove()
            except Exception as e:
                print e

            graph = ax.pcolormesh(X, Y, Z, cmap=cm.jet)
            plt.draw()
            plt.pause(0.00001)

        return plt

