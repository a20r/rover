
import numpy as np
import matplotlib
import matplotlib.pylab
import math
import random


"""
    DISCLAIMER: The crappy code below does not belong to me, I am simply
    too lazy to modify it.

"""


def make_height_func(noise_func, noise_min, noise_max):
    def height_func(i):
        return noise_func(noise_min * 2 ** -i, noise_max * 2 ** -i)
    return height_func


def make_space(width, height, height_func):
    space = np.zeros((width, height))
    corner = height_func(0)
    space[0, 0] = corner
    space[0, -1] = corner
    space[-1, 0] = corner
    space[-1, -1] = corner
    return space


def avg(*args):
    return sum(args) / len(args)


def distort_space(space, height_func):
    x_max, y_max = space.shape
    x_min = y_min = 0
    x_max -= 1
    y_max -= 1

    side = x_max
    squares = 1
    i = 0

    while side > 1:
        for x in range(squares):
            for y in range(squares):
                #Locations
                x_left = x * side
                x_right = (x + 1) * side
                y_top = y * side
                y_bottom = (y + 1) * side

                dx = side / 2
                dy = side / 2

                xm = x_left + dx
                ym = y_top + dy

                #Diamond step- create center avg for each square
                space[xm, ym] = avg(
                    space[x_left, y_top],
                    space[x_left, y_bottom],
                    space[x_right, y_top],
                    space[x_right, y_bottom]
                )

                space[xm, ym] += height_func(i)

                #Square step- create squares for each diamond
                #Top Square
                if (y_top - dy) < y_min:
                    temp = y_max - dy
                else:
                    temp = y_top - dy
                space[xm, y_top] = avg(
                    space[x_left, y_top],
                    space[x_right, y_top],
                    space[xm, ym],
                    space[xm, temp]
                )
                space[xm, y_top] += height_func(i)

                #Top Wrapping
                if y_top == y_min:
                    space[xm, y_max] = space[xm, y_top]

                #Bottom Square
                if (y_bottom + dy) > y_max:
                    temp = y_top + dy
                else:
                    temp = y_bottom - dy
                space[xm, y_bottom] = avg(space[x_left, y_bottom],
                                          space[x_right, y_bottom],
                                          space[xm, ym],
                                          space[xm, temp])
                space[xm, y_bottom] += height_func(i)

                #Bottom Wrapping
                if y_bottom == y_max:
                    space[xm, y_min] = space[xm, y_bottom]

                #Left Square
                if (x_left - dx) < x_min:
                    temp = x_max - dx
                else:
                    temp = x_left - dx
                space[x_left, ym] = avg(space[x_left, y_top],
                                        space[x_left, y_bottom],
                                        space[xm, ym],
                                        space[temp, ym])
                space[x_left, ym] += height_func(i)

                #Left Wrapping
                if x_left == x_min:
                    space[x_max, ym] = space[x_left, ym]

                #Right Square
                if (x_right + dx) > x_max:
                    temp = x_min + dx
                else:
                    temp = x_right + dx
                space[x_right, ym] = avg(space[x_right, y_top],
                                         space[x_right, y_bottom],
                                         space[xm, ym],
                                         space[temp, ym])
                space[x_right, ym] += height_func(i)

                #Right Wrapping
                if x_right == x_max:
                    space[x_min, ym] = space[x_right, ym]

        #Refine the pass
        side /= 2
        squares *= 2
        i += 1


def save_surface(space, path):
    bottom = min(space.flat)
    top = max(space.flat)
    norm = matplotlib.colors.Normalize(bottom, top)

    x, y = space.shape
    matplotlib.pylab.clf()
    fig = matplotlib.pylab.gcf()
    ax = fig.gca()
    ScalarMap = ax.pcolorfast(space, norm=norm)
    fig.colorbar(ScalarMap)
    ax.axis('image')
    fig.savefig(path + ".png", dpi=100)
    np.savetxt(path + ".out", space)
    ax.cla()


def get_space(width, height):
    width = 2 ** math.ceil(math.log(width, 2)) + 1
    height = 2 ** math.ceil(math.log(height, 2)) + 1

    noise_func = random.uniform
    noise_min = -1.0
    noise_max = 1.0
    height_func = make_height_func(noise_func, noise_min, noise_max)

    #Initialize the space with random values
    space = make_space(width, height, height_func)

    #Square-Diamond Method
    distort_space(space, height_func)
    min_val = space.min()
    max_val = space.max()
    space = (space - min_val) / (max_val - min_val)
    print space.mean()
    return space

    #save_surface(space, path)


if __name__ == "__main__":
    import sys
    dim = int(sys.argv[1])
    name = sys.argv[2]
    space = get_space(dim, dim)
    save_surface(space, name)
