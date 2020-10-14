#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time


def animate_points(my_points):
    ''' perform animation of the square dance '''
    plt.rcParams["figure.figsize"] = 10, 6

    fig, ax = plt.subplots()
    lon_points = []
    alt_points = []

    for i in range(len(my_points)):
        for p in my_points[i]:
            x, y = p
            x = (x * 1000000) - 41705051
            y = (y * 100000) + 8624054
            lon_points.append(x)
            alt_points.append(y)


    ax.set_ylim(int(min(alt_points)), int(max(alt_points)))
    ax.set_xlim(int(min(lon_points)), int(max(lon_points)))

    # create a point in the axes
    points = []

    if len(my_points) > 0:
        for x, y in my_points[0]:
            x = (x * 1000000) - 41705051
            y = (y * 100000) + 8624054
            print("about to plot: ")
            print(x)
            print(y)
            points.append(ax.plot(x, y, marker="o")[0])
        my_points.pop(0)

    def update(phi):
        # set point coordinates
        if len(my_points) > 0:
            for i in range(len(points)):
                x, y = my_points[0][i]
                x = (x * 1000000) - 41705051
                y = (y * 100000) + 8624054
                points[i].set_data([x], [y])
            my_points.pop(0)

        return points

    ani = animation.FuncAnimation(
        fig,
        update,
        interval=100,
        frames=np.arange(0, 10, 0.01),
        blit=True,
    )
    plt.show()
