#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.gridspec as gridspec

def plot_pos(data, title=None):
    x = [d[1] for d in data]
    y = [d[2] for d in data]
    z = [d[3] for d in data]
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title(title)
    ax.scatter(x[0], y[0], z[0], c='b', s=100)
    ax.scatter(0, 0, 0, c='r', s=100, marker='*')
    ax.plot(x, y, z)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    plt.show()


def plot_traj(traj):
    ts = np.arange(0, traj.duration, 0.01)
    evals = np.empty((len(ts), 15))
    for t, i in zip(ts, range(0, len(ts))):
        e = traj.eval(t)
        evals[i, 0:3] = e.pos
        evals[i, 3:6] = e.vel
        evals[i, 6:9] = e.acc
        evals[i, 9:12] = e.omega
        evals[i, 12] = e.yaw
        # evals[i, 13]   = e.roll
        # evals[i, 14]   = e.pitch

    velocity = np.linalg.norm(evals[:, 3:6], axis=1)
    acceleration = np.linalg.norm(evals[:, 6:9], axis=1)
    omega = np.linalg.norm(evals[:, 9:12], axis=1)

    # print stats
    print("max speed (m/s): ", np.max(velocity))
    print("max acceleration (m/s^2): ", np.max(acceleration))
    print("max omega (rad/s): ", np.max(omega))
    print("max roll (deg): ", np.max(np.degrees(evals[:, 13])))
    print("max pitch (deg): ", np.max(np.degrees(evals[:, 14])))

    # Create 3x1 sub plots
    gs = gridspec.GridSpec(6, 1)
    fig = plt.figure()

    ax = plt.subplot(gs[0:2, 0], projection='3d')  # row 0
    ax.plot(evals[:, 0], evals[:, 1], evals[:, 2])

    ax = plt.subplot(gs[2, 0])  # row 2
    ax.plot(ts, velocity)
    ax.set_ylabel("velocity [m/s]")

    ax = plt.subplot(gs[3, 0])  # row 3
    ax.plot(ts, acceleration)
    ax.set_ylabel("acceleration [m/s^2]")

    ax = plt.subplot(gs[4, 0])  # row 4
    ax.plot(ts, omega)
    ax.set_ylabel("omega [rad/s]")

    ax = plt.subplot(gs[5, 0])  # row 5
    ax.plot(ts, np.degrees(evals[:, 12]))
    ax.set_ylabel("yaw [deg]")

    # ax = plt.subplot(gs[6, 0]) # row 5
    # ax.plot(ts, np.degrees(evals[:,13]))
    # ax.set_ylabel("roll [deg]")

    # ax = plt.subplot(gs[7, 0]) # row 5
    # ax.plot(ts, np.degrees(evals[:,14]))
    # ax.set_ylabel("pitch [deg]")

    plt.show()
