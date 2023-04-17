#!/usr/bin/env python

import numpy as np
from pathlib import Path
import os

from crazyflie_py import *
from crazyflie_py import generate_trajectory
from crazyflie_py.uav_trajectory import Trajectory
from crazyflie_py.uav_trajectory import visualize_trajectory
from crazyflie_py import plot_trajectory

import sys

def load_trajectories(n):
    folder = 'data/intersection_fig8_{}/'.format(n)
    assert os.path.exists(folder)
    trajectories = []
    for i in range(n):
        traj = Trajectory()
        traj.loadcsv(folder+f'{i}.csv')
        trajectories.append(traj)
    return trajectories


def save_trajectories(trajectories):
    folder = 'data/intersection_fig8_{}/'.format(len(trajectories))
    if not os.path.exists(folder):
        os.makedirs(folder)
    for i, t in enumerate(trajectories):
        t.savecsv(folder+f'{i}.csv')


def get_starting_locations(traj, n):
    #Evenly distribute drones along the figure 8
    if n % 2 == 0:
        print("Even numbers of robots will cause many collisions")
        sys.exit()
    T = traj.duration
    positions = []
    for i in range(n):
        state = traj.eval(T - i*T/n)
        positions.append(state.pos)

    return np.array(positions)

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    load_traj = True

    TIMESCALE = 1.5
    CENTER = np.array((0, 0, 1.0))
    RADIUS = 1.3

    n = len(allcfs.crazyflies)

    fx = lambda t: RADIUS * np.sin(t)
    fy = lambda t: RADIUS * np.sin(t) * np.cos(t)

    position_data = np.array(generate_trajectory.generate_position_data(fx=fx, fy=fy, domain=(0, 2*np.pi)))
    print(position_data[0], position_data[-1])


    trajectories = []
    if load_traj:
        trajectories = load_trajectories(n)
    else:
        for i in range(n):
            data = np.roll(position_data, int(i*len(position_data) / n), axis=0)
            #TODO Shift start location according to beginning location?

            data[:, 0] = position_data[:, 0]
            traj = generate_trajectory.generate_trajectory(data, num_pieces=14)
            trajectories.append(traj)
            print(traj.eval(0).pos, traj.eval(2*np.pi).pos)
            print("Generated Trajectory", i)

        save_trajectories(trajectories)
    starting_positions = get_starting_locations(trajectories[0], n)
    starting_positions += CENTER
    # print("Starting Positions", starting_positions)

    for cf, traj in zip(allcfs.crazyflies, trajectories):
        cf.uploadTrajectory(0, 0, traj)
    timeHelper.sleep(2.0)

    allcfs.takeoff(targetHeight=1.0, duration=2.0)
    timeHelper.sleep(2.5)

    for cf, pos in zip(allcfs.crazyflies, starting_positions):
        cf.goTo(pos, 0, 2.0)
    timeHelper.sleep(4.0)

    allcfs.startTrajectory(0, timescale=TIMESCALE)
    timeHelper.sleep(trajectories[0].duration * TIMESCALE + 2.0)
    allcfs.startTrajectory(0, timescale=TIMESCALE, reverse=True)
    timeHelper.sleep(trajectories[0].duration * TIMESCALE + 2.0)

    for cf in allcfs.crazyflies:
        pos = cf.initialPosition
        cf.goTo(pos + np.array([0., 0., 1.0]), 0, 2.0)
    timeHelper.sleep(2.0)
    allcfs.land(targetHeight=0.06, duration=2.0)
    timeHelper.sleep(3.0)


if __name__ == "__main__":
    main()
