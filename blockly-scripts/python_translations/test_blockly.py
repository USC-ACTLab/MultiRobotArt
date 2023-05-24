
from pathlib import Path

from crazyflie_py import Crazyswarm
from generate_trajectory import generate_trajectory, plot_utils
from generate_trajectory.uav_trajectory import Trajectory
from curves_to_trajectory.curves import *
import matplotlib.pyplot as plt
import numpy as np


# noinspection PyShadowingNames
def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    load_data = False

    if load_data:
        # Method 1: Load the trajectory from a file
        traj = Trajectory()
        print("Loading trajectory")
        traj.loadcsv(Path(__file__).parent / 'data/traj_rose2.csv')
    else:
        # Method 2: Generate the trajectory from the curve
        # TODO: Edit the following line to change the curve
        flight_time = 20
        fx, fz = rose(1, 2, 1, flight_time)
        print("Generating Position Data")
        data = generate_trajectory.generate_position_data(fx=fx, fz=fz,
                                                          domain=(flight_time / 8, flight_time + flight_time / 8),
                                                          output='pos.csv')
        print("Computing trajectory")
        traj = generate_trajectory.generate_trajectory_from_file('pos.csv',
                                                                 num_pieces=12,
                                                                 approx=False)
        traj.savecsv('traj.csv')

    plot_trajectory.plot(traj)
    print("Beginning CF execution")
    for cf in allcfs.crazyflies:
        cf.uploadTrajectory(0, 0, traj)

    timeHelper.sleep(2.0)
    # for cf in allcfs.crazyflies:
    #     cf.setLEDColor(0, 1, 0)
    allcfs.takeoff(targetHeight=1.5, duration=2.)

    timeHelper.sleep(4.0)
    # for cf in allcfs.crazyflies:
    #     cf.setLEDColor(1, 0, 0)
    allcfs.startTrajectory(0, timescale=1.0)

    timeHelper.sleep(traj.duration * 1.0 + 2.0)
    # for cf in allcfs.crazyflies:
    #     cf.setLEDColor(0, 1, 0)
    allcfs.land(targetHeight=0.04, duration=2.0)


if __name__ == "__main__":
    main()
