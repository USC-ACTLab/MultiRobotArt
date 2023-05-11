from crazyflie_py import Crazyswarm
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import approximate_taylor_polynomial
from crazyflie_py.uav_trajectory import Trajectory
from crazyflie_py.generate_trajectory import *
from pathlib import Path
from crazyflie_py import plot_trajectory

TAKEOFF_DURATION = 2.
HOVER_DURATION = 2.5
TIMESCALE = 1.0
ALPHA = 0.5
<<<<<<< HEAD
K = 5/4
=======
K = 3
>>>>>>> origin/main

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

<<<<<<< HEAD
    fx = lambda x: ALPHA * np.cos((K)* (x + np.pi/K/2)) * np.cos(x)
    fy = lambda x: 0 # 0.5 * np.sin(x)
    fz = lambda x: ALPHA * np.cos((K) * (x + np.pi/K/2)) * np.sin(x)
    print("Generating Position Data")
    generate_position_data(fx, fy, fz, domain=(0, 8*np.pi + 8*np.pi / 20), output='test.csv')
    print("Computing trajectory")
    # traj = generate_trajectory_from_file('test.csv', num_pieces=10, approx=False)
    traj = Trajectory()
    traj.loadcsv('traj.csv')
    # traj.savecsv('traj.csv')
=======
    fx = lambda x: ALPHA * np.cos((5/4)* x) * np.cos(x)
    fy = lambda x: 0 # 0.5 * np.sin(x)
    fz = lambda x: ALPHA * np.cos((5/4) * x) * np.sin(x)
    print("Generating Position Data")
    generate_position_data(fx, fy, fz, domain=(0, 8*np.pi), output='test.csv')
    print("Computing trajectory")
    #traj = generate_trajectory_from_file('test.csv', num_pieces=10)
    traj = Trajectory()
    traj.loadcsv('traj_helix1.csv')
    traj.savecsv('traj_helix1.csv')
    plot_trajectory.plot(traj)
    print("Beginning CF execution")
    for cf in allcfs.crazyflies:
        cf.uploadTrajectory(0, 0, traj)
<<<<<<< HEAD
    timeHelper.sleep(2.0)
    for cf in allcfs.crazyflies:
        cf.setLEDColor(0, 1, 0)
    allcfs.takeoff(targetHeight=1., duration=2.)
    timeHelper.sleep(4.0)
    for cf in allcfs.crazyflies:
        cf.setLEDColor(1, 0, 0)
    allcfs.startTrajectory(0, timescale=TIMESCALE)
    timeHelper.sleep(traj.duration * TIMESCALE + 2.0)

    for cf in allcfs.crazyflies:
        cf.setLEDColor(0, 1, 0)
=======
    timeHelper.sleep(1.0)
    allcfs.takeoff(targetHeight=1., duration=2.)
    timeHelper.sleep(4.0)

    allcfs.startTrajectory(0, timescale=TIMESCALE)
    timeHelper.sleep(traj.duration * TIMESCALE + 2.0)
>>>>>>> origin/main
    
    allcfs.land(targetHeight=0.04, duration=2.0)

if __name__ == "__main__":
    main()
