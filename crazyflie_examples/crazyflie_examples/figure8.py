#!/usr/bin/env python

import numpy as np
from pathlib import Path

from crazyflie_py import *
from crazyflie_py.uav_trajectory import Trajectory
from crazyflie_py.uav_trajectory import visualize_trajectory
def main():
    traj1 = Trajectory()
    traj1.loadcsv(Path(__file__).parent / "data/figure8.csv")
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    for p in traj1.polynomials:
        temp_z = p.pz
        temp_y = p.py

        p.py = temp_z
        p.pz = temp_y

    TRIALS = 1
    TIMESCALE = 0.8
    for i in range(TRIALS):
        for cf in allcfs.crazyflies:
            cf.uploadTrajectory(0, 0, traj1)

        allcfs.takeoff(targetHeight=1.0, duration=2.0)
        timeHelper.sleep(2.5)
        for cf in allcfs.crazyflies:
            pos = np.array(cf.initialPosition) + np.array([0, 0, 1.0])
            cf.goTo(pos, 0, 2.0)
        timeHelper.sleep(2.5)

        allcfs.startTrajectory(0, timescale=TIMESCALE)
        timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)
        allcfs.startTrajectory(0, timescale=TIMESCALE, reverse=True)
        timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)

        allcfs.land(targetHeight=0.06, duration=2.0)
        timeHelper.sleep(3.0)


if __name__ == "__main__":
    main()
