#!/usr/bin/env python

import numpy as np
from crazyflie_py import Crazyswarm


def main():
    Z = 1.0

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=Z, duration=2.0)
    timeHelper.sleep(2.0)

    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, Z])
        cf.goTo(pos, 0, 1.0)
    timeHelper.sleep(1.0)

    allcfs.land(targetHeight=0.04, duration=2.0)
    timeHelper.sleep(2.0)


if __name__ == "__main__":
    main()
