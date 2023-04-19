#!/usr/bin/env python

import numpy as np
from crazyflie_py import *


def main():
    Z = 1.0
    
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)
    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, Z])
        cf.goTo(pos, 0, 1.0)
    timeHelper.sleep(1.0)

    while(True):
        for cf in allcfs.crazyflies:
           pos = np.array(cf.initialPosition) + np.array([0, 0, Z+1])
           cf.goTo(pos, 0, 4.0)
        timeHelper.sleep(5.0)
        for cf in allcfs.crazyflies: 
           pos = np.array(cf.initialPosition) + np.array([0, 0, Z])
           cf.goTo(pos, 0, 4.0)
        timeHelper.sleep(5.0)

    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, Z])
        cf.goTo(pos, 0, 1.0)

    allcfs.land(targetHeight=0.02, duration=1.0+Z)
    timeHelper.sleep(1.0+Z)

if __name__ == "__main__":
    main()