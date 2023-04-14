#!/usr/bin/env python

from crazyflie_py import *


def main():
    Z = 1.0
    
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # disable LED (one by one)
    for cf in allcfs.crazyflies:
        cf.setParam("motorPowerSet.m1", 0)
        cf.setParam("motorPowerSet.enable", 2)
        cf.setLEDColor(1, 0, 0)
    timeHelper.sleep(1.0)
    
    
    # enable LED (broadcast)
    for cf in allcfs.crazyflies:
        cf.setParam("motorPowerSet.m1", 65535)
        cf.setParam("motorPowerSet.m2", 65535)
        cf.setParam("motorPowerSet.m3", 65535)
        cf.setParam("motorPowerSet.m4", 65535)
    timeHelper.sleep(5.0)

    cf.setParam("motorPowerSet.m1", 0)
    cf.setParam("motorPowerSet.m2", 0)
    cf.setParam("motorPowerSet.m3", 0)
    cf.setParam("motorPowerSet.m4", 0)


if __name__ == "__main__":
    main()
