"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

# from pycrazyswarm import Crazyswarm
from crazyflie_py import Crazyswarm


TAKEOFF_DURATION = 2.
HOVER_DURATION = 2.5


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    print(allcfs.crazyflies[0].paramTypeDict)

    allcfs.crazyflies[0].setParam('motorPowerSet/enable', 1)


if __name__ == "__main__":
    main()
