from crazyflie_py import Crazyswarm


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    timeHelper.sleep(1.0)
    allcfs.takeoff(targetHeight=1., duration=2.)
    timeHelper.sleep(4.0)
    
    # Red
    for cf in allcfs.crazyflies:
        cf.setLEDColor(1, 0, 0)
    timeHelper.sleep(2.0)

    # Green
    for cf in allcfs.crazyflies:
        cf.setLEDColor(0, 1, 0)
    timeHelper.sleep(2.0)

    # Blue
    for cf in allcfs.crazyflies:
        cf.setLEDColor(0, 0, 1)
    timeHelper.sleep(2.0)

    allcfs.land(targetHeight=0.04, duration=2.0)

if __name__ == "__main__":
    main()
