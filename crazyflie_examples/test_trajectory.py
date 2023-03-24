from crazyflie_py.uav_trajectory import *
from crazyflie_py import Crazyswarm

def main():
    x_fn = lambda x : np.cos(x/2)
    y_fn = lambda x : np.sin(x)
    trajectory = auto_trajectory(x_fn, y_fn, domain=(0, 4*np.pi))
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper

    cfs = swarm.allcfs

    for cf in cfs.crazyflies:
        cf.uploadTrajectory(0, 0, trajectory)
    timeHelper.sleep(2)
    print("Takeoff")

    cfs.takeoff(1.0, 2.5)
    timeHelper.sleep(2.5)
    print("Starting trajectories!")
    cfs.startTrajectory(0, timescale=2.)
    timeHelper.sleep(trajectory.duration * 2)

    cfs.land(0.04, 2.5)
    timeHelper.sleep(2.5)

if __name__ == '__main__':
    main()
