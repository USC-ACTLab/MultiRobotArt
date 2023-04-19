from crazyflie_py.crazyflie_py import Crazyswarm
from generate_trajectory import generate_trajectory
from curves_to_trajectory.curves import *
# from crazyflie_py.crazyflie_py import plot_trajectory
from generate_trajectory import plot_trajectory


# noinspection PyShadowingNames
def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # TODO: Edit the following line to change the curve
    fx, fz = rose(1, 5, 4, 1)

    # Method 1: Generate the trajectory from the curve
    print("Generating Position Data")
    generate_trajectory.generate_position_data(fx=fx, fz=fz,
                                               domain=(0, 8 * np.pi),
                                               output='test.csv')
    print("Computing trajectory")
    traj = generate_trajectory.generate_trajectory_from_file('test.csv',
                                                             num_pieces=10,
                                                             approx=False)
    traj.savecsv('traj.csv')

    # Method 2: Load the trajectory from a file
    # traj = Trajectory()
    # traj.loadcsv('traj.csv')

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
