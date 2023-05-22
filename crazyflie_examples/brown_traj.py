from crazyflie_py import Crazyswarm
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import approximate_taylor_polynomial
from crazyflie_py.uav_trajectory import Trajectory
from pathlib import Path


X_MAX = 3
Z_MAX = 3

traj_keys = ['x', 'y', 'z', 'yaw', 'duration']

def make_trajectories_b():
    #####
    # B
    #####
    trajb = []

    # Vertical Line
    trajb.append({'z': np.zeros(8), 'duration': 2})
    trajb[0]['z'] += np.array([0, 1, 0, 0, 0, 0, 0, 0])

    # Semi Circle Downwards
    num_subdivisions = 8
    for theta in np.linspace(0, np.pi, num_subdivisions, endpoint=False):
        # Get Polynomial approximation of circle at each interval 
        taylor_z = approximate_taylor_polynomial(lambda x: -(1 - np.cos(x))/2 + 2, theta, degree=7, scale=1)
        taylor_x = approximate_taylor_polynomial(lambda x: (np.sin(x))/2, theta, degree=7, scale=1)
        # We need the coefficients in ascending order, not descending
        coefs_z = list(reversed(taylor_z.coefficients))
        coefs_x = list(reversed(taylor_x.coefficients))
        trajb.append({'x': coefs_x, 'z': coefs_z, 'duration': (np.pi) / num_subdivisions})

    for theta in np.linspace(0, np.pi, num_subdivisions, endpoint=False):
        # Get Polynomial approximation of circle at each interval 
        taylor_z = approximate_taylor_polynomial(lambda x: -(1 - np.cos(x))/2 + 1, theta, degree=7, scale=1)
        taylor_x = approximate_taylor_polynomial(lambda x: (np.sin(x)), theta, degree=7, scale=1)
        # We need the coefficients in ascending order, not descending
        coefs_z = list(reversed(taylor_z.coefficients))
        coefs_x = list(reversed(taylor_x.coefficients))
        trajb.append({'x': coefs_x, 'z': coefs_z, 'duration': (np.pi) / num_subdivisions})
    # Format accordingly...
    all_trajectories = []
    for traj in trajb:
        curr_traj = np.zeros(33)
        # Duration is required...
        curr_traj[0] = traj['duration']

        # If given x/y/z/yaw polynomials add them, else they remain 0
        if 'x' in traj:
            curr_traj[1:9] = traj['x']
        if 'y' in traj:
            curr_traj[9:17] = traj['y']
        if 'z' in traj:
            curr_traj[17:25] = traj['z']
        if 'yaw' in traj:
            curr_traj[25, 33] = traj['yaw']
        all_trajectories.append(curr_traj)
        print(all_trajectories[-1])
    all_trajectories = np.array(all_trajectories)
    np.savetxt('/home/eric/ros2_ws/src/crazyswarm2/crazyflie_examples/crazyflie_examples/data/B.csv', all_trajectories, delimiter=',', header='Duration,x^0,x^1,x^2,x^3,x^4,x^5,x^6,x^7,y^0,y^1,y^2,y^3,y^4,y^5,y^6,y^7,z^0,z^1,z^2,z^3,z^4,z^5,z^6,z^7,yaw^0,yaw^1,yaw^2,yaw^3,yaw^4,yaw^5,yaw^6,yaw^7')


def main():
    make_trajectories_b()
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper

    cfs = swarm.allcfs
    trajectories = Trajectory()
    trajectories.loadcsv('/home/eric/ros2_ws/src/crazyswarm2/crazyflie_examples/crazyflie_examples/data/B.csv')


    for cf in cfs.crazyflies:
        cf.uploadTrajectory(0, 0, trajectories)
    timeHelper.sleep(2)
    print("Takeoff")

    cfs.takeoff(1.0, 2.5)
    timeHelper.sleep(2.5)
    print("Starting trajectories!")
    cfs.startTrajectory(0, timescale=2.)
    timeHelper.sleep(trajectories.duration * 2)

    cfs.land(0.04, 2.5)
    timeHelper.sleep(2.5)

if __name__ == '__main__':
    main()
