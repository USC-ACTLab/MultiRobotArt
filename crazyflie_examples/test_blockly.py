from crazyflie_py import Crazyswarm
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import approximate_taylor_polynomial
from crazyflie_py.uav_trajectory import Trajectory
from crazyflie_py.generate_trajectory import *
from pathlib import Path
from crazyflie_py import plot_trajectory

def rose(a, k, flight_time):
    # delta_t is a time adjustment parameter. Without this, the function will start at (1, 0). We want it to start at (0, 0).
    delta_t = flight_time / (4 * k)
    if k % 2 == 0:
        x_fun = lambda t: a * np.cos(k * 2 * np.pi * (t - delta_t) / flight_time) * np.cos(2 * np.pi * (t - delta_t) / flight_time)
        y_fun = lambda t: a * np.cos(k * 2 * np.pi * (t - delta_t) / flight_time) * np.sin(2 * np.pi * (t - delta_t) / flight_time)
    else:
        x_fun = lambda t: a * np.cos(k * 2 * np.pi * (t / 2 - delta_t) / flight_time) * np.cos(2 * np.pi * (t / 2- delta_t) / flight_time)
        y_fun = lambda t: a * np.cos(k * 2 * np.pi * (t / 2 - delta_t) / flight_time) * np.sin(2 * np.pi * (t / 2 - delta_t) / flight_time)

    return x_fun, y_fun

def helix(r, flight_time, speed_z, angle_degrees=360):
    x_fun, y_fun = circle_facing_constant(r, flight_time, angle_degrees=angle_degrees)
    z_fun = lambda t: speed_z * t

    return x_fun, y_fun, z_fun

def circle_facing_constant(r, flight_time, angle_degrees=360.0):
    if flight_time <= 0:
        raise ValueError("flight_time must be positive")
    if r <= 0:
        raise ValueError("radius must be positive")

    angle_radians = angle_degrees / 360.0 * 2 * np.pi
    # # positive x_axis, clockwise
    # x_fun = lambda t:  - r * np.cos(t / flight_time  * angle_radians) + r
    # y_fun = lambda t: r * np.sin(t / flight_time * angle_radians)
    # center = (r, 0)

    # # negative x_axis, counter-clockwise
    # x_fun = lambda t: r * np.cos(t / flight_time  * angle_radians) - r
    # y_fun = lambda t: r * np.sin(t / flight_time * angle_radians)
    # center = (-r, 0)

    # # negative x_axis, clockwise
    # x_fun = lambda t: r * np.cos(t / flight_time  * angle_radians) - r
    # y_fun = lambda t:  - r * np.sin(t / flight_time * angle_radians)
    # center = (-r, 0)

    # # negative x_axis, counter-clockwise
    # x_fun = lambda t: - r * np.cos(t / flight_time  * angle_radians) + r
    # y_fun = lambda t:  - r * np.sin(t / flight_time * angle_radians)
    # center = (r, 0)

    # # positive y_axis, clockwise
    # x_fun = lambda t: - r * np.sin(t / flight_time * angle_radians)
    # y_fun = lambda t:  - r * np.cos(t / flight_time  * angle_radians) + r
    # center = (0, r)

    # # positive y_axis, counter-clockwise
    # x_fun = lambda t: r * np.sin(t / flight_time * angle_radians)
    # y_fun = lambda t:  - r * np.cos(t / flight_time  * angle_radians) + r
    # center = (0, r)

    # # negative y_axis, counter-clockwise
    # x_fun = lambda t: - r * np.sin(t / flight_time * angle_radians)
    # y_fun = lambda t:  r * np.cos(t / flight_time  * angle_radians) - r
    # center = (0, -r)

    # negative y_axis, clockwise
    x_fun = lambda t: r * np.sin(t / flight_time * angle_radians)
    y_fun = lambda t:  r * np.cos(t / flight_time  * angle_radians) - r
    center = (0, -r)

    # Display the curve:
    # plt.scatter(center[0], center[1], c='r', s=100)
    distance = r * angle_radians
    velocity = distance / flight_time

    return x_fun, y_fun

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    fx, fz = rose(1, 5/4, 1)
    print("Generating Position Data")
    generate_position_data(fx=fx, fz=fz, domain=(0, 8*np.pi), output='test.csv')
    print("Computing trajectory")
    traj = generate_trajectory_from_file('test.csv', num_pieces=10, approx=False)
    # traj = Trajectory()
    # traj.loadcsv('traj.csv')
    # traj.savecsv('traj.csv')
    plot_trajectory.plot(traj)
    print("Beginning CF execution")
    for cf in allcfs.crazyflies:
        cf.uploadTrajectory(0, 0, traj)
    timeHelper.sleep(2.0)
    for cf in allcfs.crazyflies:
        cf.setLEDColor(0, 1, 0)
    allcfs.takeoff(targetHeight=1.5, duration=2.)
    timeHelper.sleep(4.0)
    for cf in allcfs.crazyflies:
        cf.setLEDColor(1, 0, 0)
    allcfs.startTrajectory(0, timescale=1.0)
    timeHelper.sleep(traj.duration * 1.0 + 2.0)

    for cf in allcfs.crazyflies:
        cf.setLEDColor(0, 1, 0)
    
    allcfs.land(targetHeight=0.04, duration=2.0)

if __name__ == "__main__":
    main()