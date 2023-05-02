import numpy as np
import matplotlib.pyplot as plt


def plot_curve(fx, fy, flight_time):
    t = np.linspace(0, flight_time, 10000)
    x = fx(t)
    y = fy(t)
    fig, ax = plt.subplots()
    plt.scatter(x[0], y[0], c='b', s=100)
    ax.plot(x, y)
    ax.grid(True)
    ax.set_aspect('equal')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    plt.show()


def plot_curve_3d(fx, fy, fz, flight_time):
    t = np.linspace(0, flight_time, 10000)
    x = fx(t)
    y = fy(t)
    z = fz(t)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x[0], y[0], z[0], c='b', s=100)
    ax.plot(x, y, z)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()


# Circle that change velocities in two directions
# TODO: what happens if angle_degrees is not a multiple of 360?
# This implementation doesn't start at (0, 0), but at (x0, y0 - r)
def circle_facing_constant(center, flight_time, angle_degrees=360.0,
    clockwise=True):
    if flight_time <= 0:
        raise ValueError("flight_time must be positive")
    x0, y0 = center
    r = np.sqrt(x0 ** 2 + y0 ** 2)

    angle_radians = angle_degrees / 360.0 * 2 * np.pi
    sign = 1 if clockwise else -1

    x_fun = lambda t: np.sin(t / flight_time * angle_radians) * r + x0
    y_fun = lambda t: np.cos(t / flight_time * angle_radians) * r + sign * y0

    # Display the curve:
    # plt.scatter(x0, y0, c='r', s=100)
    plot_curve(x_fun, y_fun, flight_time)

    distance = r * angle_radians
    velocity = distance / flight_time

    domain = (0, flight_time)

    return x_fun, y_fun, domain


# Drone will start in position (0,0) and end up in position(distance, 0)
# Amplitude can be negative, indicating reverse direction
# Distance can be negative, indicating reverse direction
# Cycles can be a fraction (preferably a multiple of 0.5), indicating a partial cycle
def sine(amplitude, flight_time, distance, cycles=1):
    if cycles <= 0:
        raise ValueError("cycles must be positive")
    if flight_time <= 0:
        raise ValueError("flight_time must be positive")

    frequency = 1 / distance * cycles
    angular_frequency = 2 * np.pi * frequency

    x_fun = lambda t: t / flight_time * distance
    y_fun = lambda t: amplitude * np.sin(
        t / flight_time * angular_frequency * distance)
    plot_curve(x_fun, y_fun, flight_time)

    velocity = distance / flight_time
    domain = (0, flight_time)
    return x_fun, y_fun, domain


# a defines the radius of the rose.
# https://en.wikipedia.org/wiki/Rose_(mathematics)
# Make rose return a var called Domain to adjust the time
# start_center defines if the drone starts at the center of the rose or at the edge, True on default
def rose(a, n, d, flight_time, start_center=True):
    # TODO: test speed boundaries
    param = d * 2 * np.pi / flight_time
    k = n / d
    x_fun = lambda t: a * np.cos(k * param * t) * np.cos(param * t)
    y_fun = lambda t: a * np.cos(k * param * t) * np.sin(param * t)
    # plot_curve(x_fun, y_fun, flight_time)
    start_time = flight_time / (n * 4) if start_center else 0
    domain = (start_time, start_time + flight_time)
    return x_fun, y_fun, domain


# In polar coordinates, spirals are represented as r = a + b * theta
# r_max denotes the maximum distance, i.e. the drone will end up at (r_max * cos(r_max/b), r_max * sin(r_max/b))
def spiral(b, r_max, flight_time, clockwise=False):
    factor = -1 if clockwise else 1
    x_fun = lambda t: r_max * t / flight_time * np.cos(
        r_max * t / flight_time / b) * factor
    y_fun = lambda t: r_max * t / flight_time * np.sin(
        r_max * t / flight_time / b)
    domain = (0, flight_time)
    plot_curve(x_fun, y_fun, flight_time)
    return x_fun, y_fun, domain


def helix(center, flight_time, speed_z, angle_degrees=360, clockwise=True):
    x_fun, y_fun = circle_facing_constant(center, flight_time,
                                          angle_degrees=angle_degrees,
                                          clockwise=clockwise)
    z_fun = lambda t: speed_z * t

    plot_curve_3d(x_fun, y_fun, z_fun, flight_time)
    return x_fun, y_fun, z_fun


if __name__ == '__main__':
    pass
