import matplotlib.pyplot as plt

from generate_trajectory.generate_trajectory import *
from generate_trajectory.plot_utils import *
from curves_to_trajectory.curves import *


def generate_test_data():
    fx, fy, fz, domain = helix((-1, -1), 8, 0.2)
    print("Generating position data")
    data = generate_position_data(fx=fy, fy=fz, fz=fx, domain=domain, output='pos.csv')
    plot_pos(data)
    print("Generating trajectory")
    traj = generate_trajectory_from_file('pos.csv', num_pieces=24, approx=False)
    traj.savecsv('traj.csv')
    print("Plotting")
    plot_traj(traj)


if __name__ == '__main__':
    generate_test_data()
