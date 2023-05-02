import matplotlib.pyplot as plt

from generate_trajectory.generate_trajectory import *
from generate_trajectory.plot_utils import *
from curves_to_trajectory.curves import *

def generate_test_data():
    fx, fy, domain = circle_facing_constant((0, 1), 10)
    print("Generating position data")
    data = generate_position_data(fx=fx, fy=fy, domain=domain, output='pos.csv')
    plot_pos(data)
    # print("Generating trajectory")
    # traj = generate_trajectory_from_file('pos.csv', num_pieces=12, approx=False)
    # traj.savecsv('traj.csv')
    # print("Plotting")
    # plot_traj(traj)


if __name__ == '__main__':
    generate_test_data()
