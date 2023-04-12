from crazyflie_py import Crazyswarm
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import linear_sum_assignment


POS_B = np.array([(0, 0), (0, 10), (0, 20), (0, 30), (0, 40), (8, 38), (8, 20), (8, 2), (15, 10), (12.5, 15), (12.5, 5), (12.5, 25), (12.5, 35), (15, 30)])


POS_R = np.array([
            (0, 0), (0, 0.25), (0, 0.5), (0,0.75), (0, 1), (0.22, 1), (0.44, 1), (0.5, 0.75), (0.44, 0.55),
            (0.16, 0.45), (0.483, 0.15), (0.44, 0.3), (0.33, 0.45), (0.5, 0)
])

POS_O = np.array([[ 1.00000000e+00,  0.00000000e+00],
 [ 8.85456026e-01,  4.64723172e-01],
 [ 5.68064747e-01,  8.22983866e-01],
 [ 1.20536680e-01,  9.92708874e-01],
 [-3.54604887e-01,  9.35016243e-01],
 [-7.48510748e-01,  6.63122658e-01],
 [-9.70941817e-01,  2.39315664e-01],
 [-9.70941817e-01, -2.39315664e-01],
 [-7.48510748e-01, -6.63122658e-01],
 [-3.54604887e-01, -9.35016243e-01],
 [ 1.20536680e-01, -9.92708874e-01],
 [ 5.68064747e-01, -8.22983866e-01],
 [ 8.85456026e-01, -4.64723172e-01],
 [ 1.00000000e+00, -2.44929360e-16]])

POS_W = np.array([[0.,     1.    ],
 [0.4,    1.    ],
 [0.8,    1.    ],
 [0.2,    0.    ],
 [0.6,    0.    ],
 [0.05, 0.75  ],
 [0.1, 0.5  ],
 [0.15, 0.25  ],
 [0.3,  0.5  ],
 [0.49, 0.5  ],
 [0.65,  0.25  ],
 [0.7,  0.5],
 [0.746, 0.75],
 [0.746, 0.75]]
)

POS_N = np.array([[0.,   0.  ],
 [0.,   1.  ],
 [0.5,  0.  ],
 [0.5,  1.  ],
 [0.,   0.25],
 [0.,  0.5 ],
 [0.,   0.75],
 [0.5,  0.25],
 [0.5,  0.5 ],
 [0.5,  0.75],
 [0.1,  0.8 ],
 [0.2,  0.6 ],
 [0.3,  0.4 ],
 [0.4,  0.2 ]])

Z_MAX = 2
X_MAX = 2

def check_points(pos):
    plt.scatter(pos[:, 0], pos[:, 1])
    plt.xlim(pos.min(), pos.max())
    plt.ylim(pos.min(), pos.max())
    plt.show()

def to_3d(pos):
    max_x = pos[:, 0].max()
    min_x = pos[:, 0].min()
    max_z = pos[:, 1].max()
    min_z = pos[:, 1].min()
    positions = []
    for p in pos:
        x = X_MAX * p[0] / (max_x - min_x)
        z = (Z_MAX - 0.25) * (p[1] - min_z) / (max_z-min_z) + 0.25
        y = 0.2 * z
        positions.append((x, y, z))

    return np.array(positions)

def distance_matrix(points1, points2):
    return np.array([[np.linalg.norm(p1 - p2) for p1 in points1] for p2 in points2])

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper

    cfs = swarm.allcfs
    # [cf.disableCollisionAvoidance() for cf in cfs.crazyflies]
    scaled_b = to_3d(POS_B)
    scaled_b[:, 0] *= 0.5
    # check_points(POS_B)
    scaled_r = to_3d(POS_R)
    scaled_o = to_3d(POS_O)
    scaled_w = to_3d(POS_W)
    scaled_n = to_3d(POS_N)
    
    # Find minimum distance matchings for positions
    init_positions = np.array([cf.initialPosition for cf in cfs.crazyflies])
    
    dists_start_b = distance_matrix(init_positions, scaled_b)
    print(linear_sum_assignment(dists_start_b))
    scaled_b = scaled_b[linear_sum_assignment(dists_start_b)[1]]
    print(scaled_b)

    dists_b_r = distance_matrix(scaled_b, scaled_r)
    scaled_r = scaled_r[linear_sum_assignment(dists_b_r)[1]]
    print(scaled_r)

    dists_r_o = distance_matrix(scaled_r, scaled_o)
    scaled_o = scaled_o[linear_sum_assignment(dists_r_o)[1]]
    
    dists_o_w = distance_matrix(scaled_o, scaled_w)
    scaled_w = scaled_w[linear_sum_assignment(dists_o_w)[1]]
    
    dists_w_n = distance_matrix(scaled_w, scaled_n)
    scaled_n = scaled_n[linear_sum_assignment(dists_w_n)[1]]


    print("get in position!")
    cfs.takeoff(1., duration=2.5)
    print("Taking off...")
    timeHelper.sleep(2.5)

    for cf, p in zip(cfs.crazyflies, scaled_b):
        cf.goTo(goal=p, yaw=0, duration=3)
    print("B")
    timeHelper.sleep(5)

    for cf, p in zip(cfs.crazyflies, scaled_r):
        cf.goTo(goal=p, yaw=0, duration=3)
    print("R")
    timeHelper.sleep(5)

    for cf, p in zip(cfs.crazyflies, scaled_o):
        cf.goTo(goal=p, yaw=0, duration=3)
    print("O")
    timeHelper.sleep(5)
    for cf, p in zip(cfs.crazyflies, scaled_w):
        cf.goTo(goal=p, yaw=0, duration=3)
    print("W")
    timeHelper.sleep(5)
    for cf, p in zip(cfs.crazyflies, scaled_n):
        cf.goTo(goal=p, yaw=0, duration=3)
    print("N")
    timeHelper.sleep(5)

    # Land... Turns out to be a bit tricky...
    for cf in cfs.crazyflies:
        init_p = np.array(cf.initialPosition)
        cf.goTo(init_p + np.array([0., 0., 1.]), yaw=0, duration=5)
    print("returning to init positions")
    timeHelper.sleep(5)
    print("Landing")
    cfs.land(0.04, duration=2)

if __name__ == '__main__':
    main()
