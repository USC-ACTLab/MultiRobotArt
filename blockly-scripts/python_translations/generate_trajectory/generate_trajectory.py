#!/usr/bin/env python

import argparse
import numpy as np
import scipy.optimize
from .uav_trajectory import Polynomial4D, Trajectory


# computes the difference between current interpolation and desired values
def func(coefficients, times, values, piece_length):
    result = 0
    i = 0
    for t, value in zip(times, values):
        if t > (i + 1) * piece_length:
            i = i + 1
        estimate = np.polyval(coefficients[i * 8:(i + 1) * 8],
                              t - i * piece_length)
        # print(coefficients[i*8:(i+1)*8], t - i * piece_length, estimate)
        result += (value - estimate) ** 2  # np.sum((values - estimates) ** 2)
    # print(coefficients, result)
    return result


# constraint to match values between spline pieces
# def func_eq_constraint_val(coefficients, i, piece_length):
#     result = 0
#     end_val = np.polyval(coefficients[(i - 1) * 8:i * 8], piece_length)
#     start_val = np.polyval(coefficients[i * 8:(i + 1) * 8], 0)
#     return end_val - start_val


def func_eq_constraint_der(coefficients, i, piece_length, order):
    result = 0
    last_der = np.polyder(coefficients[(i - 1) * 8:i * 8], order)
    this_der = np.polyder(coefficients[i * 8:(i + 1) * 8], order)

    end_val = np.polyval(last_der, piece_length)
    start_val = np.polyval(this_der, 0)
    return end_val - start_val


def func_eq_constraint_der_value(coefficients, i, t, desired_value, order):
    result = 0
    der = np.polyder(coefficients[i * 8:(i + 1) * 8], order)

    value = np.polyval(der, t)
    return value - desired_value


# def func_eq_constraint(coefficients, tss, yawss):
#   result = 0
#   last_derivative = None
#   for ts, yaws, i in zip(tss, yawss, range(0, len(tss))):
#     derivative = np.polyder(coefficients[i*8:(i+1)*8])
#     if last_derivative is not None:
#       result += np.polyval(derivative, 0) - last_derivative
#     last_derivative = np.polyval(derivative, tss[-1])
#
#
# # apply coefficients to trajectory
# for i,p in enumerate(traj.polynomials):
#   p.pyaw.p = coefficients[i*8:(i+1)*8]
# # evaluate at each timestep and compute the sum of squared differences
# result = 0
# for t,yaw in zip(ts,yaws):
#   e = traj.eval(t)
#   result += (e.yaw - yaw) ** 2
# return result

def generate_trajectory_from_file(filename, num_pieces=5, approx=False):
    data = np.loadtxt(filename, delimiter=',', skiprows=1)
    return generate_trajectory(data, num_pieces, approx)


def generate_trajectory(data, num_pieces, approx=False):
    piece_length = data[-1, 0] / num_pieces
    # print(piece_length)
    x0 = np.zeros(num_pieces * 8)

    constraints = []
    # piecewise values and derivatives have to match
    for i in range(1, num_pieces):
        for order in range(0, 4):
            if approx:
                constraints.append({'type': 'ineq',
                                    'fun': lambda coef, i, p, l: np.abs(
                                        func_eq_constraint_der(coef, i, p,
                                                               l)) - 0.0005,
                                    'args': (i, piece_length, order)})
                # constraints.append({'type': 'ineq', 'fun': lambda coef, i, p, l: -func_eq_constraint_der(coef, i, p, l) - 0.0005, 'args': (i, piece_length, order)})
                # constraints.append({'type': 'eq', 'fun': func_eq_constraint_der, 'ub': 0.005, 'lb': 0.005, 'args': (i, piece_length, order)})
            else:
                constraints.append({'type': 'eq', 'fun': func_eq_constraint_der,
                                    'args': (i, piece_length, order)})

    # zero derivative at the beginning and end
    for order in range(1, 3):
        constraints.append({'type': 'eq', 'fun': func_eq_constraint_der_value,
                            'args': (0, 0, 0, order)})
        constraints.append({'type': 'eq', 'fun': func_eq_constraint_der_value,
                            'args': (num_pieces - 1, piece_length, 0, order)})

    # print("fitting x")
    resX = scipy.optimize.minimize(func, x0,
                                   (data[:, 0], data[:, 1], piece_length),
                                   method="SLSQP", options={"maxiter": 100},
                                   constraints=constraints
                                   )
    # print("fitting y")
    resY = scipy.optimize.minimize(func, x0,
                                   (data[:, 0], data[:, 2], piece_length),
                                   method="SLSQP", options={"maxiter": 100},
                                   constraints=constraints
                                   )
    # print("fitting z")
    resZ = scipy.optimize.minimize(func, x0,
                                   (data[:, 0], data[:, 3], piece_length),
                                   method="SLSQP", options={"maxiter": 100},
                                   constraints=constraints
                                   )

    resYaw = scipy.optimize.minimize(func, x0,
                                     (data[:, 0], data[:, 4], piece_length),
                                     method="SLSQP", options={"maxiter": 100},
                                     constraints=constraints
                                     )

    traj = Trajectory()
    traj.polynomials = [Polynomial4D(
        piece_length,
        np.array(resX.x[i * 8:(i + 1) * 8][::-1]),
        np.array(resY.x[i * 8:(i + 1) * 8][::-1]),
        np.array(resZ.x[i * 8:(i + 1) * 8][::-1]),
        np.array(resYaw.x[i * 8:(i + 1) * 8][::-1])) for i in
        range(0, num_pieces)]
    traj.duration = data[-1, 0]
    return traj


def constant_traj(t):
    return 0


def generate_position_data(fx=constant_traj, fy=constant_traj, fz=constant_traj,
                           fyaw=constant_traj, domain=(0, 1), output='test.csv'):
    t = np.linspace(*domain, int(
        domain[1] - domain[0]) * 20 + 1)  # 20 points per second to fit
    data = []
    with open(output, 'w') as f:
        f.write("t,x,y,z,yaw\n")

        for i in t:
            step = (i, fx(i) - fx(domain[0]), fy(i) - fy(domain[0]), fz(i) - fz(domain[0]), fyaw(i) - fyaw(domain[0]))
            f.write("{},{},{},{},{}\n".format(*step))
            data.append(step)
    return data


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=str,
                        help="CSV file containing time waypoints")
    parser.add_argument("output", type=str,
                        help="CSV file containing trajectory with updated yaw")
    parser.add_argument("--pieces", type=int, default=5,
                        help="number of pieces")
    args = parser.parse_args()

    data = np.loadtxt(args.input, delimiter=',', skiprows=1)
    traj = generate_trajectory(data, args.pieces)
    traj.savecsv(args.output)
