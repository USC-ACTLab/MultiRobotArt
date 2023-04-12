#!/usr/bin/env python

import numpy as np
from scipy.interpolate import approximate_taylor_polynomial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def visualize_trajectory(trajectory):
  ax = plt.figure().add_subplot(projection='3d')
  cumulative_time = 0
  for i, poly in enumerate(trajectory.polynomials):
    d = poly.duration
    time_segment = np.linspace(cumulative_time, cumulative_time + d, int(10 * d))
    print(cumulative_time)
    cumulative_time += d
    output = [poly.eval(t) for t in time_segment]
    x = [o.pos[0] for o in output]
    y = [o.pos[1] for o in output]
    z = [o.pos[2] for o in output]
    ax.plot(x, y, z, label=i)
  ax.legend()
  plt.show()

def get_values_of_functions(f_x=None, f_y=None, f_z=None, f_yaw=None, domain=(0, 1)):
  t_vals = np.linspace(*domain, int(1000*(domain[1] - domain[0])))
  # Get the ground truth values
  if not (f_x is None):
    x_vals = f_x(t_vals)
  else:
    x_vals = np.zeros_like(t_vals)
  if not (f_y is None):
    y_vals = f_y(t_vals)
  else:
    y_vals = np.zeros_like(t_vals)
  if not (f_z is None):
    z_vals = f_z(t_vals)
  else:
    z_vals = np.zeros_like(t_vals)
  if not (f_yaw is None):
    yaw_vals = f_yaw(t_vals)
  else:
    yaw_vals = np.zeros_like(t_vals)

  return x_vals, y_vals, z_vals, yaw_vals, t_vals

  
def find_breakpoints(fx=None, fy=None, fz=None, fyaw=None, domain=(0, 1)):
  if fx is None:
    f_x = lambda x: np.zeros_like(x)
  else:
    f_x = lambda x: fx(x) - fx(domain[0])
  if fy is None:
    f_y = lambda x: np.zeros_like(x)
  else:
    f_y = lambda y: fy(y) - fy(domain[0])
  if fz is None:
    f_z = lambda x: np.zeros_like(x)
  else:
    f_z = lambda z: fz(z) - fz(domain[0])
  if fyaw is None:
    f_yaw = lambda x: np.zeros_like(x)
  else:
    f_yaw = lambda yaw: fyaw(yaw) - fyaw(domain[0])
  x, y, z, yaw, t = get_values_of_functions(f_x=f_x, f_y=f_y, f_z=f_z, f_yaw=f_yaw, domain=domain)
  n_breakpoints = 16 # n segments = n_breakpoints + 1
  best_error = 1e10
  # trials for uniform guesses, take the best outcome
  for i in range(100):
    breaks = np.random.uniform(*domain, n_breakpoints)
    breaks = np.sort(breaks)
    breaks = np.append(breaks, domain[1])
    breaks = np.insert(breaks, 0, domain[0])
    tp_x, tp_y, tp_z, tp_yaw = [], [], [], []
    error = 0
    for i in range(1, len(breaks)):
      midpoint = breaks[i - 1] + (breaks[i] - breaks[i-1]) / 2
      segment = np.where(np.logical_and(t > breaks[i-1], t < breaks[i]))[0]
      if len(segment) <= 8:
          continue
      approx_x = np.polynomial.polynomial.Polynomial.fit(t[segment], x[segment], deg=7).convert()
      approx_y = np.polynomial.polynomial.Polynomial.fit(t[segment], y[segment], deg=7).convert()
      approx_z = np.polynomial.polynomial.Polynomial.fit(t[segment], z[segment], deg=7).convert()
      approx_yaw = np.polynomial.polynomial.Polynomial.fit(t[segment], yaw[segment], deg=7).convert()
      tp_x.append(approx_x)
      tp_y.append(approx_y)
      tp_z.append(approx_z)
      tp_yaw.append(approx_yaw)
      error += np.linalg.norm(approx_x(t[segment]) - x[segment])
      error += np.linalg.norm(approx_y(t[segment]) - y[segment])
      error += np.linalg.norm(approx_z(t[segment]) - z[segment])
      error += np.linalg.norm(approx_yaw(t[segment]) - yaw[segment])
    if (len(tp_x) != n_breakpoints + 1):
        continue
    if error < best_error:
      best_error = error
      best_x = tp_x
      best_y = tp_y
      best_z = tp_z
      best_yaw = tp_yaw
      breakpoints = breaks
    if best_error == 0:
        break
  return best_x, best_y, best_z, best_yaw, breakpoints



def auto_trajectory(f_x=None, f_y=None, f_z=None, f_yaw=None, domain=(0, 1)):
  """
  :param f_x: callable function f_x(t), which maps time to x-position
  :param f_y: callable function f_y(t), which maps time to y-position
  :param f_z: callable function f_z(t), which maps time to z-position
  :param f_yaw: callable function f_yaw(t), which maps time to yaw
  :param domain: the values that t takes on in the given functions
  :param num_trajectories: the number of trajectories to divide the approximations into (the more trajectories, the more accurate the approximation)

  :return: returns num_trajectories Trajectory objects that attempt to closely the original functions given.
  """
  # TODO: make better than uniform random...

  assert domain[1] > domain[0]

  approx_x, approx_y, approx_z, approx_yaw, breaks = find_breakpoints(f_x, f_y, f_z, f_yaw, domain)
  print(breaks)
  trajectory = Trajectory()
  trajectory.polynomials = []
  for i, (x, y, z, yaw, duration) in enumerate(zip(approx_x, approx_y, approx_z, approx_yaw, breaks)):
    x = np.array(list(x.coef))
    y = np.array(list(y.coef))
    z = np.array(list(z.coef))
    yaw = np.array(list(yaw.coef))
    while len(x) < 8:
        x = np.append(x, 0)
    while len(y) < 8:
        y = np.append(y, 0)
    while len(z) < 8:
        z = np.append(z, 0)
    while len(yaw) < 8:
        yaw = np.append(yaw, 0)
    poly = Polynomial4D(breaks[i+1] - breaks[i], x, y, z, yaw)
    print("duration", breaks[i+1] - breaks[i])
    trajectory.polynomials.append(poly)

  trajectory.duration = domain[1] - domain[0]
  print(domain[1] - domain[0])
  visualize_trajectory(trajectory)
  return trajectory

def normalize(v):
  norm = np.linalg.norm(v)
  assert norm > 0
  return v / norm


class Polynomial:
  def __init__(self, p):
    self.p = p

  # evaluate a polynomial using horner's rule
  def eval(self, t):
    assert t >= 0
    x = 0.0
    for i in range(0, len(self.p)):
      x = x * t + self.p[len(self.p) - 1 - i]
    return x

  # compute and return derivative
  def derivative(self):
    return Polynomial([(i+1) * self.p[i+1] for i in range(0, len(self.p) - 1)])


class TrajectoryOutput:
  def __init__(self):
    self.pos = None   # position [m]
    self.vel = None   # velocity [m/s]
    self.acc = None   # acceleration [m/s^2]
    self.omega = None # angular velocity [rad/s]
    self.yaw = None   # yaw angle [rad]


# 4d single polynomial piece for x-y-z-yaw, includes duration.
class Polynomial4D:
  def __init__(self, duration, px, py, pz, pyaw):
    self.duration = duration
    self.px = Polynomial(px)
    self.py = Polynomial(py)
    self.pz = Polynomial(pz)
    self.pyaw = Polynomial(pyaw)

  # compute and return derivative
  def derivative(self):
    return Polynomial4D(
      self.duration,
      self.px.derivative().p,
      self.py.derivative().p,
      self.pz.derivative().p,
      self.pyaw.derivative().p)

  def eval(self, t):
    result = TrajectoryOutput()
    # flat variables
    result.pos = np.array([self.px.eval(t), self.py.eval(t), self.pz.eval(t)])
    result.yaw = self.pyaw.eval(t)

    # 1st derivative
    derivative = self.derivative()
    result.vel = np.array([derivative.px.eval(t), derivative.py.eval(t), derivative.pz.eval(t)])
    dyaw = derivative.pyaw.eval(t)

    # 2nd derivative
    derivative2 = derivative.derivative()
    result.acc = np.array([derivative2.px.eval(t), derivative2.py.eval(t), derivative2.pz.eval(t)])

    # 3rd derivative
    derivative3 = derivative2.derivative()
    jerk = np.array([derivative3.px.eval(t), derivative3.py.eval(t), derivative3.pz.eval(t)])

    thrust = result.acc + np.array([0, 0, 9.81]) # add gravity

    z_body = normalize(thrust)
    x_world = np.array([np.cos(result.yaw), np.sin(result.yaw), 0])
    y_body = normalize(np.cross(z_body, x_world))
    x_body = np.cross(y_body, z_body)

    jerk_orth_zbody = jerk - (np.dot(jerk, z_body) * z_body)
    h_w = jerk_orth_zbody / np.linalg.norm(thrust)

    result.omega = np.array([-np.dot(h_w, y_body), np.dot(h_w, x_body), z_body[2] * dyaw])
    return result


class Trajectory:
  def __init__(self):
    self.polynomials = None
    self.duration = None

  def n_pieces(self):
    return len(self.polynomials)

  def loadcsv(self, filename):
    data = np.loadtxt(filename, delimiter=",", skiprows=1, usecols=range(33))
    self.polynomials = [Polynomial4D(row[0], row[1:9], row[9:17], row[17:25], row[25:33]) for row in data]
    self.duration = np.sum(data[:,0])

  def savecsv(self, filename):
    data = np.empty((len(self.polynomials), 8*4+1))
    for i, p in enumerate(self.polynomials):
      data[i,0] = p.duration
      data[i,1:9] = p.px.p
      data[i,9:17] = p.py.p
      data[i,17:25] = p.pz.p
      data[i,25:33] = p.pyaw.p
    np.savetxt(filename, data, fmt="%.6f", delimiter=",", header="duration,x^0,x^1,x^2,x^3,x^4,x^5,x^6,x^7,y^0,y^1,y^2,y^3,y^4,y^5,y^6,y^7,z^0,z^1,z^2,z^3,z^4,z^5,z^6,z^7,yaw^0,yaw^1,yaw^2,yaw^3,yaw^4,yaw^5,yaw^6,yaw^7")


  def eval(self, t):
    assert t >= 0
    assert t <= self.duration

    current_t = 0.0
    for p in self.polynomials:
      if t <= current_t + p.duration:
        return p.eval(t - current_t)
      current_t = current_t + p.duration

