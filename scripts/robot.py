import numpy as np
from math import sin, cos, asin, acos, pi, atan2, sqrt

def rad2deg(rad):
    if rad > 0:
        return rad * (180/pi)
    else:
        return rad * (180/pi) + 360
    return rad * (180/pi)

def robot_ik(coord):
    px, py = coord

    # Compute theta 2
    D = (px**2 + py**2 - 1 - 1) / 2
    theta2 = atan2(-sqrt(1 - (D**2)), D)

    s2 = sin(theta2)
    c2 = cos(theta2)

    # Compute theta 1
    theta1 = atan2(py,px) - atan2(s2,(1 + c2))

    if theta1 < -pi:
        theta1 += 2*pi
    elif theta1 > pi:
        theta1 -= 2*pi
    if theta2 < -pi:
        theta2 += 2*pi
    elif theta2 > pi:
        theta2 -= 2*pi

    return [rad2deg(theta1), rad2deg(theta2)]

def robot_fk(state):
    theta1, theta2 = state
    theta1 *= (pi/180)
    theta2 *= (pi/180)

    # Compute cartesian coordinate of end effector using joint states
    x = 1 * cos(theta1) + 1 * cos(theta1 + theta2)
    y = 1 * sin(theta1) + 1 * sin(theta1 + theta2)

    return [x,y]

def solve_coeffs(ts, tf, ps, pf):
    T = np.array([[1, ts, ts**2, ts**3],
                  [1, tf, tf**2, tf**3],
                  [0, 1,  2*ts,  3*(ts**2)],
                  [0, 1,  2*tf,  3*(tf**2)]])

    S = np.array([[ps, pf, 0, 0]])
    C = np.matmul(np.linalg.inv(T), np.transpose(S))

    print_spline(C)

    return C

def print_spline(c):
    c = np.int32(c).squeeze().tolist()
    out = str(c[0]) + " + " + str(c[1]) + "t + " + str(c[2]) + \
          "t^{2} + " + str(c[3]) + "t^{3}"
    print(out)

def generate_path(ts, tf, c1, c2, num_points=100):
    points = []
    times = np.linspace(ts, tf, num_points)
    for t in times:
        theta1 = np.sum(np.transpose(c1) * np.array([1, t, t**2, t**3]))
        theta2 = np.sum(np.transpose(c2) * np.array([1, t, t**2, t**3]))
        points.append([theta1, theta2])
    return points