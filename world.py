import numpy as np
import matplotlib.pyplot as plt
from obstacles import Square, Circle
from robot import robot_ik, robot_fk, solve_coeffs, generate_path
from math import pi
import pdb


def get_all_points(obstacles):
    ''' Retrieves points from all obstacles '''
    # Store points from all obstacles
    points = []
    for obstacle in obstacles:
        obs_points = obstacle.get_points()
        if type(points) != np.ndarray: # Check if no points yet
            points = obs_points
        else: # Concatenate to existing array
            points = np.concatenate((points, obs_points))
    return points


def plot_obstacles(obstacles):
    ''' Plots points from all obstacles '''
    for obstacle in obstacles:
        plot_obstacle(obstacle)
    obs_labels_space()


def plot_obstacle(obstacle):
    ''' Plots points from a given obstacle '''
    points = obstacle.get_points()
    plt.scatter(points[:,0], points[:,1])


def plot_obstacles_ik(obstacles):
    ''' Plots points from all obstacles in joint space '''
    for obstacle in obstacles:
        plot_obstacle_ik(obstacle)
    obs_labels_joint()


def plot_obstacle_ik(obstacle):
    ''' Plots points from a given obstacle in joint space '''
    points = obstacle.get_points()
    points_ik = []
    for point in points:
        points_ik.append(robot_ik(point))
    points_ik = np.array(points_ik)
    plt.scatter(points_ik[:,0], points_ik[:,1])

def plot_path(path):
    path = np.array(path)
    plt.scatter(path[:,0],path[:,1])

def plot_path_fk(path):
    path = np.array(path)
    fk_path = []
    for point in path:
        fk_path.append(robot_fk(point))
    fk_path = np.array(fk_path)
    plt.scatter(fk_path[:,0], fk_path[:,1])

def obs_labels_space():
    plt.text(-1.25,0.625, "A", horizontalalignment='center', verticalalignment='center')
    plt.text(-1.625,-0.3, "B", horizontalalignment='center', verticalalignment='center')
    plt.text(0.75,0, "C", horizontalalignment='center', verticalalignment='center')
    plt.text(1.125,0, "D", horizontalalignment='center', verticalalignment='center')

def obs_labels_joint():
    plt.text(200,268, "A", horizontalalignment='center', verticalalignment='center')
    plt.text(224,291, "B", horizontalalignment='center', verticalalignment='center')
    plt.text(68,224, "C", horizontalalignment='center', verticalalignment='center')
    plt.text(56,248, "D", horizontalalignment='center', verticalalignment='center')

# Populate world with obstacles
A = Square(center=(-1.25,0.625), length=0.4)
B = Circle(center=(-1.625,-0.3), radius=0.25)
C = Circle(center=(0.75,0), radius=0.125)
D = Circle(center=(1.125,0), radius=0.125)

# Define knot points
knots = [[30,  250], 
         [50,  230],
         [90,  250],
         [235, 235],
         [180, 360]]

path = []
ts = 0
tf = len(knots) - 1

for t in range(tf):
    c1 = solve_coeffs(t, t+1, knots[t][0], knots[t+1][0])
    c2 = solve_coeffs(t, t+1, knots[t][1], knots[t+1][1])
    path += generate_path(t, t+1, c1, c2)

# c1 = solve_coeffs(0, 1, 30, 50)
# c2 = solve_coeffs(0, 1, 250, 230)
# path += generate_path(0, 1, c1, c2)

# c1 = solve_coeffs(1, 2, 50, 90)
# c2 = solve_coeffs(1, 2, 230, 250)
# path += generate_path(1, 2, c1, c2)

# c1 = solve_coeffs(2, 3, 90, 235)
# c2 = solve_coeffs(2, 3, 250, 235)
# path += generate_path(2, 3, c1, c2)

# c1 = solve_coeffs(3, 4, 235, 180)
# c2 = solve_coeffs(3, 4, 235, 360)
# path += generate_path(3, 4, c1, c2)

# plot_obstacles([A,B,C,D])
# plot_path_fk(knots)
# plot_path_fk(path)

plot_obstacles_ik([A,B,C,D])
plot_path(knots)
plot_path(path)


plt.grid(True)
plt.axis('equal')
plt.show()
