import numpy as np
import matplotlib.pyplot as plt
from robot import robot_ik, robot_fk

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
    # Compute joint state using IK for each point
    for point in points:
        points_ik.append(robot_ik(point))
    points_ik = np.array(points_ik)
    plt.scatter(points_ik[:,0], points_ik[:,1])

def plot_path(path):
    path = np.array(path)
    plt.scatter(path[:,0],path[:,1], s=7)

def plot_knots(knots):
    path = np.array(knots)
    plt.scatter(path[:,0],path[:,1], s=100, marker='x')

def plot_path_fk(path):
    path = np.array(path)
    fk_path = []
    # Compute cartesian coordinate using FK for each point
    for point in path:
        fk_path.append(robot_fk(point))
    fk_path = np.array(fk_path)
    plt.scatter(fk_path[:,0], fk_path[:,1], s=7)

def plot_knots_fk(path):
    path = np.array(path)
    fk_path = []
    # Compute cartesian coordinate using FK for each point
    for point in path:
        fk_path.append(robot_fk(point))
    fk_path = np.array(fk_path)
    plt.scatter(fk_path[:,0], fk_path[:,1], s=100, marker='x')

def obs_labels_space():
    plt.text(1,-0.6, "Start", horizontalalignment='center', verticalalignment='center')
    plt.text(-1.25,0.625, "A", horizontalalignment='center', verticalalignment='center')
    plt.text(-1.625,-0.3, "B", horizontalalignment='center', verticalalignment='center')
    plt.text(0.75,0, "C", horizontalalignment='center', verticalalignment='center')
    plt.text(1.125,0, "D", horizontalalignment='center', verticalalignment='center')
    plt.text(-2,0.14, "End", horizontalalignment='center', verticalalignment='center')

def obs_labels_joint():
    plt.text(30,258, "Start", horizontalalignment='center', verticalalignment='center')
    plt.text(200,268, "A", horizontalalignment='center', verticalalignment='center')
    plt.text(224,291, "B", horizontalalignment='center', verticalalignment='center')
    plt.text(68,224, "C", horizontalalignment='center', verticalalignment='center')
    plt.text(56,248, "D", horizontalalignment='center', verticalalignment='center')
    plt.text(180,368, "End", horizontalalignment='center', verticalalignment='center')