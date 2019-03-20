import numpy as np
import matplotlib.pyplot as plt
from obstacles import Square, Circle

''' Retrieves points from all obstacles '''
def get_all_points(obstacles):
    # Store points from all obstacles
    points = []
    for obstacle in obstacles:
        obs_points = obstacle.get_points()
        if type(points) != np.ndarray: # Check if no points yet
            points = obs_points
        else: # Concatenate to existing array
            points = np.concatenate((points, obs_points))
    return points

''' Plots points from all obstacles '''
def plot_obstacles(obstacles):
    plt.grid(True, linestyle='-', linewidth=1)
    for obstacle in obstacles:
        plot_obstacle(obstacle)
    plt.axis('equal')
    plt.show()

''' Plots points from a given obstacle '''
def plot_obstacle(obstacle):
    points = obstacle.get_points()
    plt.scatter(points[:,0], points[:,1])


# Populate world with obstacles
A = Square(center=(-1.25,0.625), length=0.4)
B = Circle(center=(-1.625,-0.3), radius=0.25)
C = Circle(center=(0.75,0), radius=0.125)
D = Circle(center=(1.125,0), radius=0.125)
plot_obstacles([A,B,C,D])
