import numpy as np
import matplotlib.pyplot as plt
from obstacles import Square, Circle
from robot import solve_coeffs, generate_path
from plot_utils import *

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

# Start and end times
ts = 0
tf = len(knots) - 1

# Generate paths between all knot points with a time interval of 1
path = []
for t in range(tf):
    # Calculate coefficients for 3rd order joint trajectories
    c1 = solve_coeffs(t, t+1, knots[t][0], knots[t+1][0])
    c2 = solve_coeffs(t, t+1, knots[t][1], knots[t+1][1])
    # Calculate 100 points along trajectory during time interval
    path += generate_path(t, t+1, c1, c2)

# Plot everything in cartesian space
plt.figure(1)
plot_obstacles([A,B,C,D])
plot_path_fk(path)
plot_knots_fk(knots)
plt.title('Cartesian Space')
plt.xlabel('X')
plt.ylabel('Y')
plt.grid(True)
plt.axis('equal')
plt.savefig("../figures/cart.png")

# Plot everything in joint space
plt.figure(2)
plot_obstacles_ik([A,B,C,D])
plot_path(path)
plot_knots(knots)
plt.title('Joint Space')
plt.xlabel('Theta 1 (Degrees)')
plt.ylabel('Theta 2 (Degrees)')
plt.grid(True)
plt.axis('equal')
plt.savefig("../figures/joint.png")

# Plot theta 1 as a function of time
t = np.linspace(ts, tf, len(path))
plt.figure(3)
path = np.array(path)
plt.scatter(t, path[:,0])
plt.title('Theta 1 vs Time')
plt.xlabel('Time')
plt.ylabel('Theta 1 (Degrees)')
plt.grid(True)
plt.savefig("../figures/theta1.png")

# Plot theta 2 as a function of time
plt.figure(4)
path = np.array(path)
plt.scatter(t, path[:,1])
plt.title('Theta 2 vs Time')
plt.xlabel('Time')
plt.ylabel('Theta 2 (Degrees)')
plt.grid(True)
plt.savefig("../figures/theta2.png")

plt.show()
