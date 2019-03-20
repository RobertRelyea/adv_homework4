import numpy as np
import math

DEFAULT_NUM_POINTS = 100


class Obstacle():
    '''
    Class representing an obstacle in the robot's environment.

    Members:
        center     -- Tuple containing center of obstacle in coordinate frame.
        num_points -- Integer representing the number of points for plotting.
    '''
    def __init__(self, center, num_points):
        self.center = center
        self.num_points = num_points
        self.init_points()

    ''' Initializes points for plotting '''
    def init_points(self):
        self.points = []

    ''' Returns center of obstacle in the coordinate frame. '''
    def get_center(self):
        return self.center

    ''' Returns number of points for plotting. '''
    def get_num_points(self):
        return self.num_points
    
    ''' Returns points for plotting '''
    def get_points(self):
        return self.points


class Square(Obstacle):
    '''
    Class representing a square obstacle in the robot's environment.

    Members:
        length -- Length of square edges
    '''
    def __init__(self, center, length, num_points=DEFAULT_NUM_POINTS):
        self.length = length
        super(Square, self).__init__(center, num_points)


    def init_points(self):
        # Determine number of points on each edge
        edge_points = self.num_points / 4 + 1

        # Offset from origin to corners
        L = self.length / 2

        ### Calculate num_points points along edges of a square
        # Top right to top left
        top = np.linspace([-L,L], [L,L], edge_points)[:-1]
        # Top left to bottom left
        left = np.linspace([L,L], [L,-L], edge_points)[:-1]
        # Bottom left to bottom right
        bottom = np.linspace([L,-L], [-L,-L], edge_points)[:-1]
        # Bottom right to top right
        right = np.linspace([-L,-L], [-L,L], edge_points)[:-1]

        # Combine all points into a single numpy array and offset by object
        # center coordinate.
        self.points = np.concatenate((top, left, bottom, right))
        self.points += np.array(self.center)


class Circle(Obstacle):
    '''
    Class representing a circle obstacle in the robot's environment.

    Members:
        radius -- Radius of circle
    '''
    def __init__(self, center, radius, num_points=DEFAULT_NUM_POINTS):
        self.radius = radius
        super(Circle, self).__init__(center, num_points)

    def init_points(self):
        # Gather useful constants
        pi = math.pi
        r = self.radius
        n = self.num_points

        # Calculate num_points points along the circumference of a circle
        self.points = [(math.cos(2 * (pi/n) * d)*r, 
                        math.sin(2 * (pi/n) * d)*r) for d in range(n)]
        # Offset by obstacle center coordinate
        self.points = np.array(self.points) + np.array(self.center)