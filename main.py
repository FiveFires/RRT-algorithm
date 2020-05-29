#
# Author: Adam Ondryas
# Course: VAI
# Project: RRT path planning algorithm
# Created: 23.5. 2020
#
# Each node is defined by 4 variables: coordinates x,y and predecessor's coordinates x,y
# It can be imagined as N[Nx,Ny,Px,Py] but they are separate variables because
# I had a hard time with matrices in python (NO MORE, I MANAGED TO WORK WITH SOME)
#
# For example a predecesor node to node 10, Nx[9] Ny[9], can be found on Px[9] Py[9]
# In other words: Nx[9] = Px[10] , Ny[9] = Py[10], thats how they're linked
#
# Predecessor's coordinates are used for backtracking (backtrack function)
# This could have probably been done by giving each node a position variable in the list
# instead of keeping it's coordinates, but it's too late now (got the idea just now)
#
# Simulation can be stopped by the ESC key on the graph
#
#
# TO DO : 1) REDO THE MAX STEP WITH COLLISION THINGY, OTHERWISE DONE


import matplotlib.pyplot as plt
import random
import math
import numpy as np


class NODES:
    def __init__(self, start_pos, end_pos, max_step, tolerance, circle_obstacles, real_time_plot):

        # plotting condition, either TRUE or FALSE
        self.real_time_plot = real_time_plot

        # MAIN NODE LIST
        # root node
        self.Nx = [start_pos[0]]  # list of all nodes, x coordinate
        self.Ny = [start_pos[1]]  # list of all nodes, y coordinate

        # root node's predecessor is set to be the root node itself
        self.Px = [start_pos[0]]  # x coordinate
        self.Py = [start_pos[1]]  # y coordinate
        ##

        # simulation ends when it reaches the end node
        self.end_node_x = [end_pos[0]]  # x coordinate
        self.end_node_y = [end_pos[1]]  # y coordinate

        # backtrack path lists
        self.path_nodes_x = []
        self.path_nodes_y = []

        self.max_step = max_step  # maximum step allowed

        # n by 3 matrix, [x,y,r], stands for center of obstacle and radius,
        # each row is one obstacle
        # need this for transform computation, coordinate vector
        self.obstacles_coords = circle_obstacles[0:, 0:2]
        # just the radii of all obstacles
        self.obstacles_radius = circle_obstacles[0:, 2]

        # preloading for faster computation
        self.T_matrix = np.zeros((2, 2))

        # variable for randomly generated nodes,
        self.random_vertex = np.zeros((2, 1))
        # gets appended to Nx and Ny if all conditions are met

        # variable for predecesors of generated nodes,
        self.temp_p = np.zeros((2, 1))
        # gets appended to Px and Py if all conditions are met

        # limits for random generation
        self.up_limit_x = end_pos[0] + 2
        self.up_limit_y = end_pos[1] + 2
        self.low_limit_x = start_pos[0] - 2
        self.low_limit_y = start_pos[1] - 2

        # GRAPH INIT
        self.fig, self.ax = plt.subplots()
        # axis limits init
        self.ax.axis([self.low_limit_x, self.up_limit_x,self.low_limit_y, self.up_limit_y])
        # plotting the end location
        plt.scatter(self.end_node_x, self.end_node_y, c='red')
        end_area = plt.Circle((self.end_node_x[0], self.end_node_y[0]), radius=tolerance, color='red', alpha=0.4)
        self.ax.add_artist(end_area)
        # plotting the start location
        plt.scatter(self.Nx[0], self.Ny[0], c='red')
        start_area = plt.Circle((self.Nx[0], self.Ny[0]), radius=0.1, color='red', alpha=1)
        self.ax.add_artist(start_area)
        # plotting the obstacles
        self.number_of_obstacles, _ = circle_obstacles.shape
        for i in range(self.number_of_obstacles):
            obstacle_circle = plt.Circle(
                (self.obstacles_coords[i, 0], self.obstacles_coords[i, 1]), radius=self.obstacles_radius[i], color='black', alpha=1)
            self.ax.add_artist(obstacle_circle)

    def expand(self):  # MAIN FUNCTION
        # random point generation
        self.random_vertex[0, 0] = random.uniform(
            self.low_limit_x, self.up_limit_x)
        self.random_vertex[1, 0] = random.uniform(
            self.low_limit_y, self.up_limit_y)

        # finds the closest node for collision check and connection
        self.temp_p[0, 0], self.temp_p[1, 0] = self.find_closest()

        # rewrites the node coordinates so they satisfy the max_step condition
        self.chain()

        if self.collision() == False:  # checking against collision condition
            # path free --> appends new node and its predecesor, chains and plots
            self.Px.append(self.temp_p[0, 0])
            self.Py.append(self.temp_p[1, 0])
            self.Nx.append(self.random_vertex[0, 0])
            self.Ny.append(self.random_vertex[1, 0])
            self.plot_node()

    def collision(self):  # CHECKS COLLISIONS WITH OBSTACLES USING TRANSFORMATION MATRIX
        self.T_rotate_matrix()  # matrix used for rotating the coordinate system

        # vertex in a rotated coordinate system
        rotated_random_vertex = np.dot(self.T_matrix, self.random_vertex)

        for i in range(self.number_of_obstacles):
            # grabbing the x and y of each circle obstacle
            obstacle_cords_to_rotate = np.reshape(
                self.obstacles_coords[i, 0:2], (2, 1))
            # new obstacle coordinates
            rotated_obstacle = np.dot(self.T_matrix, obstacle_cords_to_rotate)
            # x distance to the rotated obstacle from the rotated new node
            distance_x = abs(
                (rotated_random_vertex[0, 0]-rotated_obstacle[0, 0]))
            # y distance to the rotated obstacle from the rotated new node
            distance_y = abs(
                (rotated_random_vertex[1, 0]-rotated_obstacle[1, 0]))
            if (distance_x <= self.obstacles_radius[i] and distance_y <= self.obstacles_radius[i]):
                return True

        return False

    def T_rotate_matrix(self):
        # Rotation matrix for rotating the coordinate system by the vector_angle
        vector_angle = - \
            math.atan2((self.random_vertex[1, 0] - self.temp_p[1, 0]),
                       (self.random_vertex[0, 0] - self.temp_p[0, 0]))
        self.T_matrix[0, 0] = math.cos(vector_angle)
        self.T_matrix[0, 1] = -math.sin(vector_angle)
        self.T_matrix[1, 0] = math.sin(vector_angle)
        self.T_matrix[1, 1] = math.cos(vector_angle)

    def find_closest(self):  # FIND PREDECESSOR
        # calculates vector size from the new node to all nodes and finds the closest
        vector_size = []
        for i in range(len(self.Nx)):
            vector_size.append(math.sqrt(
                (self.random_vertex[0, 0] - self.Nx[i])**2 + (self.random_vertex[1, 0] - self.Ny[i])**2))

        # returns the coordinates of the closest node
        return ([(self.Nx[vector_size.index(min(vector_size))]), self.Ny[vector_size.index(min(vector_size))]])
    def chain(self):  # CORRECTS STEP SIZE IF NEEDED
        # calculates the vector size to check against the max_step condition
        vector_x = self.random_vertex[0, 0] - self.temp_p[0, 0]
        vector_y = self.random_vertex[1, 0] - self.temp_p[1, 0]
        vector_size = math.sqrt(vector_x**2 + vector_y**2)

        # checks if step size is bigger than the allowed max_step
        if vector_size > self.max_step:
            # calculates the unit vector and multiplies by max step, then adds to the predecessor to rewrite the node coords
            unit_vector_x = (vector_x/vector_size) * self.max_step
            unit_vector_y = (vector_y/vector_size) * self.max_step
            self.random_vertex[0, 0] = self.temp_p[0, 0] + unit_vector_x
            self.random_vertex[1, 0] = self.temp_p[1, 0] + unit_vector_y

    def plot_node(self):
        # Simulation can be stopped by the ESC key
        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event: [exit(0) if event.key == 'escape' else None])

        plt.scatter(self.Nx[-1], self.Ny[-1], c='green')  # plots a node
        plt.plot([self.Px[-1], self.Nx[-1]], [self.Py[-1],
                                              self.Ny[-1]], c='blue')  # plots a line
        if self.real_time_plot == True:
            plt.pause(0.001)

    def backtrack(self):
        # finds a way HOME <3
        # takes coords of the final node and its predecesor, plots a line between them and sets the predecesor
        # as the current node, then loops through the list of all nodes to find a match and gets the
        # predecesor of this current node and plots a line again. This loops until root node == current node

        # plots a line between the last and second to last node
        self.path_nodes_x.append(self.Nx[-1])
        self.path_nodes_y.append(self.Ny[-1])
        self.path_nodes_x.append(self.Px[-1])
        self.path_nodes_y.append(self.Py[-1])
        plt.plot([self.path_nodes_x[0], self.path_nodes_x[1]], [
                 self.path_nodes_y[0], self.path_nodes_y[1]], c='red')

        # loop until root node
        while((self.path_nodes_x[-1] != self.Nx[0]) and (self.path_nodes_y[-1] != self.Ny[0])):

            # loop through the whole list of nodes to find predecesor
            for i in range(1, len(self.Nx)):
                if (self.Nx[-i] == self.path_nodes_x[-1] and self.Ny[-i] == self.path_nodes_y[-1]):
                    self.path_nodes_x.append(self.Px[-i])
                    self.path_nodes_y.append(self.Py[-i])
                    plt.plot([self.path_nodes_x[-1], self.path_nodes_x[-2]],
                             [self.path_nodes_y[-1], self.path_nodes_y[-2]], c='red')
                    if self.real_time_plot == True:
                        plt.pause(0.001)
                    break


def main():
    # INPUT PARAMS
    # REAL_TIME_PLOT, SET TO FALSE FOR JUST THE END GRAPH
    real_time_plot = input("Plot nodes in real time? [y/n]:")
    if real_time_plot == "y":
        real_time_plot = True
    else:
        real_time_plot = False

    # root node coords
    start_pos = [0, 0]
    # end node coords
    end_pos = [10, 10]
    # maximum allowed step
    max_step = 0.5
    # radius around the end_pos
    tolerance = 0.5
    # list of circular obstacles  x  y  r ---- x coordinate, y coordinate, r radius
    circle_obstacles = np.array([[8, 8, 1],
                                 [10, 8, 1],
                                 [2, 10, 1],
                                 [5, 5, 3]])
    # instancing object
    nodes = NODES(start_pos, end_pos, max_step, tolerance,
                  circle_obstacles, real_time_plot)

    # maximum number of iterations
    max_iterations = 5000
    n = 0  # iteration counter

    while (n <= max_iterations):
        nodes.expand()
        n = n+1
        # if end_pos are equal to last created node --> break loop because it reached end
        # the whole if condition is for checking if the last created node is inside the tolerances
        # for example: end_pos= (10,10), tolerance = 0.5 so the last node has to be between (9.5 - 10.5, 9.5 - 10.5)
        if (((nodes.Nx[-1] >= end_pos[0]-tolerance) and (nodes.Nx[-1] <= end_pos[0]+tolerance)) and ((nodes.Ny[-1] >= end_pos[1]-tolerance) and (nodes.Ny[-1] <= end_pos[1]+tolerance))):
            # finds and plots the way back home(root node) <3
            nodes.backtrack()

            # print out the final coords and number of iterations
            print(round(nodes.Nx[-1], 1), round(nodes.Ny[-1], 1), n)
            plt.show()
            break
    else:
        print("can't find path")
        plt.show()


if __name__ == '__main__':
    main()
