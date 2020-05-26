#
# Author: Adam Ondryas
# Course: VAI
# Project: RRT path planning algorithm
# Created: 23.5. 2020
#
# Each node is defined by 4 variables: coordinates x,y and predecessor's coordinates x,y
# It can be imagined as N[Nx,Ny,Px,Py] but they are separate variables because
# I had a hard time with matrices in python
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
# TO DO : 1) collision detection and obstacles


import matplotlib.pyplot as plt
import random
import math
from scipy.optimize import fsolve
import numpy as np

class NODES: 
    def __init__(self, start_pos, end_pos, max_step,tolerance):

        # root node
        self.Nx = [start_pos[0]] # list of all nodes, x coordinate
        self.Ny = [start_pos[1]] # list of all nodes, y coordinate

        # root node's predecessor is set to be the root node itself
        self.Px = [start_pos[0]] # x coordinate
        self.Py = [start_pos[1]] # y coordinate
        
        # simulation ends when it reaches the end node
        self.end_node_x = [end_pos[0]] # x coordinate
        self.end_node_y = [end_pos[1]] # y coordinate 

        # backtrack path lists
        self.path_nodes_x = []
        self.path_nodes_y = [] 

        # fake
        self.fake_podminka = 1
        self.max_step = max_step # maximum step allowed
        
        # limits for random generation
        self.up_limit_x = end_pos[0] + 2
        self.up_limit_y = end_pos[1] + 2 
        self.low_limit_x = start_pos[0] - 2
        self.low_limit_y = start_pos[1] - 2

        # graph init
        self.fig, self.ax = plt.subplots()
        #self.ax.axis([self.low_limit_x, self.up_limit_x, self.low_limit_y, self.up_limit_y])
        end_ring = plt.Circle((self.end_node_x[0], self.end_node_y[0]), radius=tolerance, color='red', alpha=0.4)
        self.ax.add_artist(end_ring)
        plt.scatter(self.Nx[0],self.Ny[0], c='red')
        plt.scatter(self.end_node_x,self.end_node_y, c='red')


    # MAIN FUNCTION
    def expand(self):

        # random point generation
        random_vertex_x = random.uniform(self.low_limit_x,self.up_limit_x)
        random_vertex_y = random.uniform(self.low_limit_y,self.up_limit_y)

        if self.collision() == 0: # checking against collision condition
            # path free --> appends new node
            self.Nx.append(random_vertex_x)
            self.Ny.append(random_vertex_y)
            self.find_closest() # finds the closest node to connect with, appends the predecessor to the new node
            self.chain()
            self.plot_node()

    
    def collision(self): # CHECKS COLLISIONS WITH OBSTACLES
       if self.fake_podminka == 1:
            return False

    def f_lines(self,z,random_vertex_x,random_vertex_y):
        a,b = z
        f1 = a*random_vertex_x + b - random_vertex_y
        f2 = a*self.Nx[-1] + b - self.Ny[-1]
        return [f1,f2]

        x = fsolve(f,[1,1])
        print(x)

    def f_line_circle(self,z):
        x,y = z
        f1 = (x-5)**2 + (y-5)**2 - 1
        f2 = -x - y + 1
        return [f1,f2]

        print(f([3.685,3.688]))

        x = fsolve(f,[1,1])
        print(x)

    def find_closest(self): # FIND PREDECESSOR

        # calculates vector size from the new node to all nodes and finds the closest
        vector_size = []
        for i in range(len(self.Nx)-1):
            vector_size.append(math.sqrt((self.Nx[-1] - self.Nx[i])**2 + (self.Ny[-1] - self.Ny[i])**2))
        
        # returns the coordinates of the closest node
        self.Px.append(self.Nx[vector_size.index(min(vector_size))])
        self.Py.append(self.Ny[vector_size.index(min(vector_size))])


    def chain(self): # CORRECTS STEP SIZE IF NEEDED

        # calculates the vector size to check against the max_step condition
        vector_x = self.Nx[-1] - self.Px[-1]
        vector_y = self.Ny[-1] - self.Py[-1]
        vector_size = math.sqrt(vector_x**2 + vector_y**2)

        # checks if step size is bigger than the allowed max_step
        if vector_size > self.max_step:
            # calculates the unit vector and multiplies by max step, then adds to the predecessor to rewrite the node coords
            unit_vector_x = (vector_x/vector_size) * self.max_step
            unit_vector_y = (vector_y/vector_size) * self.max_step
            self.Nx[-1] = self.Px[-1] + unit_vector_x
            self.Ny[-1] = self.Py[-1] + unit_vector_y


    def plot_node(self):

        # Simulation can be stopped by the ESC key
        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event: [exit(0) if event.key == 'escape' else None])

        plt.scatter(self.Nx[-1],self.Ny[-1],c = 'green') # plots a node
        plt.plot([self.Px[-1], self.Nx[-1]],[self.Py[-1], self.Ny[-1]], c='blue') # plots a line
        plt.pause(0.000001) 
    

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
        plt.plot([self.path_nodes_x[0], self.path_nodes_x[1]], [self.path_nodes_y[0], self.path_nodes_y[1]], c='red')

        # loop until root node
        while((self.path_nodes_x[-1] != self.Nx[0]) and (self.path_nodes_y[-1] != self.Ny[0])):
            
            for i in range(1, len(self.Nx)): # loop through the whole list of nodes to find predecesor
                if (self.Nx[-i] == self.path_nodes_x[-1] and self.Ny[-i] == self.path_nodes_y[-1]):
                    self.path_nodes_x.append(self.Px[-i])
                    self.path_nodes_y.append(self.Py[-i])
                    plt.plot([self.path_nodes_x[-1], self.path_nodes_x[-2]],[self.path_nodes_y[-1], self.path_nodes_y[-2]], c='red')
                    plt.pause(0.000001)
                    break


def main():
    # input params
    start_pos = [0, 0]
    end_pos = [10, 10]
    max_step = 0.7

    circle_obstacles = np.array([2, 2, 1, 5, 2, 2])
    # radius around the end_pos
    tolerance = 0.5
    
    # instancing object
    nodes = NODES(start_pos, end_pos, max_step, tolerance)
    
    n = 0 # number of iterations
    
    while (1):

        nodes.expand()
        n = n+1
        # if end_pos are equal to last created node --> loop breaks
        if (((nodes.Nx[-1] >= end_pos[0]-tolerance) and (nodes.Nx[-1] <= end_pos[0]+tolerance)) and ((nodes.Ny[-1] >= end_pos[1]-tolerance) and (nodes.Ny[-1] <= end_pos[1]+tolerance))):
            break

    # print out the final coords and number of iterations
    nodes.backtrack()
    print(round(nodes.Nx[-1],1),round(nodes.Ny[-1],1), n)
    plt.show()

if __name__ == '__main__':
    main()

