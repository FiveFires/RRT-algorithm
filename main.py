# Author: Adam Ondryas
# Course: VAI
# Project: RRT path planning algorithm
# Created: 23.
# Each node is defined by 4 variables: coordinates x,y and predecessor's coordinates x,y
# predecessor's coordinates are used for backtracking

# Simulation can be stopped by the ESC key


# TO DO : 1) collision detection and obstacles
#         2) backtracking line draw


import matplotlib.pyplot as plt
import random
import math




class NODES: 
    def __init__(self, start_pos, end_pos, max_step):
        # root node
        self.Nx = [start_pos[0]] # list of all nodes, x coordinate
        self.Ny = [start_pos[1]] # list of all nodes, y coordinate

        # root node's predecessor is set to be the root node itself
        self.Px = [start_pos[0]] # x coordinate
        self.Py = [start_pos[1]] # y coordinate
        
        # simulation ends when it reaches the end node
        self.end_node_x = [end_pos[0]] # x coordinate
        self.end_node_y = [end_pos[1]] # y coordinate 

        self.fake_podminka = 1
        self.max_step = max_step # maximum step allowed
        
        # limits for random generation
        self.up_limit_x = end_pos[0] + 2
        self.up_limit_y = end_pos[1] + 2 
        self.low_limit_x = start_pos[0] - 2
        self.low_limit_y = start_pos[1] - 2

        self.ax = plt.axis([self.low_limit_x, self.up_limit_x, self.low_limit_y, self.up_limit_y])
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

        # TODO vyresit soustavu rovnice primky a kruznice (prekazky)
        if self.fake_podminka == 1:
            return False

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
        plt.pause(0.001) 

def main():
    # input params
    start_pos = [0, 0]
    end_pos = [2, 10]
    max_step = 0.5
    tolerance = 0.3
    
    # instancing object
    nodes = NODES(start_pos, end_pos, max_step)
    
    n = 0 # number of iterations
    
    while (1):

        nodes.expand()
        n = n+1
        # if end_pos are equal to last created node --> loop breaks
        if (((nodes.Nx[-1] >= end_pos[0]-tolerance) and (nodes.Nx[-1] <= end_pos[0]+tolerance)) and ((nodes.Ny[-1] >= end_pos[1]-tolerance) and (nodes.Ny[-1] <= end_pos[1]+tolerance))):
            break

    # print out the final coords and number of iterations
    print(round(nodes.Nx[-1],1),round(nodes.Ny[-1],1), n)
    plt.show()

if __name__ == '__main__':
    main()

