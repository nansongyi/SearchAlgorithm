import numpy
import numpy as np
import random
import math
from matplotlib import pyplot as plt
import copy

class MapEnvironment(object):
    
    def __init__(self, mapfile, start, goal):
        # Obtain the boundary limits.
        # Check if file exists.
        M = 100
        self.goal = goal
        self.start = start
        self.map = mapfile
        dim = self.map.shape[0]
        if dim == M:
            self.xlimit = [0, numpy.shape(self.map)[0]-1] # TODO (avk): Check if this needs to flip.
            self.ylimit = [0, numpy.shape(self.map)[1]-1]
        elif dim > 2*M:
            self.xlimit = [50, 250]  # TODO (avk): Check if this needs to flip.
            self.ylimit = [50, 250]         
        # self.xlimit = [0, numpy.shape(self.map)[0]-1] # TODO (avk): Check if this needs to flip.
        # self.ylimit = [0, numpy.shape(self.map)[1]-1]

        # Check if start and goal are within limits and collision free
        if not self.state_validity_checker(start) or not self.state_validity_checker(goal):
            raise ValueError('Start and Goal state must be within the map limits');
            exit(0)

        # Display the map
        plt.imshow(self.map, interpolation='nearest')

    def path_cost2(self, path):
        """
        Cost of the unique path from path[0] to path[-1]
        :param path: list of each step of path
        :return: cost of the path
        """
        cost = 0
        for i in range(len(path)-1):
            cost += self.compute_distance(path[i], path[i+1])
        return cost

    def path_cost(self, E, a, b):
        """
        Cost of the unique path from x_init to x
        :param E: edges, in form of E[child] = parent
        :param a: initial location
        :param b: goal location
        :return: segment_cost of unique path from x_init to x
        """
        cost = 0
        while not b == a:
            # print(type(b))
            p = E[b]
            cost += self.compute_distance(b, p)
            b = p

        return cost

    def compute_distance(self, start_config, end_config):
        #
        # TODO: Implement a function which computes the distance between
        # two configurations.
        #
        dx = start_config[0] - end_config[0]
        dy = start_config[1] - end_config[1]
        distance = np.sqrt(dx**2+dy**2)
        return distance

    def state_validity_checker(self, config):
        #
        # TODO: Implement a state validity checker
        # Return true if valid.
        #
        def check_range(config):
            x, y = config[0], config[1]
            checker2 = (x >= self.xlimit[0] and x <= self.xlimit[1])
            checker3 = (y >= self.ylimit[0] and y <= self.ylimit[1])
            if not (checker2 and checker3):
                return False
            else:
                return True

        x, y = config[0], config[1]
        if isinstance(x, float) and isinstance(y, float):
            x, y = int(x), int(y)
            if not (check_range((x,y)) and check_range((x+1,y)) and check_range((x+1,y+1)) and check_range((x,y+1))):
                return False
            if self.map[x, y] == 1 or self.map[x+1, y] == 1 or self.map[x+1, y+1] == 1 or self.map[x, y+1] == 1:
                return False
            else:
                return True
        else:
            if not check_range((x,y)):
                return False
            if self.map[x, y] == 1:
                return False
            else:
                return True

    def collision_free(self, start, end, r=1):
        """ This chcker is for rrt planner.
        Check if a line segment intersects an obstacle
        :param start: starting point of line
        :param end: ending point of line
        :param r: resolution of points to sample along edge when checking for collisions
        :return: True if line segment does not intersect an obstacle, False otherwise
        """
        dist = self.compute_distance(start, end)
        # divide line between points into equidistant points at given resolution
        dim_linspaces = [np.linspace(s_i, e_i, int(math.ceil(dist / r))) for s_i, e_i in zip(start, end)]
        coll_free = all(map(self.state_validity_checker, zip(*dim_linspaces)))

        return coll_free    

    def compute_heuristic(self, config):
        #
        # TODO: Implement a function to compute heuristic.
        #
        return self.compute_distance(config, self.goal)

    def reconstruct_path(self, came_from, start, goal):
        current = goal
        path = []
        while current != start:
            # print("appending:",current)
            path.append(current)
            current = came_from[current]
        path.append(start) # optional
        path.reverse() # optional
        return path

    def sample_free(self):
        """
        Sample a location within X_free
        :return: random location within X_free
        """
        while True:  # sample until not inside of an obstacle
            x = self.sample()
            # print("free_sample:",x)
            if self.state_validity_checker(x):
                return x

    def sample(self):
        """
        Return a random location within X
        :return: random location within X (not necessarily X_free)
        """
        # x = np.empty(len(np.shape(self.map)), dtype=int)
        x = np.empty(len(np.shape(self.map)), dtype=float)
        # x[0] = int(random.uniform(self.xlimit[0], self.xlimit[1]))
        # x[1] = int(random.uniform(self.xlimit[0], self.xlimit[1]))
        x[0] = random.uniform(self.xlimit[0], self.xlimit[1])
        x[1] = random.uniform(self.xlimit[0], self.xlimit[1])
        x = tuple(x)
        # print("sample:",x)
        return x    

    def visualize_plan(self, name, plan, para=1, graph=None):
        '''
        Visualize the final path
        @param plan Sequence of states defining the plan.
        '''
        if name == 'astar':
            new_map = copy.deepcopy(self.map)
            print(len(graph.keys()))
            for point in graph:
                x = point[0]
                y = point[1]
                new_map[x, y] = 2
            plt.imshow(new_map, interpolation='nearest')
            for i in range(numpy.shape(plan)[0] - 1):
                x = [plan[i,0], plan[i+1, 0]]
                y = [plan[i,1], plan[i+1, 1]]  
                plt.plot(y, x, 'r')
            plt.title("epsilon={}".format(para))
        else:
            plt.imshow(self.map, interpolation='nearest')
            for key in graph.edges:
                x = key[0], graph.edges[key][0]
                y = key[1], graph.edges[key][1]
                plt.plot(y, x, 'g')            
            for i in range(numpy.shape(plan)[0] - 1):
                x = [plan[i,0], plan[i+1, 0]]
                y = [plan[i,1], plan[i+1, 1]]  
                plt.plot(y, x, 'r')
            plt.title("probability={}".format(para))
        plt.show()
