import numpy
import numpy as np
import random
from RRTTree import RRTTree
from RRTPlanner import RRTPlanner
from operator import itemgetter
from matplotlib import pyplot as plt


class RRTStarPlanner(RRTPlanner):

    def __init__(self, planning_env):
        super(RRTStarPlanner,self).__init__(planning_env)  
        self.name = 'rrtstar' 

    def Plan(self, start_config, goal_config, para = 0.05):
        
        # Initialize an empty plan.
        self.prob = para
        self.start_config = start_config
        self.goal_config = goal_config
        self.c_best = float('inf')  # length of best solution thus far
        plan = []
        if self.tree.samples_taken < 10:
            plan = self.rrt_search()
        plan += self.rrt_star()
        pre_cost = self.planning_env.path_cost(self.tree.edges, self.start_config, self.goal_config)
        print("cost:", pre_cost)
        return numpy.array(plan)
        
    def extend(self):
        # TODO (student): Implement an extend logic.
        pass

    def ShortenPath(self, path):
        # TODO (student): Postprocessing of the plan.

        return path

    def get_nearby_vertices(self, x_init, x_new):
        """
        Get nearby vertices to new vertex and their associated costs, number defined by rewire count
        :param tree: tree in which to search
        :param x_init: starting vertex used to calculate path cost
        :param x_new: vertex around which to find nearby vertices
        :return: list of nearby vertices and their costs, sorted in ascending order by cost
        """
        no_one, X_near = self.tree.GetKNN(x_new)
        # print("start_config!!!!", type(self.start_config))
        # print("new point type!!!!", type(x_new))
        L_near = []
        # print("X_near",X_near)
        for x_near in X_near:
            # print("x_near!!!!", type(x_near))
            cost = self.planning_env.path_cost(self.tree.edges, self.start_config, x_near) \
                 + self.planning_env.compute_distance(x_near, x_new)
            L_near.append((x_near, cost))
        # L_near = [(x_near, self.planning_env.path_cost(self.tree.edges, self.start_config, x_near) \
        #           + self.planning_env.computer_distance(x_near, x_new)) for x_near in X_near]
        # noinspection PyTypeChecker
        L_near.sort(key=itemgetter(1))
        # print(L_near)
        return L_near

    def connect_shortest_valid(self, x_new, L_near):
        """
        Connect to nearest vertex that has an unobstructed path
        :param tree: int, tree being added to
        :param x_new: tuple, vertex being added
        :param L_near: list of nearby vertices
        """
        # check nearby vertices for total cost and connect shortest valid edge
        for x_near, c_near in L_near:
            # if c_near + cost_to_go(x_near, self.x_goal) < self.c_best and self.connect_to_point(tree, x_near, x_new):
            if c_near < self.c_best and self.connect_to_point(x_near, x_new):
                # self.connect_to_goal(tree,x_near,x_new)
                break

    def rewire(self, x_new, L_near):
        """
        Rewire tree to shorten edges if possible
        Only rewires vertices according to rewire count
        :param x_new: tuple, newly added vertex
        :param L_near: list of nearby vertices used to rewire
        :return:
        """
        for x_near, c_near in L_near:
            curr_cost = self.planning_env.path_cost(self.tree.edges, self.start_config, x_near)
            tent_cost = self.planning_env.path_cost(self.tree.edges, self.start_config, x_new) \
                        + self.planning_env.compute_distance(x_near, x_new)

            # if tent_cost < curr_cost and self.planning_env.collision_free(x_near, x_new):
            #     self.tree.edges[x_near] = x_new

    def rrt_star(self):
        """
        Create and return a Rapidly-exploring Random Tree, keeps expanding until can connect to goal
        https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree
        :return: list representation of path, dict representing edges of tree in form E[child] = parent
        """
        self.tree.AddVertex(self.start_config)
        self.tree.AddEdge(self.start_config, self.start_config)
        while True:
            x_new, x_nearest = self.new_and_near()
            if x_new is None:
                # print("it's None")
                continue
            # connect shortest valid edge
            L_near = self.get_nearby_vertices(self.start_config, x_new)
            self.connect_shortest_valid(x_new, L_near)

            if x_new in self.tree.vertices:
                # rewire tree
                self.rewire(x_new, L_near)

            # probabilistically check if solution found
            if self.goal_config in self.tree.vertices:
                print("find it")
                path = self.planning_env.reconstruct_path(self.tree.edges, self.start_config, self.goal_config)
                if path is not None:
                    return path

            # # check if can connect to goal after generating max_samples
            if self.tree.samples_taken >= self.tree.max_samples:
                return []

    def visualize_plan(self, name, pre_plan, graph=None, para=0.05):

        plt.imshow(self.planning_env.map, interpolation='nearest')
        for key in graph.edges:
            x = key[0], graph.edges[key][0]
            y = key[1], graph.edges[key][1]
            plt.plot(y, x, 'g')
        for i in range(numpy.shape(pre_plan)[0] - 1):
            x = [pre_plan[i,0], pre_plan[i+1, 0]]
            y = [pre_plan[i,1], pre_plan[i+1, 1]]  
            plt.plot(y, x, 'r')             
        plt.title("probability={}".format(para))
        plt.show()