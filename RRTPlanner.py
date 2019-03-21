import numpy
import numpy as np
import random
from RRTTree import RRTTree
from matplotlib import pyplot as plt

class RRTPlanner(object):

    def __init__(self, planning_env):
        self.name = 'rrt'
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env)
        self.prob = 0.1

    def Plan(self, start_config, goal_config, para = 0.05):
        
        # Initialize an empty plan.
        self.prob = para
        self.start_config = start_config
        self.goal_config = goal_config

        pre_plan = []
        pre_plan = self.rrt_search()
        pre_cost = self.planning_env.path_cost(self.tree.edges, self.start_config, self.goal_config)
        print("pre_cost:", pre_cost)

        plan = self.ShortenPath(pre_plan)
        shorten_cost = self.planning_env.path_cost2(plan)
        print("shorten_cost:", shorten_cost)
        return np.array(pre_plan), numpy.array(plan)

    def ShortenPath(self, path):
        # TODO (student): Postprocessing of the plan.
        new_path = [path[0]]
        i = 0
        while i < len(path) - 1:
            i += 1
            for j in range(i+1, len(path)-1):
                if not self.planning_env.collision_free(path[i], path[j]):
                    new_path.append(path[j-1])
                    i = j - 2
                    break
        new_path.append(self.goal_config)
        return new_path

    def steer(self, start, goal, eta=2):
        """
        Return a point in the direction of the goal, that is distance away from start
        :param start: start location
        :param goal: goal location
        :param distance: distance away from start
        :return: point in the direction of the goal, distance away from start
        """
        if self.planning_env.compute_distance(start, self.goal_config) < eta:
            return self.goal_config
        mode = 1
        if mode == 1:
            vec = np.array([goal[0]-start[0], goal[1]-start[1]])
            if np.linalg.norm(vec) != 0:
                normed_vec = vec/np.linalg.norm(vec)
            else:
                normed_vec = vec
            endpoint = np.array([start[0],start[1]]) + eta * normed_vec

        if mode == 2:
            endpoint = goal
        # print(start)
        # print(normed_vec)
        # print(endpoint)
        # endpoint = goal

        return tuple(endpoint)

    def connect_to_point(self,sid, eid):
        if eid not in self.tree.vertices and self.planning_env.collision_free(sid, eid):
            self.tree.AddVertex(eid)
            self.tree.AddEdge(sid, eid) 
            return True
        else:
            return False

    def new_and_near(self):
        """
        Return a new steered vertex and the vertex in tree that is nearest
        :param tree: int, tree being searched
        :param q: length of edge when steering
        :return: vertex, new steered vertex, vertex, nearest vertex in tree to new vertex
        """
        if self.prob and random.random() < self.prob:
            x_rand = self.goal_config
        else:
            x_rand = self.planning_env.sample_free()
        x_nearest_id, x_nearest = self.tree.GetNearestVertex(x_rand)
        x_new = self.steer(x_nearest, x_rand)
        # check if new point is in X_free and not already in V
        # if x_new in self.tree.vertices or not self.planning_env.state_validity_checker(x_new):
        if x_new in self.tree.vertices or not self.planning_env.collision_free(x_new, x_nearest):
            return None, None

        self.tree.samples_taken += 1
        return x_new, x_nearest

    def rrt_search(self):
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
            # print("new point", x_new)
            self.connect_to_point(x_nearest, x_new)

            # probabilistically check if solution found
            if self.goal_config in self.tree.vertices:
                print("find it")
                path = self.planning_env.reconstruct_path(self.tree.edges, self.start_config, self.goal_config)
                if path is not None:
                    return path

            if self.name=='rrtstar' and self.tree.samples_taken > 10:
                return []
            # # check if can connect to goal after generating max_samples
            if self.tree.samples_taken >= self.tree.max_samples:
                return []


    def visualize_plan(self, name, pre_plan, shorten_plan, graph=None, para=0.05):

        plt.imshow(self.planning_env.map, interpolation='nearest')
        for key in graph.edges:
            x = key[0], graph.edges[key][0]
            y = key[1], graph.edges[key][1]
            plt.plot(y, x, 'g')
        for i in range(numpy.shape(pre_plan)[0] - 1):
            x = [pre_plan[i,0], pre_plan[i+1, 0]]
            y = [pre_plan[i,1], pre_plan[i+1, 1]]  
            plt.plot(y, x, 'r')             
        for i in range(numpy.shape(shorten_plan)[0] - 1):
            x = [shorten_plan[i,0], shorten_plan[i+1, 0]]
            y = [shorten_plan[i,1], shorten_plan[i+1, 1]]  
            plt.plot(y, x, 'cyan')
        plt.title("probability={}".format(para))
        plt.show()
