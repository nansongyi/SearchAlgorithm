import operator
import numpy
import numpy as np
import math

class RRTTree(object):
    
    def __init__(self, planning_env):
        
        self.planning_env = planning_env
        self.vertices = []
        self.edges = dict()
        self.samples_taken = 0
        self.max_samples = 5000

    def GetRootID(self):
        '''
        Returns the ID of the root in the tree.
        '''
        return 0

    def GetNearestVertex(self, config):
        '''
        Returns the nearest state ID in the tree.
        @param config Sampled configuration.
        '''
        dists = []
        for v in self.vertices:
            dists.append(self.planning_env.compute_distance(config, v))

        vid, vdist = min(enumerate(dists), key=operator.itemgetter(1))

        return vid, self.vertices[vid]
        
    def GetKNN(self, config, k=20):
        '''
        Return k-nearest neighbors
        @param config Sampled configuration.s
        @param k Number of nearest neighbors to retrieve.
        '''
        dists = []
        for v in self.vertices:
            # print("V type", type(v))
            dists.append(self.planning_env.compute_distance(config, v))
        dists = numpy.array(dists)

        # print("dists", dists)
        k = min(k, len(dists)-1)

        # print("kkkkkkk",k)
        knnIDs = numpy.argpartition(dists, k)[0:k]
        knnDists = [dists[i] for i in knnIDs]
        k_nearest_vertices = [self.vertices[vid] for vid in knnIDs]
        # print("hihihihihi", k_nearest_vertices)
        return knnIDs, k_nearest_vertices

    def AddVertex(self, config):
        '''
        Add a state to the tree.
        @param config Configuration to add to the tree.
        '''
        vid = len(self.vertices)
        self.vertices.append(config)
        return vid

    def AddEdge(self, sid, eid):
        '''
        Adds an edge in the tree.
        @param sid start state ID
        @param eid end state ID
        '''
        self.edges[eid] = sid
