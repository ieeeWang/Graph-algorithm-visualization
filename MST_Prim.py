# -*- coding: utf-8 -*-
"""
Prim’s Minimum Spanning Tree (MST)

Algorithm: 
1) Create a set 'mstSet' that keeps track of vertices already included in MST. 
2) Assign a key value to all vertices in the input graph. Initialize all key 
    values as INFINITE. Assign key value as 0 for the starting vertex. 
3) While 'mstSet' doesn’t include all vertices 
….a) Pick a vertex u which is not in mstSet and has minimum key value. 
….b) Include u to mstSet. 
….c) Update key value of all adjacent vertices of u. 
     if weight of edge u-v is less than the previous key value of v, update it. 
     
Time Complexity of the above program is O(V^2). If the input graph is 
represented using 'adjacency list', then the time complexity of Prim’s 
algorithm can be reduced to O(E log V) with the help of binary heap. 

More details refer to:    
https://www.geeksforgeeks.org/prims-minimum-spanning-tree-mst-greedy-algo-5/
Created on Fri Sep  4 21:27:26 2020
@author: lwang
"""
from utilis import plot_adj_matrix
import sys # Library for INT_MAX
inf = sys.maxsize 

class Graph():
 
    def __init__(self, vertices):
        self.V = vertices
        self.graph = [[0 for column in range(vertices)] 
                    for row in range(vertices)]
 
    # A utility function to print the constructed MST stored in parent[]
    def printMST(self, parent):
        print ("Edge \t Weight")
        for i in range(1, self.V):
            print (parent[i], "-", i, ", \t", self.graph[i][ parent[i] ],"\\")
 
    
    # A utility function to find the vertex with 
    # minimum key value, from the set of vertices 
    # not yet included in 'mstSet'
    def minKey(self, key, mstSet):
        # Initilaize min value
        min = inf
 
        for v in range(self.V):
            if key[v] < min and mstSet[v] == False:
                min = key[v]
                min_index = v
 
        return min_index
 
    
    # Function to construct and print MST for a graph 
    # represented using adjacency matrix representation
    def primMST(self):
        # Key values used to pick minimum weight edge in cut
        key = [inf] * self.V
        parent = [None] * self.V # List to store constructed MST
        # Make key 0 so that this vertex is picked as first vertex
        key[0] = 0
        mstSet = [False] * self.V
 
        # there is an edge between parent[i] and i
        parent[0] = -1 # First node is always the root of
 
        for cout in range(self.V):
 
            # Pick the minimum distance vertex from 
            # the set of vertices not yet processed. 
            # u is always equal to src in first iteration
            u = self.minKey(key, mstSet)
            print('u:', u)
            # Put the minimum distance vertex in 
            # the shortest path tree
            mstSet[u] = True
 
            # Update dist value of the adjacent vertices 
            # of the picked vertex only if the current 
            # distance is greater than new distance and
            # the vertex in not in the shotest path tree
            for v in range(self.V):
 
                # graph[u][v] is non zero only for adjacent vertices of m
                # mstSet[v] is false for vertices not yet included in MST
                # Update the key only if graph[u][v] is smaller than key[v]
                adj_weight = self.graph[u][v]
                if adj_weight > 0 and mstSet[v] == False and key[v] > adj_weight:
                        key[v] = adj_weight # store the min
                        # parent of v is u, i.e., an edge between v and u
                        parent[v] = u
 
        self.printMST(parent)
        return parent
 
#%%
# adjacent matrix
mygraph = [ [0, 2, 0, 6, 0],
            [2, 0, 3, 8, 5],
            [0, 3, 0, 0, 7],
            [6, 8, 0, 0, 9],
            [0, 5, 7, 9, 0]]

# mygraph  = [[0, 4, 0, 0, 0, 0, 0, 8, 0], 
#             [4, 0, 8, 0, 0, 0, 0, 11, 0], 
#             [0, 8, 0, 7, 0, 4, 0, 0, 2], 
#             [0, 0, 7, 0, 9, 14, 0, 0, 0], 
#             [0, 0, 0, 9, 0, 10, 0, 0, 0], 
#             [0, 0, 4, 14, 10, 0, 2, 0, 0], 
#             [0, 0, 0, 0, 0, 2, 0, 1, 6], 
#             [8, 11, 0, 0, 0, 0, 1, 0, 7], 
#             [0, 0, 2, 0, 0, 0, 6, 7, 0]]; 
 
plot_adj_matrix(mygraph)

N_E = len(mygraph)
g = Graph(N_E)
g.graph = mygraph
parent = g.primMST()
print(parent)
 



