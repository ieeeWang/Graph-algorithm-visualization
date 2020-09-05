# -*- coding: utf-8 -*-
"""
Dijkstra's shortest path algorithm (with graph visualization)
for undirected weighted graphs represented by adjacency matrix.

Algorithm
1) Create a set 'sptSet' (shortest path tree set) that keeps track of 
vertices included in shortest path tree.
2) Assign a distance value to all vertices in the input graph. 
Initialize all distance values as INFINITE and the source vertex as 0.
3) While sptSet doesn’t include all vertices:
….a) Pick a vertex u which is not in sptSet and has minimum distance value.
….b) Include u to sptSet.
….c) Update distance value of all adjacent vertices of u.
….d) Update 'previous node' as u if shorter distance found.    

Time Complexity of the implementation is O(V^2). If the input graph is 
represented using 'adjacency list', it can be reduced to O(E log V) with 
the help of binary heap. 
    
More details refer to: 
https://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-greedy-algo-7/
Created on Sun Aug 30 18:22:23 2020
@author: lwang
"""

import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
from utilis import plot_adj_matrix

#%% demo 1: a demo using networkx
G = nx.Graph()
G.add_edge(1,2, weight=1)
G.add_edge(1,3, weight=4)
G.add_edge(2,3, weight=1)

nx.draw(G, with_labels=True)
plt.show()

#%% this list shows the adjacency matrix representation of the graph 
graph_adj = [[0, 4, 0, 0, 0, 0, 0, 8, 0], 
            [4, 0, 8, 0, 0, 0, 0, 11, 0], 
            [0, 8, 0, 7, 0, 4, 0, 0, 2], 
            [0, 0, 7, 0, 9, 14, 0, 0, 0], 
            [0, 0, 0, 9, 0, 10, 0, 0, 0], 
            [0, 0, 4, 14, 10, 0, 2, 0, 0], 
            [0, 0, 0, 0, 0, 2, 0, 1, 6], 
            [8, 11, 0, 0, 0, 0, 1, 0, 7], 
            [0, 0, 2, 0, 0, 0, 6, 7, 0]]; 

# plot the graph represented by the adjacency matrix 'graph_adj'
plot_adj_matrix(graph_adj)

#%% Python program for Dijkstra's single source shortest path algorithm. 
#  The program is for adjacency matrix representation of the graph 
  
# Library for INT_MAX 
import sys
class Graph(): 
    def __init__(self, vertices): 
        self.V = vertices 
        self.graph = [[0 for column in range(vertices)]  
                    for row in range(vertices)] 
  
    def printSolution(self, dist, pre_node): 
        print ("Vertex \t shortest dist \t previous node")
        for node in range(self.V): 
            print (node, "-----\t", dist[node],"-----\t", pre_node[node]) 
  
    # A utility function to find the vertex with  
    # minimum distance value, from the set of vertices  
    # not yet included in shortest path tree 
    def minDistance(self, dist, sptSet):   
        # Initilaize minimum distance for next node 
        min = sys.maxsize 
        # Search not nearest vertex not in the  
        # shortest path tree 
        for v in range(self.V): 
            if dist[v] < min and sptSet[v] == False: 
                min = dist[v] 
                min_index = v 
  
        return min_index 
  
    # Funtion that implements Dijkstra's single source  
    # shortest path algorithm for a graph represented  
    # using adjacency matrix representation 
    def dijkstra(self, src): 
        dist = [sys.maxsize] * self.V 
        dist[src] = 0
        
        # sptSet (shortest path tree set) keeps track of vertices included 
        # in shortest path tree, i.e., whose minimum distance from source is 
        # calculated and finalized already. Initially, this set is empty.
        sptSet = [False] * self.V 
        
        # 'pre_node' maintains a list that store the previous node of each node 
        # in the shortest path
        pre_node = [False] * self.V
        pre_node[0] = src
        
        for _ in range(self.V): 
            # Pick the minimum distance vertex from  
            # the set of vertices not yet processed.  
            # u is always equal to src in first iteration 
            u = self.minDistance(dist, sptSet) 
            
            # Put the minimum distance vertex in the  
            # shotest path tree 
            sptSet[u] = True         
  
            # Update dist value of the adjacent vertices  
            # of the picked vertex only if the current  
            # distance is greater than new distance and 
            # the vertex in not in the shotest path tree 
            for v in range(self.V): # search throuth all adjacent vertices  
                dist_add = self.graph[u][v]
                if dist_add > 0 and sptSet[v] == False and (dist[v] > dist[u] + dist_add): 
                    dist[v] = dist[u] + dist_add
                    # update 'pre_node' if a shorter path found 
                    pre_node[v] = u
  
        self.printSolution(dist, pre_node) 
  
    
# Driver program 
g = Graph(9) 
g.graph = graph_adj; 
  
g.dijkstra(0); 
  


