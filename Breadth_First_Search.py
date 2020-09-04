# -*- coding: utf-8 -*-
"""
Created on Fri Sep  4 11:26:32 2020
Breath-first search (BFS): for traversal on graphs or tree data structures
BFS for a directed Graph with visulization. Note that undirected graphs can be 
implemented by using 'adjecnet matrix' instead of dictionary used here.

implement by using python list, dict
@author: lwang
"""

# Python3 Program to print BFS traversal 
# from a given source vertex. BFS(int s) 
# traverses vertices reachable from s. 
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
# defaultdict: dict subclass that calls a factory function to supply missing values
from collections import defaultdict  
  
# This class represents a directed graph 
# using adjacency list representation 
class Graph: 
    # Constructor 
    def __init__(self): 
        # default dictionary to store graph 
        self.graph = defaultdict(list) 
  
    # function to add an edge to graph 
    def addEdge(self,u,v): 
        self.graph[u].append(v) 
  
    # Function to print a BFS of graph 
    def BFS(self, s): 
        out_list = []
        # Mark all the vertices as not visited 
        visited = [False] * (len(self.graph)) 
        # Create a queue for BFS 
        queue = [] 
        # Mark the source node as  
        # visited and enqueue it 
        queue.append(s) 
        visited[s-1] = True
  
        while queue:   
            # Dequeue a vertex from  
            # queue and print it 
            print("currunt queue: ", queue)
            s = queue.pop(0) 
            out_list.append(s)
            print ("pop node: ", s)  
            
            # Get all adjacent vertices of the 
            # dequeued vertex s. If a adjacent 
            # has not been visited, then mark it 
            # visited and enqueue it 
            for i in self.graph[s]: 
                print(i)
                if visited[i-1] == False: 
                    queue.append(i) 
                    visited[i-1] = True
  
        return out_list

#%% Create a graph given in the above diagram 
g = Graph() 
# g.addEdge(0, 1) 
# g.addEdge(0, 2) 
# g.addEdge(1, 2) 
# g.addEdge(2, 0) 
# g.addEdge(2, 3) 
# g.addEdge(3, 3)
 
g.addEdge(1, 2)
g.addEdge(1, 3)
g.addEdge(2, 4)
g.addEdge(2, 5)
g.addEdge(3, 5)
g.addEdge(4, 5)
g.addEdge(4, 6)
g.addEdge(5, 6)  
g.addEdge(6, 6) # add this, otherwise, len(visited) = n-1

a1=g.graph 

#%% plot it using networkx
G = nx.Graph()
G.add_edge(1, 2)
G.add_edge(1, 3)
G.add_edge(2, 4)
G.add_edge(2, 5)
G.add_edge(3, 5)
G.add_edge(4, 5)
G.add_edge(4, 6)
G.add_edge(5, 6)
G.add_edge(6, 6)

nx.draw(G, with_labels=True)
plt.show()


#%%
print ("Following is Breadth First Traversal"
                  " (starting from vertex 1)") 
out_list = g.BFS(2) 
print(out_list)
