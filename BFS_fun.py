# -*- coding: utf-8 -*-
"""
Created on Fri Sep  4 12:00:30 2020
Breath-first search (BFS): for traversal on directed graphs or tree data structures
the time complexity for BFS on a graph is O(V + E)
implement by using python list, dict, refer to:
https://www.educative.io/edpresso/how-to-implement-a-breadth-first-search-in-python
@author: lwang
"""

graph = {
  'A' : ['B','C'],
  'B' : ['D', 'E'],
  'C' : ['F'],
  'D' : [],
  'E' : ['F'],
  'F' : []
}


def bfs(visited, graph, node):
    
  #visited is a list that is used to keep track of visited nodes.
  visited.append(node) 
  #queue is a list that is used to keep track of nodes currently in the queue.
  queue.append(node)

  while queue:
    s = queue.pop(0) # remove the first entered node
    print (s, end = " ") 

    for neighbour in graph[s]:
      if neighbour not in visited:
        visited.append(neighbour)
        queue.append(neighbour)

  return visited


# Driver Code
visited = [] # List to keep track of visited nodes.
queue = []     #Initialize a queue
_visited = bfs(visited, graph, 'A')
_visited