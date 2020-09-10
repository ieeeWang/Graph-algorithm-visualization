# -*- coding: utf-8 -*-
"""
Created on Fri Sep  4 12:00:30 2020
Breath-first search (BFS) and depth-first search (DFS): 
    for traversal on directed graphs (for simplicity).

the time complexity for both B/DFS on a graph is O(V + E)
implement by using build-in python data strucures:
    list (for queue or stack) 
    dict (for adj list)

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
    
  #visited is a list that is used to keep track of visited nodes from current nodes
  visited.append(node) 
  # queue keep track of nodes currently in the queue or stack
  queue = []     #Initialize a queue
  queue.append(node)
  
  # curr_visited keep track of currently visiting nodes
  curr_visiting = []     #Initialize a queue

  while queue:
    # BFS:
    # s = queue.pop(0) # remove the first entered node, i.e., queue
    # DFS:
    s = queue.pop() # remove the last entered node, i.e., stack
    
    print (s, end = " ") 
    curr_visiting.append(s) 

    for neighbour in graph[s]:
      if neighbour not in visited:
        visited.append(neighbour)
        queue.append(neighbour)

  return curr_visiting


# Driver Code
visited = [] # List to keep track of visited nodes.
_visited = bfs(visited, graph, 'A')
print(_visited)


#%% test list.pop()
print ('=test=') 
mystack = [ 1, 2, 3, 4, 5, 6 ] 
mystack.insert(0,'a') # insert at 0 
# mystack.append('a') # append at last

for _ in range(len(mystack)):
    # s = mystack.pop(0) # remove the first entered node, i.e., queue
    s = mystack.pop() # remove the last entered node, i.e., stack
    print (s, end = " ") 