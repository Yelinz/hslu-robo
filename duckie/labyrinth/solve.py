# BFS to find the shortest path
# 0 no node
# 1 node
# 2 start
# 3 end
# every red dot is a node, yellow lines are vectors
labyrinth = [
  [0, 0, 1, 1, 1, 1, 2],
  [0, 0, 1, 0, 0, 0, 0],
  [0, 1, 1, 1, 1, 1, 0],
  [0, 0, 1, 0, 0, 1, 0],
  [3, 1, 1, 1, 0, 1, 1],
  [0, 0, 1, 0, 0, 0, 0],
]

# djisktra representation
# Distance matrix (Adjacency matrix)
labyrinth = [
# 1  2  3  4
  [0, 3, 0, 0, 0, 0, 0], # 1
  [0, 0, 2, 0, 0, 0, 0], # 2
  [0, 0, 0, 1, 2, 0, 2],
  [0, 0, 0, 0, 0, 2, 0],
  [0, 0, 0, 0, 0, 0, 0],
  [0, 0, 0, 0, 0, 0, 0], # n
]
START_NODE = 1
END_NODE = 3

# construct graph from matrix

# drive and check position based on nodes
