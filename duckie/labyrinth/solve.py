from duckie.labyrinth.graph import Graph, Vertex

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
graph = Graph()

a = Vertex(0, 0, name='a')
b = Vertex(5, 0, name='b')
c = Vertex(-5, 0, name='c')
d = Vertex(0, 5, name='d')
e = Vertex(5, 5, name='e')
f = Vertex(-5, 5, name='f')
g = Vertex(5, 10, name='g')
h = Vertex(0, 10, name='h')

# Set edges manually

graph.connect_vertices(a, b) # set weight with weight=x
graph.connect_vertices(a, c)
graph.connect_vertices(a, d)
graph.connect_vertices(b, e)
graph.connect_vertices(d, e)
graph.connect_vertices(e, g)
graph.connect_vertices(g, h)

# Set edges via matrix

# graph.apply_edges(
#   [a, b, c, d, e, f, g, h],
#   [
#     [0, 5, 5, 5, 0, 0, 0, 0],
#     [5, 0, 0, 0, 5, 0, 0, 0],
#     [5, 0, 0, 0, 0, 5, 0, 0],
#     [5, 0, 0, 0, 5, 5, 0, 0],
#     [0, 5, 0, 5, 0, 0, 5, 0],
#     [0, 0, 5, 5, 0, 0, 0, 0],
#     [0, 0, 0, 0, 5, 0, 0, 5],
#     [0, 0, 0, 0, 0, 0, 5, 0],
#   ]
# )

path = graph.find_shortest_path(a, h)

points = [[v.x, v.y] for v in path]

# drive and check position based on nodes
