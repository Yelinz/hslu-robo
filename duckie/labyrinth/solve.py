from duckie.labyrinth.graph import Graph, Vertex
# construct graph from matrix
graph = Graph()

start = Vertex(0, 0, name='start') # start
a = Vertex(3, 0, name='a')
b = Vertex(7, 0, name='b')
c = Vertex(8, 0, name='c')
d = Vertex(9, 0, name='d')
end = Vertex(11, 0, name='end') # end
e = Vertex(9, -1, name='e')
f = Vertex(1, 2, name='f')
g = Vertex(3, 2, name='g')
h = Vertex(7, 2, name='h')
i = Vertex(9, 2, name='i')
j = Vertex(10, 2, name='j')
k = Vertex(1, 4, name='k')
l = Vertex(3, 4, name='l')
m = Vertex(9, 4, name='m')

# Set edges manually

graph.connect_vertices(start, a) # set weight with weight=x
graph.connect_vertices(a, b)
graph.connect_vertices(a, g)
graph.connect_vertices(b, h)
graph.connect_vertices(d, c)
graph.connect_vertices(d, e)
graph.connect_vertices(d, end)
graph.connect_vertices(g,f)
graph.connect_vertices(g,l)
graph.connect_vertices(i,j)
graph.connect_vertices(i,h)
graph.connect_vertices(i,m)
graph.connect_vertices(f,k)
graph.connect_vertices(l,m)

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
