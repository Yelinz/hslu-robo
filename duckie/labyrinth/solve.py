from graph import Graph, Vertex
# construct graph from matrix
graph = Graph()

start = Vertex(0, 0)
a = Vertex(3, 0)
b = Vertex(7, 0)
c = Vertex(8, 0)
d = Vertex(9, 0)
e = Vertex(9, 1)
f = Vertex(1, -2)
g = Vertex(3, -2)
h = Vertex(7, -2)
i = Vertex(9, -2)
j = Vertex(10, -2)
k = Vertex(1, -4)
l = Vertex(3, -4)
m = Vertex(9, -4)
end = Vertex(11, 0)

graph.connect_vertices(start, a)
graph.connect_vertices(a, b)
graph.connect_vertices(a, g)
graph.connect_vertices(b, h)
graph.connect_vertices(d, c)
graph.connect_vertices(d, e)
graph.connect_vertices(d, end)
graph.connect_vertices(g, f)
graph.connect_vertices(g, l)
graph.connect_vertices(i, j)
graph.connect_vertices(i, h)
graph.connect_vertices(i, m)
graph.connect_vertices(i, d)
graph.connect_vertices(f, k)
graph.connect_vertices(l, m)

path = graph.find_shortest_path(start, end)

points = [[v.x, v.y] for v in path]

# drive and check position based on nodes
