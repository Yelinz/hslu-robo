import math


class Vertex:
    def __init__(self, x, y, name=''):
        self.name = name
        self.x = x
        self.y = y

        self.cost = math.inf
        self.visited = False
        self.predecessor = None

    def set_predecessor(self, predecessor, cost):
        self.predecessor = predecessor
        self.cost = cost

    def visit(self):
        self.visited = True

    def get_path(self):
        if self.predecessor is None:
            return [self]

        path = self.predecessor.get_path()
        path.append(self)

        return path


class Edge:
    def __init__(self, from_vertex, to_vertex, weight):
        self.from_vertex = from_vertex
        self.to_vertex = to_vertex
        self.weight = weight


class Graph:
    def __init__(self):
        self.vertices = set()
        self.edges = []

    def connect_vertices(self, from_vertex, to_vertex, weight=1, direction='bidirectional'):
        self.edges.append(Edge(from_vertex, to_vertex, weight))
        if direction == 'bidirectional':
            self.edges.append(Edge(to_vertex, from_vertex, weight))

        self.vertices.add(from_vertex)
        self.vertices.add(to_vertex)
    
    def apply_edges(self, vertices, matrix):
        self.vertices = vertices

        for from_vertex_i, to_list in enumerate(matrix):
            for to_vertex_i, weight in enumerate(to_list):
                if weight == 0:
                    continue

                self.edges.append(Edge(vertices[from_vertex_i], vertices[to_vertex_i], weight))

    def unvisited(self):
        return filter(lambda v: v.visited is False, self.vertices)

    def vertex_with_lowest_cost(self):
        return min(self.unvisited(), key=lambda v: v.cost)

    def unvisited_neighbors(self, vertex):
        return [
            edge for edge in self.edges
            if (edge.from_vertex == vertex and edge.to_vertex.visited is False)
        ]

    def find_shortest_path(self, start_vertex, end_vertex):
        start_vertex.cost = 0

        while end_vertex.visited is False:
            vertex = self.vertex_with_lowest_cost()
            if vertex is None:
                raise 'start and end vertices are not connected in graph'

            edges = self.unvisited_neighbors(vertex)

            for edge in edges:
                if edge.to_vertex.cost > vertex.cost + edge.weight:
                    edge.to_vertex.set_predecessor(vertex, edge.weight)

            vertex.visit()

        return end_vertex.get_path()
