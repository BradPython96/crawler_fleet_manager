import math, random

class Point(object):
    __slots__ = ('x', 'y','id')

    def __init__(self, x, y, id = -1):
        self.x = float(x)
        self.y = float(y)
        self.id = id

    def __repr__(self):
        return "Point(%.2f, %.2f)" % (self.x, self.y)

    def __str__(self):
        return "(%.2f, %.2f)" % (self.x, self.y)

    def dual_line(self):
        return Line(self.x/10, -self.y/10)

class Line():
    def __init__(self, m, b):
        self.m = m*10
        self.b = b*10



class Edge(object):
    __slots__ = ('p1', 'p2')

    def __init__(self, point1, point2):
        self.p1 = point1
        self.p2 = point2

    def get_adjacent(self, point):
        if point == self.p1:
            return self.p2
        return self.p1

    def __contains__(self, point):
        return self.p1 == point or self.p2 == point

    def __eq__(self, edge):
        if self.p1 == edge.p1 and self.p2 == edge.p2:
            return True
        if self.p1 == edge.p2 and self.p2 == edge.p1:
            return True
        return False

    def __ne__(self, edge):
        return not self.__eq__(edge)

    def __str__(self):
        return "({}, {})".format(self.p1, self.p2)

    def __repr__(self):
        return "Edge({!r}, {!r})".format(self.p1, self.p2)

    def __hash__(self):
        return self.p1.__hash__() ^ self.p2.__hash__()


class BMGraph:
    def __init__(self,n, xlist = None, ylist = None,edges = None):
        self.n = n
        if xlist != None:
            self.xlist = [item for item in xlist]
        if ylist != None:
            self.ylist = [item for item in ylist]

        self.X = [x+1 for x in range(n)]
        self.Y = [-y-1 for y in range(n)]
        self.edges = []
        self.neighbors = {}

        if edges != None:
            self.edges = [item for item in edges]
        else:
            for i in range(1, n+1):
                for j in range(1, n+1):
                    self.edges.append((i, -j))

        for e in self.edges:
            x, y = e
            self.neighbors[x] = self.neighbors.get(x, []) + [y]
            self.neighbors[y] = self.neighbors.get(y, []) + [x]

    def updateEdges(self, edges):
        self.edges = [item for item in edges]
        for i in self.neighbors.keys():
            self.neighbors[i] = []
        for e in self.edges:
            x, y = e
            self.neighbors[x] = self.neighbors.get(x, []) + [y]
            self.neighbors[y] = self.neighbors.get(y, []) + [x]


class Matching:

    def __init__(self, k, edges):
        self.k = k
        self.edges = [e for e in edges]
        self.update()

    def update(self):
        self.X_pair = {x:y for (x,y) in self.edges}
        self.Y_pair = {y:x for (x,y) in self.edges}

    def size(self):
        return len(self.edges)


