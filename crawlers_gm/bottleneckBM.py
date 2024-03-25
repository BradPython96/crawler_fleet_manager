import collections
from .bmgraph import *

class BottleneckBM(object):
    def __init__(self, G, distance_table):
        self.G = G
        self.solution = []
        self.distance_table = distance_table
        G_prime = BMGraph(G.n)
        self.HopcroftKarp = HopcroftKarp(G_prime)
        self.solution = self.HopcroftKarp.match(self.G.n)

    def match(self):
        if self.G.n < 2:
            return self.getSolution()

        edges_pools = [e for e in self.G.edges]
        while len(self.solution) >= self.G.n:
            self.solution.sort(key=lambda e: self.distance_table[e[0]-1][-e[1]-1])
            edge_length = [self.distance_table[e[0]-1][-e[1]-1] for e in self.solution]
            max_length = max(edge_length)
            index = edge_length.index(max_length)
            if index < 1:
                break

            new_edges = []

            for e in edges_pools:
                if self.distance_table[e[0]-1][-e[1]-1] < edge_length[index]:
                    new_edges.append(e)

            edges_pools = [e for e in new_edges]

            self.HopcroftKarp.G.updateEdges(new_edges)

            existing_match = self.solution[:index]

            result = self.HopcroftKarp.match(self.G.n)
            if len(result) < self.G.n:
                break
            else:
                self.solution = result

        return self.getSolution()

    def getSolution(self):
        return self.solution


class HopcroftKarp(object):

    INFINITY = -1

    def __init__(self, G):
        self.G = G

    def match(self, k, existing_match = None):
        self.pair = {}
        self.dist = {}
        self.q = collections.deque()
        for v in self.G.X + self.G.Y:
            self.pair[v] = None
            self.dist[v] = HopcroftKarp.INFINITY


        if existing_match != None :
            for e in existing_match:
                self.pair[e[0]] = e[1]
                self.pair[e[1]] = e[0]
            matching = len(existing_match)
        else:
            matching = 0

        while matching < k and self.bfs():
            for v in self.G.X:
                if matching >= k:
                    break
                if self.pair[v]==None and self.dfs(v):
                    matching = matching + 1
                    if matching == k:
                        break

        edges = [(u, self.pair[u]) for u in self.pair.keys() if u > 0 and self.pair[u] != None]
        return edges

    def dfs(self, v):
        if v != None:
            for u in self.G.neighbors[v]:
                if self.dist[self.pair[u]] == self.dist[v] + 1 and self.dfs(self.pair[u]):
                    self.pair[u] = v
                    self.pair[v] = u
                    return True
            self.dist[v] = HopcroftKarp.INFINITY
            return False

        return True

    def bfs(self):
        for v in self.G.X:
            if self.pair[v] == None:
                self.dist[v] = 0
                self.q.append(v)
            else:
                self.dist[v] = HopcroftKarp.INFINITY

        self.dist[None] = HopcroftKarp.INFINITY

        while len(self.q) > 0:
            v = self.q.popleft()
            if v != None:
                for u in self.G.neighbors[v]:
                    if self.dist[self.pair[u]] == HopcroftKarp.INFINITY:
                        self.dist[self.pair[u]] = self.dist[v]+1
                        self.q.append(self.pair[u])

        return self.dist[None] != HopcroftKarp.INFINITY
