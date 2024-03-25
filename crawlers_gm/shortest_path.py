from .geometrics import path_length, edge_distance, check_path_intersect
from heapq import heapify, heappush, heappop
from .visible_vertices import check_valid_vertex
try:
    dict.iteritems
except AttributeError:
    # Python 3
    def iteritems(d):
        return iter(d.items())
else:
    # Python 2
    def iteritems(d):
        return d.iteritems()

def dijkstra_single(graph, origin, destination, add_to_visgraph):
    D = {}
    P = {}
    Q = priority_dict()
    Q[origin] = 0

    for v in Q:
        D[v] = Q[v]
        if v == destination: break

        edges = graph[v]
        if add_to_visgraph != None and len(add_to_visgraph[v]) > 0:
            edges = add_to_visgraph[v] | graph[v]
        for e in edges:
            w = e.get_adjacent(v)
            elength = D[v] + edge_distance(v, w)
            if w in D:
                if elength < D[w]:
                    raise ValueError
            elif w not in Q or elength < Q[w]:
                Q[w] = elength
                P[w] = v
    return (D, P)


def shortest_single_path(graph, origin, destination, add_to_visgraph=None):
    D, P = dijkstra_single(graph, origin, destination, add_to_visgraph)
    path = []
    while 1:
        path.append(destination)
        if destination == origin: break
        destination = P[destination]
    path.reverse()
    return [path, path_length(path)]


def shortest_path(graph, origin, destination_list, add_to_visgraph=None):
    D, P = dijkstra(graph, origin, add_to_visgraph)
    paths = []
    for destination in destination_list:
        path = []
        while 1:
            path.append(destination)
            if destination == origin: break
            destination = P[destination]
        path.reverse()
        paths.append([path, path_length(path)])
    return paths


def dijkstra(graph, origin, add_to_visgraph):
    D = {}
    P = {}
    Q = priority_dict()
    Q[origin] = 0

    for v in Q:
        D[v] = Q[v]
        edges = graph[v]
        if add_to_visgraph != None and len(add_to_visgraph[v]) > 0:
            edges = add_to_visgraph[v] | graph[v]
        for e in edges:
            w = e.get_adjacent(v)
            elength = D[v] + edge_distance(v, w)
            if w in D:
                if elength < D[w]:
                    raise ValueError
            elif w not in Q or elength < Q[w]:
                Q[w] = elength
                P[w] = v
    return (D, P)


def dfs(paths, path, is_visited, graph, origin, destination, contraint, add_to_visgraph):
    if path_length(path) > contraint["cable_length"]:
        return
    is_visited[origin] = True
    path.append(origin)
    if len(path) > 2:
        result = check_valid_vertex(graph, path[-2], path[-3], path[-1])
        if not result:
            path.pop()
            is_visited[origin] = False
            return

    if origin == destination:
        d = path_length(path)
        if d <= contraint["cable_length"]:
            paths.append([path.copy(), d])
        else:
            path.pop()
            is_visited[origin] = False
            return
        if len(paths) > contraint["num_path"]:
            list_ = [e[1] for e in paths]
            index_ = list_.index(max(list_))
            del paths[index_]
            del list_[index_]
            contraint["cable_length"] = max(list_)
    else:
        edges = graph.visgraph[origin]
        if add_to_visgraph != None and len(add_to_visgraph[origin]) > 0:
            edges = add_to_visgraph[origin] | graph.visgraph[origin]
        for e in edges:
            i = e.get_adjacent(origin)
            if not is_visited[i]:
                dfs(paths, path, is_visited, graph, i, destination, contraint, add_to_visgraph)
    path.pop()
    is_visited[origin] = False
    return


def valid_path(graph, origin, destination, contraint, add_to_visgraph=None):
    paths = []
    path = []
    is_visited = {}
    is_visited[origin] = False
    is_visited[destination] = False
    for v in graph.graph.get_points():
        is_visited[v] = False
    dfs(paths, path, is_visited, graph, origin, destination, contraint, add_to_visgraph)

    ## remove duplicate elements:
    paths_newlist = []
    ## remove invalid path
    for e in paths:
        is_valid = check_path_intersect(e[0])
        if is_valid:
            paths_newlist.append(e)
    paths_newlist.sort(key=lambda x: x[1])
    return paths_newlist


class priority_dict(dict):
    """Dictionary that can be used as a priority queue.

    Keys of the dictionary are items to be put into the queue, and values
    are their respective priorities. All dictionary methods work as expected.
    The advantage over a standard heapq-based priority queue is that priorities
    of items can be efficiently updated (amortized O(1)) using code as
    'thedict[item] = new_priority.'

    Note that this is a modified version of
    https://gist.github.com/matteodellamico/4451520 where sorted_iter() has
    been replaced with the destructive sorted iterator __iter__ from
    https://gist.github.com/anonymous/4435950
    """
    def __init__(self, *args, **kwargs):
        super(priority_dict, self).__init__(*args, **kwargs)
        self._rebuild_heap()

    def _rebuild_heap(self):
        self._heap = [(v, k) for k, v in iteritems(self)]
        heapify(self._heap)

    def smallest(self):
        heap = self._heap
        v, k = heap[0]
        while k not in self or self[k] != v:
            heappop(heap)
            v, k = heap[0]
        return k

    def pop_smallest(self):
        heap = self._heap
        v, k = heappop(heap)
        while k not in self or self[k] != v:
            v, k = heappop(heap)
        del self[k]
        return k

    def __setitem__(self, key, val):
        super(priority_dict, self).__setitem__(key, val)

        if len(self._heap) < 2 * len(self):
            heappush(self._heap, (val, key))
        else:
            self._rebuild_heap()

    def setdefault(self, key, val):
        if key not in self:
            self[key] = val
            return val
        return self[key]

    def update(self, *args, **kwargs):
        super(priority_dict, self).update(*args, **kwargs)
        self._rebuild_heap()

    def __iter__(self):
        def iterfn():
            while len(self) > 0:
                x = self.smallest()
                yield x
                del self[x]
        return iterfn()
