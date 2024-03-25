from shortest_path import priority_dict
from geometrics import path_length, edge_distance, check_path_intersect
from visible_vertices import check_valid_vertex

import sys


class HPoint(object):
    def __init__(self, point, homotopy_class=[]):
        self.point = point
        self.homotopy_class = homotopy_class

    def __eq__(self, hpoint):
        return hpoint and self.point == hpoint.point and self.homotopy_class == hpoint.homotopy_class

    def __ne__(self, hpoint):
        return not self.__eq__(hpoint)

    def __lt__(self, hpoint):
        """ This is only needed for shortest path calculations where heapq is
            used. When there are two points of equal distance, heapq will
            instead evaluate the Points, which doesnt work in Python 3 and
            throw a TypeError."""
        return hash(self) < hash(hpoint)

    def __str__(self):
        return "(%.2f, %.2f)" % (self.point.x, self.point.y)

    def __hash__(self):
        return self.point.x.__hash__() ^ self.point.y.__hash__()

    def __repr__(self):
        return "HPoint(%.2f, %.2f)" % (self.point.x, self.point.y)


def update_h_signature(hpoint1, hpoint2, rep_points):
    """
        :param hpoint1:  precedent node
        :param hpoint2: successor node
        :param rep_points: representative points in obstacle
        :return:
        """
    x_min = min(hpoint1.point.x, hpoint2.point.x)
    x_max = max(hpoint1.point.x, hpoint2.point.x)
    y_min = min(hpoint1.point.y, hpoint2.point.y)
    y_max = max(hpoint1.point.y, hpoint2.point.y)
    hpoint2.homotopy_class = hpoint1.homotopy_class.copy()
    if x_min == x_max:
        return
    # find candidate rep_points :
    rep_points_selected = []
    rep_points_candidate = []
    for point in rep_points:
        if x_min < point.x < x_max and point.y <= y_max:
            rep_points_selected.append(point)
        elif x_min == point.x or x_max == point.x:
            sys.exit('representative points error!')
    # detection in two cases as in Oren Zalman's paper
    k = (hpoint2.point.y - hpoint1.point.y) / (hpoint2.point.x - hpoint1.point.x)
    if k >= 0:
        b = 0
    else:
        b = y_max - y_min
    for point in rep_points_selected:
        if (point.y - y_min) <= k * (point.x - x_min) + b:
            rep_points_candidate.append(point)
    # sort the candidate points
    if hpoint1.point.x < hpoint2.point.x:
        newlist = sorted(rep_points_candidate, key=lambda pt: pt.x, reverse=False)
        inverse = 1
    elif hpoint1.point.x > hpoint2.point.x:
        newlist = sorted(rep_points_candidate, key=lambda pt: pt.x, reverse=True)
        inverse = -1
    for point in newlist:
        if len(hpoint2.homotopy_class) > 0 and hpoint2.homotopy_class[-1] == -point.polygon_id * inverse:
            hpoint2.homotopy_class.pop()
        else:
            hpoint2.homotopy_class.append(point.polygon_id * inverse)
    return


def path_homotopy(path, rep_points):
    hpath = [HPoint(p) for p in path]
    for i in range(len(path)-1):
        update_h_signature(hpath[i], hpath[i + 1], rep_points)
    return hpath[-1].homotopy_class


def hsignature_concat(h1, h2):
    if len(h1) == 0:
        return h2
    else:
        for item in h2:
            if len(h1) > 0 and item == -h1[-1]:
                h1.pop()
            else:
                h1.append(item)
    return h1


def shortest_homotopy_path(graph, init_config, tar_config, rep_points, add_to_visgraph = None):
    """
    :param ini_config: initial cable configuration, with the form of [v_1, v_2,...,vn]
    :param tar_config: target cable configuration
    :param graph:
    :param add_to_visgraph:
    :return:
    """
    if len(init_config) > 1:
        for i in range(len(init_config)-1):
            update_h_signature(init_config[i], init_config[i+1], rep_points)
    if len(tar_config) > 1:
        for i in range(len(tar_config)-1):
            update_h_signature(tar_config[i], tar_config[i+1], rep_points)

    h = hsignature_concat([-x for x in init_config[-1].homotopy_class], tar_config[-1].homotopy_class)

    origin = HPoint(init_config[-1].point)
    destination = HPoint(tar_config[-1].point)

    origin.homotopy_class = []
    destination.homotopy_class = h

    print("init homotopy class : ", origin.homotopy_class)
    print("target homotopy class : ", destination.homotopy_class)

    D, P =  dijkstra_homo(graph, origin, destination, rep_points, add_to_visgraph)

    path = []
    if destination in P.keys():
        while 1:
            path.append(destination)
            if destination == origin: break
            destination = P[destination]
    path.reverse()
    return path

def valid_path_homotopy(graph, origin, destination, contraint, rep_points, add_to_visgraph=None):
    paths = []
    path = []
    is_visited = {}
    is_visited[origin] = False
    is_visited[destination] = False
    for v in graph.graph.get_points():
        is_visited[v] = False
    dfs_homotopy(paths, path, is_visited, graph, origin, destination, contraint, rep_points, add_to_visgraph)

    ## remove duplicate elements:
    paths_newlist = []
    ## remove invalid path
    for e in paths:
        is_valid = check_path_intersect(e[0])
        if is_valid:
            paths_newlist.append(e)

    return paths_newlist


def dfs_homotopy(paths, path, is_visited, graph, origin, destination, contraint, rep_points, add_to_visgraph):
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
        h = path_homotopy(path, rep_points)
        if d <= contraint["cable_length"] and h==contraint["homotopy"] and check_path_intersect(path):
            paths.append([path.copy(), d])
        else:
            path.pop()
            is_visited[origin] = False
            return
        if len(paths) > 1:
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
                dfs_homotopy(paths, path, is_visited, graph, i, destination, contraint, rep_points, add_to_visgraph)
    path.pop()
    is_visited[origin] = False
    return




def astar_homo(graph,origin, destination, rep_points, add_to_visgraph):
    P = {}  #came_from
    D = {}  # gScore
    D[origin] = 0
    Q = priority_dict()
    Q[origin] = 0

    for v in Q:
        if v in destination: break
        edges = graph[v.point]
        if add_to_visgraph != None and len(add_to_visgraph[v.point]) > 0:
            edges = add_to_visgraph[v.point] | graph[v.point]
        for e in edges:
            w = e.get_adjacent(v.point)
            w_homo = HPoint(w)
            update_h_signature(v, w_homo, rep_points)
            elength = D[v] + 10*edge_distance(v.point, w_homo.point)
            if w_homo not in D or (w_homo in D and elength < D[w_homo]):
                D[w_homo] = elength
                priority = elength + edge_distance(w_homo.point, destination[-1].point)
                Q[w_homo] = priority
                P[w_homo] = v
    index = destination.index(v)
    for i in range(index+1, len(destination)):
        D[destination[i]] = D[destination[i-1]] + edge_distance(destination[i-1].point, destination[i].point)
        P[destination[i]] = destination[i-1]
    return (D, P)


def dijkstra_search(graph, origin, destination):
    P = {}  # came_from
    D = {}  # gScore
    D[origin] = 0
    Q = priority_dict()
    Q[origin] = 0
    for v in Q:

        D[v] = Q[v]
        if v == destination: break
        successors = []
        if v in graph.keys():
            successors.append(graph[v])
        for item in graph.keys():
            if graph[item] == v:
                successors.append(item)

        for w in successors:
            if w not in D:
                elength = D[v] + edge_distance(v.point, w.point)
                if w not in Q or Q[w]<elength:
                    Q[w] = elength
                    P[w] = v
    return (D, P)


def dijkstra_homo(graph, origin, destination, rep_points, add_to_visgraph):
    L_MAX = 2700
    D = {}
    P = {}
    Q = priority_dict()
    Q[origin] = 0
    count = 1
    for v in Q:
        D[v] = Q[v]
        if Q[v] > L_MAX: break

        edges = graph[v.point]

        if add_to_visgraph != None and len(add_to_visgraph[v.point]) > 0:
            edges = add_to_visgraph[v.point] | graph[v.point]

        for e in edges:
            w = e.get_adjacent(v.point)
            w_homo = HPoint(w)
            update_h_signature(v, w_homo, rep_points)
            elength = D[v] + edge_distance(v.point, w_homo.point)

            if elength < L_MAX:
                if w_homo in D:
                    if elength < D[w_homo]:
                        raise ValueError
                elif w_homo not in Q or elength < Q[w_homo]:
                    Q[w_homo] = elength
                    P[w_homo] = v
                    count = count+1
    print("count :", count)
    return (D, P)


