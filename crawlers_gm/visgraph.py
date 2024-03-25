import copy
from sys import stdout, version_info
from multiprocessing import Pool
from tqdm import tqdm
from .graph import Graph
from .geometrics import *
from .visible_vertices import visible_vertices, point_in_polygon, edge_intersect, closest_point
from .shortest_path import shortest_path, valid_path, shortest_single_path
from .shortest_homotopy_path import valid_path_homotopy

PYTHON3 = version_info[0] == 3
if PYTHON3:
    xrange = range
    import pickle
else:
    import cPickle as pickle


class VisGraph(object):

    def __init__(self):
        self.graph = None
        self.visgraph = None
        self.length = 99999999

    def load(self, filename):
        """Load obstacle graph and visibility graph. """
        with open(filename, 'rb') as load:
            self.graph, self.visgraph = pickle.load(load)

    def save(self, filename):
        """Save obstacle graph and visibility graph. """
        with open(filename, 'wb') as output:
            pickle.dump((self.graph, self.visgraph), output, -1)

    def build(self, input, workers=1, status=True):
        """Build visibility graph based on a list of polygons.

        The input must be a list of polygons, where each polygon is a list of
        in-order (clockwise or counter clockwise) Points. It only one polygon,
        it must still be a list in a list, i.e. [[Point(0,0), Point(2,0),
        Point(2,1)]].
        Take advantage of processors with multiple cores by setting workers to
        the number of subprocesses you want. Defaults to 1, i.e. no subprocess
        will be started.
        Set status=False to turn off the statusbar when building.
        """

        self.graph = Graph(input)
        self.visgraph = Graph([])

        points = self.graph.get_points()
        batch_size = 10

        x_max = max([point.x for point in points])
        x_min = min([point.x for point in points])
        y_max = max([point.y for point in points])
        y_min = min([point.y for point in points])
        self.length = sqrt((x_max - x_min) ** 2 + (y_max - y_min) ** 2) * 4

        if workers == 1:
            for batch in tqdm([points[i:i + batch_size]
                               for i in xrange(0, len(points), batch_size)],
                            disable=not status):
                for edge in _vis_graph(self.graph, batch):
                    if not check_reduced(self.graph, edge, self.length):
                         self.visgraph.add_edge(edge)

        else:
            pool = Pool(workers)
            batches = [(self.graph, points[i:i + batch_size])
                       for i in xrange(0, len(points), batch_size)]

            results = list(tqdm(pool.imap(_vis_graph_wrapper, batches), total=len(batches),
                disable=not status))
            for result in results:
                for edge in result:
                    if not check_reduced(self.graph, edge, self.length):
                        self.visgraph.add_edge(edge)

    def find_visible(self, point):
        """Find vertices visible from point."""
        return visible_vertices(point, self.graph)

    def update(self, points, origin=None, destination=None):
        """Update visgraph by checking visibility of Points in list points."""

        for p in points:
            for v in visible_vertices(p, self.graph, origin=origin,
                                      destination=destination):
                self.visgraph.add_edge(Edge(p, v))
        return

    def djikstra_path(self, origin, destination_list, add_to_visg):
        '''
        origin_exists = origin in self.visgraph
        add_to_visg = Graph([])
        for destination in destination_list:
            dest_exists = destination in self.visgraph
            orgn = None if origin_exists else origin
            dest = None if dest_exists else destination
            if not origin_exists:
                for v in visible_vertices(origin, self.graph, destination=dest):
                    add_to_visg.add_edge(Edge(origin, v))
            if not dest_exists:
                for v in visible_vertices(destination, self.graph, origin=orgn):
                    add_to_visg.add_edge(Edge(destination, v))
        '''

        return shortest_path(self.visgraph, origin, destination_list, add_to_visg)

    def djikstra_single_path(self, origin, destination):
        origin_exists = origin in self.visgraph
        dest_exists = destination in self.visgraph
        if origin_exists and dest_exists:
            return shortest_path(self.visgraph, origin, destination)
        orgn = None if origin_exists else origin
        dest = None if dest_exists else destination
        add_to_visg = Graph([])
        if not origin_exists:
            for v in visible_vertices(origin, self.graph, destination=dest):
                add_to_visg.add_edge(Edge(origin, v))
        if not dest_exists:
            for v in visible_vertices(destination, self.graph, origin=orgn):
                add_to_visg.add_edge(Edge(destination, v))
        return shortest_single_path(self.visgraph, origin, destination, add_to_visg)

    def candidate_path(self, origin, destination, cable_length, num_path):
        origin_exists = origin in self.visgraph
        dest_exists = destination in self.visgraph
        if origin_exists and dest_exists:
            return shortest_path(self.visgraph, origin, destination)
        orgn = None if origin_exists else origin
        dest = None if dest_exists else destination
        add_to_visg = Graph([])
        if not origin_exists:
            for v in visible_vertices(origin, self.graph, destination=dest):
                add_to_visg.add_edge(Edge(origin, v))
        if not dest_exists:
            for v in visible_vertices(destination, self.graph, origin=orgn):
                add_to_visg.add_edge(Edge(destination, v))
            # print("destination visible points : ", len(add_to_visg.get_edges()))
        contraint = {}
        contraint["cable_length"] = cable_length
        contraint["num_path"] = num_path
        res = valid_path(self, origin, destination, contraint, add_to_visg)
        return res

    def complete_graph(self, origin_list, destination_list):
        add_to_visg = Graph([])
        for origin in origin_list:
            origin_exists = origin in self.visgraph
            if not origin_exists:
                for v in visible_vertices(origin, self.graph, destination_list=destination_list):
                    add_to_visg.add_edge(Edge(origin, v))
        for destination in destination_list:
            dest_exists = destination in self.visgraph
            if not dest_exists:
                for v in visible_vertices(destination, self.graph):
                    add_to_visg.add_edge(Edge(destination, v))
        return add_to_visg

    def homotopic_path(self, origin, destination, cable_length, homotopy, rep_points):
        origin_exists = origin in self.visgraph
        dest_exists = destination in self.visgraph
        if origin_exists and dest_exists:
            return shortest_single_path(self.visgraph, origin, destination)
        orgn = None if origin_exists else origin
        dest = None if dest_exists else destination
        add_to_visg = Graph([])
        if not origin_exists:
            for v in visible_vertices(origin, self.graph, destination=dest):
                add_to_visg.add_edge(Edge(origin, v))
            # print("origin visible points : ",len(add_to_visg.get_edges()))
        if not dest_exists:
            for v in visible_vertices(destination, self.graph, origin=orgn):
                add_to_visg.add_edge(Edge(destination, v))
            # print("destination visible points : ", len(add_to_visg.get_edges()))
        contraint = {}
        contraint["cable_length"] = cable_length
        contraint["homotopy"] = copy.deepcopy(homotopy)
        res = valid_path_homotopy(self, origin, destination, contraint, rep_points, add_to_visg)
        return res


def _vis_graph_wrapper(args):
    try:
        return _vis_graph(*args)
    except KeyboardInterrupt:
        pass


def _vis_graph(graph, points):
    visible_edges = []
    for p1 in points:
        for p2 in visible_vertices(p1, graph, scan='half'):
            visible_edges.append(Edge(p1, p2))
    return visible_edges


def check_reduced(graph, edge, length):
    polygon_id1 = edge.p1.polygon_id
    polygon_id2 = edge.p2.polygon_id
    is_reduced = False
    if polygon_id1 == polygon_id2:
        is_reduced = not check_bitangent(graph, edge, polygon_id1, length)
    else:
        check_polygon1 = check_bitangent(graph, edge, polygon_id1, length)
        check_polygon2 = check_bitangent(graph, edge, polygon_id2, length)
        is_reduced = not (check_polygon1 and check_polygon2)
    return is_reduced


def check_bitangent(graph, edge, polygon_id, length):
    is_bitangent = True
    v = unit_vector(edge.p1, edge.p2)
    extend1 = Point(edge.p1.x + (v.x * length), edge.p1.y + (v.y * length))
    extend2 = Point(edge.p2.x - (v.x * length), edge.p2.y - (v.y * length))
    polygon_points = graph.get_polygon_points()[polygon_id]
    polygon_edges = graph.get_polygons()[polygon_id]
    points_collinear = []
    for point in polygon_points:
        if ccw(edge.p1, extend1, point) == 0:
            points_collinear.append(point)
    points_collinear.append(edge.p1)
    points_collinear.append(edge.p2)
    for item in polygon_edges:
        if item.p1 not in points_collinear and item.p2 not in points_collinear:
            if edge_intersect(edge.p1, extend1, item) or edge_intersect(edge.p2, extend2, item):
                is_bitangent = False
                break
    return is_bitangent


def find_edges_in_angle(edges, pb, pv, pa):
    res= []
    ba = [pa.x - pb.x, pa.y - pb.y]
    bv = [pv.x - pb.x, pv.y - pb.y]
    for e in edges:
        w = e.get_adjacent(pb)
        if w == pa:
            continue
        bw = [w.x - pb.x, w.y - pb.y]

        dist = edge_distance(pb, w)
        c1 = (bw[0] * bv[1] - bw[1] * bv[0]) / (ba[0] * bv[1] - ba[1] * bv[0])
        c2 = - (bw[0] * ba[1] - bw[1] * ba[0]) / (ba[0] * bv[1] - ba[1] * bv[0])
        if c1 >= 0 and c2 >= 0:
            alpha = (bw[0] * ba[0] + bw[1] * ba[1]) / dist
            res.append([e, alpha])

    res.sort(key=lambda item: item[1], reverse=True)
    res = [e[0] for e in res]
    return res




