from geometrics import *
from visgraph import VisGraph
from scipy.optimize import linear_sum_assignment
from schgraph import *
from shortest_homotopy_path import path_homotopy
import copy
from bottleneckBM import BottleneckBM, Matching
from vns import SearchVNS, SearchVNSSimple
import func_timeout


def generate_vis_graph(polygon_list, display_range):
    polygons = []
    for id_, polygon in enumerate(polygon_list):
        item = []
        for point in polygon:
            tmp = point[1]
            item.append(Point(point[0], display_range[1] - tmp, id_ + 1))
        polygons.append(item)
    g = VisGraph()
    g.build(polygons, status=False)
    return g


def min_weight_match(cost_matrix):
    n = len(cost_matrix)
    row_ind, col_ind = linear_sum_assignment(np.array(cost_matrix))
    solution = [(i, col_ind[i]) for i in range(n)]
    solution.sort(key=lambda e: cost_matrix[e[0]][e[1]])
    return solution


def shortest_configs(x_pts, y_pts, display_range, g):
    cost_matrix = []
    min_weight_edges = {}
    n = len(x_pts)
    start_list = [Point(x_pts[i][0], display_range[1] - x_pts[i][1]) for i in range(n)]
    end_list = [Point(y_pts[j][0],  display_range[1] - y_pts[j][1]) for j in range(n)]
    add_to_visg = g.complete_graph(start_list, end_list)

    for i in range(n):
        start = start_list[i]
        shortest_config = g.djikstra_path(start, end_list, add_to_visg)
        e = []
        for j in range(n):
            min_weight_edges[(i, j)] = [copy.deepcopy(shortest_config[j])]
            e.append(shortest_config[j][1])
        cost_matrix.append(e)
    return [cost_matrix, min_weight_edges]


def homotopy_shorten(path, g, rep_points):
    n = len(path)
    if not check_path_intersect(path):
        # return shortest path
        shortest_config = g.djikstra_single_path(path[0], path[-1])
        return shortest_config[0]
    if n < 3:
        return path
    else:
        h = path_homotopy(path, rep_points)
        new_path = g.homotopic_path(path[0], path[-1], path_length(path), h, rep_points)
        if new_path == []:
            print("error occurs")
            print(path)
        return new_path[0][0]


def remove_crossing(pathset, g, rep_points):
    cross = cross_exist(pathset)
    cnt = 0
    while cross[0] >= 0:
        cnt = cnt +1
        p1 = pathset[cross[1]]
        p2 = pathset[cross[2]]
        if cross[0]==0:
            p1_i = p1[cross[3]]
            p2_i = p1[cross[3]+1]
            p1_j = p2[cross[4]]
            p2_j = p2[cross[4]+1]
            intersection_pt = find_intersect_point(p1_i,p2_i,p1_j,p2_j)
            new_path1 = p1[:cross[3]+1] + [intersection_pt] + p2[cross[4]+1:]
            new_path2 = p2[:cross[4]+1] + [intersection_pt] + p1[cross[3]+1:]
        else:
            new_path1 = p1[:cross[3]+1] + p2[cross[4]+1:]
            new_path2 = p2[:cross[4]+1] + p1[cross[3]+1:]
        pathset[cross[1]] = homotopy_shorten(new_path1, g, rep_points)
        pathset[cross[2]] = homotopy_shorten(new_path2, g, rep_points)
        cross = cross_exist(pathset)
    cross = cross_exist(pathset)
    print("cross: {} , cnt: {}".format(cross[0], cnt))
    return


def search_paths(x_pts, y_pts, g, rep_points, display_range, vns_param):
    '''
    x_pts:  depart points
    y_pts:  arrival points
    g: visibility graph
    rep_points: reference points to compute the homotopy class
    vns_param: a dictionary of parameters for VNS
    return: a set of paths
    '''
    # calculate distance between anchor points:
    n = len(x_pts)
    assert len(x_pts) == len(y_pts)
    dist_anchor = [[[(x_pts[j][0] - x_pts[i][0]) ** 2 + (x_pts[j][1] - x_pts[i][1]) ** 2, i] for i in range(n)] for j in
                   range(n)]
    for e in dist_anchor:
        e.sort(key=lambda x: x[0])

    # compute upper bound by lsap
    cost_matrix, min_weight_edges = shortest_configs(x_pts, y_pts, display_range, g)
    solution_lsap = min_weight_match(cost_matrix)
    pathset = []
    for i in range(n):
        pathset.append(min_weight_edges[(solution_lsap[i][0], solution_lsap[i][1])][0][0])
    solution_lsap.sort(key=lambda e: e[0])
    remove_crossing(pathset, g, rep_points)


    res = search_subgraph(pathset)
    result = []
    for e in res:
        ms = computeSubgraphsMakespan(pathset, e)
        result.append(ms)
    sup_bound_lsap = max(result)
    sup_bound_index = res[result.index(sup_bound_lsap)]

    match = [[] for _ in range(n)]
    crossing_table = np.ones((n * n, n * n), int) * (-1)
    index_list = []
    xpts_index = {}
    ypts_index = {}
    for i in range(n):
        xpts_index[x_pts[i]] = i
        ypts_index[y_pts[i]] = i

    subgraph_index = []
    for path_num, p in enumerate(pathset):
        start = (p[0].x, display_range[1] - p[0].y)
        end = (p[-1].x, display_range[1] - p[-1].y)
        if start in xpts_index.keys():
            start_index = xpts_index[start]
            end_index = ypts_index[end]
        else:
            start_index = ypts_index[start]
            end_index = xpts_index[end]
        if path_num in sup_bound_index:
            subgraph_index.append(start_index)
        match[start_index] = [end_index, 0]
        index_list.append(start_index * n + end_index)
    for i in range(n - 1):
        for j in range(i + 1, n):
            crossing_table[index_list[i], index_list[j]] = 0
            crossing_table[index_list[j], index_list[i]] = 0

    # calculate distance between anchor-destination points
    dist_anchor_dest = [[[min_weight_edges[(j, i)][0][1], i] for i in range(n)] for j in range(n)]
    for e in dist_anchor_dest:
        e.sort(key=lambda x: x[0])
    sorted_dist_anchor_dest = [[0 for i in range(n)] for j in range(n)]
    for i in range(n):
        for j in range(n):
            sorted_dist_anchor_dest[i][dist_anchor_dest[i][j][1]] = j

    kmax = vns_param["kmax"]
    neighbor = vns_param["neighbor"]
    num_path = vns_param["vns2_num_path"]


    # VNS search, combine vns1 + vns2
    # vns1 search
    print("vns1 start")
    vns1 = SearchVNSSimple(g, match, kmax, crossing_table, min_weight_edges, x_pts, y_pts, subgraph_index,
                           math.ceil(sup_bound_lsap), neighbor, 1)
    try:
        vns1.local_search()
    except func_timeout.exceptions.FunctionTimedOut:
        print("vns1 timeout")

    match = [[e[0], e[1]] for e in vns1.solution]
    res = findSubgraphs(n, min_weight_edges, match)
    result = []
    for e in res:
        ms = findSubgraphsMakespan(min_weight_edges, match, e)
        result.append(ms)
    sup_bound_index1 = res[result.index(max(result))]
    sup_bound_vns1 = math.ceil(vns1.sup_bound)
    print("vns2 start")


    # VNS2 search
    crossing_table = [[defaultdict() for _ in range(n * n)] for _ in range(n * n)]
    for i in range(n):
        for j in range(n):
            index1 = (match[i][0]) * n + match[i][1]
            index2 = (match[j][0]) * n + match[j][1]
            if index1 < index2:
                crossing_table[index1][index2][(0, 0)] = [0, 0]
            elif index1 > index2:
                crossing_table[index2][index1][(0, 0)] = [0, 0]


    vns2 = SearchVNS(g, match, kmax, crossing_table, min_weight_edges, x_pts, y_pts, sup_bound_index1,
                     sup_bound_vns1, neighbor, num_path, display_range)


    try:
        vns2.local_search()
    except func_timeout.exceptions.FunctionTimedOut:
        print("vns2 timeout")

    paths = []
    for i in range(n):
        paths.append(vns2.configs[(i, vns2.solution[i][0])][vns2.solution[i][1]][0])
    return paths
