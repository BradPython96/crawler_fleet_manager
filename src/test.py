from util import search_paths, generate_vis_graph
from geometrics import Point as pt
from collections import defaultdict
import matplotlib.pyplot as plt
from schgraph import Schgraph
from random import random
from math import sqrt

def path_finding(outer_range, obstacles, departs, arrivals):
    # to adapt the coordinate system
    x_pts = [(e[0], e[1]) for e in departs]
    y_pts = [(e[0], e[1]) for e in arrivals]

    # build visibility graph
    g = generate_vis_graph(obstacles, outer_range)
    # rep points are defined to compute the homotopy class
    rep_points = []
    random_offset = [random() for i in range(len(obstacles))]
    for i, polygon in enumerate(obstacles):
        sum_x = 0
        sum_y = 0
        for pg in polygon:
            sum_x = sum_x + pg[0]
            sum_y = sum_y + outer_range[1] - pg[1]
        rep_points.append(pt(sum_x / len(polygon) + (sqrt(i)/len(obstacles) + random_offset[i]*0.1)*0.5 / len(obstacles), sum_y /len(polygon) + 0.2, i + 1))


    vns_param = {
        "kmax": 7,
        "neighbor": 10,
        "vns2_num_path": 100
    }

    paths = search_paths(x_pts, y_pts, g, rep_points, outer_range, vns_param)
    paths = [[(e.x, outer_range[1] - e.y) for e in p] for p in paths]
    
    paths_for_graph = []
    for path in paths :
    	paths_for_graph.append([pt(p[0],p[1]) for p in path])
    	
    
    schgraph = Schgraph(paths_for_graph)
    schgraph.construct()
    
    graph_res=defaultdict(list)
    for g in schgraph.graph:
    
        graph_res[(g[0], (g[1].x,g[1].y))] =[(int(schgraph.graph[g][i][0]),(int(schgraph.graph[g][i][1].x),int(schgraph.graph[g][i][1].y))) for i in range(len(schgraph.graph[g]))]
        
    
    return paths, graph_res


def test():
    '''
      input coordinate system
      -------- x
      |
      |
      |
      | y
    '''
    inputs = {
        "range": [326,235],
        "obstacles": [[(121,80),(132,80),(132,91),(121,91)],
        [(188,78),(199,78),(199,87),(188,87)],
        [(162,103),(172,103),(172,119),(162,119)]],
        "departs":  [(184,108), (194,104)],
        "arrivals":  [(127,123), (140, 126)]

    }

    outer_range = inputs["range"]
    obstacles = inputs["obstacles"]
    departs = inputs["departs"]
    arrivals = inputs["arrivals"]

    paths, graph = path_finding(outer_range, obstacles, departs, arrivals)

    
    prio_dict=defaultdict(list)
    for o in obstacles :
        for p in o:
            prio_dict[(p[0],p[1])] = []
            for i in range (6):
                for k in graph :
                    if int(k[0])==i and (int(k[1][0]), int(k[1][1])) == p:
                        prio = len(graph[i,p])-1
                        while(len(prio_dict[(p[0],p[1])])<=prio):
                            prio_dict[(p[0],p[1])].append(-1)   
                        prio_dict[(p[0],p[1])][prio] = i
    
    debug_visualization = False
    if debug_visualization:
        plt.figure()
        for p in obstacles:
            obs_x = [e[0] for e in p] + [p[0][0]]
            obs_y = [e[1] for e in p] + [p[0][1]]
            plt.fill(obs_x, obs_y, 'gray')
        for path in paths:
            for i in range(len(path)-1):
                plt.plot([path[i][0], path[i+1][0]], [path[i][1], path[i+1][1]], 'green')

        plt.show()


if __name__=="__main__":
    test()
