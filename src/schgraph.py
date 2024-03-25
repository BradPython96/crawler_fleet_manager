import math
from collections import defaultdict, deque
from geometrics import *

class Schgraph(object):
    def __init__(self, path_list):
        # path list contains only non intersecting paths
        self.graph = defaultdict(list)
        self.graph_inv = defaultdict(list)
        self.V = []
        self.tsort = []
        self.path_list = path_list

    def construct(self):
        path_list = self.path_list
        n = len(path_list)
        for i in range(n):
            m = len(path_list[i])
            self.V.append((i, path_list[i][0]))
            for j in range(m-1):
                self.V.append((i, path_list[i][j+1]))
                self.graph[(i,path_list[i][j])].append((i, path_list[i][j+1]))
                self.graph_inv[(i, path_list[i][j+1])].append((i, path_list[i][j]))

        for i in range(n-1):
            for j in range(i+1,n):
                if len(path_list[i]) > 2 and len(path_list[j]) > 2:
                    common_sequence = common_vertex(path_list[i], path_list[j])
                    for e in common_sequence:
                        priority_order = priority_motion(path_list[i], path_list[j], e)
                        for k in range(len(e[0])):
                            if priority_order[k] == 1:
                                self.graph[(i,path_list[i][e[0][k]])].append((j,path_list[i][e[0][k]]))
                                self.graph_inv[(j, path_list[i][e[0][k]])].append((i, path_list[i][e[0][k]]))
                            else:
                                self.graph[(j, path_list[i][e[0][k]])].append((i, path_list[i][e[0][k]]))
                                self.graph_inv[(i, path_list[i][e[0][k]])].append((j, path_list[i][e[0][k]]))
        return

    def dfs(self, v, visited, stack):
        visited[v] = True
        for i in self.graph[v]:
            if not visited[i]:
                self.dfs(i, visited, stack)
        stack.append(v)

    def check_cycle(self):
        # https: // www.geeksforgeeks.org/detect-cycle-in-directed-graph-using-topological-sort/
        visited = dict()
        for v in self.V:
            visited[v] = False
        stack = []
        for i in self.V:
            if not visited[i]:
                self.dfs(i, visited, stack)

        pos = dict()
        ind = 0
        while len(stack)!=0:
            pos[stack[-1]] = ind
            self.tsort.append(stack[-1])
            ind += 1
            stack.pop()
        for v in self.V:
            first = 0 if v not in pos else pos[v]
            for it in self.graph[v]:
                second = 0 if it not in pos else pos[it]
                if first > second:
                    return True
        return False

    def makespan(self):
        res = 0
        dt = 4
        pass_time = dict()
        for i in self.V:
            pass_time[i] = 0
        if not self.check_cycle():
            for v in self.tsort:
                for i in self.graph_inv[v]:
                    dist = math.sqrt((i[1].x - v[1].x) ** 2 + (i[1].y - v[1].y) ** 2)
                    if dist==0:
                        dist = dt
                    if pass_time[v] < pass_time[i] + dist:
                        pass_time[v] = pass_time[i] + dist

        for i in range(len(self.path_list)):
            if pass_time[(i,self.path_list[i][-1])] > res:
                res = pass_time[(i,self.path_list[i][-1])]
        return res


def priority_motion(config1, config2, common_list):
    # take the order of config1
    #  1 : config1 > config2 , -1: config1 < config2
    # if the nodes of obstacles are collinear, the relation geometric change
    priority_order = []
    if len(common_list[0])==0:
         print("common list in not empty")

    elif len(common_list[0])==1:
        alpha1 = cosinus_alpha(config1[common_list[0][0]-1], config1[common_list[0][0]], config1[common_list[0][0]+1])
        alpha2 = cosinus_alpha(config2[common_list[1][0]-1], config2[common_list[1][0]], config2[common_list[1][0]+1])
        if alpha1 < alpha2:
            priority_order.append(-1)
        else:
            priority_order.append(1)

    else:
        alpha1 = cosinus_alpha(config1[common_list[0][0] - 1], config1[common_list[0][0]],
                               config1[common_list[0][0] + 1])
        alpha2 = cosinus_alpha(config2[common_list[1][0] - 1], config2[common_list[1][0]],
                               config2[common_list[1][0] + 1])
        if alpha1 < alpha2:
            priority_order.append(-1)
        else:
            priority_order.append(1)

        if config1[common_list[0][0]+1] == config2[common_list[1][0]+1]:
            orientation = ccw(config1[common_list[0][0]], config1[common_list[0][0]-1], config2[common_list[1][0]-1])
        else:
            orientation = ccw(config1[common_list[0][0]], config1[common_list[0][0] - 1],
                              config2[common_list[1][0]+1])

        for i in range(1, len(common_list[0])):
            alpha = ccw(config1[common_list[0][i]], config1[common_list[0][i]-1], config1[common_list[0][i]+1])
            if alpha == orientation:
                priority_order.append(-1)
            else:
                priority_order.append(1)
    return priority_order


def common_vertex(config1, config2):
    n = len(config1)
    m = len(config2)
    common_list1 = []
    common_list2 = []
    for i in range(n):
        for j in range(m):
            if config1[i] == config2[j]:
                common_list1.append(i)
                common_list2.append(j)
                break
    if len(common_list1)>0:
        index_list1 = np.diff(common_list1, n=1, axis=-1, prepend=common_list1[0])
        i = 0
        e1 = []
        e2 = []
        sequence_list = []
        while i < len(index_list1):
            if index_list1[i] == 1:
                e1.append(common_list1[i])
                e2.append(common_list2[i])
            else:
                sequence_list.append([e1, e2])
                e1 = [common_list1[i]]
                e2 = [common_list2[i]]
            i = i + 1
        sequence_list.append([e1, e2])
        return sequence_list[1:]
    else:
        return []


def search_subgraph(configs):
    n = len(configs)
    visited = {}
    checked = {}
    sub_graphs = []
    for i in range(n):
        visited[i] = False
        checked[i] = False

    for i in range(n):
        open_list = deque([])
        closed_list = []
        if not visited[i]:
            visited[i] = True
            open_list.append(i)
            while len(open_list) > 0:
                j = open_list.popleft()
                checked[j] = True
                closed_list.append(j)
                for item in range(n):
                    if not visited[item]:
                        config1 = configs[item]
                        config2 = configs[j]
                        if has_common_vertex(config1, config2):
                            open_list.append(item)
                            visited[item] = True
            sub_graphs.append(closed_list)
    return sub_graphs


def computeSubgraphsMakespan(configs, subgraph):
    if len(subgraph)==1:
        i = subgraph[0]
        res = path_length(configs[i])
    else:
        paths = []
        for i in subgraph:
            path = configs[i]
            paths.append(path)

        sg = Schgraph(paths)
        sg.construct()
        deadlock = sg.check_cycle()
        if deadlock:
            res = -1
        else:
            res = math.ceil(sg.makespan())
    return res


def findSubgraphs(n, configs, match):
    visited = {}
    checked = {}
    sub_graphs = []
    for i in range(n):
        visited[i] = False
        checked[i] = False

    for i in range(n):
        open_list = deque([])
        closed_list = []
        if not visited[i]:
            visited[i] = True
            open_list.append(i)
            while len(open_list) > 0:
                j = open_list.popleft()
                checked[j] = True
                closed_list.append(j)
                for item in range(n):
                    if not visited[item]:
                        config1 = configs[(item, match[item][0])][match[item][1]][0]
                        config2 = configs[(j, match[j][0])][match[j][1]][0]
                        if has_common_vertex(config1, config2):
                            open_list.append(item)
                            visited[item] = True
            sub_graphs.append(closed_list)
    return sub_graphs


def findSubgraphsMakespan(configs, match, subgraph):
    if len(subgraph)==1:
        i = subgraph[0]
        res = configs[(i, match[i][0])][match[i][1]][1]
    else:
        paths = []
        for i in subgraph:
            path = configs[(i, match[i][0])][match[i][1]][0]
            paths.append(path)

        sg = Schgraph(paths)
        sg.construct()
        deadlock = sg.check_cycle()
        if deadlock:
            res = -1
        else:
            res = math.ceil(sg.makespan())
    return res


def makespan2path(p1, p2):
    configs = [p1, p2]
    sg = Schgraph(configs)
    sg.construct()
    deadlock = sg.check_cycle()
    if deadlock:
        return -1
    else:
        return sg.makespan()
