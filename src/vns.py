from func_timeout import func_set_timeout
import time
import numpy as np
import math
from schgraph import findSubgraphs, findSubgraphsMakespan, makespan2path, config_intersection
from geometrics import Point


def find_anchor_list(connected_graph_index, anchor, xpts, num):
    # to find k nearest points from the current anchor points
    k = num
    index_list = [i for i in range(len(xpts))]
    index_list.sort(key=lambda x: (xpts[x][0] - xpts[anchor][0]) ** 2 + (xpts[x][1] - xpts[anchor][1]) ** 2)
    res = [e for e in connected_graph_index]
    for i in index_list:
        if i not in connected_graph_index:
            res.append(i)
        if len(res) > k:
            break
    return res


class SearchVNSSimple(object):
    def __init__(self, g, sol, k, arr1, arr2, arr3, arr4, indice, ms, num, path_num):
        self.g = g
        self.crossing_table = arr1
        self.makespan_table = np.full_like(arr1, 0)
        self.configs = arr2
        self.solution = [[e[0], e[1]] for e in sol]
        self.kmax = k
        self.xpts = arr3
        self.ypts = arr4
        # self.path_table : O not generated yet ; 1 already generate
        self.path_table = [[1 for _ in range(len(sol))] for _ in range(len(sol))]
        self.sup_bound = ms
        self.max_edge_index = indice.copy()
        self.neighbor = num
        self.path_num = path_num

    def find_local_solution(self, k):
        # find neighbors
        n = len(self.solution)
        graph = {}
        is_visited = {}

        dists = []
        for i in self.max_edge_index:
            dists.append(self.configs[(i, self.solution[i][0])][self.solution[i][1]][1])
        ind_ = dists.index(max(dists))
        start = self.max_edge_index[ind_]
        destination = self.solution[start][0]
        anchor_list = find_anchor_list(self.max_edge_index, start, self.xpts, self.neighbor)
        dest_list = [self.solution[e][0] for e in anchor_list]

        # find relevant anchor points
        local_solution = {}

        for i in anchor_list:
            graph[i + 1] = []
            is_visited[i + 1] = False
            local_solution[i + 1] = -self.solution[i][0] - 1
            local_solution[-self.solution[i][0] - 1] = i + 1

            for j in dest_list:
                is_visited[-j - 1] = False
                dist = self.configs[(i, j)][0][1]
                if dist < self.sup_bound:
                    graph[i + 1] = graph.get(i + 1, []) + [-j - 1]
                    graph[-j - 1] = graph.get(-j - 1, []) + [i + 1]

        perm = []
        perms = []
        # find sequence
        self.find_perm(start + 1, -destination - 1, k, perm, perms, is_visited, graph, local_solution)
        if len(perms) > 0:
            res = findSubgraphs(n, self.configs, self.solution)
            result = []
            for e in res:
                ms = findSubgraphsMakespan(self.configs, self.solution, e)
                result.append(ms)
            self.max_edge_index = res[result.index(max(result))]
            self.sup_bound = math.ceil(max(result))
            return True
        else:
            return False

    def find_path(self, free_path, a_side, b_side, n):
        path_pool = []

        def dfs(i):
            # evaluate path pool
            if i == len(a_side):
                solution_eval = [[] for _ in range(n)]

                for _ in range(n):
                    solution_eval[_] = [self.solution[_][0], self.solution[_][1]]

                for _ in range(len(path_pool)):
                    solution_eval[path_pool[_][0]] = [path_pool[_][1], path_pool[_][2]]

                subgraph = [_ for _ in range(n)]
                ms = findSubgraphsMakespan(self.configs, solution_eval, subgraph)

                if ms >= self.sup_bound or ms <0:
                    return False
                return True

            for p in free_path[i]:
                config1 = self.configs[(a_side[i], b_side[i])][p][0]
                index1 = a_side[i] * n + b_side[i]
                is_valid = True
                for e in path_pool:
                    config2 = self.configs[(e[0], e[1])][e[2]][0]
                    index2 = e[0] * n + e[1]
                    if index1 < index2:
                        if self.crossing_table[index1, index2] == -1:
                            if not config_intersection(config1, config2):
                                self.crossing_table[index1, index2] = 1
                                self.makespan_table[index1, index2] = 0
                                is_valid = False
                                break
                            else:
                                tmp = makespan2path(config1, config2)
                                self.crossing_table[index1, index2] = 0
                                self.makespan_table[index1, index2] = tmp
                                if tmp < 0:
                                    is_valid = False
                                    break

                        else:
                            if self.crossing_table[index1, index2] == 1 or self.makespan_table[index1, index2]<0:
                                is_valid = False
                                break

                    else:
                        if self.crossing_table[index2][index1] == -1:
                            if not config_intersection(config1, config2):
                                self.crossing_table[index2, index1] = 1
                                self.makespan_table[index2, index1] = 0
                                is_valid = False
                                break
                            else:
                                tmp = makespan2path(config1, config2)
                                self.crossing_table[index2, index1] = 0
                                self.makespan_table[index2, index1] = tmp
                                if tmp < 0:
                                    is_valid = False
                                    break
                        else:
                            if self.crossing_table[index2, index1] == 1 or self.makespan_table[index2, index1] < 0:
                                is_valid = False
                                break

                if is_valid:
                    path_pool.append([a_side[i], b_side[i], p])
                    if dfs(i + 1):
                        return True
                    else:
                        path_pool.pop()

            return False

        if dfs(0):
            # update the new solution
            for ele in path_pool:
                self.solution[ele[0]] = [ele[1], ele[2]]
            return True
        else:
            return False

    def check_perm(self, perm):
        n = len(self.solution)
        a_side = [e - 1 for e in perm[::2]]
        b_side = [-e - 1 for e in perm[1::2]]
        p_length = len(a_side)
        # first step generate paths for the existing sequence pair:

        # detect if the new added path cross the existing path
        free_path = []
        for i in range(p_length):
            f = []
            for k in range(len(self.configs[(a_side[i], b_side[i])])):
                config2 = self.configs[(a_side[i], b_side[i])][k][0]
                if self.configs[(a_side[i], b_side[i])][k][1] < self.sup_bound:
                    is_valid = True
                    for j in range(n):
                        if j not in a_side:
                            config1 = self.configs[(j, self.solution[j][0])][self.solution[j][1]][0]
                            index2 = a_side[i] * n + b_side[i]
                            index1 = j * n + self.solution[j][0]
                            if index1 < index2:
                                if self.crossing_table[index1, index2] == -1:
                                    if not config_intersection(config1, config2):
                                        self.crossing_table[index1, index2] = 1
                                        self.makespan_table[index1, index2] = 0
                                        is_valid = False
                                        break
                                    else:
                                        tmp = makespan2path(config1, config2)
                                        self.crossing_table[index1, index2] = 0
                                        self.makespan_table[index1, index2] = tmp
                                        if tmp < 0:
                                            is_valid = False
                                            break

                                else:
                                    if self.crossing_table[index1, index2] ==1 or self.makespan_table[index1, index2] < 0:
                                        is_valid = False
                                        break

                            else:
                                if self.crossing_table[index2, index1] == -1:
                                    if not config_intersection(config1, config2):
                                        self.crossing_table[index2, index1] = 1
                                        self.makespan_table[index2, index1] = 0
                                        is_valid = False
                                        break
                                    else:
                                        tmp = makespan2path(config1, config2)
                                        self.crossing_table[index2, index1] = 0
                                        self.makespan_table[index2, index1] = tmp
                                        if tmp < 0:
                                            is_valid = False
                                            break

                                else:
                                    if self.crossing_table[index2, index1] == 1 or self.makespan_table[index2, index1] < 0:
                                        is_valid = False
                                        break
                    if is_valid:

                        f.append(k)
            free_path.append(f)

        # un algo to search possible paths
        return self.find_path(free_path, a_side, b_side, n)

    def find_perm(self, start, dest, k, perms, perm, is_visited, graph, match):
        if len(perms) == 1:
            return

        if len(perm) == (2 * k):
            is_visited[perm[-1]] = False
            perm.pop()
            return

        perm.append(start)
        is_visited[start] = True
        successors = graph[start]
        neig_length = len(successors)
        if neig_length > 1:
            indice = np.random.randint(0, neig_length - 1)
            successors = successors[indice:] + successors[0:indice]

        for e in successors:
            if not is_visited[e]:
                perm.append(e)
                is_visited[e] = True
                if e == dest:
                    if len(perm) == (2 * k):
                        if self.check_perm(perm):
                            perms.append(perm.copy())
                            return
                        else:
                            is_visited[e] = False
                            perm.pop()
                    else:
                        is_visited[e] = False
                        perm.pop()

                else:
                    self.find_perm(match[e], dest, k, perms, perm, is_visited, graph, match)
                    if len(perms) == 1:
                        return
        is_visited[start] = False
        perm.pop()
        if len(perm) > 0:
            is_visited[perm[-1]] = False
            perm.pop()
        return

    @func_set_timeout(60)
    def local_search(self):
        i = 1
        cnt_improvement = 0
        start_time = time.time()

        while i <= self.kmax:
            if self.find_local_solution(i):
                i = 1
                cnt_improvement = cnt_improvement + 1
                print("VNSX time :{}, makespan :{}".format(time.time(), math.ceil(self.sup_bound)))
            else:
                i = i+1
        return cnt_improvement


class SearchVNS(object):
    def __init__(self, g, sol, k, arr1, arr2, arr3, arr4, indice, ms, num, path_num, display_range):
        self.g = g
        self.crossing_table = arr1
        self.configs = arr2
        self.solution = [[e[0], e[1]] for e in sol]
        self.kmax = k
        self.xpts = arr3
        self.ypts = arr4
        self.path_table = [[0 for _ in range(len(sol))] for _ in range(len(sol))]
        self.sup_bound = ms
        self.max_edge_index = indice.copy()
        self.neighbor = num
        self.path_num = path_num
        self.display_range = display_range

    def find_local_solution(self, k):
        # find neighbors
        n = len(self.solution)
        graph = {}
        is_visited = {}

        dists = []
        for i in self.max_edge_index:
            dists.append(self.configs[(i, self.solution[i][0])][self.solution[i][1]][1])
        ind_ = dists.index(max(dists))
        start = self.max_edge_index[ind_]
        destination = self.solution[start][0]
        anchor_list = find_anchor_list(self.max_edge_index, start, self.xpts, self.neighbor)
        dest_list = [self.solution[e][0] for e in anchor_list]

        # find relevant anchor points
        local_solution = {}

        for i in anchor_list:
            graph[i + 1] = []
            is_visited[i + 1] = False
            local_solution[i + 1] = -self.solution[i][0] - 1
            local_solution[-self.solution[i][0] - 1] = i + 1

            for j in dest_list:
                is_visited[-j - 1] = False
                dist = self.configs[(i, j)][0][1]
                if dist < self.sup_bound:
                    graph[i + 1] = graph.get(i + 1, []) + [-j - 1]
                    graph[-j - 1] = graph.get(-j - 1, []) + [i + 1]

        perm = []
        perms = []
        # find sequence
        self.find_perm(start + 1, -destination - 1, k, perm, perms, is_visited, graph, local_solution)
        if len(perms) > 0:
            res = findSubgraphs(n, self.configs, self.solution)
            result = []
            for e in res:
                ms = findSubgraphsMakespan(self.configs, self.solution, e)
                result.append(ms)
            self.max_edge_index = res[result.index(max(result))]
            self.sup_bound = math.ceil(max(result))
            return True
        else:
            return False

    def find_path(self, free_path, a_side, b_side, n):
        path_pool = []

        def dfs(i):
            # evaluate path pool
            if i == len(a_side):
                solution_eval = [[] for _ in range(n)]

                for _ in range(n):
                    solution_eval[_] = [self.solution[_][0], self.solution[_][1]]

                for _ in range(len(path_pool)):
                    solution_eval[path_pool[_][0]] = [path_pool[_][1], path_pool[_][2]]

                subgraph = [_ for _ in range(n)]
                ms = findSubgraphsMakespan(self.configs, solution_eval, subgraph)

                if ms >= self.sup_bound or ms <0:
                    return False
                return True

            for p in free_path[i]:
                config1 = self.configs[(a_side[i], b_side[i])][p][0]
                index1 = a_side[i] * n + b_side[i]
                is_valid = True
                for e in path_pool:
                    config2 = self.configs[(e[0], e[1])][e[2]][0]
                    index2 = e[0] * n + e[1]
                    if index1 < index2:
                        if (p, e[2]) not in self.crossing_table[index1][index2].keys():
                            if not config_intersection(config1, config2):
                                self.crossing_table[index1][index2][(p, e[2])] = [1, 0]
                                is_valid = False
                                break
                            else:
                                tmp = makespan2path(config1, config2)
                                self.crossing_table[index1][index2][(p, e[2])] = [0, tmp]
                                if tmp < 0:
                                    is_valid = False
                                    break

                        else:
                            if self.crossing_table[index1][index2][(p, e[2])][0]==1 or self.crossing_table[index1][index2][(p, e[2])][1]<0:
                                is_valid = False
                                break

                    else:
                        if (e[2], p) not in self.crossing_table[index2][index1].keys():
                            if not config_intersection(config1, config2):
                                self.crossing_table[index2][index1][(e[2], p)] = [1,0]
                                is_valid = False
                                break
                            else:
                                tmp = makespan2path(config1, config2)
                                self.crossing_table[index2][index1][(e[2], p)] = [0, tmp]
                                if tmp < 0:
                                    is_valid = False
                                    break
                        else:
                            if self.crossing_table[index2][index1][(e[2], p)][0] == 1 or self.crossing_table[index2][index1][(e[2], p)][1] < 0:
                                is_valid = False
                                break

                if is_valid:
                    path_pool.append([a_side[i], b_side[i], p])
                    if dfs(i + 1):
                        return True
                    else:
                        path_pool.pop()

            return False

        if dfs(0):
            # update the new solution
            for ele in path_pool:
                self.solution[ele[0]] = [ele[1], ele[2]]
            return True
        else:
            return False


    def check_perm(self, perm):
        n = len(self.solution)
        a_side = [e - 1 for e in perm[::2]]
        b_side = [-e - 1 for e in perm[1::2]]
        p_length = len(a_side)
        # first step generate paths for the existing sequence pair:
        # self.path_table : O not generated yet ; 1 already generate

        for i in range(p_length):
            if not self.path_table[a_side[i]][b_side[i]]:
                start = Point(self.xpts[a_side[i]][0], self.display_range[1] - self.xpts[a_side[i]][1])
                end = Point(self.ypts[b_side[i]][0], self.display_range[1] - self.ypts[b_side[i]][1])
                cand_config = self.g.candidate_path(start, end, self.sup_bound, self.path_num)
                self.configs[(a_side[i], b_side[i])] = cand_config
                self.path_table[a_side[i]][b_side[i]] = 1

        # detect if the new added path cross the existing path
        free_path = []
        for i in range(p_length):
            f = []
            for k in range(len(self.configs[(a_side[i], b_side[i])])):
                config2 = self.configs[(a_side[i], b_side[i])][k][0]
                if self.configs[(a_side[i], b_side[i])][k][1] < self.sup_bound:
                    is_valid = True
                    for j in range(n):
                        if j not in a_side:
                            config1 = self.configs[(j, self.solution[j][0])][self.solution[j][1]][0]
                            index2 = a_side[i] * n + b_side[i]
                            index1 = j * n + self.solution[j][0]
                            if index1 < index2:
                                if (self.solution[j][1], k) not in self.crossing_table[index1][index2].keys():
                                    if not config_intersection(config1, config2):
                                        self.crossing_table[index1][index2][(self.solution[j][1], k)] = [1, 0]
                                        is_valid = False
                                        break
                                    else:
                                        tmp = makespan2path(config1, config2)
                                        self.crossing_table[index1][index2][(self.solution[j][1], k)] = [0, tmp]
                                        if tmp < 0:
                                            is_valid = False
                                            break

                                else:
                                    if self.crossing_table[index1][index2][(self.solution[j][1], k)][0]==1 or self.crossing_table[index1][index2][(self.solution[j][1], k)][1]<0:
                                        is_valid = False
                                        break

                            else:
                                if (k, self.solution[j][1]) not in self.crossing_table[index2][index1].keys():
                                    if not config_intersection(config1, config2):
                                        self.crossing_table[index2][index1][(k, self.solution[j][1])] = [1, 0]
                                        is_valid = False
                                        break
                                    else:
                                        tmp = makespan2path(config1, config2)
                                        self.crossing_table[index2][index1][(k, self.solution[j][1])] = [0, tmp]
                                        if tmp < 0:
                                            is_valid = False
                                            break

                                else:
                                    if self.crossing_table[index2][index1][(k, self.solution[j][1])][0] == 1 or self.crossing_table[index2][index1][(k, self.solution[j][1])][1] < 0:
                                        is_valid = False
                                        break
                    if is_valid:
                        f.append(k)
            free_path.append(f)

        # un algo to search possible paths
        return self.find_path(free_path, a_side, b_side, n)

    def find_perm(self, start, dest, k, perms, perm, is_visited, graph, match):
        if len(perms) == 1:
            return

        if len(perm) == (2 * k):
            is_visited[perm[-1]] = False
            perm.pop()
            return

        perm.append(start)
        is_visited[start] = True
        successors = graph[start]
        neig_length = len(successors)
        if neig_length > 1:
            indice = np.random.randint(0, neig_length - 1)
            successors = successors[indice:] + successors[0:indice]

        for e in successors:
            if not is_visited[e]:
                perm.append(e)
                is_visited[e] = True
                if e == dest:
                    if len(perm) == (2 * k):
                        if self.check_perm(perm):
                            perms.append(perm.copy())
                            return
                        else:
                            is_visited[e] = False
                            perm.pop()
                    else:
                        is_visited[e] = False
                        perm.pop()

                else:
                    self.find_perm(match[e], dest, k, perms, perm, is_visited, graph, match)
                    if len(perms) == 1:
                        return
        is_visited[start] = False
        perm.pop()
        if len(perm) > 0:
            is_visited[perm[-1]] = False
            perm.pop()
        return

    @func_set_timeout(60)
    def local_search(self):
        i = 1
        cnt_improvement = 0
        while i <= self.kmax:
            if self.find_local_solution(i):
                i = 1
                cnt_improvement = cnt_improvement + 1
                print("VNSX time :{}, makespan :{}".format(time.time(), math.ceil(self.sup_bound)))
            else:
                i = i+1
        return cnt_improvement

