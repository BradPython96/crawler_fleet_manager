from math import pi, sqrt, atan, acos
import numpy as np

INF = 10000
CCW = 1
CW = -1
COLLINEAR = 0

"""Due to floating point representation error, some functions need to
   truncate floating point numbers to a certain tolerance."""
COLIN_TOLERANCE = 10
T = 10 ** COLIN_TOLERANCE
T2 = 10.0 ** COLIN_TOLERANCE


class Point(object):
    __slots__ = ('x', 'y', 'polygon_id')

    def __init__(self, x, y, polygon_id=-1):
        self.x = float(x)
        self.y = float(y)
        self.polygon_id = polygon_id

    def __eq__(self, point):
        return point and self.x == point.x and self.y == point.y

    def __ne__(self, point):
        return not self.__eq__(point)

    def __lt__(self, point):
        """ This is only needed for shortest path calculations where heapq is
            used. When there are two points of equal distance, heapq will
            instead evaluate the Points, which doesnt work in Python 3 and
            throw a TypeError."""
        return hash(self) < hash(point)

    def __str__(self):
        return "(%.2f, %.2f)" % (self.x, self.y)

    def __hash__(self):
        return self.x.__hash__() ^ self.y.__hash__()

    def __repr__(self):
        return "Point(%.2f, %.2f)" % (self.x, self.y)


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


def ccw(a, b, c):
    """Return 1 if counter clockwise, -1 if clock wise, 0 if collinear """
    #  Rounding this way is faster than calling round()
    area = int(((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)) * T) / T2
    if area > 0:
        return 1
    if area < 0:
        return -1
    return 0


def cosinus_alpha(a, b, c):
    ab = sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y))
    cb = sqrt((c.x - b.x) * (c.x - b.x) + (c.y - b.y) * (c.y - b.y))
    abc = ((a.x - b.x) * (c.x - b.x) + (a.y - b.y) * (c.y - b.y)) / (ab * cb)
    return abc


def on_segment(p, q, r):
    """Given three collinear points p, q, r, the function checks if point q
    lies on line segment 'pr'."""
    if (q.x <= max(p.x, r.x)) and (q.x >= min(p.x, r.x)):
        if (q.y <= max(p.y, r.y)) and (q.y >= min(p.y, r.y)):
            return True
    return False


def segment_intersect(p1, q1, p2, q2):
    o1 = ccw(p1, q1, p2)
    o2 = ccw(p1, q1, q2)
    o3 = ccw(p2, q2, p1)
    o4 = ccw(p2, q2, q1)
    # General case
    if o1 != o2 and o3 != o4:
        return True
    # p1, q1 and p2 are collinear and p2 lies on segment p1q1
    if o1 == COLLINEAR and on_segment(p1, p2, q1):
        return True
    # p1, q1 and q2 are collinear and q2 lies on segment p1q1
    if o2 == COLLINEAR and on_segment(p1, q2, q1):
        return True
    # p2, q2 and p1 are collinear and p1 lies on segment p2q2
    if o3 == COLLINEAR and on_segment(p2, p1, q2):
        return True
    # p2, q2 and q1 are collinear and q1 lies on segment p2q2
    if o4 == COLLINEAR and on_segment(p2, q1, q2):
        return True
    return False


def segment_intersect1(p1, q1, p2, q2):
    if p1 == p2 or p1 == q2 or q1 == p2 or q1 == q2:
        return False
    o1 = ccw(p1, q1, p2)
    o2 = ccw(p1, q1, q2)
    o3 = ccw(p2, q2, p1)
    o4 = ccw(p2, q2, q1)
    # General case
    if o1 != o2 and o3 != o4:
        return True
    # p1, q1 and p2 are collinear and p2 lies on segment p1q1
    if o1 == COLLINEAR and on_segment(p1, p2, q1):
        return True
    # p1, q1 and q2 are collinear and q2 lies on segment p1q1
    if o2 == COLLINEAR and on_segment(p1, q2, q1):
        return True
    # p2, q2 and p1 are collinear and p1 lies on segment p2q2
    if o3 == COLLINEAR and on_segment(p2, p1, q2):
        return True
    # p2, q2 and q1 are collinear and q1 lies on segment p2q2
    if o4 == COLLINEAR and on_segment(p2, q1, q2):
        return True
    return False



def edge_intersect(p1, q1, edge):
    """Return True if edge from A, B interects edge.
    http://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/"""
    p2 = edge.p1
    q2 = edge.p2
    return segment_intersect(p1, q1, p2, q2)


def angle2(point_a, point_b, point_c):
    """Return angle B (radian) between point_b and point_c.
           c
         /  \
       /    B\
      a-------b
    """
    a = (point_c.x - point_b.x)**2 + (point_c.y - point_b.y)**2
    b = (point_c.x - point_a.x)**2 + (point_c.y - point_a.y)**2
    c = (point_b.x - point_a.x)**2 + (point_b.y - point_a.y)**2
    cos_value = (a + c - b) / (2 * sqrt(a) * sqrt(c))
    return acos(int(cos_value*T)/T2)


def angle(center, point):
    """Return the angle (radian) of point from center of the radian circle.
     ------p
     |   /
     |  /
    c|a/
    """
    dx = point.x - center.x
    dy = point.y - center.y
    if dx == 0:
        if dy < 0:
            return pi * 3 / 2
        return pi / 2
    if dy == 0:
        if dx < 0:
            return pi
        return 0
    if dx < 0:
        return pi + atan(dy / dx)
    if dy < 0:
        return 2 * pi + atan(dy / dx)
    return atan(dy / dx)


def unit_vector(c, p):
    magnitude = edge_distance(c, p)
    return Point((p.x - c.x) / magnitude, (p.y - c.y) / magnitude)


def edge_distance(p1, p2):
    """Return the Euclidean distance between two Points."""
    return sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)


def check_collinear(p0, p1, p2):
    p0p1 = [p1.x - p0.x, p1.y - p0.y]
    p1p2 = [p2.x - p1.x, p2.y - p1.y]
    if abs(p0p1[0]*p1p2[1] - p0p1[1]*p1p2[0]) < 1e-6:
        return True
    else:
        return False


def find_intersect_point(p0, p1, q0, q1):
    # calculate intersection point between segment p0p1 and q0q1
    p1q0 = [q0.x - p1.x, q0.y - p1.y]
    p1q1 = [q1.x - p1.x, q1.y - p1.y]
    p0p1 = [p1.x - p0.x, p1.y - p0.y]

    lambda_ = (p1q1[0] * p0p1[1] - p1q1[1] * p0p1[0]) / (p1q0[1] * p0p1[0] - p1q0[0] * p0p1[1] + p1q1[0] * p0p1[1] - p1q1[1] * p0p1[0])
    if lambda_ < 0 or lambda_ > 1:
        return
    x_cor = lambda_ * q0.x + (1 - lambda_) * q1.x
    y_cor = lambda_ * q0.y + (1 - lambda_) * q1.y
    intersect_point = Point(x_cor, y_cor)
    return intersect_point


def point_in_triangle(p, p0, p1, p2):
    # http://totologic.blogspot.com/2014/01/accurate-point-in-triangle-test.html
    denominator = (p1.y  - p2.y)*(p0.x -p2.x) + (p2.x - p1.x)*(p0.y - p2.y)
    a = ((p1.y - p2.y)*(p.x - p2.x) + (p2.x - p1.x)*(p.y - p2.y))/denominator
    b = ((p2.y - p0.y)*(p.x - p2.x) + (p0.x - p2.x)*(p.y - p2.y))/denominator
    c = 1 - a - b
    return a>=0 and a<=1 and b>=0 and b<=1 and c>=0 and c<=1


def check_path_intersect(path):
    if len(path) < 4:
        return True
    for i in range(2, len(path)-1):
         j = 0
         while j < i-1:
             if segment_intersect1(path[i], path[i+1], path[j], path[j+1]):
                 return False
             j = j+1
    return True


def path_length(p):
    length = 0
    if len(p)<2:
        return length
    for i in range(1,len(p)):
        length = length + edge_distance(p[i-1], p[i])
    return length


def config_intersection1(config1, config2):
    if len(config1) <= len(config2):
        return intersection_detect1(config1, config2)
    else:
        result = intersection_detect1(config2, config1)
        if len(result) >1:
            return [result[0], result[2], result[1]]
        else:
            return result

def config_intersection(config1, config2):
    if len(config1) <= len(config2):
        return intersection_detect(config1, config2)
    else:
        return intersection_detect(config2, config1)

def intersection_detect(config1, config2):
    # return 0 : intersection, return 1 : no intersection, return 3 : disjoint
    # len(config1) <= len(config2), anchor points, destination points can't be polygon vertex
    # take the direction of config1
    # find index for common vertices
    n = len(config1)
    m = len(config2)
    for i in range(n - 1):
        for j in range(m - 1):
            if segment_intersect1(config1[i], config1[i + 1], config2[j], config2[j + 1]):
                return 0
    if n <= 2 or m <= 2:
        return 3
    if n > 2 and m > 2:
        common_list1 = []
        common_list2 = []
        for i in range(n):
            for j in range(m):
                if config1[i] == config2[j]:
                    common_list1.append(i)
                    common_list2.append(j)
                    break
        # find singular vertex and vertex sequence
        if len(common_list1) == 0:
            return 3
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
        # to check intersection
        if len(sequence_list) <= 1:
            return 3
        else:
            for e in sequence_list[1:]:
                if len(e[0]) == 1:
                    cf1_cf2 = check_vertex_common(config1, config2, e[0][0], e[1][0])
                elif len(e[0]) > 1:
                    cf1_cf2 = check_segment_common(config1, config2, e[0], e[1])
                if cf1_cf2 == 0:
                    return 0
    return 1


def intersection_detect1(config1, config2):
    # return flag : segment cross 0; singular node or common sequence 1; no cross -1;
    # len(config1) <= len(config2), anchor points, destination points can't be polygon vertex
    # take the direction of config1
    # find index for common vertices
    n = len(config1)
    m = len(config2)
    for i in range(n - 1):
        for j in range(m - 1):
            if segment_intersect1(config1[i], config1[i + 1], config2[j], config2[j + 1]):
                return [0, i, j]
    if n <= 2 or m <= 2:
        return [-1]
    if n > 2 and m > 2:
        common_list1 = []
        common_list2 = []
        for i in range(n):
            for j in range(m):
                if config1[i] == config2[j]:
                    common_list1.append(i)
                    common_list2.append(j)
                    break
        # find singular vertex and vertex sequence
        if len(common_list1) == 0:
            return [-1]
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
        # to check intersection
        if len(sequence_list) <= 1:
            return [-1]
        else:
            for e in sequence_list[1:]:
                if len(e[0]) == 1:
                    cf1_cf2 = check_vertex_common(config1, config2, e[0][0], e[1][0])
                elif len(e[0]) > 1:
                    cf1_cf2 = check_segment_common(config1, config2, e[0], e[1])
                if cf1_cf2 == 0:
                    return [1, e[0][0], e[1][0]]
    return [-1]


def check_vertex_common(config1, config2, i, j):
    # return 0 : intersection, return 1 : config1 is exterior, return 2 : config2 is exterior
    # for the case more general:
    a0 = [config1[i - 1].x - config1[i].x, config1[i - 1].y - config1[i].y]
    a1 = [config1[i + 1].x - config1[i].x, config1[i + 1].y - config1[i].y]
    b0 = [config2[j - 1].x - config2[j].x, config2[j - 1].y - config2[j].y]
    b1 = [config2[j + 1].x - config2[j].x, config2[j + 1].y - config2[j].y]

    o1 = inclu_in_angle(a0, b0, b1)
    o2 = inclu_in_angle(a1, b0, b1)
    o3 = inclu_in_angle(b0, a0, a1)
    o4 = inclu_in_angle(b1, a0, a1)

    if b0[0] * b1[0] + b0[1] * b1[1] > 0 and b0[0] * b1[1] - b0[1] * b1[0] == 0:
        if o3:
            return 0
    elif b0[0] * b1[0] + b0[1] * b1[1] < 0 and b0[0] * b1[1] - b0[1] * b1[0] == 0:
        if o3 or o4:
            return 0
    else:
        if not ((o1 and o2) or (not o1 and not o2)):
            return 0

    if a0[0] * a1[0] + a0[1] * a1[1] > 0 and a0[0] * a1[1] - a0[1] * a1[0] == 0:
        if o1:
            return 0
    elif a0[0] * a1[0] + a0[1] * a1[1] < 0 and a0[0] * a1[1] - a0[1] * a1[0] == 0:
        if o1 or o2:
            return 0
    else:
        if not ((o3 and o4) or (not o3 and not o4)):
            return 0
    alpha1 = cosinus_alpha(config1[i - 1], config1[i], config1[i + 1])
    alpha2 = cosinus_alpha(config2[j - 1], config2[j], config2[j + 1])
    if alpha1 <= alpha2:
        return 1
    else:
        return 2


def check_segment_common(config1, config2, seq1, seq2):
    # return 0 : intersection, return 1 : no intersection
    # config1[seq1[0]] = config2[seq2[0]], config1[seq1[-1]] = config2[seq2[-1]]
    # take the direction of config1, that seq1 increase
    if config1[seq1[0]-1] == config2[seq2[0]-1]:
        o1 = ccw(config1[seq1[0]], config1[seq1[0]-1], config2[seq2[0]+1])
    else:
        o1 = ccw(config1[seq1[0]], config1[seq1[0]-1], config2[seq2[0]-1])

    if config1[seq1[-1]+1] == config2[seq2[-1]+1]:
        o2 = ccw(config1[seq1[-1]], config1[seq1[-1]+1], config2[seq2[-1]-1])
    else:
        o2 = ccw(config1[seq1[-1]], config1[seq1[-1]+1], config2[seq2[-1]+1])
    if o1 == o2:
        return 0
    else:
        return 1


def has_common_vertex(config1, config2):
    result = False
    for x in config1:
        for y in config2:
            if x == y:
                result = True
                return result
    return result


def cross_exist(paths):
    # return [flag, index of path1, index of path2, index of node in path1, index of node in path2]
    n = len(paths)
    for i in range(n-1):
        for j in range(i+1, n):
            cross = config_intersection1(paths[i], paths[j])
            if cross[0] >= 0:
                return [cross[0], i, j, cross[1], cross[2]]
    return [-1]


def inclu_in_angle(a, b, c):
    avb = a[0] * b[1] - a[1] * b[0]
    avc = a[0] * c[1] - a[1] * c[0]
    bvc = b[0] * c[1] - b[1] * c[0]
    if avb * bvc <= 0 <= avc * bvc:
        return True
    else:
        return False

