import numpy as np

def check_winding_order(vertices):
    # Determines the winding order for the polygon defined by the input vertices
    # Returns 1 for clockwise, -1 for anticlockwise, and 0 otherwise
    sum = 0
    for i in range(len(vertices)):
        v = vertices[i]
        w = vertices[(i + 1) % len(vertices)]
        sum += (w[0] - v[0]) * (w[1] + v[1])
    if sum > 0:
        return 1
    elif sum < 0:
        return -1
    else:
        return 0

def check_self_intersect(vertices):
    # Checks if any pair of edges of the polygon defined by the input vertices intersects
    # Returns True if intersection detected, otherwise False
    intersect = False
    for i in range(len(vertices) - 2):
        v = vertices[i]
        w = vertices[i + 1]

        for j in range(i + 1, len(vertices)):
            p = vertices[j]
            q = vertices[(j + 1) % len(vertices)]

            if segment_intersect_proper(v, w, p, q):
                intersect = True
                break
        
        if intersect: break
    
    return intersect

def remove_colinearity(vertices):
    # Checks if any two adjacent edges of the polygon defined by the input vertices are colinear.
    # Returns a list of vertices representing the input polygon with any colinearity removed.
    colinear_vertices = []
    for i in range(len(vertices)):
        v = vertices[i]
        w = vertices[(i + 1) % len(vertices)]
        x = vertices[(i + 2) % len(vertices)]
        
        if on_segment(v, x, w):
            colinear_vertices.append(w)
    
    for cv in colinear_vertices:
        vertices.remove(cv)

    return vertices

def segment_intersect_proper(p1, p2, p3, p4):
    # Returns True if two line segments properly intersect, otherwise False
    d1 = np.cross(np.subtract(p1, p3), np.subtract(p4, p3))
    d2 = np.cross(np.subtract(p2, p3), np.subtract(p4, p3))
    d3 = np.cross(np.subtract(p3, p1), np.subtract(p2, p1))
    d4 = np.cross(np.subtract(p4, p1), np.subtract(p2, p1))

    return ((d1 < 0 and d2 > 0) or (d1 > 0 and d2 < 0)) and ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0))

def on_segment(v, w, p, exclusive = False):
    # Returns True is point p lies on the line segment defined by v and w, otherwise False
    # Input exclusive determines how intersection at endpoints is handled
    parallel = np.isclose(np.cross(np.subtract(p, v), np.subtract(w, v)), 0)

    if exclusive:
        bounded = (min(v[0], w[0]) <= p[0] <= max(v[0], w[0]) and min(v[1], w[1]) < p[1] < max(v[1], w[1])) or \
                (min(v[0], w[0]) < p[0] < max(v[0], w[0]) and min(v[1], w[1]) <= p[1] <= max(v[1], w[1]))
    else:
        bounded = min(v[0], w[0]) <= p[0] <= max(v[0], w[0]) and min(v[1], w[1]) <= p[1] <= max(v[1], w[1])
    
    return bounded and parallel

def point_in_triangle(a, b, c, p):
    # Returns True if point p is contained within the triangle defined by vertices a, b, c
    ab = np.subtract(b, a)
    bc = np.subtract(c, b)
    ca = np.subtract(a, c)

    ap = np.subtract(p, a)
    bp = np.subtract(p, b)
    cp = np.subtract(p, c)

    cross_a = np.cross(ab, ap)
    cross_b = np.cross(bc, bp)
    cross_c = np.cross(ca, cp)

    if cross_a > 0 or cross_b > 0 or cross_c > 0:
        return False
    
    return True

def in_bounds(v, x_size, y_size):
    # Returns True if point v lies in the positive bounds defined by x_size and y_size
    if v[0] > x_size or v[0] < 0:
        return False
    elif v[1] > y_size or v[1] < 0:
        return False
    else:
        return True