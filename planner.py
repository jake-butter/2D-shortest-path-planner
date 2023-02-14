import matplotlib.pyplot as plt
import matplotlib.patches as patch
import numpy as np
import yaml
import argparse
from scipy.sparse.csgraph import dijkstra
from geometry_utilities import *

def visgraph(start, goal, obstacles, external_edges, internal_edges, x_space_size, y_space_size):
    # Computes a visibility graph in the form of a weighted adjacency matrix for the given points and obstacles
    
    # Construct list of all graph vertices
    vertices = [start, goal]
    for o in obstacles:
        for ov in o:
            vertices.append(ov)

    # Initialise adjacency matrix
    n = len(vertices)
    adj_mat = np.zeros((n, n))

    for i in range(len(vertices)):
        v = np.array(vertices[i])
        for j in range(len(vertices)):
            if i == j or adj_mat[i, j] > 0: continue
            w = np.array(vertices[j])
            if not in_bounds(v, x_space_size, y_space_size):
                continue
            if not in_bounds(w, x_space_size, y_space_size):
                continue

            intersect = False
            for ee in external_edges:
                if segment_intersect_proper(v, w, np.array(ee[0]), np.array(ee[1])):
                    intersect = True
                    break
            for ie in internal_edges:
                ie0 = np.array(ie[0])
                ie1 = np.array(ie[1])
                if segment_intersect_internal(v, w, ie0, ie1):
                    intersect = True
                    break
                if (np.array_equal(v, ie0) and np.array_equal(w, ie1)) or (np.array_equal(v, ie1) and np.array_equal(w, ie0)):
                    intersect = True
                    break
            
            if not intersect:
                dist = np.linalg.norm(np.array(v) - np.array(w))
                adj_mat[i, j] = dist
                adj_mat[j, i] = dist
    
    return vertices, adj_mat

def segment_intersect_internal(p1, p2, p3, p4):
    # Special case of segment intersection for internal edges
    if segment_intersect_proper(p1, p2, p3, p4):
        return True
    elif on_segment(p3, p4, p1, True):
        return True
    elif on_segment(p3, p4, p2, True):
        return True
    elif on_segment(p1, p2, p3, True):
        return True
    elif on_segment(p1, p2, p4, True):
        return True
    else:
        return False

def triangulate(obstacle):
    index_list = []
    for i in range(len(obstacle)):
        index_list.append(i)

    external_edges = []
    internal_edges = []

    for i in range(len(obstacle)):
        curr = obstacle[i]
        next = obstacle[(i + 1) % len(obstacle)]
        external_edges.append([curr, next])

    while len(index_list) > 3:
        for i in range(len(index_list)):
            curr_i = index_list[i]
            prev_i = index_list[i - 1 if i > 0 else len(index_list) - 1]
            next_i = index_list[(i + 1) % len(index_list)]

            curr = np.array(obstacle[curr_i])
            prev = np.array(obstacle[prev_i])
            next = np.array(obstacle[next_i])

            # Check for ear
            # Internal angle of ears is convex
            curr_to_prev = np.array(prev) - np.array(curr)
            curr_to_next = np.array(next) - np.array(curr)
            if np.cross(curr_to_prev, curr_to_next) < 0: continue

            # Check if any of the other obstacle vertices are contained within the triangle defined by curr, prev, and next
            containing = False
            for j in range(len(obstacle)):
                if j in [curr_i, prev_i, next_i]: continue

                p = obstacle[j]
                if point_in_triangle(prev, curr, next, p):
                    containing = True
                    break
            if containing: continue
            
            # Vertex is an ear, add triangle edges and remove vertex from list
            if not np.any(np.all([prev, next] == external_edges)):
                internal_edges.append([prev.tolist(), next.tolist()])
            index_list.pop(i)
            break
    
    return external_edges, internal_edges

if __name__=="__main__":
    parser = argparse.ArgumentParser(prog='shortestPathFinding', description='Find the Shortest Path that avoids obstacles')
    parser.add_argument('inputyaml')
    parser.add_argument('output', nargs='?', default='solution.txt')
    parser.add_argument('--plot', action='store_true')
    args = parser.parse_args()

    # Load input
    indata = yaml.load(open(args.inputyaml), yaml.Loader)
    obstacles = indata['list_obstacles']
    start = [indata['x_start'], indata['y_start']]
    goal = [indata['x_goal'], indata['y_goal']]
    x_space_size = indata['x_space_size']
    y_space_size = indata['y_space_size']

    # Check validity of start and goal
    if not in_bounds(start, x_space_size, y_space_size):
        raise ValueError('Start point out of bounds.')
    if not in_bounds(goal, x_space_size, y_space_size):
        raise ValueError('Goal point out of bounds.')

    # Check validity of input obstacle polygons
    for i in range(len(obstacles)):
        # Remove duplicate vertices
        deduplicated = []
        for ov in obstacles[i]:
            if ov not in deduplicated:
                deduplicated.append(ov)
        obstacles[i] = deduplicated

        # Ensure obstacle has at least 3 vertices
        if len(obstacles[i]) < 3:
            raise ValueError('Obstacles must have at least 3 vertices.')

        # Disallow self-intersection
        if check_self_intersect(obstacles[i]):
            raise ValueError('Self-intersecting obstacles not supported.')
        
        # Triangulation requires clockwise winding order
        wo = check_winding_order(obstacles[i])
        if wo < 0:
            obstacles[i].reverse()
        
        # Triangulation requires no colinear adjacent edges
        obstacles[i] = remove_colinearity(obstacles[i])

    # Decompose obstacles to triangle edges
    external_edges = []
    internal_edges = []
    for obstacle in obstacles:
        ee, ie = triangulate(obstacle)
        external_edges += ee
        internal_edges += ie

    # Construct weighted visibility graph
    vertices, adj_mat = visgraph(start, goal, obstacles, external_edges, internal_edges, x_space_size, y_space_size)

    # Get shortest path in visibility graph
    shortest, predecessors = dijkstra(adj_mat, indices = [0], return_predecessors = True)
    shortest = np.squeeze(shortest)
    predecessors = np.squeeze(predecessors)

    if np.isinf(shortest[1]):
        raise ValueError('No feasible path')

    path = [goal]
    prev = 1
    while True:
        v = vertices[prev]
        prev = predecessors[prev]
        if prev < 0: break
        w = vertices[prev]
        path.append(w)
    path.reverse()

    # Write output
    with open(args.output, "w") as f:
        f.write(str(path))

    # Visualize
    if args.plot:
        fig, ax = plt.subplots()
        ax.set_xlim([0, indata['x_space_size']])
        ax.set_ylim([0, indata['y_space_size']])

        for obstacle in obstacles:
            p = patch.Polygon(np.array(obstacle))
            ax.add_patch(p)
        
        plt.savefig('scene.png')

        #for i, v in enumerate(vertices):
        #    for j, w in enumerate(vertices):
        #        if i == j: continue
        #        if adj_mat[i][j] > 0:
        #            ax.plot([v[0],w[0]],[v[1],w[1]], 'b.-', alpha=0.05)
        
        path = np.array(path)
        ax.plot(path[:,0], path[:,1], 'r.-')

        #for ie in internal_edges:
        #    ax.plot([ie[0][0], ie[1][0]],[ie[0][1], ie[1][1]], 'g.--', alpha=0.0)

        plt.savefig('solution.png')
        plt.show()