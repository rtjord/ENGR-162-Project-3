from scipy.sparse.csgraph import dijkstra
from scipy.sparse import csr_matrix
import numpy as np


# Convert node index to x, y coordinates
def node_index_to_coordinates(index, num_cols, origin=(0, 0)):
    origin_x = origin[0]
    origin_y = origin[1]
    x = index % num_cols - origin_x
    y = int(index / num_cols) - origin_y
    return x, y


# Convert x, y coordinates to node index
def coordinates_to_node_index(x, y, num_cols, origin=(0, 0)):
    origin_x = origin[0]
    origin_y = origin[1]
    return (x + origin_x) + num_cols * (y + origin_y)


# Create a grid graph with dimensions m by n
def grid_graph(m, n):

    # the edges connect each node to every other node
    num_edges = m * n

    # initialize the array for the edges
    arr = np.zeros((num_edges, num_edges))

    # Compare the x and y coordinates of each node to those of every other node
    for i1 in range(num_edges):
        x1, y1 = node_index_to_coordinates(i1, n)
        for i2 in range(num_edges):
            x2, y2 = node_index_to_coordinates(i2, n)

            # if the nodes are the same
            if x1 == x2 and y1 == y2:

                # the distance between the nodes is zero
                arr[i1][i2] = 0

            # if the nodes are adjacent
            elif 1 >= abs(x2 - x1) != abs(y2 - y1) <= 1:

                # the distance between the nodes is 1
                arr[i1][i2] = 1

            # if the nodes are not the same or adjacent
            else:

                # the distance between the nodes is unknown
                arr[i1][i2] = np.inf

    # create a graph from the array of edges
    graph = csr_matrix(arr)
    return graph


# remove node = (x, y) from the graph
def remove_node(graph, node, num_cols, origin):

    # get x and y coordinates of node to remove
    x = node[0]
    y = node[1]

    # get index of node to remove
    index = coordinates_to_node_index(x, y, num_cols, origin)

    # get edge array for graph
    arr = graph.toarray()

    # set the row and column corresponding to the node
    # to be removed equal to infinity
    for i in range(num_cols**2):
        arr[index][i] = np.inf
        arr[i][index] = np.inf

    # turn the modified edge array back into a graph
    graph = csr_matrix(arr)
    return graph


def find_nearest_unknown(graph, source, num_cols, origin, known_points):
    start_index = coordinates_to_node_index(source[0], source[1], num_cols, origin)
    distance_matrix, predecessors = dijkstra(graph, directed=False, indices=start_index, return_predecessors=True)

    nearest_index = distance_matrix.index(min(distance_matrix))
    nearest_point = node_index_to_coordinates(nearest_index, num_cols, origin)

    while nearest_point in known_points:
        distance_matrix[nearest_index] = np.inf
        nearest_index = distance_matrix.index(min(distance_matrix))
        nearest_point = node_index_to_coordinates(nearest_index, num_cols, origin)

    return nearest_point


# find a path from the source to the target
def find_path(graph, source, target, num_cols, origin):
    path = []  # initialize path

    # get the graph indices for the source and target
    start_index = coordinates_to_node_index(source[0], source[1], num_cols, origin)
    target_index = coordinates_to_node_index(target[0], target[1], num_cols, origin)

    # find paths from start_index to all the other points in the graph
    distance_matrix, predecessors = dijkstra(graph, directed=False, indices=start_index, return_predecessors=True)

    # start at target and work backwards to find path from target to source
    while target_index != start_index and target_index != -9999:
        path.append(target_index)
        target_index = predecessors[target_index]

    # append start index to path because the while loop ends before it can be appended
    path.append(start_index)

    # convert graph indices to x, y coordinates
    path = [node_index_to_coordinates(index, num_cols, origin) for index in path]

    # reverse to get path from source to target
    path.reverse()
    return path
