from scipy.sparse.csgraph import dijkstra
from scipy.sparse import csr_matrix
import numpy as np


# Convert node index to x, y coordinates
def index_to_node(index, num_cols):
    x = index % num_cols
    y = int(index / num_cols)
    return x, y


# Convert x, y coordinates to node index
def node_to_index(x, y, num_cols):
    return x + num_cols * y


# Create a grid graph with dimensions m by n
def grid_graph(m, n):

    # the edges connect each node to every other node
    num_edges = m * n

    # initialize the array for the edges
    arr = np.zeros((num_edges, num_edges))

    # Compare the x and y coordinates of each node to those of every other node
    for i1 in range(num_edges):
        x1, y1 = index_to_node(i1, n)
        for i2 in range(num_edges):
            x2, y2 = index_to_node(i2, n)

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
def remove_node(graph, node, num_cols):

    # get x and y coordinates of node to remove
    x = node[0]
    y = node[1]

    # get index of node to remove
    index = node_to_index(x, y, num_cols)

    # get edge array for graph
    arr = graph.toarray()
    num_nodes = len(arr)
    # set the row and column corresponding to the node
    # to be removed equal to infinity
    for i in range(num_nodes):
        arr[index][i] = np.inf
        arr[i][index] = np.inf

    # turn the modified edge array back into a graph
    graph = csr_matrix(arr)
    return graph


def find_nearest_unknown(graph, source, num_cols, known_nodes):
    start_index = node_to_index(source[0], source[1], num_cols)
    distance_matrix, predecessors = dijkstra(graph, directed=False, indices=start_index, return_predecessors=True)

    min_distance = min(distance_matrix)
    nearest_index = np.where(distance_matrix == min_distance)[0][0]
    nearest_node = index_to_node(nearest_index, num_cols)

    while nearest_node in known_nodes:
        distance_matrix[nearest_index] = np.inf

        min_distance = min(distance_matrix)
        if min_distance == np.inf:
            return None
        nearest_index = np.where(distance_matrix == min_distance)[0][0]
        nearest_node = index_to_node(nearest_index, num_cols)
    return nearest_node


# find a path from the source to the target
def find_path(graph, source, target, num_cols):
    path = []  # initialize path

    # get the graph indices for the source and target
    start_index = node_to_index(source[0], source[1], num_cols)
    target_index = node_to_index(target[0], target[1], num_cols)

    # find paths from start_index to all the other points in the graph
    distance_matrix, predecessors = dijkstra(graph, directed=False, indices=start_index, return_predecessors=True)

    # start at target and work backwards to find path from target to source
    while target_index != start_index and target_index != -9999:
        path.append(target_index)
        target_index = predecessors[target_index]

    # append start index to path because the while loop ends before it can be appended
    path.append(start_index)

    # convert graph indices to x, y coordinates
    path = [index_to_node(index, num_cols) for index in path]

    # reverse to get path from source to target
    path.reverse()
    return path


def indices_to_node(row, col, num_rows):
    x_coordinate = col
    y_coordinate = num_rows - row - 1
    return x_coordinate, y_coordinate


def node_to_indices(x_coordinate, y_coordinate, num_rows):
    row = num_rows - y_coordinate - 1
    col = x_coordinate
    return row, col
