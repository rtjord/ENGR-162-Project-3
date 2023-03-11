from scipy.sparse.csgraph import dijkstra
from scipy.sparse import csr_matrix
import numpy as np

GEARS = 'G'
PATH = 'X'
UNKNOWN = 'U'
WALL = '!'
ORIGIN = 'O'
WAYPOINT = 'W'
CLEAR = ' '
TARGET = 'T'


# Convert graph index to x, y coordinates
def graph_index_to_coordinates(index, num_cols):
    x = index % num_cols
    y = int(index / num_cols)
    return x, y


# Convert x, y coordinates to graph index
def graph_coordinates_to_index(x, y, num_cols):
    return x + num_cols * y


# Create a grid graph with dimensions m by n
def grid_graph(m, n):

    # the edges connect each node to every other node
    num_edges = m * n

    # initialize the array for the edges
    arr = np.zeros((num_edges, num_edges))

    # Compare the x and y coordinates of each node to those of every other node
    for i1 in range(num_edges):
        x1, y1 = graph_index_to_coordinates(i1, n)
        for i2 in range(num_edges):
            x2, y2 = graph_index_to_coordinates(i2, n)

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
    index = graph_coordinates_to_index(x, y, num_cols)

    # get edge array for graph
    arr = graph.toarray()

    # set the row and column corresponding to the node
    # to be removed equal to zero
    for i in range(num_cols**2):
        arr[index][i] = 0
        arr[i][index] = 0

    # turn the modified edge array back into a graph
    graph = csr_matrix(arr)
    return graph


# find a path from the source to the target
def find_path(graph, source, target, num_cols):
    path = []  # initialize path

    # get the graph indices for the source and target
    start_index = graph_coordinates_to_index(source[0], source[1], num_cols)
    target_index = graph_coordinates_to_index(target[0], target[1], num_cols)

    # find paths from start_index to all the other points in the graph
    distance_matrix, predecessors = dijkstra(graph, indices=start_index, return_predecessors=True)

    # start at target and work backwards to find path from target to source
    while target_index != start_index and target_index != -9999:
        path.append(target_index)
        target_index = predecessors[target_index]

    # append start index to path because the while loop ends before it can be appended
    path.append(start_index)

    # convert graph indices to x, y coordinates
    path = [graph_index_to_coordinates(index, num_cols) for index in path]

    # reverse to get path from source to target
    path.reverse()
    return path


# must pass a list for points
def find_unknowns(arr, row, col, points):

    # make a copy of the array to edit
    arr = arr.copy()

    # Check out of bounds
    if row < 0 or row >= len(arr) or col < 0 or col >= len(arr[0]):
        return

    # Ran into wall
    if arr[row][col] == WALL:
        return

    # found unknown
    if arr[row][col] == UNKNOWN:
        points.append((row, col))
        return

    # prevent doubling back
    arr[row][col] = WALL

    # return coordinates if at an unexplored area
    find_unknowns(arr, row + 1, col, points)
    find_unknowns(arr, row - 1, col, points)
    find_unknowns(arr, row, col + 1, points)
    find_unknowns(arr, row, col - 1, points)
    return


