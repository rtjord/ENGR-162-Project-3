import numpy as np
from path_finding import *

GEARS = 'G'
PATH = 'X'
UNKNOWN = 'U'
WALL = '!'
ORIGIN = 'O'
WAYPOINT = 'W'
CLEAR = ' '
TARGET = 'T'

arr = np.array([['U', '!', ' '],
                ['U', '!', 'U'],
                ['!', 'G', '!'],
                ['U', '!', 'U'],
                ['X', 'X', ' '],
                [' ', 'X', ' '],
                [' ', 'X', 'U'],
                [' ', 'X', ' '],
                [' ', 'O', ' '],
                ['U', ' ', ' ']]
               )

# construct a graph to represent the map
num_rows = 10
num_cols = 3
graph = grid_graph(num_rows, num_cols)

# Get the indices of marks of the map
walls = np.array(np.where(arr == WALL)).T

# indices of all known points

# Clear marks are considered known
known_indices = np.array(np.where(arr != UNKNOWN)).T

# convert indices to coordinates
wall_nodes = [indices_to_node(row, col, num_rows) for row, col in walls]
known_nodes = [indices_to_node(row, col, num_rows) for row, col in known_indices]
print('Known nodes:', known_nodes)
print('Wall nodes:', wall_nodes)

# remove the walls from the graph
for node in wall_nodes:
    graph = remove_node(graph, node, num_cols)

source_node = indices_to_node(2, 1, num_rows)  # start at GEARS

# set target to the nearest unknown node
target_node = find_nearest_unknown(graph, source_node, num_cols, known_nodes)
print(target_node)
if target_node is None:
    print('Could not locate unknown point. Expanding map.')
    arr = np.pad(arr, [(1, 1), (1, 1)], mode='constant', constant_values=UNKNOWN)
    print(arr)
else:
    print('Target node:', target_node)
    # find a path from GEARS to the target
    node_path = find_path(graph, source_node, target_node, num_cols)
    print('Node path:', node_path)
