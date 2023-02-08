import numpy as np
import random

map = np.zeros((8, 16))
map = np.pad(map, [(1, 1), (1, 1)], mode='constant', constant_values=-1)

map[3][6] = 2

for i in range(200):
    row = random.randint(1, 9)
    col = random.randint(1, 17)
    if map[row][col] != 2:
        map[row][col] = 1
print(map)

target_found = False


def find_path(map, row, col, path):
    global target_found
    # Out of bounds
    if row < 0 or row >= len(map) or col < 0 or col >= len(map[0]):
        return 0

    # Ran into wall
    if map[row][col] == 0 or map[row][col] == -1:
        return 0

    # Found target
    if map[row][col] == 2:
        path.append((row, col))
        target_found = True
        return 1

    # Prevent doubling back
    map[row][col] = 0
    if not target_found:
        path.append((row, col))
    # Keep searching
    return 0 + find_path(map, row+1, col, path) + \
        find_path(map, row, col+1, path) + \
        find_path(map, row-1, col, path) + \
        find_path(map, row, col-1, path)


my_path = []
path_exists = find_path(map.copy(), 1, 1, my_path)
print(f'Path exists: {bool(path_exists)}')
print(f'Number of clear squares adjacent to path: {path_exists}')

print(my_path)
