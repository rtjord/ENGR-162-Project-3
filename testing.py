import numpy as np
import json

arr = np.array([[-1, -1, -1, -1],
                [1, 1, 1, 0],
                [-1, 0, 0, -1],
                [-1, -1, -1, -1]])


def find_unknown(arr, row, col):
    # Check out of bounds
    if row < 0 or row >= len(arr) or col < 0 or col >= len(arr[0]):
        return -1, -1

    # Ran into wall
    if arr[row][col] == -1:
        return -1, -1

    # found unknown
    if arr[row][col] == 0:
        return row, col

    # prevent doubling back
    arr[row][col] = -1

    # return coordinates if at an unexplored area
    return find_unknown(arr.copy(), row + 1, col), find_unknown(arr.copy(), row - 1, col), find_unknown(arr.copy(), row, col + 1), find_unknown(arr.copy(), row, col - 1)


points = find_unknown(arr.copy(), 1, 0)


def unpack(nested_tuple, flattened_list):
    for t in nested_tuple:
        if isinstance(t, tuple):
            unpack(t, flattened_list)
        else:
            flattened_list.append(t)
    return flattened_list


flattened_list = []
flattened_list = unpack(points, flattened_list)

points = []
for i in range(0, len(flattened_list), 2):
    points.append([flattened_list[i], flattened_list[i+1]])

while [-1, -1] in points:
    points.remove([-1, -1])
print(points)