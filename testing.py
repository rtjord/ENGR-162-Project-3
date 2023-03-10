import numpy as np
import json


# Get the magnitude of a vector of arbitrary length
def get_magnitude(*args):
    return np.sqrt(np.dot(args, args))


# Get the distance between two vectors
def get_distance(vec1, vec2):
    displacement = np.subtract(vec1, vec2)
    return get_magnitude(*displacement)


def find_unknowns(arr, row, col, points):
    arr = arr.copy()
    # Check out of bounds
    if row < 0 or row >= len(arr) or col < 0 or col >= len(arr[0]):
        return

    # Ran into wall
    if arr[row][col] == -1:
        return

    # found unknown
    if arr[row][col] == 0:
        points.append((row, col))
        return

    # prevent doubling back
    arr[row][col] = -1

    # return coordinates if at an unexplored area
    find_unknowns(arr, row + 1, col, points)
    find_unknowns(arr, row - 1, col, points)
    find_unknowns(arr, row, col + 1, points)
    find_unknowns(arr, row, col - 1, points)
    return


def get_nearest_unknown():
    unknown_points = []
    find_unknowns(arr, 0, 0, unknown_points)

    if len(unknown_points) == 0:
        return None

    gears = np.array([0, 0])
    nearest_point = np.array(unknown_points[0])
    min_distance = get_distance(gears, nearest_point)

    for point in unknown_points:
        point = np.array(point)
        distance = get_distance(gears, point)
        if distance < min_distance:
            nearest_point = point
            min_distance = distance
    return nearest_point


def is_accessible(arr, row, col, target_row, target_col):
    arr = arr.copy()

    # Check out of bounds
    if row < 0 or row >= len(arr) or col < 0 or col >= len(arr[0]):
        return False

    # Check for walls
    if arr[row][col] == -1:
        return False

    if row == target_row and col == target_col:
        return True

    arr[row][col] = -1

    return is_accessible(arr, row+1, col, target_row, target_col) or is_accessible(arr, row-1, col, target_row, target_col) or \
        is_accessible(arr, row, col+1, target_row, target_col) or is_accessible(arr, row, col-1, target_row, target_col)


def find_path(arr, row, col, target_row, target_col, path):
    arr = arr.copy()

    # Check out of bounds
    if row < 0 or row >= len(arr) or col < 0 or col >= len(arr[0]):
        return

    if arr[row][col] == -1:
        return

    if row == target_row and col == target_col:
        path.append((row, col))
        return

    if is_accessible(arr, row, col, target_row, target_col):
        path.append((row, col))

    arr[row][col] = -1
    find_path(arr, row+1, col, target_row, target_col, path)
    find_path(arr, row-1, col, target_row, target_col, path)
    find_path(arr, row, col+1, target_row, target_col, path)
    find_path(arr, row, col-1, target_row, target_col, path)
    return


arr = np.array([[0, 5, 0, 0],
 [5, 2, 5, 0],
 [0, 0, 0, 0],
 [0, 4, 0, 0],
 [5, 1, 5, 0],
 [5, 4, 5, 0],
 [5, 4, 4, 5],
 [4, 1, 1, 5],
 [5, 5, 5, 5]]
)


print(arr)
nearest_unknown = get_nearest_unknown()
print(nearest_unknown)
r, c = nearest_unknown

path = []
find_path(arr, 1, 1, r, c, path)
print(path)

