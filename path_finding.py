
# Determine if (target_row, target_col) is accessible from (row, col)
def is_accessible(arr, row, col, target_row, target_col):

    # create a copy of the array to avoid conflict between recursive calls
    arr = arr.copy()

    # Check out of bounds
    if row < 0 or row >= len(arr) or col < 0 or col >= len(arr[0]):
        return False

    # Check for walls
    if arr[row][col] == -1:
        return False

    # Check for target
    if row == target_row and col == target_col:
        return True

    # prevent doubling back
    arr[row][col] = -1

    # continue search in all directions
    return is_accessible(arr, row+1, col, target_row, target_col) or is_accessible(arr, row-1, col, target_row, target_col) or \
        is_accessible(arr, row, col+1, target_row, target_col) or is_accessible(arr, row, col-1, target_row, target_col)


# Find a path from (row, col) to (target_row, target_col)
def find_path(arr, row, col, target_row, target_col, path):

    # create a copy of the array to avoid conflict between recursive calls
    arr = arr.copy()

    # Check out of bounds
    if row < 0 or row >= len(arr) or col < 0 or col >= len(arr[0]):
        return

    # Check for walls
    if arr[row][col] == -1:
        return

    # Check for target
    if row == target_row and col == target_col:
        path.append((row, col))
        return

    # Only include points that are not dead ends
    if is_accessible(arr, row, col, target_row, target_col):
        path.append((row, col))

    # prevent doubling back
    arr[row][col] = -1

    # Continue search in all directions
    find_path(arr, row+1, col, target_row, target_col, path)
    find_path(arr, row-1, col, target_row, target_col, path)
    find_path(arr, row, col+1, target_row, target_col, path)
    find_path(arr, row, col-1, target_row, target_col, path)
    return
