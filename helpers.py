import numpy as np


# Convert linear speed to angular speed
def get_dps(linear_speed, wheel_radius):
    circumference = 2 * np.pi * wheel_radius
    return linear_speed * 360 / circumference


# Get the magnitude of a vector of arbitrary length
def get_magnitude(*args):
    return np.sqrt(np.dot(args, args))


# Get the distance between two vectors
def get_distance(vec1, vec2):
    displacement = np.subtract(vec1, vec2)
    return get_magnitude(*displacement)
