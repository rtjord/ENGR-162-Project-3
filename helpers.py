import numpy as np


# Convert linear speed to angular speed
def get_dps(linear_speed, wheel_radius):
    circumference = 2 * np.pi * wheel_radius
    return linear_speed * 360 / circumference


# Convert sensor reading to measurement using linear regression model
def linear_regression(sensor_reading, slope, y_int):
    return slope * sensor_reading + y_int


# Convert sensor reading to measurement using power regression model
def power_regression(sensor_reading, coefficient, power):
    return coefficient * sensor_reading ** power


# Convert sensor reading to measurement using quadratic regression model
def quadratic_regression(sensor_reading, a, b, c):
    return a * sensor_reading ** 2 + b * sensor_reading + c


# Get the magnitude of a vector of arbitrary length
def get_magnitude(*args):
    return np.sqrt(np.dot(args, args))


# Get the distance between two vectors
def get_distance(vec1, vec2):
    displacement = np.subtract(vec1, vec2)
    return get_magnitude(*displacement)


# round floats with decimal part .5 away from 0
def round_half_up(n):
    if n >= 0:
        return int(n + 0.5)
    if n < 0:
        return -1 * int(abs(n) + 0.5)


# round angle to nearest 90 degrees
def round_to_90(angle):
    return round(angle / 90) * 90
