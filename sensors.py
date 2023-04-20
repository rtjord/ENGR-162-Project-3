import grovepi
from helpers import linear_regression, exponential_regression, power_regression, get_magnitude
import numpy as np


# Return the distance to the nearest wall detected by the ultrasonic sensor
def read_ultrasonic(port):
    sensor_reading = grovepi.ultrasonicRead(port)
    distance = linear_regression(sensor_reading, 1.0965, -1.5181)
    return distance


# Return the distance to the IR beacon
def read_infrared(port1, port2):
    reading1 = grovepi.analogRead(port1)
    reading2 = grovepi.analogRead(port2)
    avg = (reading1 + reading2) / 2  # calculate average IR value
    distance = exponential_regression(avg, 303.0348, 0.9726)
    return distance, avg


# return the distance
def read_imu(imu):
    magnet = imu.readMagnet()  # Read the magnet
    x, y, z = magnet['x'], magnet['y'], magnet['z']

    # flip x-axis to use right hand rule
    x *= -1
    magnitude = get_magnitude(x, y, z)
    magnitude = max(magnitude, 0.01)
    distance = power_regression(magnitude, 178.2277, -0.4372)
    direction = np.array([x, y, z]) / magnitude

    # Get the magnitude of the reading
    return magnitude, distance, direction
