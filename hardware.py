import grovepi
from helpers import linear_regression


# Return the distance to the nearest wall detected by the ultrasonic sensor
def read_ultrasonic(port):
    sensor_reading = grovepi.ultrasonicRead(port)
    distance = linear_regression(sensor_reading, 0.9423, 2.2666)
    return distance
