from brickpi3 import BrickPi3
import grovepi
from MPU9250 import MPU9250
import numpy as np
from time import sleep


# Convert linear speed to angular speed
def get_dps(linear_speed, wheel_radius):
    circumference = 2 * np.pi * wheel_radius
    return linear_speed * 360 / circumference


# Convert sensor reading to measurement using linear regression model
# Default settings for converting ultrasonic sensor reading to cm
def linear_regression(sensor_reading, slope, y_int):
    return slope * sensor_reading + y_int


# Get the magnitude of a vector of arbitrary length
def get_magnitude(*args):
    return np.sqrt(np.dot(args, args))


class Gears(BrickPi3):
    def __init__(self, mode='auto', buffer_time=0.02):

        # Initialize parent class
        BrickPi3.__init__(self)

        # MOTORS AND WHEELS
        # Assign wheels to BrickPi ports
        # Example shown below
        self.wheel = self.PORT_A

        # Reset all motor encoders to 0
        self.reset_motor_encoders()

        # SENSORS
        # Magnetic Sensor
        self.imu = MPU9250()  # IMU (magnetic sensor)
        self.magnet_magnitude = 1  # magnitude of IMU magnet reading
        self.magnet_zero_count = 0  # number of consecutive zeros read by IMU
        self.magnet_detected = False  # Is the IMU currently detecting a magnet (reading 0)?

        # Ultrasonic Sensor
        self.ultrasonic_sensor_port = 4  # Assign ultrasonic sensor to port 4

        # Map
        self.map = np.zeros((8, 16))  # Initialize the map as an 8 x 16 array of zeros
        self.map = np.pad(map, [(1, 1), (1, 1)], mode='constant', constant_value=1)
        self.x_position = 0
        self.y_position = 0
        self.orientation = 0
        self.distance_traveled = 0

        # ADDITIONAL ATTRIBUTES
        self.on = False
        self.buffer_time = buffer_time  # Set time between run cycles (seconds)
        self.mode = mode
        self.mode_list = ['auto', 'manual']

    # Reset a single motor encoder to 0
    def reset_encoder(self, port):
        self.offset_motor_encoder(port, self.get_motor_encoder(port))

    # Reset all motor encoders to 0
    def reset_motor_encoders(self):
        pass

    # BASIC MOVEMENT METHODS
    def move_forward(self):
        pass

    def reverse(self):
        pass

    def turn_left(self):
        pass

    def turn_right(self):
        pass

    def stop(self):
        pass

    def detect_obstacles(self):
        pass

    def avoid_obstacles(self):
        pass

    # Possible implementation shown. Need to decide on wheel design.
    def track_position(self):
        current_distance = self.get_motor_encoder(self.wheel)
        delta = current_distance - self.distance_traveled
        self.x_position += delta * np.cos(self.orientation)
        self.x_position += delta * np.sin(self.orientation)
        self.distance_traveled = current_distance

    # Parameters
    # 1. x: x-coordinate of GEARS
    # 2. y: y-coordinate of GEARS
    # Description: Update the map with the position of GEARS
    def update_map(self, x, y):
        self.map[x][y] = 'X'

    # Check if GEARS has completed the mission
    def check_finished(self):
        pass

    # Set position and/or dps for all motors
    # This is the only method that interfaces directly with the motors
    def update_motors(self):
        pass

    # Main logic for the rover during the primary demonstration
    # Later method calls have higher priority
    def run(self):

        # Check that a valid mode is selected
        if self.mode not in self.mode_list:
            print('Unknown mode. Switching to auto')
            self.mode = 'auto'

        # If GEARS is on
        if self.on:
            # BASE METHODS

            # MODE DEPENDENT METHODS
            if self.mode == 'auto':
                pass
            elif self.mode == 'manual':
                pass

        # If GEARS is off
        else:
            self.stop()

        self.update_motors()  # Update dps values for the motors (Directly interfaces with motors)
        sleep(self.buffer_time)  # Wait several milliseconds before repeating
