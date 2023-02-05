from numpy import pi, sqrt, dot, zeros
from time import sleep


# Convert linear speed to angular speed
def get_dps(linear_speed, wheel_radius):
    circumference = 2 * pi * wheel_radius
    return linear_speed * 360 / circumference


# Convert sensor reading to measurement using linear regression model
def linear_regression(sensor_reading, slope, y_int):
    return slope * sensor_reading + y_int


# Get the magnitude of a vector of arbitrary length
def get_magnitude(*args):
    return sqrt(dot(args, args))


class VirtualGears():
    def __init__(self, max_speed=20, min_obstacle_dist=5, back_radius=4, mode='auto', buffer_time=0.02):

        # MOTORS AND WHEELS
        # Assign wheels to BrickPi ports
        #
        #

        # SENSORS
        # Magnetic Sensor
        self.magnet_magnitude = 1  # magnitude of IMU magnet reading
        self.magnet_detected = False  # Is the IMU currently detecting a magnet (reading 0)?
        self.magnet_zero_count = 0  # number of consecutive zeros read by IMU

        # Ultrasonic Sensor
        self.ultrasonic_sensor_port = 4  # Assign ultrasonic sensor to port 4
        self.min_obstacle_dist = min_obstacle_dist  # Set minimum obstacle distance (cm)

        # Map
        self.map = zeros((8, 16))  # Initialize the map as an 8 x 16 array of zeros

        # ADDITIONAL ATTRIBUTES
        self.on = False
        self.buffer_time = buffer_time  # Set time between run cycles (seconds)
        self.mode = mode
        self.mode_list = ['auto', 'manual']

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

    # Parameters
    # 1. x: x-coordinate of GEARS
    # 2. y: y-coordinate of GEARS
    # Description: Update the map with the position of GEARS
    def update_map(self, x, y, obstacle_type):
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

        # Check that mode is valid
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
