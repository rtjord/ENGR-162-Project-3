from brickpi3 import BrickPi3
import grovepi
from MPU9250 import MPU9250
import numpy as np
from time import sleep
import os

GEARS = 2
PATH = 1
UNKNOWN = 0
OBSTACLE = -1


# Convert linear speed to angular speed
def get_dps(linear_speed, wheel_radius):
    circumference = 2 * np.pi * wheel_radius
    return linear_speed * 360 / circumference


# Convert sensor reading to measurement using linear regression model
def linear_regression(sensor_reading, slope, y_int):
    return slope * sensor_reading + y_int


# Get the magnitude of a vector of arbitrary length
def get_magnitude(*args):
    return np.sqrt(np.dot(args, args))


class Gears(BrickPi3):
    def __init__(self, mode='auto', max_speed=15, wheel_radius=2, track_width=4, buffer_time=0.01):

        # Initialize parent class
        BrickPi3.__init__(self)

        # MOTORS AND WHEELS
        # Assign wheels to BrickPi ports
        self.left_wheel = self.PORT_D
        self.right_wheel = self.PORT_A

        self.all_motors = self.left_wheel + self.right_wheel

        # Reset all motor encoders to 0
        self.reset_motor_encoders()

        # Radius of wheels (assumes all wheels have the same radius)
        self.wheel_radius = wheel_radius
        self.wheel_circumference = 2 * np.pi * self.wheel_radius  # cm

        # Distance between two wheels on the same axle
        self.track_width = track_width

        # Motor Speeds
        self.max_dps = get_dps(max_speed, wheel_radius)
        self.left_dps = 0
        self.right_dps = 0

        # SENSORS
        # Magnetic Sensor
        # self.imu = MPU9250()  # IMU (magnetic sensor)
        # self.magnet_magnitude = 1  # magnitude of IMU magnet reading
        # self.magnet_zero_count = 0  # number of consecutive zeros read by IMU
        # self.magnet_detected = False  # Is the IMU currently detecting a magnet (reading 0)?

        # Ultrasonic Sensor
        self.ultrasonic_sensor_port = 2  # Assign ultrasonic sensor to port 2

        # MAP
        self.map = np.zeros((8, 16))  # Initialize the map as an 8 x 16 array of zeros
        self.map = np.pad(self.map, [(1, 1), (1, 1)], mode='constant', constant_values=OBSTACLE)
        self.x_position = 0
        self.y_position = 0
        self.orientation = 0
        self.heading = 0
        self.prev_left_encoder = 0
        self.prev_right_encoder = 0
        self.tile_width = 4  # width of a tile on the map (cm)
        self.hazards = {'type': [], 'parameter': [], 'value': [], 'x': [], 'y': []}

        # ADDITIONAL ATTRIBUTES
        self.on = False
        self.buffer_time = buffer_time  # Set time between cycles (seconds)
        self.mode = mode
        self.mode_list = ['auto', 'manual']

    # Reset a single motor encoder to 0
    def reset_encoder(self, port):
        self.offset_motor_encoder(port, self.get_motor_encoder(port))

    # Reset all motor encoders to 0
    def reset_motor_encoders(self):
        self.reset_encoder(self.left_wheel)
        self.reset_encoder(self.right_wheel)

    # BASIC MOVEMENT METHODS
    # Move straight forward at a constant speed
    def move_forward(self):
        self.orientation = round(self.orientation / 90) * 90
        self.left_dps = self.max_dps
        self.right_dps = self.max_dps

    # Move straight backward at a constant speed
    def reverse(self):
        self.orientation = round(self.orientation / 90) * 90
        self.left_dps = -1 * self.max_dps
        self.right_dps = -1 * self.max_dps

    def turn_left(self):
        if not self.turning:
            self.heading += 90

    def turn_right(self):
        if not self.turning:
            self.heading -= 90

    # Make GEARS turn until its orientation matches its heading
    def correct_orientation(self):
        # Use proportional control to minimize the difference between the orientation
        # and desired heading of GEARS
        error = self.heading - self.orientation
        if abs(error) < 2:
            return

        dps = 5 * error
        # limit the dps of the motors
        dps = min(self.max_dps, max(-1 * self.max_dps, dps))
        # Set the motor dps values
        self.left_dps = -1 * dps
        self.right_dps = dps

    # Do not move
    def stop(self):
        self.left_dps = 0
        self.right_dps = 0

    def detect_obstacles(self):

        # Read the ultrasonic sensor
        sensor_reading = grovepi.ultrasonicRead(self.ultrasonic_sensor_port)

        # Convert sensor reading to distance in cm
        # TODO: Determine parameters for linear regression model
        obstacle_distance = linear_regression(sensor_reading, 0.4, -0.7)
        if obstacle_distance < self.tile_width:
            if 315 < self.orientation <= 359 or 0 <= self.orientation < 45:
                self.update_map(self.x_position + self.tile_width, self.y_position, OBSTACLE)
            elif 45 <= self.orientation < 135:
                self.update_map(self.x_position, self.y_position + self.tile_width, OBSTACLE)
            elif 135 <= self.orientation < 225:
                self.update_map(self.x_position - self.tile_width, self.y_position, OBSTACLE)
            else:
                self.update_map(self.x_position, self.y_position + self.tile_width, OBSTACLE)

    def detect_hazards(self):
        pass

    def record_hazard(self, hazard_type, parameter, value, x, y):
        self.hazards['type'].append(hazard_type)
        self.hazards['parameter'].append(parameter)
        self.hazards['value'].append(value)
        self.hazards['x'].append(x)
        self.hazards['y'].append(y)

    def avoid_obstacles(self):
        pass

    def update_orientation(self):

        # Position of the left wheels in degrees
        left_encoder = self.get_motor_encoder(self.left_wheel)

        # Position of the right wheels in degrees
        right_encoder = self.get_motor_encoder(self.right_wheel)

        # the orientation of GEARS should be a constant multiple of the difference in motor encoder values
        # TODO: Convert motor encoder difference of wheels to orientation of GEARS
        self.orientation = 0.2 * (right_encoder - left_encoder)

        # # Keep orientation between 0 and 359 degrees
        # self.orientation %= 360

    def update_position(self):
        # Get position of the left and right motors (measured in degrees)
        left_encoder = self.get_motor_encoder(self.left_wheel)
        right_encoder = self.get_motor_encoder(self.right_wheel)

        # Determine how much the positions of the left and right motors have changed
        # since the last cycle
        left_change = left_encoder - self.prev_left_encoder
        right_change = right_encoder - self.prev_right_encoder

        # If GEARS is moving in a straight line, the average change is non-zero,
        # so the position of GEARS will be updated.
        # If GEARS is turning, the average change is zero, so the position of
        # GEARS will not change.
        average_change = (left_change + right_change) / 2

        # Convert degrees to cm to get the linear change in distance
        delta = average_change / 360 * self.wheel_circumference

        # Update x and y position of GEARS
        self.x_position += delta * np.cos(np.radians(self.orientation))
        self.y_position += delta * np.sin(np.radians(self.orientation))

        # Record the encoder values to use for reference on the next cycle
        self.prev_left_encoder = left_encoder
        self.prev_right_encoder = right_encoder

    # Mark the current position of GEARs on the map
    # Assumes GEARS started in the lower left hand corner of the map
    def update_map(self, x_pos, y_pos, mark):
        # Convert x and y positions to row and column indices for map
        row = int(len(self.map) - y_pos / self.tile_width) - 2
        col = int(x_pos / self.tile_width) + 1
        row = max(1, min(len(self.map)-2, row))
        col = max(1, min(len(self.map[0])-2, col))

        # If marking the position of GEARS
        if mark == 2:
            # Replace previous GEARS mark with path mark
            self.map[self.map == 2] = 1

        self.map[row][col] = mark

    def display_map(self):
        for row in range(8):
            for col in range(16):
                if self.map[row+1][col+1] == 1:
                    print('X', end=' ')
                else:
                    print(' ', end=' ')
            print()

    def set_heading(self, degrees):
        self.heading = degrees % 360

    # Check if GEARS has completed the mission
    def check_finished(self):
        pass

    # Set position and/or dps for all motors
    # This is the only method that interfaces directly with the motors
    def update_motors(self):
        self.set_motor_dps(self.left_wheel, self.left_dps)
        self.set_motor_dps(self.right_wheel, self.right_dps)

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
            self.update_orientation()  # Get the new orientation
            self.update_position()  # Get the new x and y coordinates
            self.update_map(self.x_position, self.y_position, 2)  # Mark the position on the map
            self.correct_orientation()  # Make GEARS turn to face the desired heading
            self.detect_obstacles()  # Detect obstacles with the ultrasonic sensor
            self.check_finished()

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
