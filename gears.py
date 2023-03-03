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
START = 3
WAYPOINT = 4


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
    def __init__(self, mode='auto', max_speed=5, wheel_radius=2, buffer_time=0.01):

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
        self.map = np.array([[START]])  # Initialize the map
        self.start_row = 0  # row index of start position
        self.start_col = 0  # column index of start position
        self.x_position = 0  # x position in cm
        self.y_position = 0  # y position in cm
        self.x_min = 0  # left edge of map (cm)
        self.x_max = 0  # right edge of map (cm)
        self.y_min = 0  # bottom edge of map (cm)
        self.y_max = 0  # top edge of map (cm)
        self.row = 0  # current row of GEARS on map
        self.col = 0  # current column of GEARS on map
        self.orientation = 0  # actual orientation (degrees)
        self.heading = 0  # desired orientation (degrees)
        self.turning = False  # Is GEARS currently executing a turn?
        self.prev_left_encoder = 0  # value of left encoder from previous cycle
        self.prev_right_encoder = 0  # value of right encoder from previous cycle
        self.tile_width = 4  # width of a tile on the map (cm)
        self.hazards = {'type': [], 'parameter': [], 'value': [], 'x': [], 'y': []}  # Dictionary of hazards

        # ADDITIONAL ATTRIBUTES
        self.on = False  # Is GEARS on?
        self.buffer_time = buffer_time  # time between cycles (seconds)
        self.mode = mode  # current mode
        self.mode_list = ['auto', 'manual']  # list of known modes

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
            self.turning = True

    def turn_right(self):
        if not self.turning:
            self.heading -= 90
            self.turning = True

    # Do not move
    def stop(self):
        self.left_dps = 0
        self.right_dps = 0

    # Make GEARS turn until its orientation matches its heading
    def correct_orientation(self):
        # Use proportional control to minimize the difference between the orientation
        # and desired heading of GEARS
        if self.turning:
            error = self.heading - self.orientation
            dps = 5 * error
            # limit the dps of the motors
            dps = min(self.max_dps, max(-1 * self.max_dps, dps))
            # Set the motor dps values
            self.left_dps = -1 * dps
            self.right_dps = dps
            if abs(error) < 1:
                self.stop()
                self.turning = False

    def record_hazard(self, hazard_type, parameter, value, x, y):
        self.hazards['type'].append(hazard_type)
        self.hazards['parameter'].append(parameter)
        self.hazards['value'].append(value)
        self.hazards['x'].append(x)
        self.hazards['y'].append(y)

    def update_map_pos(self):
        try:
            r, c = np.where(self.map == 2)
            self.row = r[0]
            self.col = c[0]
        except IndexError:
            r, c = np.where(self.map == 3)
            self.row = r[0]
            self.col = c[0]

    def detect_obstacles(self):

        # Read the ultrasonic sensor
        sensor_reading = grovepi.ultrasonicRead(self.ultrasonic_sensor_port)

        # Convert sensor reading to distance in cm
        # TODO: Determine parameters for linear regression model
        obstacle_distance = linear_regression(sensor_reading, 0.4, -0.7)

        orientation = self.orientation % 360
        if orientation < 0:
            orientation += 360

        if obstacle_distance < self.tile_width:
            if 315 < orientation <= 359 or 0 <= orientation < 45:
                self.update_map(self.x_position + self.tile_width, self.y_position, OBSTACLE)
            elif 45 <= orientation < 135:
                self.update_map(self.x_position, self.y_position + self.tile_width, OBSTACLE)
            elif 135 <= orientation < 225:
                self.update_map(self.x_position - self.tile_width, self.y_position, OBSTACLE)
            else:
                self.update_map(self.x_position, self.y_position - self.tile_width, OBSTACLE)

    def avoid_obstacles(self):
        r = round(self.row - np.sin(np.radians(self.heading)))
        c = round(self.col + np.cos(np.radians(self.heading)))
        try:
            front_mark = self.map[r][c]
        except IndexError:
            front_mark = UNKNOWN

        if r < 0 or c < 0:
            front_mark = UNKNOWN

        if front_mark == OBSTACLE or front_mark == PATH:
            self.turn_left()

    def get_map_indices(self, x_pos, y_pos):
        on_map = self.x_min <= x_pos <= self.x_max and self.y_min <= y_pos <= self.y_max
        if not on_map:
            print('Error. Position is not on map')
            return -1, -1

        row = self.start_row - int(x_pos / self.tile_width)
        col = self.start_col + int(y_pos / self.tile_width)

        return row, col

    def expand_map(self, x_pos, y_pos):

        # If mark is to the right of the map
        if round(x_pos) > self.x_max:
            num_cols = round((x_pos - self.x_max) / self.tile_width)
            for i in range(num_cols):
                # Add zeroes to the left side of the map
                self.map = np.pad(self.map, [(0, 0), (0, 1)], mode='constant', constant_values=UNKNOWN)

                # Update the max x position
                self.x_max += self.tile_width

        # If mark is to the left of the map
        if round(x_pos) < self.x_min:
            num_cols = round((self.x_min - x_pos) / self.tile_width)
            # Add a column of zeroes to the right side of hte map
            for i in range(num_cols):
                self.map = np.pad(self.map, [(0, 0), (1, 0)], mode='constant', constant_values=UNKNOWN)

                # Update the min x position
                self.x_min -= self.tile_width

        # If mark is above the map
        if round(y_pos) > self.y_max:
            num_rows = round((y_pos - self.y_max) / self.tile_width)
            for i in range(num_rows):
                self.map = np.pad(self.map, [(1, 0), (0, 0)], mode='constant', constant_values=UNKNOWN)
                self.y_max += self.tile_width

        # If gears is below the map
        if round(y_pos) < self.y_min:
            num_rows = round((self.y_min - y_pos) / self.tile_width)
            for i in range(num_rows):
                self.map = np.pad(self.map, [(0, 1), (0, 0)], mode='constant', constant_values=UNKNOWN)
                self.y_min -= self.tile_width

    def update_position(self):
        # If GEARS is turning, do not update its position
        if self.turning:
            return
        # Get position of the left and right motors (measured in degrees)
        left_encoder = self.get_motor_encoder(self.left_wheel)
        right_encoder = self.get_motor_encoder(self.right_wheel)

        # Determine how much the positions of the left and right motors have changed
        # since the last cycle
        left_change = left_encoder - self.prev_left_encoder
        right_change = right_encoder - self.prev_right_encoder

        average_change = (left_change + right_change) / 2

        # Convert degrees to cm to get the linear change in distance
        delta = average_change / 360 * self.wheel_circumference

        # Update x and y position of GEARS
        self.x_position += delta * np.cos(np.radians(self.heading))
        self.y_position += delta * np.sin(np.radians(self.heading))

        # Record the encoder values to use for reference on the next cycle
        self.prev_left_encoder = left_encoder
        self.prev_right_encoder = right_encoder

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

    # Mark the current position of GEARs on the map
    # Assumes GEARS started in the lower left hand corner of the map
    def update_map(self, x_pos, y_pos, mark):

        # If the marking is off the map, expand the map
        self.expand_map(x_pos, y_pos)

        # Get the row and column indices of the start position
        start_row, start_col = np.where(self.map == START)
        self.start_row = start_row[0]
        self.start_col = start_col[0]

        # Get the current row and col indices of GEARS
        current_row = int(start_row - y_pos / self.tile_width)
        current_col = int(start_col + x_pos / self.tile_width)

        current_row = max(0, min(len(self.map)-1, current_row))
        current_col = max(0, min(len(self.map[0])-1, current_col))

        # If marking the position of GEARS
        if mark == GEARS:
            # Replace previous GEARS mark with path mark
            self.map[self.map == GEARS] = PATH

        # Do not overwrite initial position marking
        if current_row != start_row or current_col != start_col:
            self.map[current_row][current_col] = mark

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

    def point_turn(self):
        self.heading = float(input('Enter the desired angle: '))
        self.turning = True

    def mark_waypoint(self):
        x_coordinate = int(input('Enter the x-coordinate: '))
        y_coordinate = int(input('Enter the y-coordinate: '))

        x_pos = x_coordinate * self.tile_width
        y_pos = y_coordinate * self.tile_width

        self.map[self.map == WAYPOINT] = UNKNOWN  # Clear previous waypoint

        self.update_map(x_pos, y_pos, WAYPOINT)

    def goto_point(self):
        x_coordinate = int(input('Enter the x-coordinate: '))
        y_coordinate = int(input('Enter the y-coordinate: '))

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
            self.update_orientation()  # Get the new orientation of GEARS
            self.update_position()  # Get the new x and y coordinates of GEARS
            self.update_map(self.x_position, self.y_position, 2)  # Mark the position of GEARS on the map
            self.update_map_pos()  # Get the new row and column of GEARS on the map
            self.detect_obstacles()  # Detect obstacles with the ultrasonic sensor and mark them on the map

            # MODE DEPENDENT METHODS
            if self.mode == 'auto':
                self.move_forward()
                self.avoid_obstacles()  # Turn left if facing an obstacle
            elif self.mode == 'manual':
                pass

            self.correct_orientation()  # Make GEARS turn to face the desired heading
        # If GEARS is off
        else:
            self.stop()

        self.update_motors()  # Update dps values for the motors (Directly interfaces with motors)
        sleep(self.buffer_time)  # Wait several milliseconds before repeating
