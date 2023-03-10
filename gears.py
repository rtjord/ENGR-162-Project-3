from brickpi3 import BrickPi3
import grovepi
import numpy as np
from time import sleep
import pandas as pd
from path_finding import find_unknowns, find_path

# GEARS = 2
# PATH = 1
# UNKNOWN = 0
# WALL = -1
# ORIGIN = 3
# WAYPOINT = 4
# CLEAR = 5
# TARGET = 6

GEARS = 'G'
PATH = 'X'
UNKNOWN = 'U'
WALL = '!'
ORIGIN = 'O'
WAYPOINT = 'W'
CLEAR = ' '
TARGET = 'T'

FRONT = 0
LEFT = 90
BACK = 180
RIGHT = 270


# Convert linear speed to angular speed
def get_dps(linear_speed, wheel_radius):
    circumference = 2 * np.pi * wheel_radius
    return linear_speed * 360 / circumference


# Convert sensor reading to measurement using linear regression model
def linear_regression(sensor_reading, slope, y_int):
    return slope * sensor_reading + y_int


# Return the distance to the nearest wall detected by the ultrasonic sensor
def read_ultrasonic(port):
    sensor_reading = grovepi.ultrasonicRead(port)
    distance = linear_regression(sensor_reading, 0.9423, 2.2666)
    return distance


# Get the magnitude of a vector of arbitrary length
def get_magnitude(*args):
    return np.sqrt(np.dot(args, args))


# Get the distance between two vectors
def get_distance(vec1, vec2):
    displacement = np.subtract(vec1, vec2)
    return get_magnitude(*displacement)


class Gears(BrickPi3):
    def __init__(self, mode='auto', max_speed=15, wheel_radius=3, buffer_time=0.01):

        # Initialize parent class
        BrickPi3.__init__(self)

        # MOTORS AND WHEELS
        # Assign wheels to BrickPi ports
        self.left_wheel = self.PORT_D
        self.right_wheel = self.PORT_A

        self.all_motors = self.left_wheel + self.right_wheel

        # Reset all motor encoders to 0
        self.reset_motor_encoders()

        # Wheels (assumes all wheels have the same radius)
        self.wheel_radius = wheel_radius
        self.wheel_circumference = 2 * np.pi * self.wheel_radius  # cm
        self.turning_constant = 0.135  # convert motor encoder difference to orientation

        # Motor Speeds
        self.max_dps = get_dps(max_speed, wheel_radius)
        self.left_dps = 0
        self.right_dps = 0

        # SENSORS
        # Ultrasonic Sensor
        self.front_ultrasonic = 2
        self.left_ultrasonic = 3
        self.right_ultrasonic = 7

        # MAP
        self.map = np.array([[ORIGIN]])  # Initialize the map
        self.origin_row = 0  # row of start position on map array
        self.origin_col = 0  # column of start position on map array
        self.x_position = 0  # cm right of the origin
        self.y_position = 0  # cm left of the origin
        self.x_coordinate = 0 # tile widths right of origin
        self.y_coordinate = 0 # tile widths above origin
        self.row = 0  # current row of GEARS on map
        self.col = 0  # current column of GEARS on map
        self.orientation = 0  # actual orientation (degrees)
        self.heading = 0  # desired orientation (degrees)
        self.turning = False  # Is GEARS currently executing a turn?
        self.origin_marked = False  # Is GEARS returning to the origin?
        self.prev_left_encoder = 0  # value of left encoder from previous cycle
        self.prev_right_encoder = 0  # value of right encoder from previous cycle
        self.tile_width = 40  # width of a tile on the map (cm)
        self.hazards = pd.DataFrame(columns=['type', 'parameter', 'value', 'x', 'y'])
        self.target_x = self.x_coordinate
        self.target_y = self.y_coordinate
        self.lead_x = self.x_coordinate
        self.lead_y = self.y_coordinate
        self.path = []
        self.path_index = 0

        # ADDITIONAL ATTRIBUTES
        self.on = False  # Is GEARS on?
        self.buffer_time = buffer_time  # time between cycles (seconds)
        self.mode = mode  # current mode
        self.mode_list = ['auto', 'manual', 'waypoint']  # list of known modes

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
        self.left_dps = self.max_dps
        self.right_dps = self.max_dps

    # Move straight backward at a constant speed
    def reverse(self):
        self.left_dps = -1 * self.max_dps
        self.right_dps = -1 * self.max_dps

    # Turn 90 degrees left
    def turn_left(self):
        if not self.turning:
            self.heading += 90
            self.turning = True

    # Turn 90 degrees right
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
        gain = 5

        if self.turning:
            error = self.heading - self.orientation
            dps = gain * error

            # limit the dps of the motors
            dps = min(self.max_dps, max(-1 * self.max_dps, dps))

            # Set the motor dps values
            self.left_dps = -1 * dps
            self.right_dps = dps

            # if the error is less than 0.5 degree
            if abs(error) < 0.5:

                # Stop turning
                self.stop()
                self.turning = False

    def wait_for_turn(self):
        while self.turning:
            self.correct_orientation()

    def record_hazard(self, hazard_type, parameter, value, x, y):

        # Create new row
        new_entry = pd.DataFrame([[hazard_type, parameter, value, x, y]], columns=self.hazards.columns)

        # Insert new row
        self.hazards = pd.concat([self.hazards, new_entry], ignore_index=True)

    def get_neighbor_coordinates(self, direction):
        x_position = self.x_position + self.tile_width * np.cos(np.radians(self.orientation + direction))
        y_position = self.y_position + self.tile_width * np.sin(np.radians(self.orientation + direction))
        x_coordinate, y_coordinate = self.position_to_coordinates(x_position, y_position)
        return x_coordinate, y_coordinate

    def detect_walls(self):

        # Get wall distance
        front_distance = read_ultrasonic(self.front_ultrasonic)
        left_distance = read_ultrasonic(self.left_ultrasonic)
        right_distance = read_ultrasonic(self.right_ultrasonic)

        # Get the coordinates of the tiles that sensors are pointing at
        front_x, front_y = self.get_neighbor_coordinates(FRONT)
        left_x, left_y = self.get_neighbor_coordinates(LEFT)
        right_x, right_y = self.get_neighbor_coordinates(RIGHT)

        # If the sensors detect a wall, mark it
        # Otherwise, mark the tile as clear

        if front_distance < self.tile_width / 2:
            self.update_map(front_x, front_y, WALL)
        else:
            self.update_map(front_x, front_y, CLEAR)

        if left_distance < self.tile_width / 2:
            self.update_map(left_x, left_y, WALL)
        else:
            self.update_map(left_x, left_y, CLEAR)

        if right_distance < self.tile_width / 2:
            self.update_map(right_x, right_y, WALL)
        else:
            self.update_map(right_x, right_y, CLEAR)

    def avoid_walls(self):
        r = round(self.row - np.sin(np.radians(self.heading)))
        c = round(self.col + np.cos(np.radians(self.heading)))
        try:
            front_mark = self.map[r][c]
        except IndexError:
            front_mark = UNKNOWN

        if r < 0 or c < 0:
            front_mark = UNKNOWN

        if front_mark == WALL:
            self.turn_left()

    def expand_map(self, row, col):
        num_rows = len(self.map)
        num_cols = len(self.map[0])

        # If mark is to the right of the map
        if col >= num_cols:

            # Add columns of zeroes to the right side of the map
            for i in range(col - num_cols + 1):
                self.map = np.pad(self.map, [(0, 0), (0, 1)], mode='constant', constant_values=UNKNOWN)

        # If mark is to the left of the map
        if col < 0:

            # Add columns of zeroes to the left side of the map
            for i in range(-1 * col):
                self.map = np.pad(self.map, [(0, 0), (1, 0)], mode='constant', constant_values=UNKNOWN)
                self.origin_col += 1

        # If mark is above the map
        if row < 0:

            # Add rows of zeroes to the top of the map
            for i in range(-1 * row):
                self.map = np.pad(self.map, [(1, 0), (0, 0)], mode='constant', constant_values=UNKNOWN)
                self.origin_row += 1

        # If mark is below the map
        if row >= num_rows:

            # Add rows of zeroes to the bottom of the map
            for i in range(row - num_rows + 1):
                self.map = np.pad(self.map, [(0, 1), (0, 0)], mode='constant', constant_values=UNKNOWN)

    # Convert position (cm) to coordinates (tile widths)
    def position_to_coordinates(self, x_position, y_position):
        x_coordinate = x_position / self.tile_width
        y_coordinate = y_position / self.tile_width

        return x_coordinate, y_coordinate

    def coordinates_to_position(self, x_coordinate, y_coordinate):
        return self.tile_width * x_coordinate, self.tile_width * y_coordinate

    # Convert coordinates (tile widths) to row and column indices
    def coordinates_to_indices(self, x_coordinate, y_coordinate):
        # Get the current row and col indices of GEARS
        row = round(self.origin_row - y_coordinate)
        col = round(self.origin_col + x_coordinate)

        return row, col

    def indices_to_coordinates(self, row, col):
        y_coordinate = self.origin_row - row
        x_coordinate = col - self.origin_col

        return x_coordinate, y_coordinate

    def position_to_indices(self, x_position, y_position):
        x_coordinate, y_coordinate = self.position_to_coordinates(x_position, y_position)
        row, col = self.coordinates_to_indices(x_coordinate, y_coordinate)
        return row, col

    def indices_to_position(self, row, col):
        x_coordinate, y_coordinate = self.indices_to_coordinates(row, col)
        x_position, y_position = self.coordinates_to_position(x_coordinate, y_coordinate)
        return x_position, y_position

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

        # Convert position to coordinates
        self.x_coordinate, self.y_coordinate = self.position_to_coordinates(self.x_position, self.y_position)

        # Convert coordinates to indices
        self.row, self.col = self.coordinates_to_indices(self.x_coordinate, self.y_coordinate)

        # Mark GEARS on map
        self.update_map(self.x_coordinate, self.y_coordinate, GEARS)

    def update_orientation(self):

        # Position of the left wheels in degrees
        left_encoder = self.get_motor_encoder(self.left_wheel)

        # Position of the right wheels in degrees
        right_encoder = self.get_motor_encoder(self.right_wheel)

        # the orientation of GEARS should be a constant multiple of the difference in motor encoder values
        self.orientation = self.turning_constant * (right_encoder - left_encoder)

        # # Keep orientation between 0 and 359 degrees
        # self.orientation %= 360

    # Mark the current position of GEARs on the map
    # Assumes GEARS started in the lower left hand corner of the map
    def update_map(self, x_coordinate, y_coordinate, mark):
        row, col = self.coordinates_to_indices(x_coordinate, y_coordinate)

        # If the marking is off the map, expand the map
        self.expand_map(row, col)

        # Get new indices after expanding map
        row, col = self.coordinates_to_indices(x_coordinate, y_coordinate)

        # Do not overwrite the origin or walls
        safe_list = [ORIGIN, WALL]
        if self.map[row][col] in safe_list:
            return

        # Replace previous gears or target with path
        replace_list = [GEARS, TARGET]
        if mark in replace_list:

            # Replace previous mark with path mark
            self.map[self.map == mark] = PATH

        # If placing a clear mark
        if mark == CLEAR:
            # If the target tile is not marked unknown
            if self.map[row][col] != UNKNOWN:
                # Do not overwrite that mark
                return

        self.map[row][col] = mark

    def display_map(self):
        map_copy = self.map.copy()
        map_copy[map_copy == ORIGIN] = PATH
        map_copy[map_copy != PATH] = CLEAR
        print('---' * len(map_copy[0]))
        for row in map_copy:
            print('|', end='')
            print(*row, sep=', ', end='|\n')
        print('---' * len(map_copy[0]))

    def set_heading(self, degrees):
        self.heading = degrees % 360
        self.turning = True

    def point_turn(self):
        self.heading = float(input('Enter the desired angle: '))
        self.turning = True

    def mark_waypoint(self, x_coordinate, y_coordinate):
        self.map[self.map == WAYPOINT] = UNKNOWN  # Clear previous waypoint

        self.update_map(x_coordinate, y_coordinate, WAYPOINT)

    def goto_waypoint(self):
        r, c = np.where(self.map == WAYPOINT)

        try:
            row = r[0]
            col = c[0]

        # Either there was no waypoint or GEARS reached the waypoint and
        # the waypoint mark was replaced with the GEARS mark
        except IndexError:
            # Stop
            self.stop()
            # Exit
            return

        # Get the coordinates of the waypoint
        x_position, y_position = self.indices_to_position(row, col)

        if abs(self.x_position - x_position) > 1:
            delta_x = x_position - self.x_position
            delta_y = 0
        elif abs(self.y_position - y_position) > 1:
            delta_x = 0
            delta_y = y_position - self.y_position
        else:
            return

        self.heading = np.degrees(np.arctan2(delta_y, delta_x))

        if abs(self.orientation - self.heading) > 1:
            self.turning = True

        # Move toward the waypoint
        # correct_orientation should be called later to ensure GEARS is facing the
        # correct direction
        self.move_forward()

    def get_nearest_unknown(self):
        points = []
        find_unknowns(self.map, self.row, self.col, points)
        if len(points) == 0:
            return None
        nearest_point = np.array(points[0])
        gears = np.array([self.row, self.col])
        min_distance = get_distance(gears, nearest_point)
        for point in points:
            np.array(point)
            distance = get_distance(gears, point)
            if distance < min_distance:
                nearest_point = point
                min_distance = distance
        return nearest_point

    # Check if GEARS has completed the mission
    def check_finished(self):
        pass

    def calibrate_turns(self):

        # Get the turning constant from the user
        self.turning_constant = float(input('Enter the turning constant: '))

        # reset all relevant variables
        self.reset_motor_encoders()
        self.heading = 0
        self.orientation = 0

        # Turn 180 degrees
        self.set_heading(180)
        self.wait_for_turn()

        left_encoder = self.get_motor_encoder(self.left_wheel)
        right_encoder = self.get_motor_encoder(self.right_wheel)
        difference = right_encoder - left_encoder
        print('Encoder difference:', difference, 'degrees')
        print('Turning constant:', self.turning_constant)
        print('Orientation:', self.orientation)
        print('Heading:', self.heading)

    # Set dps for all motors
    # This is the only method that interfaces directly with the motors
    def update_motors(self):

        # If GEARS is off and the motor dps is being set to a nonzero value
        if not self.on and (self.left_dps != 0 or self.right_dps != 0):

            # Do not update the motor dps values
            return

        # Update the motor dps values
        self.set_motor_dps(self.left_wheel, self.left_dps)
        self.set_motor_dps(self.right_wheel, self.right_dps)

    # Turn off and reset all motors
    def exit(self):
        self.on = False
        self.stop()
        self.update_motors()
        self.reset_all()

    def get_new_target(self):
        nearest_unknown = self.get_nearest_unknown()  # Get the nearest unknown point on the map
        # If an unknown point was found
        if nearest_unknown is not None:
            self.target_x, self.target_y = self.indices_to_coordinates(*nearest_unknown)
            self.update_map(self.target_x, self.target_y, TARGET)

    def update_lead(self):
        # if at the target, get a new path
        if self.at_target():
            self.path = []
            self.path_index = 0
            target_row, target_col = self.coordinates_to_indices(self.target_x, self.target_y)
            # Find a path from GEARS to the target
            find_path(self.map, self.row, self.col, target_row, target_col, self.path)
            if len(self.path) >= 2:
                target_index = self.path.index((target_row, target_col))
                self.path = self.path[1:target_index+1]

        elif self.at_lead():
            self.path_index += 1

        try:
            self.lead_x, self.lead_y = self.path[self.path_index]
        except IndexError:
            self.lead_x, self.lead_y = self.target_x, self.target_y

    def follow_lead(self):
        x_position, y_position = self.coordinates_to_position(self.lead_x, self.lead_y)

        if abs(self.x_position - x_position) > 1:
            delta_x = x_position - self.x_position
            delta_y = 0
        elif abs(self.y_position - y_position) > 1:
            delta_x = 0
            delta_y = y_position - self.y_position
        else:
            return

        self.heading = np.degrees(np.arctan2(delta_y, delta_x))

        if abs(self.orientation - self.heading) > 1:
            self.turning = True

        # Move toward the waypoint
        # correct_orientation should be called later to ensure GEARS is facing the
        # correct direction
        self.move_forward()

    def at_target(self):
        return abs(self.x_coordinate - self.target_x) < 0.1 and abs(self.y_coordinate - self.target_y) < 0.1

    def at_lead(self):
        abs(self.x_coordinate - self.lead_x) < 0.1 and abs(self.y_coordinate - self.lead_y) < 0.1

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
            self.update_position()  # Get the new position of GEARS

            # MODE DEPENDENT METHODS
            if self.mode == 'auto':
                self.detect_walls()  # Detect walls with the ultrasonic sensor and mark them on the map
                if self.at_target():
                    self.get_new_target()
                self.update_lead()
                self.follow_lead()

            elif self.mode == 'waypoint':
                self.goto_waypoint()
            elif self.mode == 'manual':
                pass

            self.correct_orientation()  # Make GEARS turn to face the desired heading
        # If GEARS is off
        else:
            self.stop()

        self.update_motors()  # Update dps values for the motors (Directly interfaces with motors)
        sleep(self.buffer_time)  # Wait several milliseconds before repeating
