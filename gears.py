from brickpi3 import BrickPi3
import numpy as np
from time import sleep
import pandas as pd
from helpers import get_dps, read_ultrasonic, get_distance
from path_finding import grid_graph, remove_node, find_nearest_unknown, find_path


# ORIGIN = 3
# PATH = 1
# GEARS = 2
# LEAD = 4
# TARGET = 6
# WALL = -1
# CLEAR = 5
# UNKNOWN = 0

ORIGIN = 'O'
PATH = 'X'
GEARS = 'G'
LEAD = 'L'
TARGET = 'T'
WALL = '!'
CLEAR = ' '
UNKNOWN = 'U'

FRONT = 0
LEFT = 90
BACK = 180
RIGHT = 270


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
        self.wheel_circumference = 2 * np.pi * wheel_radius  # cm
        self.turning_constant = 0.135  # convert motor encoder difference to orientation
        self.turning_gain = 5

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
        self.x_coordinate = 0  # tile widths right of origin
        self.y_coordinate = 0  # tile widths above origin
        self.row = 0  # current row of GEARS on map
        self.col = 0  # current column of GEARS on map
        self.orientation = 0  # actual orientation (degrees)
        self.heading = 0  # desired orientation (degrees)
        self.turning = False  # Is GEARS currently executing a turn?
        self.origin_marked = False  # Is GEARS returning to the origin?
        self.prev_left_encoder = 0  # value of left encoder from previous cycle
        self.prev_right_encoder = 0  # value of right encoder from previous cycle
        self.tile_width = 40  # width of a tile on the map (cm)
        self.path = []
        self.target_x = self.x_coordinate
        self.target_y = self.y_coordinate
        self.lead_x = self.x_coordinate
        self.lead_y = self.y_coordinate
        self.path_index = 1
        self.hazards = pd.DataFrame(columns=['type', 'parameter', 'value', 'x', 'y'])

        # ADDITIONAL ATTRIBUTES
        self.on = False  # Is GEARS on?
        self.buffer_time = buffer_time  # time between cycles (seconds)
        self.mode = mode  # current mode
        self.mode_list = ['auto', 'walls', 'point_turn', 'waypoint', 'manual']  # list of known modes

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
            self.set_heading(self.heading + 90, turn=True)

    # Turn 90 degrees right
    def turn_right(self):
        if not self.turning:
            self.set_heading(self.heading + 90, turn=True)

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
            dps = self.turning_gain * error

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

    # Pause the program until the turn is complete
    # Only use for calibration and troubleshooting
    def wait_for_turn(self):
        while self.turning:
            self.correct_orientation()

    # record hazard in a dataframe
    def record_hazard(self, hazard_type, parameter, value, x, y):

        # Create new row
        new_entry = pd.DataFrame([[hazard_type, parameter, value, x, y]], columns=self.hazards.columns)

        # Insert new row
        self.hazards = pd.concat([self.hazards, new_entry], ignore_index=True)

    # get the coordinates of the tile adjacent to GEARS in a certain direction
    def get_neighbor_coordinates(self, direction):
        x_position = self.x_position + self.tile_width * np.cos(np.radians(self.orientation + direction))
        y_position = self.y_position + self.tile_width * np.sin(np.radians(self.orientation + direction))
        x_coordinate, y_coordinate = self.position_to_coordinates(x_position, y_position)
        return x_coordinate, y_coordinate

    # detect walls with the ultrasonic sensor and record them on the map
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

    # default behavior when not following path
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

    # expand the map to include the given row and column indices
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

    # Convert coordinates (tile widths) to position (cm)
    def coordinates_to_position(self, x_coordinate, y_coordinate):
        return self.tile_width * x_coordinate, self.tile_width * y_coordinate

    # Convert coordinates (tile widths) to row and column indices
    def coordinates_to_indices(self, x_coordinate, y_coordinate):
        row = round(self.origin_row - y_coordinate)
        col = round(self.origin_col + x_coordinate)
        return row, col

    # Convert indices to coordinates (tile widths)
    def indices_to_coordinates(self, row, col):
        y_coordinate = self.origin_row - row
        x_coordinate = col - self.origin_col
        return x_coordinate, y_coordinate

    # Convert position (cm) to indices
    def position_to_indices(self, x_position, y_position):
        x_coordinate, y_coordinate = self.position_to_coordinates(x_position, y_position)
        row, col = self.coordinates_to_indices(x_coordinate, y_coordinate)
        return row, col

    # Convert indices to position (cm)
    def indices_to_position(self, row, col):
        x_coordinate, y_coordinate = self.indices_to_coordinates(row, col)
        x_position, y_position = self.coordinates_to_position(x_coordinate, y_coordinate)
        return x_position, y_position

    def update_position(self):

        # If GEARS is turning, do not update its position
        if self.turning:
            return

        # Get the position of the left and right motors (measured in degrees)
        left_encoder = self.get_motor_encoder(self.left_wheel)
        right_encoder = self.get_motor_encoder(self.right_wheel)

        # Determine how far the left and right motors have turned
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

    # Track the orientation of GEARS based on the difference between the left and right motor encoders
    def update_orientation(self):

        # Position of the left wheel in degrees
        left_encoder = self.get_motor_encoder(self.left_wheel)

        # Position of the right wheel in degrees
        right_encoder = self.get_motor_encoder(self.right_wheel)

        # the orientation of GEARS is a constant multiple of the difference between the motor encoder values
        # find the turning constant during calibration
        self.orientation = self.turning_constant * (right_encoder - left_encoder)

    # Mark the current position of GEARs on the map
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

        # If marking the lead
        if mark == LEAD:
            self.map[self.map == LEAD] = PATH  # Clear previous lead

        self.map[row][col] = mark

    # Display a polished map output
    def display_map(self):
        map_copy = self.map.copy()
        map_copy[map_copy == ORIGIN] = PATH
        map_copy[map_copy != PATH] = CLEAR
        print('---' * len(map_copy[0]))
        for row in map_copy:
            print('|', end='')
            print(*row, sep=', ', end='|\n')
        print('---' * len(map_copy[0]))

    # Set the heading and turn to face it
    def set_heading(self, degrees, turn=False):
        self.heading = degrees % 360
        self.turning = turn

    # Get a new path from GEARS to the nearest unknown point
    def get_new_path(self):

        # construct a graph to represent the map
        num_rows = len(self.map)
        num_cols = len(self.map[0])
        graph = grid_graph(num_rows, num_cols)

        # Get the indices of marks of the map
        walls = np.array(np.where(self.map == WALL)).T
        origin = np.array(np.where(self.map == ORIGIN)).T
        path_points = np.array(np.where(self.map == PATH)).T
        gears = np.array(np.where(self.map == GEARS)).T

        # indices of all known points
        known_points = np.concatenate((origin, path_points, gears))

        # convert indices to coordinates
        known_points = [self.indices_to_coordinates(row, col) for row, col in known_points]

        # x and y coordinates of origin relative to bottom left corner of map
        origin = (len(self.map) - self.origin_row, self.origin_col)

        # remove the walls from the graph
        for wall in walls:
            graph = remove_node(graph, wall, num_cols, origin)

        source = (self.x_coordinate, self.y_coordinate)  # start at GEARS

        # set target to the nearest unknown point
        self.target_x, self.target_y = find_nearest_unknown(graph, source, num_cols, origin, known_points)
        target = (self.target_x, self.target_y)

        # find a path from GEARS to the target
        self.path = find_path(graph, source, target, num_cols, origin)

        self.lead_x, self.lead_y = self.path[1]
        self.path_index = 1

    # Set lead to the correct point on the path
    def update_lead(self):

        # if gears has reached the lead
        if self.near(self.lead_x, self.lead_y):

            # move the lead to the next point on the path
            self.path_index += 1

        # set the lead to a point on the path
        self.lead_x, self.lead_y = self.path[self.path_index]

    # Move to the lead (no diagonals)
    def follow_lead(self):
        x_position, y_position = self.coordinates_to_position(self.lead_x, self.lead_y)

        # if GEARS is more than 1 cm away from the lead in the x direction
        if not np.isclose(self.x_position, x_position, 0, 1):
            delta_x = x_position - self.x_position
            delta_y = 0

        # if GEARS is more than 1 cm away from the lead in the y direction
        elif not np.isclose(self.y_position, y_position, 0, 1):
            delta_x = 0
            delta_y = y_position - self.y_position

        # if GEARS is within 1 cm of the lead in both directions, do nothing
        else:
            return

        # get the heading
        heading = np.degrees(np.arctan2(delta_y, delta_x))

        # turn if the orientation is not within 0.5 degrees of the heading
        turn = not np.close(self.orientation, self.heading, 0, 0.5)

        self.set_heading(heading, turn=turn)

        # move forward toward the lead
        # correct_orientation should be called later to ensure GEARS is facing the
        # correct direction
        self.move_forward()

    # Determine if GEARS is near (x_coordinate, y_coordinate) within a certain tolerance
    def near(self, x_coordinate, y_coordinate, tolerance):
        return np.isclose(self.x_coordinate, x_coordinate, 0, tolerance) and \
            np.isclose(self.y_coordinate, y_coordinate, 0, tolerance)

    # Determine the turning constant
    def calibrate_turns(self):

        # Get the turning constant from the user
        self.turning_constant = float(input('Enter the turning constant: '))

        # reset all relevant variables
        self.reset_motor_encoders()
        self.heading = 0
        self.orientation = 0

        # Turn 180 degrees
        self.set_heading(180, turn=True)
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
            # main demo, Task 5 (no cargo), and Integration Task 5/6 (cargo)
            if self.mode == 'auto':
                self.detect_walls()  # Detect walls with the ultrasonic sensor and mark them on the map

                # If at the target
                if self.near(self.target_x, self.target_y, 0.1):

                    # Get a new path
                    self.get_new_path()

                self.update_lead()  # move the lead to the next coordinate on the path
                self.follow_lead()  # move GEARS to the lead

            # Task 1 and Integration Task 1/2
            elif self.mode == 'walls':

                # Avoid hitting the walls
                self.avoid_walls()

            # Task 2 (no cargo) and Task 6 (cargo)
            elif self.mode == 'point_turn':

                # get a heading from the user
                angle = float(input('Enter the desired angle: '))

                # turn to face that heading
                self.set_heading(angle, turn=True)

            # TODO: PoC Task 3, Avoid Hazards
            # 1. Get the magnetic sensor working
            # 2. Create vector fields for the magnetic field
            # 3. Determine direction of magnet from vector field
            # 4. Determine distance from magnitude of vector field
            # 5. Mark magnet on map
            # 6. Surround magnet with walls out to specified radius
            # 7. Experiment with IR sensor
            # 8. Mark IR beacon on the map
            # 9. Surround IR beacon with walls out to specified radius
            # 10. Integrate with path finding algorithm to navigate around hazards
            # 11. Get the target from the user

            # PoC Task 4
            # TODO: Integrate with path finding system
            # Should be able to get target from user and navigate to that target intelligently
            # Differs from main demo because target may be well beyond the explored area of map
            # Will have to update path while exploring to navigate around walls as they are discovered
            elif self.mode == 'waypoint':

                # get an x, y coordinate pair from the user
                self.lead_x = float(input('Enter the x-coordinate: '))
                self.lead_y = float(input('Enter the y-coordinate: '))

                # mark the coordinate on the map
                self.update_map(self.lead_x, self.lead_y, LEAD)

                # go to the coordinate
                self.follow_lead()

            # for testing
            elif self.mode == 'manual':
                pass

            self.correct_orientation()  # Make GEARS turn to face the desired heading

        # If GEARS is off
        else:
            self.stop()

        self.update_motors()  # Update dps values for the motors (Directly interfaces with motors)
        sleep(self.buffer_time)  # Wait several milliseconds before repeating
