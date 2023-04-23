from brickpi3 import BrickPi3
import grovepi
from MPU9250 import MPU9250
from time import sleep, time
import pandas as pd
import math
from helpers import get_dps, round_half_up, get_magnitude, round_to_90, power_regression, quadratic_regression
from path_finding import *
from sensors import read_ultrasonic, read_infrared, read_imu
from constants import *


class Gears(BrickPi3):
    def __init__(self, mode='auto', max_speed=10, wheel_radius=4, buffer_time=0.01):

        # Initialize parent class
        BrickPi3.__init__(self)

        # MOTORS
        # Assign wheels to BrickPi ports
        self.left_wheel = self.PORT_D
        self.right_wheel = self.PORT_A

        # Assign gate to BrickPi port
        self.gate = self.PORT_B
        self.gate_open = False

        # Reset all motor encoders to 0
        self.reset_motor_encoders()

        # Wheels (assumes all wheels have the same radius)
        self.wheel_circumference = 2 * np.pi * wheel_radius  # cm
        self.turning_constant = 0.220  # convert motor encoder difference to orientation
        self.turning_gain = 5

        # Motor Speeds
        self.max_dps = get_dps(max_speed, wheel_radius)
        self.left_dps = 0
        self.right_dps = 0
        self.gate_pos = 0
        self.set_motor_limits(self.gate, dps=20)  # Limit max speed of gate

        # SENSORS
        # Ultrasonic Sensor
        self.front_ultrasonic = 2
        self.left_ultrasonic = 3
        self.right_ultrasonic = 7

        # IR Sensor (A0)
        self.right_infrared = 14
        self.left_infrared = 15
        grovepi.pinMode(self.right_infrared, "INPUT")
        grovepi.pinMode(self.left_infrared, "INPUT")

        # Magnetic Sensor
        self.imu = MPU9250()  # IMU (magnetic sensor)
        self.magnet_count = 0

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
        self.orientation_correction = 0
        self.turning = False  # Is GEARS currently executing a turn?
        self.prev_left_encoder = 0  # value of left encoder from previous cycle
        self.prev_right_encoder = 0  # value of right encoder from previous cycle
        self.tile_width = 40  # width of a tile on the map (cm)
        self.path = []
        self.target_x = self.x_coordinate
        self.target_y = self.y_coordinate
        self.lead_x = self.x_coordinate
        self.lead_y = self.y_coordinate
        self.path_index = 1
        self.target_fails = 0
        self.walls = []
        self.wall_distances = []
        self.hazards = pd.DataFrame(columns=['Resource Type',
                                             'Parameter of Interest',
                                             'Parameter',
                                             'Resource X Coordinate',
                                             'Resource Y Coordinate'])
        self.last_alignment = time()
        # ADDITIONAL ATTRIBUTES
        self.on = False  # Is GEARS on?
        self.buffer_time = buffer_time  # time between cycles (seconds)
        self.mode = mode  # current mode
        self.mode_list = ['auto', 'walls', 'point_turn', 'target', 'manual', 'dance']  # list of known modes

    # Reset a single motor encoder to 0
    def reset_encoder(self, port):
        self.offset_motor_encoder(port, self.get_motor_encoder(port))

    # Reset all motor encoders to 0
    def reset_motor_encoders(self):
        self.reset_encoder(self.left_wheel)
        self.reset_encoder(self.right_wheel)
        self.reset_encoder(self.gate)

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
            self.set_heading(self.heading - 90, turn=True)

    # Do not move
    def stop(self):
        self.left_dps = 0
        self.right_dps = 0

    # open the gate
    def open_gate(self):
        self.gate_pos = -70
        self.gate_open = True

    # close the gate
    def close_gate(self):
        self.gate_pos = 0
        self.gate_open = False

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

    # Keep GEARS from exiting through the entrance
    def setup(self):

        # Place a phantom wall at the entrance to keep GEARS from exiting through the entrance.
        # This solution is more flexible than simply marking the origin with a wall because it
        # allows GEARS to return to the origin.
        direction = ''

        while type(direction) != float:
            try:
                self.display_map()
                direction = float(input('What is the direction of the entrance relative to the origin? (degrees): '))
            except ValueError:
                print(direction)
                print('Error: Please enter a number')

        x, y = self.get_neighbor_coordinates(direction)
        self.update_map(x, y, BARRIER)
        self.update_position()
        self.walls.append(((self.x_coordinate, self.y_coordinate), (x, y)))

    # record hazard in a dataframe
    def record_hazard(self, hazard_type, parameter, value, x, y):

        # Create new row
        new_entry = pd.DataFrame([[hazard_type, parameter, value, x, y]], columns=self.hazards.columns)

        # Insert new row
        self.hazards = pd.concat([self.hazards, new_entry], ignore_index=True)

    # get the coordinates of the point adjacent to GEARS in a certain direction
    def get_neighbor_coordinates(self, direction):
        row = round(self.row - np.sin(np.radians(self.orientation + direction)))
        col = round(self.col + np.cos(np.radians(self.orientation + direction)))
        x_coordinate, y_coordinate = self.indices_to_coordinates(row, col)
        return x_coordinate, y_coordinate

    # detect walls with the ultrasonic sensor and record them on the map
    def detect_walls(self):

        near_half = np.isclose(self.x_coordinate % 1, 0.5, 0, 0.4) or np.isclose(self.y_coordinate % 1, 0.5, 0, 0.4)

        # do not detect walls while turning or near the halfway mark between tiles
        if self.turning or near_half:
            return

        # Get distance to wall in each direction
        front_distance = read_ultrasonic(self.front_ultrasonic)
        left_distance = read_ultrasonic(self.left_ultrasonic)
        right_distance = read_ultrasonic(self.right_ultrasonic)

        # Get the coordinates of the points that sensors are pointing at
        front_x, front_y = self.get_neighbor_coordinates(FRONT)
        left_x, left_y = self.get_neighbor_coordinates(LEFT)
        right_x, right_y = self.get_neighbor_coordinates(RIGHT)

        # If the sensors detect a wall, mark it
        # Otherwise, mark the point as clear
        x_coord = round_half_up(self.x_coordinate)
        y_coord = round_half_up(self.y_coordinate)

        if front_distance < (self.tile_width / 2):
            wall = ((x_coord, y_coord), (front_x, front_y))
            if wall not in self.walls:
                self.update_map(front_x, front_y, WALL)
                self.walls.append(wall)
        else:
            self.update_map(front_x, front_y, CLEAR)

        if left_distance < (self.tile_width / 2):
            wall = ((x_coord, y_coord), (left_x, left_y))
            if wall not in self.walls:
                self.update_map(left_x, left_y, WALL)
                self.walls.append(wall)
        else:
            self.update_map(left_x, left_y, CLEAR)

        if right_distance < (self.tile_width / 2):
            wall = ((x_coord, y_coord), (right_x, right_y))
            if wall not in self.walls:
                self.update_map(right_x, right_y, WALL)
                self.walls.append(wall)
        else:
            self.update_map(right_x, right_y, CLEAR)

    def detect_infrared(self):
        distance, magnitude = read_infrared(self.right_infrared, self.left_infrared)  # read right IR sensor
        if distance == np.inf:
            return
        # if the average reading is greater than the threshold
        if distance <= self.tile_width:

            # get direction IR sensor is pointing on map
            nearest90 = round_to_90(self.orientation)

            # if the source direction is within 10 degrees of a cardinal direction,
            if np.isclose(self.orientation, nearest90, 0, 10):
                flag = nearest90 % 360 / 90
                if flag == 0:
                    direction = FRONT
                elif flag == 1:
                    direction = LEFT
                elif flag == 2:
                    direction = BACK
                else:
                    direction = RIGHT

                x_coordinate, y_coordinate = self.get_neighbor_coordinates(direction)
                self.update_map(x_coordinate, y_coordinate, HEAT)
                self.record_hazard('Caesium-137', 'Radiated Power (W)', magnitude, x_coordinate, y_coordinate)
            # else, wait for the sensor to rotate closer to a cardinal direction

    def detect_magnets(self):

        # get the distance and direction to the nearest magnetic source
        magnitude, distance, direction_vector = read_imu(self.imu)
        x, y, z = direction_vector

        if distance == np.inf:
            return

        if distance <= self.tile_width:
            self.magnet_count += 1
        else:
            self.magnet_count = 0

        # if the magnetic source is in an adjacent tile
        if self.magnet_count >= 5:
            # determine the direction of the magnet relative to GEARS
            source_direction = self.orientation + math.atan2(y, x)
            nearest90 = round_to_90(source_direction)

            # wait until the angle to the magnet is near a multiple of 90 before marking it on the map
            if np.isclose(source_direction, nearest90, 0, 20):
                flag = nearest90 % 360 / 90
                if flag == 0:
                    direction = FRONT
                elif flag == 1:
                    direction = LEFT
                elif flag == 2:
                    direction = BACK
                else:
                    direction = RIGHT

                x_coordinate, y_coordinate = self.get_neighbor_coordinates(direction)
                self.update_map(x_coordinate, y_coordinate, MAGNET)
                self.record_hazard('MRI', 'Field Strength (uT)', magnitude, x_coordinate, y_coordinate)
                self.magnet_count = 0

    def get_alignment(self):

        # read the distance to the left wall
        left_distance = read_ultrasonic(self.left_ultrasonic)

        # if stops detecting a wall to the left
        if self.turning or left_distance > self.tile_width:

            # clear the log of wall distances
            self.wall_distances = []

            # wait until the next cycle
            return -1

        # log the distance to the left wall and the x, y coordinates of GEARS
        self.wall_distances.append({'distance': left_distance,
                                    'x': self.x_coordinate,
                                    'y': self.y_coordinate})

        # if there are not at least 20 measurements, wait until the next cycle
        if len(self.wall_distances) <= 20:
            return -1

        initial = self.wall_distances[0]
        current = self.wall_distances[-1]

        forward_travel = max(abs(current['x'] - initial['x']), abs(current['y'] - initial['y']))
        forward_travel *= self.tile_width
        sideways_travel = initial['distance'] - current['distance']

        if forward_travel > 0:
            angle = math.atan(sideways_travel / forward_travel)
            return np.degrees(angle)

        return -1

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

        self.update_position()

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
        row = round_half_up(self.origin_row - y_coordinate)
        col = round_half_up(self.origin_col + x_coordinate)
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

    # Track the orientation of GEARS based on the difference between the left and right motor encoders
    def update_orientation(self):

        # Position of the left wheel in degrees
        left_encoder = self.get_motor_encoder(self.left_wheel)

        # Position of the right wheel in degrees
        right_encoder = self.get_motor_encoder(self.right_wheel)

        # the orientation of GEARS is a constant multiple of the difference between the motor encoder values
        # find the turning constant during calibration

        # get the angle between GEARS and the left wall
        angle = self.get_alignment()

        if (time() - self.last_alignment) > 3 and angle != -1:
            self.orientation_correction = angle
            self.last_alignment = time()

        self.orientation = self.turning_constant * (right_encoder - left_encoder) + self.orientation_correction

    # Mark the current position of GEARs on the map
    # Mark the current position of GEARs on the map
    def update_map(self, x_coordinate, y_coordinate, mark):
        row, col = self.coordinates_to_indices(x_coordinate, y_coordinate)

        # If the marking is off the map, expand the map
        self.expand_map(row, col)

        # Get new indices after expanding map
        row, col = self.coordinates_to_indices(x_coordinate, y_coordinate)

        # Replace previous gears or target with path
        replace_list = [GEARS, TARGET]
        if mark in replace_list:
            # Replace previous mark with path mark
            self.map[self.map == mark] = PATH

        # Do not overwrite the origin, path, heat sources, magnetic sources, or barriers
        safe_list = [ORIGIN, PATH, HEAT, MAGNET, BARRIER]
        if self.map[row][col] in safe_list:
            return

        # If placing a clear mark
        if mark == CLEAR:
            # Only walls and unknown points can be marked as clear
            if not (self.map[row][col] == WALL or self.map[row][col] == UNKNOWN):
                # Do not overwrite that mark
                return

        # place the mark on the map
        self.map[row][col] = mark

    # Display a polished map output
    def display_map(self, show_coordinates=False):
        map_copy = self.map.copy()

        if show_coordinates:
            for col in range(map_copy.shape[1]):
                x, y = self.indices_to_coordinates(0, col)
                print(f'{x:3.0f}', end='')
            print()

        print('---' * map_copy.shape[1])

        # print the map in color
        for i, row in enumerate(map_copy):
            print('|', end='')
            for j, char in enumerate(row):
                coordinates = self.indices_to_coordinates(i, j)
                if char == ORIGIN:
                    color = CYAN
                elif char == GEARS:
                    color = BLUE
                elif char == PATH:
                    color = GREEN
                elif char == WALL:
                    color = RED
                elif char == TARGET:
                    color = PURPLE
                elif char == HEAT:
                    color = ORANGE
                elif char == MAGNET:
                    color = PINK
                elif coordinates in self.path:
                    color = YELLOW
                else:
                    color = ''

                print(color + char + RESET + ', ', end='')

            if not show_coordinates:
                print(f'|')
            if show_coordinates:
                x, y = self.indices_to_coordinates(i, 0)
                print(f'|{y:2.0f}')
        print('---' * map_copy.shape[1])

    # Write the map to a file
    def write_map(self):
        print('\nWriting map to file')
        output_map = np.zeros(self.map.shape, dtype='int32')
        output_map[self.map == PATH] = 1
        output_map[self.map == ORIGIN] = 5
        output_map[self.map == HEAT] = 2
        output_map[self.map == MAGNET] = 3
        output_map[self.map == GEARS] = 4  # exit

        # trim border
        row_indices, col_indices = np.where(output_map != 0)
        output_map = output_map[min(row_indices):max(row_indices) + 1, min(col_indices):max(col_indices) + 1]

        map_number = str(input('Map number: '))
        output_file = f"maps/outputs/map{map_number}.csv"
        notes = str(input('Notes: '))

        num_rows, num_cols = output_map.shape
        rows, cols = np.where(output_map == 5)
        row = rows[0]
        col = cols[0]
        x = col
        y = num_rows - row - 1

        with open(output_file, "w") as f:
            f.write("Team: 04\n")
            f.write(f"Map: {map_number}\n")
            f.write(f"Unit Length: {self.tile_width}\n")
            f.write("Unit: cm\n")
            f.write(f"Origin: ({x}, {y})\n")
            f.write(f'Notes: {notes}\n')

            for row in range(num_rows):
                for col in range(num_cols):
                    f.write(str(output_map[row][col]))
                    if col < num_cols - 1:
                        f.write(",")
                f.write("\n")

    def write_hazards(self):
        print('\nWriting hazards to file')

        filename = 'maps/outputs/team04_hazards.csv'
        map_number = str(input('Map number: '))
        notes = str(input('Notes: '))

        hazards = self.hazards.to_csv()
        hazards = hazards[1:]
        hazards = hazards.replace(',', ', ')
        hazards = hazards.replace('\r', '')

        with open(filename, 'w') as f:
            f.write('Team: 04\n')
            f.write(f'Map: {map_number}\n')
            f.write(f'Notes: {notes}\n\n')
            f.write(hazards)

    # Set the heading and turn to face it
    def set_heading(self, degrees, turn=False):
        self.heading = degrees
        self.turning = turn

    # construct a graph to represent the map
    def construct_graph(self):

        # update position to ensure GEARS row and column values are correct
        self.update_position()

        # construct a graph to represent the map
        num_rows, num_cols = self.map.shape
        graph = grid_graph(num_rows, num_cols)

        # Get the indices of heat sources of the map
        heat_sources = np.array(np.where(self.map == HEAT)).T
        magnets = np.array(np.where(self.map == MAGNET)).T

        wall_edges = []
        for (x1, y1), (x2, y2) in self.walls:

            # convert coordinates to map indices
            row1, col1 = self.coordinates_to_indices(x1, y1)
            row2, col2 = self.coordinates_to_indices(x2, y2)

            # map indices to nodes
            node1 = indices_to_node(row1, col1, num_rows)
            node2 = indices_to_node(row2, col2, num_rows)

            # get the index for each node in the graph
            i = node_to_index(node1[0], node1[1], num_cols)
            j = node_to_index(node2[0], node2[1], num_cols)
            wall_edges.append((i, j))

        # convert indices to nodes
        heat_nodes = [indices_to_node(row, col, num_rows) for row, col in heat_sources]
        magnet_nodes = [indices_to_node(row, col, num_rows) for row, col in magnets]

        # remove the walls from the graph
        graph = remove_edges(graph, wall_edges)

        # remove the heat sources from the graph
        graph = remove_nodes(graph, heat_nodes, num_cols)

        # remove the magnets sources from the graph
        graph = remove_nodes(graph, magnet_nodes, num_cols)

        return graph

    # get the target from the user when in target mode
    def set_target(self):
        if self.mode != 'target':
            print(f'Warning: mode is not set to target')
            decision = str(input('Proceed? (y/n): '))
            if decision == 'n':
                return

        # get the coordinates from the user
        self.target_x = float(input('Enter the x-coordinate: '))
        self.target_y = float(input('Enter the y-coordinate: '))

        # mark the target on the map
        self.update_map(self.target_x, self.target_y, TARGET)

    # get the nearest unknown point on the map
    def get_nearest_unknown(self):

        # construct the graph
        graph = self.construct_graph()
        num_rows, num_cols = self.map.shape

        # convert gears indices to node
        source_node = indices_to_node(self.row, self.col, num_rows)

        # Get indices of all known points
        # Clear marks and walls are considered unknown
        map_copy = self.map.copy()
        map_copy[self.map == CLEAR] = UNKNOWN
        map_copy[self.map == WALL] = UNKNOWN
        known_indices = np.array(np.where(self.map != UNKNOWN)).T
        known_nodes = [indices_to_node(row, col, num_rows) for row, col in known_indices]

        target_node = find_nearest_unknown(graph, source_node, num_cols, known_nodes)

        if target_node is None:
            return None

        # convert target node to coordinates
        target_row, target_col = node_to_indices(target_node[0], target_node[1], num_rows)
        target_x, target_y = self.indices_to_coordinates(target_row, target_col)

        return target_x, target_y

    # handle cases where a path to an unknown point could not be found
    def target_failure(self):
        self.target_fails += 1

        # if this is the first failure
        if self.target_fails == 1:
            # Expand the map in all directions, marking each new point as unknown
            self.map = np.pad(self.map, [(1, 1), (1, 1)], mode='constant', constant_values=UNKNOWN)

            # update origin indices after expanding map
            self.origin_row += 1
            self.origin_col += 1

            # try again on the next cycle

        # if unable to find a new target after expanding the map
        if self.target_fails == 2:
            print('GEARS may be trapped')
            self.stop()

    # Get a new path from GEARS to the nearest unknown point
    def get_path(self, target_x, target_y):

        # update target
        self.target_x = target_x
        self.target_y = target_y

        # mark target on the map
        self.update_map(target_x, target_y, TARGET)

        # construct the graph
        graph = self.construct_graph()
        num_rows, num_cols = self.map.shape

        # convert gears indices to node
        source_node = indices_to_node(self.row, self.col, num_rows)

        # Convert target coordinates to node
        row, col = self.coordinates_to_indices(target_x, target_y)
        target_node = indices_to_node(row, col, num_rows)

        # find a path from GEARS to the target
        node_path = find_path(graph, source_node, target_node, num_cols)

        # if unable to find a path
        if node_path is None:
            self.target_failure()  # record the failure
            self.path = []  # make path empty
            return  # try again

        # if successful, reset fails
        self.target_fails = 0

        # convert nodes to indices
        indices_path = [node_to_indices(x, y, num_rows) for x, y in node_path]

        # convert indices to coordinates
        self.path = [self.indices_to_coordinates(row, col) for row, col in indices_path]

        # reset path index
        self.path_index = 1

    # Set lead to the correct point on the path
    def update_lead(self):

        # if GEARS has not reached the end of the path
        if self.path_index < len(self.path):

            # set the lead to the current point on the path
            self.lead_x, self.lead_y = self.path[self.path_index]

        # if gears has reached the lead
        if self.near(self.lead_x, self.lead_y, 0.1):

            # move the lead to the next point on the path
            self.path_index += 1

    # Move to the lead (no diagonals)
    def follow_lead(self):

        # if GEARS is more than 0.1 tile widths away from the lead in the x direction
        if not np.isclose(self.x_coordinate, self.lead_x, 0, 0.1):
            delta_x = self.lead_x - self.x_coordinate
            delta_y = 0

        # if GEARS is more than 0.1 tile widths away from the lead in the y direction
        elif not np.isclose(self.y_coordinate, self.lead_y, 0, 0.1):
            delta_x = 0
            delta_y = self.lead_y - self.y_coordinate

        # if GEARS is within 1 cm of the lead in both directions, do nothing
        else:
            self.stop()
            return

        # get the heading
        heading = np.degrees(np.arctan2(delta_y, delta_x))

        # turn if the orientation is not within 1 degree of the heading
        turn = not np.isclose(self.orientation, self.heading, 0, 1)

        self.set_heading(heading, turn=turn)

        # move forward toward the lead
        # correct_orientation should be called later to ensure GEARS is facing the
        # correct direction
        self.move_forward()

    def check_path_blocked(self):
        for x, y in self.path:
            row, col = self.coordinates_to_indices(x, y)
            mark = self.map[row][col]
            if mark == WALL or mark == HEAT or mark == MAGNET:
                return True
        return False

    # Determine if GEARS is near (x_coordinate, y_coordinate) within a certain tolerance
    def near(self, x_coordinate, y_coordinate, tolerance):
        return np.isclose(self.x_coordinate, x_coordinate, 0, tolerance) and \
            np.isclose(self.y_coordinate, y_coordinate, 0, tolerance)

    def check_finished(self):
        self.update_position()
        # create a copy of the map
        map_copy = self.map.copy()

        # get the number of rows and columns in the map
        num_rows, num_cols = map_copy.shape

        # boundaries of a 3x3 array around GEARS
        min_row = max(self.row - 1, 0)
        max_row = min(self.row + 2, num_rows-1)
        min_col = max(self.col - 1, 0)
        max_col = min(self.col + 2, num_cols - 1)

        # get the 3x3 array around gears
        local_region = map_copy[min_row:max_row, min_col:max_col]

        num_walls = len(np.where(local_region == WALL)[0])
        num_magnets = len(np.where(local_region == MAGNET)[0])
        num_heat = len(np.where(local_region == HEAT)[0])

        num_hazards = num_walls + num_magnets + num_heat

        if num_hazards == 0:
            return True

        return False

    # Determine the turning constant
    def calibrate_turns(self):

        if not self.on:
            print('Warning: GEARS is not on. Calibrating while off may cause infinite loop.')
            decision = str(input('Proceed? (y/n): '))
            if decision == 'n':
                return

        # Get the turning constant from the user
        valid_input = False

        while not valid_input:
            try:
                self.turning_constant = float(input('Enter the turning constant: '))
                valid_input = True
            except ValueError:
                print('Error. Please enter a number.')

        # reset all relevant variables
        self.reset_motor_encoders()
        self.heading = 0
        self.orientation = 0

        # Turn 180 degrees
        self.set_heading(180, turn=True)
        self.wait_for_turn()
        self.stop()

        difference = self.get_motor_encoder(self.left_wheel) - self.get_motor_encoder(self.right_wheel)
        print('Encoder difference:', difference, 'degrees')
        print('Turning constant:', self.turning_constant)
        print('Orientation:', self.orientation)
        print('Heading:', self.heading)

    def calibrate_ultrasonic(self):
        # Get distance to wall in each direction
        while True:
            front_distance = read_ultrasonic(self.front_ultrasonic)
            left_distance = read_ultrasonic(self.left_ultrasonic)
            right_distance = read_ultrasonic(self.right_ultrasonic)
            print(f'\rfront: {front_distance:.2f} cm, left: {left_distance:.2f} cm, right: {right_distance:.2f} cm', end='')
            sleep(0.05)

    def calibrate_infrared(self):
        while True:
            distance = read_infrared(self.right_infrared, self.left_infrared)
            print(f'\rdistance: {distance:8.2f}', end='')
            sleep(0.05)

    def calibrate_imu(self):
        while True:
            magnitude, distance, direction_vector = read_imu(self.imu)
            x, y, z = direction_vector
            if distance != np.inf:
                print(f'\rmagnitude: {magnitude:8.2f}, distance: {distance:8.2f}', end='')

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

        # update gate position
        self.set_motor_position(self.gate, self.gate_pos)

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
            self.update_map(self.x_coordinate, self.y_coordinate, GEARS)  # Mark GEARS on map

            # MODE DEPENDENT METHODS
            # main demo, Task 1, Task 5, Integration Task 1/2, Integration Task 5/6
            if self.mode == 'auto':

                # mark walls and hazards on the map
                self.detect_walls()  # detect walls with the ultrasonic sensor
                self.detect_infrared()  # detect heat sources with the IR sensor
                self.detect_magnets()  # detect magnets with the IMU

                # check if the path is blocked by an obstacle or hazard
                path_blocked = self.check_path_blocked()

                # check if GEARS has reached the end of the path
                end_of_path = self.path_index >= len(self.path)

                # check if the target has changed
                target_changed = len(self.path) > 0 and self.path[-1] != (self.target_x, self.target_y)

                if path_blocked or end_of_path or target_changed:
                    self.stop()  # stop while a new path is calculated
                    self.update_motors()
                    # Get a new target
                    target = self.get_nearest_unknown()

                    # if unable to find a new target
                    if target is None:
                        self.target_failure()  # handle the failure
                        return  # try again

                    # reset number of target fails
                    self.target_fails = 0

                    # update target
                    self.target_x, self.target_y = target

                    # Get a path to the new target
                    self.get_path(self.target_x, self.target_y)

                    # reset the path index
                    self.path_index = 1

                self.update_lead()  # move the lead to the next coordinate on the path
                self.follow_lead()  # move GEARS to the lead

                # if self.check_finished():
                #     self.stop()
                #     self.open_gate()

                # if self.get_motor_encoder(self.gate) == -70:
                #     self.stop()

            # Task 1 and Integration Task 1/2
            elif self.mode == 'walls':

                # detect walls
                self.detect_walls()

                # Avoid hitting the walls
                self.avoid_walls()

            # Task 2, Task 6
            elif self.mode == 'point_turn':

                # get a heading from the user
                if not self.turning:
                    angle = float(input('Enter the desired angle: '))
                    # turn to face that heading
                    self.set_heading(angle, turn=True)

            # Task 3, Task 4, Integration Task 3/4
            # Get target from user and navigate to that target while avoiding hazards
            # Differs from main demo because the target is chosen by the user and
            # may be well beyond the explored area of map
            # Will have to update path while exploring to navigate around hazards as they are discovered
            # Call method "set_target" to tell GEARS where to go
            elif self.mode == 'target':

                # mark walls and hazards on the map
                self.detect_walls()  # detect walls with the ultrasonic sensor

                # check if the path is blocked by an obstacle or hazard
                path_blocked = self.check_path_blocked()

                # check if GEARS has reached the end of the path
                end_of_path = self.path_index >= len(self.path)

                # check if the target has changed
                target_changed = len(self.path) > 0 and self.path[-1] != (self.target_x, self.target_y)

                # check if GEARS is at the target
                at_target = self.near(self.target_x, self.target_y, 0.1)

                if path_blocked or end_of_path or target_changed:
                    if at_target:
                        return

                    print('Recalculating', end='\r')

                    # get a new path to the target
                    self.get_path(self.target_x, self.target_y)

                self.update_lead()  # move the lead to the next coordinate on the path
                self.follow_lead()  # move GEARS to the lead

            # allow the user to manually control GEARS
            elif self.mode == 'manual':
                pass

            elif self.mode == 'dance':
                self.move_forward()
                self.update_motors()
                sleep(0.5)
                self.reverse()
                self.update_motors()
                sleep(0.5)
                self.open_gate()

            if not self.mode == 'dance':
                self.correct_orientation()  # Make GEARS turn to face the desired heading

        # If GEARS is off
        else:
            self.stop()

        self.update_motors()  # Update dps values for the motors (Directly interfaces with motors)
        sleep(self.buffer_time)  # Wait several milliseconds before repeating
