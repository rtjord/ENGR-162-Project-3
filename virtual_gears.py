import pandas as pd
from helpers import get_dps
from path_finding import *
from constants import *
import os


def read_ultrasonic():
    return np.inf


class VirtualGears:
    def __init__(self, mode='auto', max_speed=500, wheel_radius=3, buffer_time=0.01, visualizer=False):

        # MOTORS AND WHEELS
        # Assign wheels to BrickPi ports
        self.left_wheel = None
        self.right_wheel = None

        # reset motor encoders to 0
        self.left_encoder = 0
        self.right_encoder = 0

        # Wheels (assumes all wheels have the same radius)
        self.wheel_circumference = 2 * np.pi * wheel_radius  # cm
        self.turning_constant = 0.135  # convert motor encoder difference to orientation
        self.turning_gain = 100

        # Motor Speeds
        self.max_dps = get_dps(max_speed, wheel_radius)
        self.left_dps = 0
        self.right_dps = 0

        # SENSORS
        # Ultrasonic Sensor
        self.front_ultrasonic_reading = np.inf
        self.left_ultrasonic_reading = np.inf
        self.right_ultrasonic_reading = np.inf

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
        self.target_fails = 0
        self.hazards = pd.DataFrame(columns=['Resource Type',
                                             'Parameter of Interest',
                                             'Parameter',
                                             'Resource X Coordinate',
                                             'Resource Y Coordinate'])
        self.map_number = 0
        self.notes = ''
        self.visualizer = visualizer

        # ADDITIONAL ATTRIBUTES
        self.on = False  # Is GEARS on?
        self.buffer_time = buffer_time  # time between cycles (seconds)
        self.mode = mode  # current mode
        self.mode_list = ['auto', 'walls', 'point_turn', 'target', 'manual']  # list of known modes

    # Reset all motor encoders to 0
    def reset_motor_encoders(self):
        self.left_encoder = 0
        self.right_encoder = 0

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
        front_distance = read_ultrasonic()
        left_distance = read_ultrasonic()
        right_distance = read_ultrasonic()

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

        # Determine how far the left and right motors have turned
        # since the last cycle
        left_change = self.left_encoder - self.prev_left_encoder
        right_change = self.right_encoder - self.prev_right_encoder

        average_change = (left_change + right_change) / 2

        # Convert degrees to cm to get the linear change in distance
        delta = average_change / 360 * self.wheel_circumference

        # Update x and y position of GEARS
        self.x_position += delta * np.cos(np.radians(self.heading))
        self.y_position += delta * np.sin(np.radians(self.heading))

        # Record the encoder values to use for reference on the next cycle
        self.prev_left_encoder = self.left_encoder
        self.prev_right_encoder = self.right_encoder

        # Convert position to coordinates
        self.x_coordinate, self.y_coordinate = self.position_to_coordinates(self.x_position, self.y_position)

        # Convert coordinates to indices
        self.row, self.col = self.coordinates_to_indices(self.x_coordinate, self.y_coordinate)

    # Track the orientation of GEARS based on the difference between the left and right motor encoders
    def update_orientation(self):

        # the orientation of GEARS is a constant multiple of the difference between the motor encoder values
        # find the turning constant during calibration
        self.orientation = self.turning_constant * (self.right_encoder - self.left_encoder)

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

        # Do not overwrite the origin or walls
        safe_list = [ORIGIN, WALL]
        if self.map[row][col] in safe_list:
            return

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
        print('---' * map_copy.shape[1])

        for row in map_copy:
            print('|', end='')
            for char in row:
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
                else:
                    color = ''

                print(color + char + RESET + ', ', end='')
            print('|')
        print('---' * map_copy.shape[1])

    def write_map(self):
        print('Writing map to file')
        output_map = np.zeros(self.map.shape, dtype='int32')
        output_map[self.map == PATH] = 1
        output_map[self.map == ORIGIN] = 5
        output_map[self.map == HEAT] = 2
        output_map[self.map == MAGNET] = 3
        output_map[self.map == GEARS] = 4  # exit

        # trim border
        row_indices, col_indices = np.where(output_map != 0)
        output_map = output_map[min(row_indices):max(row_indices) + 1, min(col_indices):max(col_indices) + 1]

        self.get_map_number()
        self.get_notes()
        output_file = f"maps/outputs/map{self.map_number}.csv"

        with open(output_file, "w") as f:
            f.write("Team: 04\n")
            f.write(f"Map: {self.map_number}\n")
            f.write(f"Unit Length: {self.tile_width}\n")
            f.write("Unit: cm\n")

            num_rows, num_cols = output_map.shape
            rows, cols = np.where(output_map == 5)
            row = rows[0]
            col = cols[0]
            x = col
            y = num_rows - row - 1
            f.write(f"Origin: ({x}, {y})\n")
            f.write(f'Notes: {self.notes}\n')

            for row in range(num_rows):
                for col in range(num_cols):
                    f.write(str(output_map[row][col]))
                    if col < num_cols - 1:
                        f.write(",")
                f.write("\n")

    def get_map_number(self):
        self.map_number = input('Map number: ')

    def get_notes(self):
        self.notes = str(input('Notes: '))

    def write_hazards(self):
        print('Writing hazards to file')
        self.get_map_number()
        self.get_notes()

        filename = 'maps/outputs/team04_hazards.csv'

        hazards = self.hazards.to_csv()
        hazards = hazards[1:]
        hazards = hazards.replace(',', ', ')
        hazards = hazards.replace('\r', '')

        with open(filename, 'w') as f:
            f.write('Team: 04\n')
            f.write(f'Map: {self.map_number}\n')
            f.write(f'Notes: {self.notes}\n\n')
            f.write(hazards)

    # Set the heading and turn to face it
    def set_heading(self, degrees, turn=False):
        self.heading = degrees % 360
        self.turning = turn

    def construct_graph(self):

        # update position to ensure GEARS row and column values are correct
        self.update_position()

        # construct a graph to represent the map
        num_rows, num_cols = self.map.shape
        graph = grid_graph(num_rows, num_cols)

        # Get the indices of walls of the map
        walls = np.array(np.where(self.map == WALL)).T

        # convert indices to nodes
        wall_nodes = [indices_to_node(row, col, num_rows) for row, col in walls]

        # remove the walls from the graph
        for node in wall_nodes:
            graph = remove_node(graph, node, num_cols)

        return graph

    def set_target(self):
        self.target_x = float(input('Enter the x-coordinate: '))
        self.target_y = float(input('Enter the y-coordinate: '))
        self.update_map(self.target_x, self.target_y, TARGET)

    def get_nearest_unknown(self):

        # construct the graph
        graph = self.construct_graph()
        num_rows, num_cols = self.map.shape

        # convert gears indices to node
        source_node = indices_to_node(self.row, self.col, num_rows)

        # Clear marks are considered known
        known_indices = np.array(np.where(self.map != UNKNOWN)).T
        known_nodes = [indices_to_node(row, col, num_rows) for row, col in known_indices]

        target_node = find_nearest_unknown(graph, source_node, num_cols, known_nodes)

        if target_node is None:
            return None

        # convert target node to coordinates
        target_row, target_col = node_to_indices(target_node[0], target_node[1], num_rows)
        target_x, target_y = self.indices_to_coordinates(target_row, target_col)

        return target_x, target_y

    def target_failure(self):
        self.target_fails += 1

        # if this is the first failure
        if self.target_fails == 1:
            # Expand the map in all directions, marking each new tile as unknown
            self.map = np.pad(self.map, [(1, 1), (1, 1)], mode='constant', constant_values=UNKNOWN)

            # update origin indices after expanding map
            self.origin_row += 1
            self.origin_col += 1

            # try again on the next cycle

        # if unable to find a new target after expanding the map
        if self.target_fails == 2:
            print('GEARS may be trapped')

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

        # convert nodes to indices
        indices_path = [node_to_indices(x, y, num_rows) for x, y in node_path]

        # convert indices to coordinates
        path = [self.indices_to_coordinates(row, col) for row, col in indices_path]

        # reset path index
        self.path_index = 1

        return path

    # Set lead to the correct point on the path
    def update_lead(self):

        # if GEARS has not reached the end of the path
        if self.path_index < len(self.path):

            # set the lead to the current point on the path
            self.lead_x, self.lead_y = self.path[self.path_index]

        # if gears has reached the lead
        if self.near(self.lead_x, self.lead_y, 0.1):
            if self.visualizer:
                os.system("cls")
                self.display_map()
            # move the lead to the next point on the path
            self.path_index += 1

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
            self.stop()
            return

        # get the heading
        heading = np.degrees(np.arctan2(delta_y, delta_x))

        # turn if the orientation is not within 0.5 degrees of the heading
        turn = not np.isclose(self.orientation, self.heading, 0, 0.5)

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

        difference = self.right_encoder - self.left_encoder
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

        # Update the motor encoder values
        self.left_encoder += self.left_dps * self.buffer_time
        self.right_encoder += self.right_dps * self.buffer_time

    # Turn off and reset all motors
    def exit(self):
        self.on = False
        self.stop()
        self.update_motors()

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
            # main demo, Task 5 (no cargo), and Integration Task 5/6 (cargo)
            if self.mode == 'auto':
                self.detect_walls()  # Detect walls with the ultrasonic sensor and mark them on the map

                # If at the target
                if self.near(self.target_x, self.target_y, 0.1):

                    # Get a new target
                    target = self.get_nearest_unknown()

                    # if unable to find a new target
                    if target is None:

                        # handle the failure
                        self.target_failure()

                        # try again
                        return

                    # reset number of target fails
                    self.target_fails = 0

                    self.target_x, self.target_y = target

                    # Get a path to the new target
                    self.path = self.get_path(self.target_x, self.target_y)

                    # reset the path index
                    self.path_index = 1

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
            # Should be able to get target from user and navigate to that target intelligently
            # Differs from main demo because target may be well beyond the explored area of map
            # Will have to update path while exploring to navigate around walls as they are discovered
            # Call method "set_target" to tell GEARS where to go
            elif self.mode == 'target':

                self.detect_walls()  # Detect walls with the ultrasonic sensor and mark them on the map

                # get a path to the target
                self.path = self.get_path(self.target_x, self.target_y)

                self.update_lead()  # move the lead to the next coordinate on the path
                self.follow_lead()  # move GEARS to the lead

            # for testing
            elif self.mode == 'manual':
                pass

            self.correct_orientation()  # Make GEARS turn to face the desired heading

        # If GEARS is off
        else:
            self.stop()

        self.update_motors()  # Update dps values for the motors (Directly interfaces with motors)
