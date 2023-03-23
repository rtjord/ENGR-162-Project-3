#!/usr/bin/env python3

from virtual_gears import VirtualGears
from terminal import Terminal
import numpy as np
from constants import *
import os
import platform
from time import sleep


def load_map(filename):
    with open(filename, "r") as f:
        lines = f.readlines()
        map_list = []
        for line in lines:
            char_list = line.split()
            map_list.append(char_list)

        map_array = np.asarray(map_list)
        return map_array


class VirtualUltrasonic:
    def __init__(self, filename, tile_width):
        self.sim_map = load_map(filename)
        self.tile_width = tile_width

        start = np.where(self.sim_map == ORIGIN)
        self.origin_row = start[0][0]
        self.origin_col = start[1][0]

    def read(self, x_coordinate, y_coordinate, direction):
        row, col = self.coordinates_to_indices(x_coordinate, y_coordinate)
        row = round(row - np.sin(np.radians(direction)))
        col = round(col + np.cos(np.radians(direction)))

        in_bounds = 0 <= row < self.sim_map.shape[0] and 0 <= col < self.sim_map.shape[1]
        if not in_bounds:
            # GEARS is requesting information beyond the sim map
            return np.inf  # no obstacle detected

        mark = self.sim_map[row][col]

        if mark == WALL:
            return self.tile_width / 4
        return np.inf

    # Convert coordinates (tile widths) to row and column indices
    def coordinates_to_indices(self, x_coordinate, y_coordinate):
        row = round(self.origin_row - y_coordinate)
        col = round(self.origin_col + x_coordinate)
        return row, col


class Simulator:
    def __init__(self, gears, filename, visualizer=True):
        self.gears = gears
        self.origin_row = 0
        self.origin_col = 0
        self.exit_x = np.inf
        self.exit_y = np.inf
        self.visualizer = visualizer
        self.finished = False
        self.sim_map = load_map(filename)
        self.setup_map()
        self.display_map()

    # Convert indices to coordinates (tile widths)
    def indices_to_coordinates(self, row, col):
        y_coordinate = self.origin_row - row
        x_coordinate = col - self.origin_col
        return x_coordinate, y_coordinate

    def setup_map(self):
        start = np.where(self.sim_map == ORIGIN)
        self.origin_row = start[0][0]
        self.origin_col = start[1][0]

        exit_point = np.where(self.sim_map == EXIT)
        try:
            exit_row = exit_point[0][0]
            exit_col = exit_point[1][0]
            self.exit_x, self.exit_y = self.indices_to_coordinates(exit_row, exit_col)
        except IndexError:
            print('Exit not marked')
            self.exit_x = np.inf
            self.exit_y = np.inf

    # Display a polished map output
    def display_map(self, show_coordinates=False):
        map_copy = self.sim_map.copy()

        if show_coordinates:
            for col in range(map_copy.shape[1]):
                x, y = self.indices_to_coordinates(0, col)
                print(f'{x:3.0f}', end='')
            print()

        print('---' * map_copy.shape[1])

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
                elif coordinates in self.gears.path:
                    color = LIGHT_GREY
                else:
                    color = ''

                print(color + char + RESET + ', ', end='')

            if not show_coordinates:
                print(f'|')
            if show_coordinates:
                x, y = self.indices_to_coordinates(i, 0)
                print(f'|{y:2.0f}')
        print('---' * map_copy.shape[1])

    # main logic for the simulator
    def run(self):

        if self.visualizer:
            # make a copy of the map at the start of the cycle
            map_copy = self.gears.map.copy()

        self.gears.run()  # advance the state of GEARS by one timestep

        # if GEARS found the exit and is in auto mode
        if self.gears.near(self.exit_x, self.exit_y, 0.1) and self.gears.mode == 'auto':
            print(f'\nGears successfully exited the maze')
            self.finished = True

        if self.visualizer:
            # check if the map has changed during this cycle
            map_changed = map_copy.size != self.gears.map.size or not np.all(map_copy == self.gears.map)

            # if the map has changed
            if map_changed:
                if platform.system() == 'Windows':
                    os.system('cls')  # clear the terminal
                elif platform.system() == 'Linux':
                    os.system('clear')
                else:
                    print('Unrecognized system')

                self.gears.display_map()  # display the new map


def sync_with_hardware(virtual_gears, physical_gears):
    physical_gears.on = virtual_gears.on

    # sync physical motor dps with virtual motor dps
    physical_gears.left_dps = virtual_gears.left_dps
    physical_gears.right_dps = virtual_gears.right_dps

    # sync virtual motor encoder with physical motor encoder
    virtual_gears.left_encoder = physical_gears.get_motor_encoder(physical_gears.left_wheel)
    virtual_gears.right_encoder = physical_gears.get_motor_encoder(physical_gears.right_wheel)

    physical_gears.update_motors()  # update the physical motors


def main():
    filename = 'maps/inputs/map1.csv'
    max_speed = 10  # cm/s
    tile_width = 40  # cm
    timestep = 0.01  # s
    hardware_sync = True

    if hardware_sync:
        from gears import Gears
        physical_gears = Gears(mode='manual', max_speed=max_speed)

    ultrasonic = VirtualUltrasonic(filename=filename, tile_width=tile_width)
    virtual_gears = VirtualGears(ultrasonic=ultrasonic, max_speed=max_speed, timestep=timestep)
    simulator = Simulator(virtual_gears, filename=filename, visualizer=True)

    # create a Terminal object
    terminal = Terminal(virtual_gears, on_startup=[virtual_gears.setup], on_exit=[virtual_gears.display_map,
                                                                                  virtual_gears.write_map,
                                                                                  virtual_gears.write_hazards,
                                                                                  virtual_gears.exit])
    terminal.start()  # start the terminal

    try:
        while terminal.active:  # while the terminal is active
            simulator.run()  # run the simulator

            if hardware_sync:  # if syncing with the hardware
                sync_with_hardware(virtual_gears, physical_gears)  # update the physical GEARS

            if simulator.finished:  # if the simulation finishes
                terminal.exit()  # exit the terminal

    except KeyboardInterrupt:  # if the user presses Ctrl+C
        terminal.exit()  # exit the terminal


if __name__ == '__main__':
    main()
