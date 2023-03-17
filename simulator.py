#!/usr/bin/env python3

from virtual_gears import VirtualGears
from terminal import Terminal
import numpy as np
from constants import *
import os
import platform


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
        print(self.sim_map)

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

    def run(self):

        if self.visualizer:
            # make a copy of the map at the start of the cycle
            map_copy = self.gears.map.copy()

        self.gears.run()  # run main logic for rover

        # if GEARS found the exit
        if self.gears.near(self.exit_x, self.exit_y, 0.1):
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


def main():
    filename = 'maps/inputs/map1.csv'
    ultrasonic = VirtualUltrasonic(filename=filename, tile_width=40)
    gears = VirtualGears(ultrasonic=ultrasonic, max_speed=500, buffer_time=0.01)  # create a VirtualGears object
    simulator = Simulator(gears, filename, visualizer=True)  # create a simulator object

    # create a Terminal object
    terminal = Terminal(gears, on_startup=[gears.setup], on_exit=[gears.display_map,
                                                                  gears.write_map,
                                                                  gears.write_hazards,
                                                                  gears.exit])
    terminal.start()  # start the terminal

    try:
        while terminal.active:  # while the terminal is active
            simulator.run()  # run the simulator
            if simulator.finished:  # if the simulation finishes
                terminal.exit()  # exit the terminal

    except KeyboardInterrupt:  # if the user presses Ctrl+C
        terminal.exit()  # exit the terminal


if __name__ == '__main__':
    main()
