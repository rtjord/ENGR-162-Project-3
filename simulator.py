#!/usr/bin/env python3

from virtual_gears import VirtualGears
from terminal import Terminal
import numpy as np
from constants import *
import os
import platform


class Simulator:
    def __init__(self, gears, filename):
        self.gears = gears
        self.exit_x = np.inf
        self.exit_y = np.inf
        self.finished = False
        self.sim_map = self.load_map(filename)
        self.setup_map()

    def load_map(self, filename):
        with open(filename, "r") as f:
            lines = f.readlines()
            map_list = []
            for line in lines:
                char_list = line.split()
                map_list.append(char_list)

            map_array = np.asarray(map_list)
            return map_array

    def setup_map(self):
        self.gears.map = self.sim_map
        start = np.where(self.sim_map == ORIGIN)
        self.gears.origin_row = start[0][0]
        self.gears.origin_col = start[1][0]
        self.gears.row = self.gears.origin_row
        self.gears.col = self.gears.origin_col

        exit_row, exit_col = np.where(self.sim_map == EXIT)
        try:
            exit_row = exit_row[0]
            exit_col = exit_col[0]
            self.exit_x, self.exit_y = self.gears.indices_to_coordinates(exit_row, exit_col)
            self.gears.update_map(self.exit_x, self.exit_y, UNKNOWN)
        except IndexError:
            print('Exit not marked')
            self.exit_x = np.inf
            self.exit_y = np.inf

    def run(self):
        # make a copy of the map at the start of the cycle
        map_copy = self.gears.map.copy()

        self.gears.run()  # run main logic for rover

        # if GEARS found the exit
        if self.gears.near(self.exit_x, self.exit_y, 0.1):
            print(f'\nGears successfully exited the maze')
            self.finished = True

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
    gears = VirtualGears(max_speed=500, buffer_time=0.01)  # create a VirtualGears object
    simulator = Simulator(gears, 'maps/inputs/map6.csv')  # create a simulator object

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
