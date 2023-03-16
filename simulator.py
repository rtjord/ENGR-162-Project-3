#!/usr/bin/env python3

from virtual_gears import VirtualGears
from terminal import Terminal
import numpy as np
from constants import *


def load_map(filename):
    with open(filename, "r") as f:
        lines = f.readlines()
        map_list = []
        for line in lines:
            char_list = line.split()
            map_list.append(char_list)

        map_array = np.asarray(map_list)
        return map_array


def setup_map(sim_map, gears):
    gears.map = sim_map
    start = np.where(sim_map == ORIGIN)
    gears.origin_row = start[0][0]
    gears.origin_col = start[1][0]
    gears.row = gears.origin_row
    gears.col = gears.origin_col

    return gears


def main():
    gears = VirtualGears(max_speed=500, buffer_time=0.01, visualizer=True)  # create a VirtualGears object
    sim_map = load_map('maps/inputs/map1.csv')  # load map

    gears = setup_map(sim_map, gears)  # give map to GEARS

    # locate the exit
    exit_row, exit_col = np.where(sim_map == EXIT)
    try:
        exit_row = exit_row[0]
        exit_col = exit_col[0]
        exit_x, exit_y = gears.indices_to_coordinates(exit_row, exit_col)
    except IndexError:
        print('Exit not marked')
        exit_x = np.inf
        exit_y = np.inf

    gears.display_map()

    # create a Terminal object
    terminal = Terminal(gears, on_startup=[gears.setup], on_exit=[gears.display_map,
                                                                  gears.write_map,
                                                                  gears.write_hazards,
                                                                  gears.exit])
    terminal.start()  # start the terminal

    try:
        while terminal.active:  # while the terminal is active
            gears.run()  # run main logic for rover

            # if GEARS found the exit
            if gears.near(exit_x, exit_y, 0.1):
                print(f'\nGears successfully exited the maze')
                terminal.exit()

    except KeyboardInterrupt:  # if the user presses Ctrl+C
        terminal.exit()  # exit the terminal


if __name__ == '__main__':
    main()
