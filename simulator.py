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


def setup_map(sim_map, gears):
    gears.map = sim_map
    start = np.where(sim_map == ORIGIN)
    gears.origin_row = start[0][0]
    gears.origin_col = start[1][0]
    gears.row = gears.origin_row
    gears.col = gears.origin_col

    exit_row, exit_col = np.where(sim_map == EXIT)
    try:
        exit_row = exit_row[0]
        exit_col = exit_col[0]
        exit_x, exit_y = gears.indices_to_coordinates(exit_row, exit_col)
        gears.update_map(exit_x, exit_y, UNKNOWN)
    except IndexError:
        print('Exit not marked')
        exit_x = np.inf
        exit_y = np.inf

    return gears, exit_x, exit_y


def main():
    gears = VirtualGears(max_speed=500, buffer_time=0.01)  # create a VirtualGears object
    sim_map = load_map('maps/inputs/map6.csv')  # load map

    gears, exit_x, exit_y = setup_map(sim_map, gears)  # give map to GEARS

    gears.display_map()

    # create a Terminal object
    terminal = Terminal(gears, on_startup=[gears.setup], on_exit=[gears.display_map,
                                                                  gears.write_map,
                                                                  gears.write_hazards,
                                                                  gears.exit])
    terminal.start()  # start the terminal

    try:
        while terminal.active:  # while the terminal is active

            # make a copy of the map at the start of the cycle
            map_copy = gears.map.copy()

            gears.run()  # run main logic for rover

            # if GEARS found the exit
            if gears.near(exit_x, exit_y, 0.1):
                print(f'\nGears successfully exited the maze')
                terminal.exit()

            # check if the map has changed during this cycle
            map_changed = map_copy.size != gears.map.size or not np.all(map_copy == gears.map)

            # if the map has changed
            if map_changed:
                if platform.system() == 'Windows':
                    os.system('cls')  # clear the terminal
                elif platform.system() == 'Linux':
                    os.system('clear')
                else:
                    print('Unrecognized system')

                gears.display_map()  # display the new map

    except KeyboardInterrupt:  # if the user presses Ctrl+C
        terminal.exit()  # exit the terminal


if __name__ == '__main__':
    main()
