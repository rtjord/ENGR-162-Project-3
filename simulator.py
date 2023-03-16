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
    x, y = gears.get_neighbor_coordinates(BACK)
    gears.update_map(x, y, WALL)

    return gears


def main():
    gears = VirtualGears(max_speed=500, buffer_time=0.01, visualizer=True)  # create a VirtualGears object
    sim_map = load_map('maps/inputs/map5.csv')  # load map
    gears = setup_map(sim_map, gears)  # give map to GEARS

    sim_map = gears.map
    # create a Terminal object
    terminal = Terminal(gears, on_startup=[], on_exit=[gears.display_map,
                                                       gears.exit])
    terminal.start()  # start the terminal

    try:
        while terminal.active:  # while the terminal is active
            gears.run()  # run main logic for rover

            # the map has expanded but not because of path finding failure
            if gears.map.size > sim_map.size and gears.target_fails == 0:
                print(f'\nGears successfully exited the maze')  # alert the user
                terminal.exit()  # exit the terminal

    except KeyboardInterrupt:  # if the user presses Ctrl+C
        terminal.exit()  # exit the terminal


if __name__ == '__main__':
    main()
