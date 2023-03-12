from virtual_gears import VirtualGears
from terminal import Terminal
import numpy as np
import os
import time

WALL = '!'


def load_map(filename):
    with open(filename) as f:
        lines = f.readlines()
        map_list = []
        for line in lines:
            char_list = line.split()
            map_list.append(char_list)

        map_array = np.asarray(map_list)
        return map_array


def setup_map(sim_map, gears):
    gears.map = sim_map
    start = np.where(sim_map == 'O')
    gears.origin_row = start[0][0]
    gears.origin_col = start[1][0]
    gears.row = gears.origin_row
    gears.col = gears.origin_col
    return gears


def main():
    gears = VirtualGears()  # Create a VirtualGears object
    sim_map = load_map('maps/map4.txt')
    gears = setup_map(sim_map, gears)

    # Create a Terminal object
    terminal = Terminal(gears, on_startup=[], on_exit=[gears.exit])
    terminal.start()  # Start the terminal

    path_not_found = False

    try:
        while terminal.active:  # While the terminal is active
            gears.run()  # Run main logic for rover

            # GEARS appears to have hit a wall on the sim map and the GEARS map has not expanded
            if sim_map[gears.row][gears.col] == WALL and gears.path_fails == 0:
                print(f'\nCollision detected at row: {gears.row}, column: {gears.col}')
                terminal.exit()

            # the map has expanded but not because of path finding failure
            if gears.map.size != sim_map.size and gears.path_fails == 0:
                print(f'\n Gears successfully exited the maze')
                gears.display_map()
                terminal.exit()

    except KeyboardInterrupt:  # If the user presses Ctrl+C
        terminal.exit()  # Exit the terminal


if __name__ == '__main__':
    main()
