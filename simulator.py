from virtual_gears import VirtualGears
from terminal import Terminal
import numpy as np

WALL = '!'


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
    start = np.where(sim_map == 'O')
    gears.origin_row = start[0][0]
    gears.origin_col = start[1][0]
    gears.row = gears.origin_row
    gears.col = gears.origin_col
    return gears


def main():
    gears = VirtualGears()  # Create a VirtualGears object
    sim_map = load_map('maps/map1.txt')
    gears = setup_map(sim_map, gears)

    # Create a Terminal object
    terminal = Terminal(gears, on_startup=[], on_exit=[gears.display_map, gears.write_map, gears.exit])
    terminal.start()  # Start the terminal

    try:
        while terminal.active:  # While the terminal is active
            gears.run()  # Run main logic for rover

            # the map has expanded but not because of path finding failure
            if gears.map.size != sim_map.size and gears.target_fails == 0:
                print(f'\n Gears successfully exited the maze')
                terminal.exit()

    except KeyboardInterrupt:  # If the user presses Ctrl+C
        terminal.exit()  # Exit the terminal


if __name__ == '__main__':
    main()
