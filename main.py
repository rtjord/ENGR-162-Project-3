#!/usr/bin/env python3

from gears import Gears
from terminal import Terminal
import os
import platform
import numpy as np


def main():
    gears = Gears(max_speed=10, wheel_radius=4)  # Create a Gears object

    # Create a Terminal object
    terminal = Terminal(gears, on_startup=[gears.setup], on_exit=[gears.display_map,
                                                                  gears.write_map,
                                                                  gears.write_hazards,
                                                                  gears.exit])
    terminal.start()  # Start the terminal

    try:
        while terminal.active:  # While the terminal is active

            # make a copy of the map at the start of the cycle
            map_copy = gears.map.copy()

            gears.run()  # Run main logic for rover

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

    except KeyboardInterrupt:  # If the user presses Ctrl+C
        terminal.exit()  # Exit the terminal


if __name__ == '__main__':
    main()
