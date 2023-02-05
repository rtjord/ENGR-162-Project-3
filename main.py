#!/usr/bin/env python3

from gears import Gears
from terminal import Terminal


def main():
    gears = Gears()  # Create a Gears object

    # Create a Terminal object
    terminal = Terminal(gears, on_startup=[], on_exit=[gears.reset_all])
    terminal.start()  # Start the terminal

    try:
        while terminal.active:  # While the terminal is active
            gears.run()  # Run main logic for rover
    except KeyboardInterrupt:  # If the user presses Ctrl+C
        terminal.exit()  # Exit the terminal


if __name__ == '__main__':
    main()
