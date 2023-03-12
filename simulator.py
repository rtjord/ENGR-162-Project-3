from virtual_gears import VirtualGears
from terminal import Terminal
import numpy as np

def main():
    gears = VirtualGears()  # Create a VGears object

    # Create a Terminal object
    terminal = Terminal(gears, on_startup=[], on_exit=[gears.exit])
    terminal.start()  # Start the terminal

    sim_map = np.array([['!', '!', '!', '!', '!', '!', '!', 'U', '!', '!'],
                        ['!', '!', 'U', '!', 'U', '!', '!', 'U', '!', '!'],
                        ['!', '!', 'U', '!', '!', '!', '!', 'U', '!', '!'],
                        ['!', 'U', 'U', 'U', 'U', 'U', 'U', 'U', 'U', '!'],
                        ['!', '!', 'U', '!', 'U', '!', 'U', '!', 'U', '!'],
                        ['!', '!', 'U', 'U', 'U', '!', 'U', '!', 'U', '!'],
                        ['!', 'O', 'U', '!', '!', '!', 'U', 'U', 'U', '!'],
                        ['!', '!', '!', '!', '!', '!', '!', '!', '!', '!']])
    gears.map = sim_map
    start = np.where(sim_map == 'O')
    gears.origin_row = start[0][0]
    gears.origin_col = start[1][0]
    gears.row = gears.origin_row
    gears.col = gears.origin_col

    try:
        while terminal.active:  # While the terminal is active
            gears.run()  # Run main logic for rover
    except KeyboardInterrupt:  # If the user presses Ctrl+C
        terminal.exit()  # Exit the terminal


if __name__ == '__main__':
    main()
