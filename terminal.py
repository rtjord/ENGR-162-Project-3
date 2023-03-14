#!/usr/bin/env python3

from threading import Thread, get_ident
from virtual_gears import VirtualGears
import os


# Cast var to a new data type
def cast(data_type, var):
    try:
        if data_type == 'str':
            return str(var)
        elif 'int' in data_type:
            return int(var)
        elif 'float' in data_type:
            return float(var)
        elif data_type == 'bool':
            return var == 'True'
        else:
            print('Unknown data type')
    except ValueError:
        print(f'Cannot convert {var} to type {data_type}')


# Convert items in the list to ints and floats if possible
def list_to_numeric(my_list):
    for i, item in enumerate(my_list):
        try:
            # If the item is an integer
            if int(item) == float(item):
                my_list[i] = int(item)
            else:
                my_list[i] = float(item)
        except ValueError:
            continue
    return my_list


# Terminal to interact with the GEARS in real time
# object must have 'self.on' as an instance variable
class Terminal:
    def __init__(self, obj, on_startup=(), on_exit=()):
        if on_startup is None:
            on_startup = []
        self.obj = obj  # object that the terminal is controlling
        self.obj_class_name = type(obj).__name__  # object's class name

        # known commands
        self.commands = {'on': self.on_cmd,
                         'off': self.off_cmd,
                         'get': self.get_cmd,
                         'set': self.set_cmd,
                         'run': self.run_cmd,
                         'list': self.list_cmd,
                         'log': self.log_cmd,
                         'reset': self.reset_cmd,
                         'clear': self.clear_cmd,
                         'exit': self.exit}

        self.active = True  # set false to deactivate the terminal
        self.log = []  # list of previous commands
        self.input_thread = Thread(target=self.get_input)  # Create a thread to receive user input

        self.on_startup = on_startup  # object methods to run on startup
        self.on_exit = on_exit  # object methods to run on exit

    # Start the terminal
    def start(self):

        # run start up methods
        for method in self.on_startup:
            method()

        self.print_commands()  # print all commands
        self.input_thread.start()  # start listening for user input

    # List all commands
    def print_commands(self):
        print('Known commands: ' + ' '.join(cmd for cmd in self.commands))

    # Get user input
    def get_input(self):
        while self.active:

            # Wait for user input
            try:
                user_input = input('>>> ')  # prompt the user to enter a command

            # A KeyboardInterrupt also raises an EOFError
            # Handle the EOFError here
            except EOFError:
                self.active = False  # Deactivate the terminal
                continue  # skip the rest of the loop body

            self.log.append(user_input)  # Log user input
            input_list = user_input.split()  # Split user input into list of strings

            if len(input_list) == 0:  # If the user did not enter a command
                continue  # skip the rest of the loop body

            cmd = input_list[0]  # the command is the first word the user typed

            # If the user entered an unknown command
            if cmd not in self.commands:
                print('unknown command')  # Alert the user
                continue  # Skip the rest of the loop body

            args = input_list[1:]  # Any words after the command are additional arguments
            self.commands[cmd](*args)  # Execute the correct command

    # Turn on the object
    def on_cmd(self, *args):
        if len(args) > 0:
            print('Warning: on command requires no additional arguments')

        # If the object is off
        if not self.obj.on:
            self.obj.on = True
            print(f'{self.obj_class_name} on')

        # If the object is already on
        else:
            print('Already on')  # Alert the user

    # Turn off the object
    def off_cmd(self, *args):
        if len(args) > 0:
            print('Warning: off command requires no additional arguments')

        # If GEARS is on
        if self.obj.on:
            self.obj.on = False
            print(f'{self.obj_class_name} off')

        # If GEARS is already off
        else:
            print('Already off')  # Alert the user

    # Get the value of an instance variable
    def get_cmd(self, *args):

        try:
            name = args[0]  # Get the instance variable name
        except IndexError:  # If the user did not provide an instance variable
            print('usage: get [instance_var]')  # Remind the user how to use the command
            return  # End the method execution

        if name in vars(self.obj):  # If the instance var is valid
            print(getattr(self.obj, name))  # Get the value of the instance var
        else:  # If the instance var is not valid
            print(f'{name} not found')  # Alert the user

    # Set the value of an instance variable
    def set_cmd(self, *args):
        try:
            name = args[0]  # first argument is the variable to change
            value = args[1]  # second argument is the value to assign

        # If the user did not enter enough arguments
        except IndexError:
            print('usage: set [inst_var] [value]')  # Remind the user how to use the command
            return  # skip the rest of the method

        attr_dict = vars(self.obj)  # dictionary of instance variables
        if name in attr_dict:  # if the variable name entered by the user is valid
            data_type = type(attr_dict[name]).__name__  # get the data type of the variable
            value = cast(data_type, value)  # Cast value to correct data type

            if value is not None:  # If casting was successful

                # Set the instance variable to the specified value
                setattr(self.obj, name, value)
        else:
            print(f"Attribute '{name}' not found")

    # Call a method
    def run_cmd(self, *args):
        try:
            method_name = args[0]
            method = getattr(self.obj, method_name)
        except IndexError:
            print('usage: run [method_name]')
            return
        except AttributeError:
            print(f"Method not found")
            return

        if callable(method):

            # Assumes any strings that can be converted to float should be converted to float
            method_args = list_to_numeric(list(args[1:]))
            method(*method_args)
        else:
            print(f"'{method_name}' is not callable")

    # List instance variables or methods
    def list_cmd(self, *args):
        try:
            info = args[0]
        except IndexError:
            print('usage: list [info]')
            print('[info]: vars, methods, cmds')
            return

        # If the user asked for the instance variables
        if info == 'vars':
            inst_vars = vars(self.obj)  # get a list dictionary of instance variables
            for var in inst_vars:  # for each instance variable
                print(var)  # print the variable name

        # Else if the user asked for the methods
        elif info == 'methods':
            method_list = []

            for attr_name in dir(self.obj):  # for each attribute name
                attribute = getattr(self.obj, attr_name)  # get the actual attribute

                # If the attribute is callable and does not start with "__"
                if callable(attribute) and not attr_name.startswith('__'):
                    method_list.append(attr_name)  # add the attribute name to the list of methods

            for method_name in method_list:
                print(method_name + '()')  # print each method name

        elif info == 'cmds':
            self.print_commands()  # Print all known commands

        else:
            print(f"unknown argument '{info}'")

    def log_cmd(self, *args):
        if len(args) > 0:
            print('Warning: log command requires no additional arguments')

        print(self.log)

    # Reset the object
    def reset_cmd(self, *args):
        if len(args) > 0:
            print('Warning: reset command requires no additional arguments')

        self.obj.__init__()

    def clear_cmd(self, *args):
        if len(args) > 0:
            print('Warning: clear command requires no additional arguments')

        os.system("cls")
        self.print_commands()

    # Exit the terminal
    def exit(self, *args):
        if len(args) > 0:
            print('Warning: exit command requires no additional arguments')

        self.active = False  # Disable user input

        # If exit was not called by typing "exit" in the terminal
        if get_ident() != self.input_thread.ident:

            # prompt the user to press enter so that the input thread can finish executing
            print('\nPress Enter to continue...', end='')
            self.input_thread.join()  # Wait for the user to press enter
        # Call the objects exit methods
        for method in self.on_exit:
            method()


if __name__ == '__main__':
    gears = VirtualGears()  # Create a test object

    # Create a terminal object
    terminal = Terminal(gears, on_startup=[])
    terminal.start()  # Start the terminal

    try:
        while terminal.active:  # While the terminal is active
            gears.run()  # Execute the object's run method
    except KeyboardInterrupt:  # If the user presses Ctrl+C
        terminal.exit()  # Exit the terminal
