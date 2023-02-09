import os
import numpy as np
from time import sleep
import random

myMap = np.zeros((10, 18))


def display_map(map):
    r = random.randint(0, 7)
    c = random.randint(0, 15)
    map[r][c] = 1

    for row in range(8):
        for col in range(16):
            if map[row+1][col+1] == 1:
                print('X', end=' ')
            else:
                print(' ', end=' ')
        print()


for i in range(10):
    os.system('cls')
    display_map(myMap)
    sleep(0.1)
