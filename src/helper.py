#! /usr/bin/env python

import numpy as np
import sys, select, tty, termios

def get_key():
    orig_settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin)
    select.select([sys.stdin], [], [], 0)
    x = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)
    return x

class FIFO:
    '''1D FIFO array'''
    def __init__(self, size):
        self.size = size
        self.data = np.array([])

    def append(self, num):
        self.data = np.append(self.data, num)
        if self.data.size > self.size:
            self.data = self.data[1:]
