#! /usr/bin/env python

import numpy as np

class FIFO:
    '''1D FIFO array'''
    def __init__(self, size):
        self.size = size
        self.data = np.array([])

    def append(self, num):
        self.data = np.append(self.data, num)
        if self.data.size > self.size:
            self.data = self.data[1:]

