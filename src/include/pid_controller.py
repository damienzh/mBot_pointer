#! /usr/bin/env python

class pid:
    def __init__(self,p,i,d):
        self.p = p
        self.i = i
        self.d = d

        self.error_last = 0
        self.error_i = 0

    def update_param(self, P, I, D):
        self.p = P
        self.i = I
        self.d = D

    def update(self, ref, feedback):
        error = ref - feedback
        error_d = error - self.error_last
        self.error_i += error
        u = self.p * error + self.i * self.error_i + self.d * error_d
        self.error_last = error
        return u

if __name__ == '__main__':
    pass