#! /usr/bin/env python

class pid:
    def __init__(self):
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0

        self.error_last = 0
        self.error_i = 0

        self.output_max = 0
        self.output_min = 0

    def set_params(self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        print 'update parameters', self.Kp, self.Ki, self.Kd

    def set_limit(self, upper, lower):
        self.output_max = upper
        self.output_min = lower

    def update(self, ref, feedback):
        error = ref - feedback
        error_d = error - self.error_last
        self.error_i += error
        u = self.Kp * error + self.Ki * self.error_i + self.Kd * error_d
        self.error_last = error

        if u > self.output_max:
            u = self.output_max
        elif u < self.output_min:
            u = self.output_min

        return u

