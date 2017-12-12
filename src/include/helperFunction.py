#! /usr/bin/env python

import cv2
import math
import numpy as np
import sys, select, tty, termios

def get_key():
    orig_settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin)
    select.select([sys.stdin], [], [], 0)
    x = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)
    return x


def quaternion_to_euler_angle(w, x, y, z):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    return X, Y, Z


def cvShow(img):
    cv2.imshow('img', img)
    cv2.waitKey(0)