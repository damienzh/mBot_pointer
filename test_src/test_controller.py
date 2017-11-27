#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray
from nav_msgs.msg import Odometry


class PID:
    def __init__(self):
        self.p = 0
        self.i = 0
        self.d = 0
        self.error = [0, 0]
        self.error_i = 0
        self.error_d = 0

    def update_param(self, P, I, D):
        self.p = P
        self.i = I
        self.d = D

    def update(self, set_point, feedback):
        error = set_point - feedback
        u = self.p * error + self.i * self.error_i - self.d * self.error_d
        self.error_i += error


if __name__ == '__main__':
    rospy.init_node('test_controller')