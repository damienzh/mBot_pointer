#! /usr/bin/env python

''' Subscribe cmd_vel twist message
    Publish PWM motor control signal'''

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int16
from math import atan2

wheelbase = 0.145

class mBotController():
    def __init__(self):
        rospy.init_node('mBot_controller')

        self.v_cmd = 0
        self.w_cmd = 0
        self.v_cur = 0
        self.w_cur = 0

        cmd_pub = rospy.Publisher('/vel_cmd_pwm', Int16MultiArray, queue_size=1)
        rospy.Subscriber('/cmd_vel', Twist, self.velCmdCallback)
        rospy.Subscriber('/encoder_l', Int16, self.encCallbackL)
        rospy.Subscriber('/encoder_r', Int16, self.encCallbackR)

    def velCmdCallback(self, msg):
        self.v_cmd = msg.linear.x
        self.w_cmd = msg.angular.z
        self.vr_cmd = (2 * self.v_cmd + self.w_cmd * wheelbase) / 2
        self.vl_cmd = 2 * self.v_cmd - self.vr_cmd

    def encCallbackL(self,msg):
        self.v_l =msg.data

    def encCallbackR(self,msg):
        self.v_r = msg.data

    def controller(self):
        self.trv_rot = atan2((self.v_r - self.v_l), wheelbase)
        self.trv_dis = (self.v_l + self.v_r) / 2

if __name__ == '__main__':
    pass