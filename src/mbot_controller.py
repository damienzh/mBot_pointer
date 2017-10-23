#! /usr/bin/env python

''' Subscribe cmd_vel twist message
    Publish PWM motor control signal'''

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int16
from mbot_pointer.msg import Encoder
from mbot_pointer.srv import Twist2cmd
from dynamic_reconfigure.server import Server as Dyna_Server
from mbot_pointer.cfg import mbot_controllerConfig
from math import atan2
from config import *


class mBotController():
    def __init__(self):
        self.pwm_control = rospy.get_param('~pwm_control', False)

        self.v_cmd = 0
        self.w_cmd = 0
        self.v_cur = 0
        self.w_cur = 0

        self.cmd_rpm_left = 0
        self.cmd_rpm_right = 0

        self.pwm_pub = rospy.Publisher('/vel_cmd_pwm', Int16MultiArray, queue_size=1)
        self.rpm_pub = rospy.Publisher('/vel_cmd_rpm', Int16MultiArray, queue_size=1)
        rospy.Subscriber('/cmd_vel', Twist, self.velCmdCallback)
        rospy.Subscriber('/encoder', Encoder, self.encCallback)
        rospy.Subscriber('/motor_rpm', Encoder, self.rpmcallback)

        self.left_p = 0
        self.left_i = 0
        self.left_d = 0
        self.right_p = 0
        self.right_i = 0
        self.right_d = 0

        self.dynamic_param_server = Dyna_Server(mbot_controllerConfig, self.dyna_callback)

        self.left_controller = pid(self.left_p, self.left_i, self.left_d)
        self.right_controller = pid(self.right_p, self.right_i, self.right_d)
        self.rpm_msg_now = 0
        self.rpm_msg_then = 0

    def dyna_callback(self, config, level):
        self.left_p = config['pid_left_p']
        self.left_i = config['pid_left_i']
        self.left_d = config['pid_left_d']
        self.right_p = config['pid_right_p']
        self.right_i = config['pid_right_i']
        self.right_d = config['pid_right_d']

        return config

    def velCmdCallback(self, msg):
        self.v_cmd = msg.linear.x
        self.w_cmd = msg.angular.z
        self.vr_cmd = (2 * self.v_cmd + self.w_cmd * wheelbase) / 2
        self.vl_cmd = 2 * self.v_cmd - self.vr_cmd


    def encCallback(self,msg):
        self.count_l = msg.left
        self.count_r = msg.right

    def rpmcallback(self, msg):
        self.v_l = self.rpm2vel(msg.left)
        self.v_r = self.rpm2vel(msg.right)
        self.rpm_msg_now = msg.header.stamp.to_sec()
        self.dt = self.rpm_msg_now - self.rpm_msg_then
        self.update()
        self.rpm_msg_then = self.rpm_msg_now

    def update(self):
        if self.pwm_control:
            ul = self.left_controller.update(self.vl_cmd, self.v_l, self.dt)
            ur = self.right_controller.update(self.vr_cmd, self.v_r, self.dt)
            u = Int16MultiArray()
            u.data = [self.pwm(ul), self.pwm(ur)]
            self.pwm_pub.publish(u)
        else:
            u = Int16MultiArray()
            u.data = [self.vel2rpm(self.vl_cmd), -self.vel2rpm(self.vr_cmd)]
            self.rpm_pub.publish(u)

    def rpm2vel(self, rpm):
        '''convert rpm to m/s'''
        vel = rpm*360*count2dis/60
        return vel

    def vel2rpm(self, vel):
        '''m/s to rpm'''
        rpm = (vel * 60 / count2dis) / 360
        return int(rpm)

    def pwm(self, c):
        i = int(c)
        if i > 255:
            i = 255
        elif i < -255:
            i = -255
        return i


class pid:
    def __init__(self,p,i,d):
        self.p = p
        self.i = i
        self.d = d

        self.set_point = 0
        self.error = [0, 0]
        self.feedback = 0
        self.dt = 0

    def update(self, ref, feedback, dt):
        self.error[1] = ref - feedback
        self.error[0] = self.error[0] + self.error[1]

        u = self.p * self.error[1] + self.i * self.error[0] + self.d * self.error[1]/ self.dt

        return u

if __name__ == '__main__':
    rospy.init_node('mBot_controller')
    controller = mBotController()
    rospy.spin()