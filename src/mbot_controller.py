#! /usr/bin/env python

''' Subscribe cmd_vel twist message
    Publish PWM motor control signal'''

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray
from mbot_pointer.msg import Encoder
from nav_msgs.msg import Odometry
from dynamic_reconfigure.server import Server as Dyna_Server
from mbot_pointer.cfg import mbot_controllerConfig
from math import atan2
import numpy as np
from copy import deepcopy
from config import *
from include.pid_controller import pid


class mBotController():
    def __init__(self):
        self.pwm_control = rospy.get_param('~pwm_control', False)

        '''init callback data'''
        self.v_cmd = 0
        self.w_cmd = 0
        self.v_l = 0
        self.v_r = 0
        self.linear_v = 0
        self.angular_w = 0

        self.cmd_rpm_left = 0
        self.cmd_rpm_right = 0

        self.init_pid()

        self.msg_now = 0
        self.msg_then = 0

        self.pwm_pub = rospy.Publisher('/cmd_vel_pwm', Int16MultiArray, queue_size=1)
        self.rpm_pub = rospy.Publisher('/cmd_vel_rpm', Int16MultiArray, queue_size=1)
        self.dynamic_param_server = Dyna_Server(mbot_controllerConfig, self.dyna_callback)
        rospy.Subscriber('/cmd_vel', Twist, self.velCmdCallback)
        rospy.Subscriber('/encoder', Encoder, self.encCallback)
        rospy.Subscriber('/motor_rpm', Encoder, self.rpmcallback)
        rospy.Subscriber('/mbot_odom', Odometry, self.odom_callback)

    def init_pid(self):
        self.left_p = 0
        self.left_i = 0
        self.left_d = 0
        self.right_p = 0
        self.right_i = 0
        self.right_d = 0
        self.linear_p = 0
        self.linear_i = 0
        self.linear_d = 0
        self.angular_p = 0
        self.angular_i = 0
        self.angular_d = 0

        self.left_controller = pid(self.left_p, self.left_i, self.left_d)
        self.right_controller = pid(self.right_p, self.right_i, self.right_d)
        self.linear_controller = pid(self.linear_p, self.linear_i, self.linear_d)
        self.angular_controller = pid(self.angular_p, self.angular_i, self.angular_d)

    def dyna_callback(self, config, level):
        self.left_p = config['pid_left_p']
        self.left_i = config['pid_left_i']
        self.left_d = config['pid_left_d']
        self.right_p = config['pid_right_p']
        self.right_i = config['pid_right_i']
        self.right_d = config['pid_right_d']
        self.linear_p = config['pid_linear_p']
        self.linear_i = config['pid_linear_i']
        self.linear_d = config['pid_linear_d']
        self.angular_p = config['pid_angular_p']
        self.angular_i = config['pid_angular_i']
        self.angular_d = config['pid_angular_d']
        self.max_v = config['max_linear_speed']
        self.max_w = config['max_angular_speed']

        self.left_controller.update_param(self.left_p, self.left_i, self.left_d)
        self.right_controller.update_param(self.right_p, self.right_i, self.right_d)
        self.linear_controller.update_param(self.linear_p, self.linear_i, self.linear_d)
        self.angular_controller.update_param(self.angular_p, self.angular_i, self.angular_d)

        return config

    def velCmdCallback(self, msg):
        self.v_cmd = msg.linear.x
        if self.v_cmd > self.max_v:
            self.v_cmd = self.max_v
        elif self.v_cmd < -self.max_v:
            self.v_cmd = -self.max_v

        self.w_cmd = msg.angular.z
        if self.w_cmd > self.max_w:
            self.w_cmd = self.max_w
        elif self.w_cmd < -self.max_w:
            self.w_cmd = -self.max_w

        rospy.loginfo('linear command velocity %0.2f', self.v_cmd)
        rospy.loginfo('angular command velocity %0.2f', self.w_cmd)

    def encCallback(self,msg):
        self.count_l = msg.left
        self.count_r = msg.right

    def rpmcallback(self, msg):
        self.v_l = self.rpm2vel(msg.left)
        self.v_r = self.rpm2vel(msg.right)
        #self.rpm_msg_now = msg.header.stamp.to_sec()
        #self.dt = self.rpm_msg_now - self.rpm_msg_then
        #self.update()
        #self.rpm_msg_then = self.rpm_msg_now

    def odom_callback(self,msg):
        self.linear_v = msg.twist.twist.linear.x
        self.angular_w = msg.twist.twist.angular.z
        self.msg_now = msg.header.stamp.to_sec()
        self.dt = self.msg_now - self.msg_then
        uv = self.linear_control()
        uw = self.angular_control()
        (cmd_l, cmd_r) = self.u_to_cmd(uv, uw)
        self.motor_control(cmd_l, cmd_r)
        rospy.loginfo('desire speed for motor %d, %d', cmd_l, cmd_r)
        self.msg_then = deepcopy(self.msg_now)

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

    def linear_control(self):
        uv = self.linear_controller.update(self.v_cmd, self.linear_v)
        return uv

    def angular_control(self):
        uw = self.angular_controller.update(self.w_cmd, self.angular_w)
        return uw

    def motor_control(self, left_cmd, right_cmd):
        rpm_cmd_left = self.vel2rpm(left_cmd)
        rpm_cmd_right = self.vel2rpm(right_cmd)
        ul_motor = self.left_controller.update(rpm_cmd_left, self.v_l)
        ur_motor = self.right_controller.update(rpm_cmd_right, self.v_r)
        u = Int16MultiArray()
        u.data = [self.pwm(ul_motor), self.pwm(ur_motor)]
        self.pwm_pub.publish(u)

    def u_to_cmd(self, uv, uw):
        '''A = [0.5 0.5; -1/B 1/B]
        y = inv(A)*u'''
        yl = uv - 0.0835*uw
        yr = uv + 0.0835*uw

        return (yl, yr)

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


if __name__ == '__main__':
    rospy.init_node('mBot_controller')
    controller = mBotController()
    rospy.spin()