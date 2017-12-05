#! /usr/bin/env python
'''subscribe vel_cmd
 publish pwm control for two motors'''

import rospy
from dynamic_reconfigure.server import Server as DynaServer
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Twist
from mbot_pointer.msg import Encoder
from mbot_pointer.cfg import mbot_controllerConfig
from include.pid_controller import pid
from config import count2dis

class mbotDriver:
    def __init__(self):

        self.init_controller()

        self.cmd_l = 0
        self.cmd_r = 0
        self.feedback_l = 0
        self.feedback_r = 0

        self.dynamic_param_server = DynaServer(mbot_controllerConfig, self.dyna_callback)
        self.rpm_sub = rospy.Subscriber('/motor_rpm', Encoder, self.rpm_callback)
        self.cmd_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_callback)
        self.pwm_pub = rospy.Publisher('/cmd_vel_pwm', Int16MultiArray, queue_size=1)

    def init_controller(self):
        self.controller_left = pid()
        self.controller_left.set_limit(255, -255)
        #self.controller_left.set_params(0.3, 0.5, 0.16)
        #print 'left pid:', self.controller_left.Kp, self.controller_left.Ki, self.controller_left.Kd

        self.controller_right = pid()
        self.controller_right.set_limit(255, -255)
        #self.controller_right.set_params(0.3, 0.5, 0.16)
        #print 'right pid:', self.controller_right.Kp, self.controller_right.Ki, self.controller_right.Kd

    def dyna_callback(self, config, level):
        left_p = config['pid_left_p']
        left_i = config['pid_left_i']
        left_d = config['pid_left_d']
        right_p = config['pid_right_p']
        right_i = config['pid_right_i']
        right_d = config['pid_right_d']

        self.controller_left.set_params(left_p, left_i, left_d)
        self.controller_right.set_params(right_p, right_i, right_d)

        return config

    def cmd_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        leftV = linear - 0.0835 * angular
        rightV = linear + 0.0835 * angular
        self.cmd_l = self.vel2rpm(leftV)
        self.cmd_r = self.vel2rpm(rightV)


    def rpm_callback(self, msg):
        self.feedback_l = msg.left
        self.feedback_r = msg.right

        ul = self.controller_left.update(self.cmd_l, self.feedback_l)
        ur = self.controller_right.update(self.cmd_r, self.feedback_r)
        rospy.loginfo('left: %d %d,  right: %d %d', self.cmd_l, self.feedback_l, self.cmd_r, self.feedback_r)

        u = Int16MultiArray()
        u.data = [ul, ur]
        self.pwm_pub.publish(u)

    def vel2rpm(self, vel):
        '''m/s to rpm'''
        rpm = (vel * 60 / count2dis) / 360
        return int(rpm)


if __name__ == '__main__':
    rospy.init_node('mbot_driver')
    driver = mbotDriver()
    rospy.spin()