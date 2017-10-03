#! /usr/bin/env python

import rospy
from mbot_pointer.srv import Twist2PWM, Twist2PWMResponse

wheelbase = 0.145

def convert(req):
    v = req.linear_x
    w = req.angular_z
    vr = (2*v + w*wheelbase) / 2
    vl = 2*v - vr

    return Twist2PWMResponse(vl, vr)

if __name__ == '__main__':
    rospy.init_node('twist2pwm')
    service = rospy.Service('twist2pwm', Twist2PWM, convert)
    rospy.spin()