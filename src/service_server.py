#! /usr/bin/env python

import math
import rospy
from mbot_pointer.srv import Twist2cmd, Twist2cmdResponse
from std_srvs.srv import Empty, EmptyRequest
from config import count2dis, wheelbase

max_rpm = 180

def twist2pwm(req):
    v = req.linear_x
    w = req.angular_z
    vr = (2*v + w*wheelbase) / 2
    vl = 2*v - vr

    return Twist2cmdResponse(vl, vr)

def twist2rpm(req):
    v = req.linear_x
    w = req.angular_z
    vr = (2 * v + w * wheelbase) / 2
    vl = 2 * v - vr
    cmd_l = v2rpm(vl)
    cmd_r = v2rpm(vr)

    return Twist2cmdResponse(cmd_l, cmd_r)

def v2rpm(vel):
    rpm = int((vel/count2dis) * 360 * 60)
    if abs(rpm) > max_rpm:
        rpm = math.copysign(max_rpm, rpm)
    return rpm

if __name__ == '__main__':
    rospy.init_node('mbot_service')
    rospy.Service('twist2pwm', Twist2cmd, twist2pwm)
    rospy.Service('twist2rpm', Twist2cmd, twist2rpm)
    rospy.spin()