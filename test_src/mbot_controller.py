#! /usr/bin/env python

''' Subscribe cmd_vel twist message
    Publish PWM motor control signal'''

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray

def vel_callback(msg):
    pwm = Int16MultiArray()
    pwm.data = [0, 0]
    v = msg.linear.x
    w = msg.angular.z



if __name__ == '__main__':
    rospy.init_node('mBot_controller')
    cmdVel_sub = rospy.Subscriber('cmd_vel', Twist, vel_callback)
    cmdPWM_pub = rospy.Publisher('cmd_vel_pwm', Int16MultiArray, queue_size=1)