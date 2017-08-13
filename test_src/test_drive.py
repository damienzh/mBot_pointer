#! /usr/bin/env python

import sys, select, tty, termios
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

def callback(msg):
    vel = Twist()
    if msg.range > 0.4:
        vel.linear.x = 0.1
    else:
        vel.linear.x = 0

    cmd_pub.publish(vel)

def get_key():
    orig_settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin)
    select.select([sys.stdin], [], [], 0)
    x = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)
    return x

KEY_MAP = {'w':[1,0], 's':[-1,0], 'a':[0,-1], 'd':[0,1], ' ':[0,0]}

if __name__ == '__main__':
    rospy.init_node('mbot_driver')
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    args = sys.argv
    if len(args) == 1:
        print ('autonomous')
        range_sub = rospy.Subscriber('terarangerone', Range, callback)
    else:
        print ('tele-op')
        while(1):
            keyin = get_key()
            if keyin in KEY_MAP.keys():
                print ('pressed',keyin)
                linear = KEY_MAP[keyin][0] * 0.5
                angular = KEY_MAP[keyin][1] * 0.5
            else:
                print (keyin, 'is not a valid key')
                linear = 0
                angular = 0
                if keyin == chr(27):
                    break

            vel = Twist()
            vel.linear.x = linear
            vel.angular.z = angular
            cmd_pub.publish(vel)

    rospy.spin()