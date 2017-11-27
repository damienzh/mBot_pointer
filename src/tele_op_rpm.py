#! /usr/bin/env python

import rospy
import sys, select, tty, termios
from std_msgs.msg import Int16MultiArray

def get_key():
    orig_settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin)
    select.select([sys.stdin], [], [], 0)
    x = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)
    return x


def vel2rpm(cmd, vel):
    '''vel [-100,100], rpm [-255,255]'''
    p = [cmd[0]*vel, -cmd[1]*vel]

    return p

V = 60
KEY_MAP_M = {'w':[1, 1], 's':[-1, -1], 'a':[-1, 1], 'd':[1, -1], ' ':[0, 0]}
KEY_MAP_V = {'q':10, 'e':-10}

if __name__ == '__main__':
    rospy.init_node('tele_op_rpm')

    pub_rpm = rospy.Publisher('cmd_vel_rpm', Int16MultiArray, queue_size=1)
    # init rpm value
    rpm = Int16MultiArray()
    rpm.data = [0,0]

    info = "=======================================================\n"\
           "  w/s: move forward or backward\n" \
           "  a/d: turn left or right\n" \
           "  q/e: increase or decrease speed by 10 rpm\n" \
           "  space: stop\n" \
           "  Esc: exit\n" \
           "  Default speed 60 rpm\n"\
           "=======================================================\n"
    print info
    pub_rpm.publish(rpm)

    while not rospy.is_shutdown():
        keyin = get_key()
        if keyin in KEY_MAP_V:
            V = V + KEY_MAP_V[keyin]
            print "current speed:", V,"rpm"
        elif keyin in KEY_MAP_M:
            #print KEY_MAP[keyin]
            rpm.data = vel2rpm(KEY_MAP_M[keyin], V)
            pub_rpm.publish(rpm)
        elif keyin == chr(27):
            print "Ctrl-C to exit"
            break
        else:
            print "Not valid key"

    rpm.data = [0, 0]
    pub_rpm.publish(rpm)

    rospy.spin()