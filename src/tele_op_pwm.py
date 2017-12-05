#! /usr/bin/env python

import rospy
from std_msgs.msg import Int16MultiArray

from include.helper import get_key


# def get_key():
#     orig_settings = termios.tcgetattr(sys.stdin)
#     tty.setraw(sys.stdin)
#     select.select([sys.stdin], [], [], 0)
#     x = sys.stdin.read(1)
#     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)
#     return x


def vel2pwm(cmd, vel):
    '''vel [-100,100], pwm [-255,255]'''
    v = vel
    if v >= 100:
        v = 100
    p = [cmd[0]*v, cmd[1]*v]

    p[0] = p[0] * 255 / 100
    p[1] = -p[1] * 255 / 100

    return p

V = 50
KEY_MAP_M = {'w':[1, 1], 's':[-1, -1], 'a':[-1, 1], 'd':[1, -1], ' ':[0, 0]}
KEY_MAP_V = {'q':10, 'e':-10}

if __name__ == '__main__':
    rospy.init_node('tele_op_pwm')

    pub_pwm = rospy.Publisher('cmd_vel_pwm', Int16MultiArray, queue_size=1)
    # init pwm value
    pwm = Int16MultiArray()
    pwm.data = [0,0]

    info = "=======================================================\n"\
           "  w/s: move forward or backward\n" \
           "  a/d: turn left or right\n" \
           "  q/e: increase or decrease speed by 10% of full speed\n" \
           "  space: stop\n" \
           "  Esc: exit\n" \
           "  Default speed 50% full\n"\
           "=======================================================\n"
    print info
    pub_pwm.publish(pwm)

    while not rospy.is_shutdown():
        keyin = get_key()
        if keyin in KEY_MAP_V:
            V = V + KEY_MAP_V[keyin]
            print "current speed:", V,"% full speed"
        elif keyin in KEY_MAP_M:
            #print KEY_MAP[keyin]
            pwm.data = vel2pwm(KEY_MAP_M[keyin], V)
            pub_pwm.publish(pwm)
        elif keyin == chr(27):
            print "Ctrl-C to exit"
            break
        else:
            print "Not valid key"

    pwm.data = [0, 0]
    pub_pwm.publish(pwm)

    rospy.spin()