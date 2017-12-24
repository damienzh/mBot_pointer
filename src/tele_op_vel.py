#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from include.helperFunction import get_key

def vel2rpm(cmd, vel):
    '''vel [-100,100], rpm [-255,255]'''
    p = [cmd[0]*vel, -cmd[1]*vel]

    return p

V = 0.15
W = 0.1
KEY_MAP_M = {'w':[1, 0], 's':[-1, 0], 'a':[0, 1], 'd':[0, -1], ' ':[0, 0]}
KEY_MAP_V = {'q':0.05, 'e':-0.05}
KEY_MAP_W = {'r':0.1, 't':-0.1}

if __name__ == '__main__':
    rospy.init_node('tele_op_rpm')

    pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    # init rpm value
    vel = Twist()

    info = "=======================================================\n"\
           "  w/s: move forward or backward\n" \
           "  a/d: turn left or right\n" \
           "  q/e: increase or decrease linear speed by 0.05 m/s\n" \
           "  r/t: increase or decrease angular speed by 0.1 rad/s\n"\
           "  space: stop\n" \
           "  Esc: exit\n" \
           "  Default linear speed 0.15 m/s\n"\
           "  Default angular speed 0.1 rad/s\n"\
           "=======================================================\n"
    print info
    pub_vel.publish(vel)

    while not rospy.is_shutdown():
        keyin = get_key()
        if keyin in KEY_MAP_V:
            V = V + KEY_MAP_V[keyin]
            print "current speed:", V,"m/s"
        elif keyin in KEY_MAP_W:
            W = W + KEY_MAP_W[keyin]
            print "current angular speed:", W,"rad/s"
        elif keyin in KEY_MAP_M:
            #print KEY_MAP[keyin]
            vel.linear.x = KEY_MAP_M[keyin][0] * V
            vel.angular.z = KEY_MAP_M[keyin][1] * W
            pub_vel.publish(vel)
        elif keyin == chr(27):
            print "Ctrl-C to exit"
            break
        else:
            print "Not valid key"

    vel.linear.x = 0
    vel.angular.z = 0
    pub_vel.publish(vel)

    rospy.spin()