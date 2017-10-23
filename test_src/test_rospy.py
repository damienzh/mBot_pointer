#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32
from mbot_pointer.srv import Twist2cmd
from dynamic_reconfigure.client import Client, DoubleParameter

def test_param():
    p = rospy.get_param('~pid_p', default=10.2)
    i = rospy.get_param('~pid_i', default=0.5)
    d = rospy.get_param('~pid_d', default=0.1)

    return p+i+d


if __name__ == '__main__':
    rospy.init_node('test_rospy', anonymous=True)

    rate = rospy.Rate(30)
    test_pub = rospy.Publisher('test_pub', Float32, queue_size=1)
    t2c_srv = rospy.ServiceProxy('twist2pwm', Twist2cmd)
    t2c_srv.wait_for_service()

    while not rospy.is_shutdown():
        f = Float32()
        f.data = test_param()
        test_pub.publish(f)


        rate.sleep()