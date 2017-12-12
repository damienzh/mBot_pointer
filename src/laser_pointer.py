#! /usr/bin/env python

import rospy
import tf2_ros, tf
from std_msgs.msg import Int16
from config import SERVO_0, SERVO_90, SERVO_180

class PointerDriver:
    def __init__(self):

        self.anglePub = rospy.Publisher('pointer_angle', Int16, queue_size=1)



if __name__ == '__main__':
    rospy.init_node('pointer_driver')
    r = rospy.Rate(1)
    p = PointerDriver()
    r.sleep()
    rospy.spin()