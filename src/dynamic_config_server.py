#! /usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from mbot_pointer.cfg import mbot_paramsConfig

def callback(config, level):
    '''rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},\
          {str_param}, {bool_param}, {size}""".format(**config))'''
    return config

if __name__ == '__main__':
    rospy.init_node('mbot_pointer', anonymous=True)

    server = Server(mbot_paramsConfig, callback)

    rospy.spin()