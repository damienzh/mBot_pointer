#! /usr/bin/env python

import rospy
import tf
import cv2, cv_bridge
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from std_srvs.srv import Empty
import numpy as np

class Identifier:
    def __init__(self, rgb=None, depth=None, pcl=None):
        '''rgb, depth are cv image
            pcl is numpy array'''
        self.color_img = rgb
        self.depth_img = depth
        self.pcl = pcl

    def in_sight(self):

        pass

def getTarget():
    pass


if __name__ == '__main__':
    rospy.init_node('identifier')
    rospy.Service('getTarget', Empty, getTarget)
    rospy.spin()