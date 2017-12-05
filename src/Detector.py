#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from std_srvs.srv import Empty
import tf2_ros, tf
import numpy as np
import cv2
from include.CvDetector import CvDetector

class Detector:
    def __init__(self, rgb=None, depth=None, pcl=None):
        '''rgb, depth are cv image
            pcl is numpy array'''
        self.color_img = rgb
        self.depth_img = depth
        self.pcl = pcl

    def cvDetect(self):
        d = CvDetector()
        d.detect(self.color_img)
        self.objects = d.objects
        self.object_num = self.objects.__len__


def getTarget():
    pass


if __name__ == '__main__':
    rospy.init_node('detector')
    rospy.Service('getTarget', Empty, getTarget)
    rospy.spin()