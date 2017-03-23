#! /usr/bin/env python

import rospy
import cv2, cv_bridge
import std_msgs
from sensor_msgs.msg import Image


class Identifier:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.img_color = rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback)
        self.img_depth = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)

    def color_callback(self, msg):
        image_color = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def depth_callback(self, msg):
        depth_data = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
