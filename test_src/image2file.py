#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge
import time
import os
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_srvs.srv import Empty, EmptyResponse
import tf2_ros
import numpy as np
import sys
from matplotlib import pyplot as plt


class ImageSaver:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.rgb_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.rgb_callback)
        self.dep_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.dep_callback)
        self.depreg_sub = rospy.Subscriber('/camera/depth_registered/image_raw', Image, self.depreg_callback)
        self.pcl_sub = rospy.Subscriber('/camera/depth/points', PointCloud2, self.pcl_callback)
        self.pclreg_sub = rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self.pclreg_callback)

        rospy.Service('saveImages', Empty, self.saveImages)
        rospy.loginfo('call service "saveImage" to save images to disk')

    def rgb_callback(self, msg):
        self.rgb_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def dep_callback(self, msg):
        self.dep_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def depreg_callback(self, msg):
        self.depreg_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def pcl_callback(self, msg):
        self.pcl = msg

    def pclreg_callback(self, msg):
        self.pclreg = msg

    def convertPCL(self, pcl):
        '''read points from PointCloud2 message'''
        p = np.array([])
        for pts in point_cloud2.read_points(pcl, skip_nans=False):
            p = np.append(p, pts[0:3])
        return p.reshape(-1, 3)

    def saveImages(self, req):
        timestr = time.strftime("%Y%m%d-%H%M%S")

        c_filename = 'color_image_' + timestr + '.png'
        d_filename = 'depth_image_' + timestr + '.png'
        dreg_filename = 'depth_registered_image_' + timestr + '.png'
        pc_filename = 'point_cloud_' + timestr + '.txt'
        pcreg_filename = 'point_cloud_' + timestr + '.txt'

        path = os.path.join(os.path.expanduser('~'), 'catkin_ws/src/mbot_pointer/test_src/Images')

        rospy.loginfo('converting point cloud to numpy array')
        pcl = self.convertPCL(self.pcl)
        pcl_reg = self.convertPCL(self.pclreg)

        rospy.loginfo('saving files')
        cv2.imwrite(os.path.join(path, c_filename), self.rgb_img)
        cv2.imwrite(os.path.join(path, d_filename), self.dep_img)
        cv2.imwrite(os.path.join(path, dreg_filename), self.depreg_img)

        np.savetxt(os.path.join(path, pc_filename), pcl)
        np.savetxt(os.path.join(path, pcreg_filename), pcl_reg)

        rospy.loginfo('Images saved')

        return EmptyResponse()


if __name__ == '__main__':
    rospy.init_node('save_image')
    saver = ImageSaver()
    rospy.spin()