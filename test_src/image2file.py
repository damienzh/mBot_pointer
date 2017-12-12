#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge
import time
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import tf
import tf2_ros, tf2_sensor_msgs
import numpy as np
import sys

rospy.init_node('test_node')
# name = sys.argv

dp_im = rospy.wait_for_message('/camera/depth_registered/image_raw', Image)
c_im = rospy.wait_for_message('/camera/rgb/image_raw', Image)
clouds = rospy.wait_for_message('/camera/depth_registered/points', PointCloud2)

b = cv_bridge.CvBridge()

im_d = b.imgmsg_to_cv2(dp_im, desired_encoding='passthrough')
im_c = b.imgmsg_to_cv2(c_im, desired_encoding='bgr8')

#print type(im_d)
# print type(im_c)

timestr = time.strftime("%Y%m%d-%H%M%S")

c_filename = 'color_image'+timestr+'.png'
d_filename = 'depth_image'+timestr+'.png'
pc_filename = 'point_cloud'+timestr+'.txt'

cv2.imwrite(c_filename, im_c)
print('saved color image')
cv2.imwrite(d_filename, im_d)
print('saved depth image')
#cv2.waitKey(0)
#np.savetxt('sample_depth_image', im)

tf_buffer = tf2_ros.Buffer()
tf_l = tf2_ros.TransformListener(tf_buffer)

p = np.array([])
for pts in point_cloud2.read_points(clouds, skip_nans=False):
    p = np.append(p, pts[0:3])
p = p.reshape(-1, 3)
'''point cloud frame is optical frame, z represent depth, x right, y down'''
#print(p.dtype)
np.savetxt(pc_filename, p)
print 'saved point cloud'