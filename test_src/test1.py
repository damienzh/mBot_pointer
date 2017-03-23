#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge
import numpy as np

rospy.init_node('test_node')

dp_im = rospy.wait_for_message('/camera/depth/image_raw', Image)

print dp_im.encoding

b = cv_bridge.CvBridge()

im = b.imgmsg_to_cv2(dp_im,desired_encoding='passthrough')

print type(im)

np.savetxt('sample_depth_image', im)