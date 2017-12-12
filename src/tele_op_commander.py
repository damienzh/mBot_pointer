#! /usr/bin/env python
'''operate robot to search target,
    confirm when target in sight'''

import cv2
import cv_bridge
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2

from src.include.helperFunction import get_key

KEY_MAP_M = {'w':[1, 0], 's':[-1, 0], 'a':[0, 1], 'd':[0, 1], ' ':[0, 0]}
V = 0.1
W = 0.1

info = "=======================================================\n"\
           "  w/s: move forward or backward\n" \
           "  a/d: turn left or right\n" \
           "  space: stop\n" \
           "  g: confirm target in sight\n" \
           "  Esc: exit\n" \
           "=======================================================\n"

camera_frame_id = 'camera_link'

class commander:
    def __init__(self):

        self.rgb = None
        self.depth = None
        self.pcl = np.array([])
        self.bridge = cv_bridge.CvBridge()

        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def get_rgb(self):
        rgb_msg = rospy.wait_for_message('/camera/rgb/image', Image)
        self.rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        self.rgb_frame = rgb_msg.header.frame_id

    def get_depth(self):
        dep_msg = rospy.wait_for_message('/camera/depth_registered/image', Image)
        self.depth = self.bridge.imgmsg_to_cv2(dep_msg, desired_encoding='passthrough')
        self.depth_frame = dep_msg.header.frame_id

    def get_pcl(self):
        pcl_msg = rospy.wait_for_message('/camera/depth_registered/points', PointCloud2)
        for pts in point_cloud2.read_points(pcl_msg, skip_nans=False):
            self.pcl = np.append(self.pcl, pts[0:3])
        self.pcl = self.pcl.reshape(-1, 3)
        self.pcl_frame = pcl_msg.header.frame_id

    def key_listener(self):
        key_in = get_key()
        cmd = Twist()
        while not rospy.is_shutdown():
            if key_in in KEY_MAP_M:
                cmd.linear.x = KEY_MAP_M[key_in][0] * V
                cmd.angular.z = KEY_MAP_M[key_in][1] * W
                self.cmd_pub.publish(cmd)
            elif key_in == 'g':
                self.get_target()
            elif key_in == chr(27):
                print "Ctrl-C to exit"
                break
        rospy.spin()

    def get_target(self):
        tf_buffer = tf2_ros.Buffer()
        tl = tf2_ros.TransformListener(tf_buffer)
        self.get_rgb()
        self.get_depth()
        self.get_pcl()

        depth = np.float32(self.depth)
        # use kmeans find object and background in depth image
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        ret, labels, centers = cv2.kmeans(depth.flatten(), 3, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
        #extract the object pixels
        label_ind = np.where(centers==np.median(centers))[0][0]
        ob = (labels==label_ind).reshape(self.depth.shape)
        # extract object points in point cloud
        ob_pts = self.pcl[ob.flatten()]
        # remove nan points
        ob_pts = ob_pts[~np.isnan(ob_pts).any(axis=1)]
        # select points suit for robot height
        ob_points = ob_pts[np.logical_and(ob_pts[:,1]<0.1, ob_pts[:,1]> -0.2)]

    def handle_pcl(self):
        '''z front ydown x right'''
        shape = self.depth.shape
        pcl = self.pcl.reshape(shape[0], shape[1], 3)
        pass

if __name__ == '__main__':
    rospy.init_node('tele_commander')

    print info
