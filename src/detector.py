#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import TransformStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
import actionlib
import tf2_ros, tf
from tf.transformations import quaternion_from_euler
import numpy as np
import math
import cv2
import cv_bridge
from include.CvDetector import CvDetector
from include.helperClass import PointCluster

class Detector:
    def __init__(self, test=False):
        self.test = test

        self.color_img = None
        self.depth_img = None
        self.pcl = None
        self.pcl_msg = None
        self.found_object = False
        self.target_label = ''

        self.bridge = cv_bridge.CvBridge()
        self.d = CvDetector()

        rgbTopic = rospy.get_param('ImageTopic')
        depTopic = rospy.get_param('DepthTopic')
        pclTopic = rospy.get_param('PointCloudTopic')

        rospy.Subscriber(rgbTopic, Image, self.rgb_callback)
        rospy.Subscriber(depTopic, Image, self.dep_callback)
        rospy.Subscriber(pclTopic, PointCloud2, self.pcl_callback)

        rospy.Service('/objectDetection', Empty, self.cvDetect)

        if not self.test:
            self.moveClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            self.moveClient.wait_for_server()


    def getData(self):
        if self.pcl_msg != None:
            self.pcl = self.convertPCL(self.pcl_msg)

        self.shape = self.depth_img.shape

    def rgb_callback(self, msg):
        self.color_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def dep_callback(self, msg):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def pcl_callback(self, msg):
        self.pcl_msg = msg

    def convertPCL(self, pcl):
        '''read points from PointCloud2 message to numpy array'''
        p = np.array([])
        for pts in point_cloud2.read_points(pcl, skip_nans=False):
            p = np.append(p, pts[0:3])
        return p.reshape(-1, 3)

    def cvDetect(self, req):
        rospy.loginfo('start get images')
        self.getData()
        rospy.loginfo('start detection')
        detcetedImg = self.d.detect(self.color_img)
        self.objects = self.d.objects
        self.object_num = len(self.objects)
        if self.test:
            print self.object_num
        rospy.loginfo('done detection')

        if self.object_num > 0:
            center_w = self.shape[1] / 2
            centerObj = self.objects[0]
            centerObjX = centerObj.center[0]
            for obj in self.objects[1:]:
                centerX = obj.center[0]
                if abs(centerX - center_w) < abs(centerObjX - center_w):
                    centerObj = obj
            self.target_label = centerObj.label
            self.target_box = centerObj.box
            self.target_center = centerObj.center

            self.found_object = True
            rospy.loginfo('selected object')

        if self.test:
            cv2.imshow('detected img', detcetedImg)
            cv2.waitKey(0)

        if self.found_object:
            rospy.loginfo('sending transform')
            self.getObjPos()

        return EmptyResponse()

    def checkPassage(self):
        windowWidth = int(self.shape[1]/2)
        checkWindow = (self.shape[1]/2 - windowWidth/2, self.shape[1]/2 + windowWidth/2)

    def getObjPos(self):
        #Crop depth image of target object
        depthCrop = self.CropDepth(self.depth_img, self.target_box)

        #Kmeans segment object on cropped depth image 3 clusters: background, object, floor
        label, center = self.depthKMeans(depthCrop, 3)
        print('kmeans done')
        #median label represent pixel indices of object in depth image
        n = np.argwhere(center == np.median(center))[0][0]
        targetDepth = self.selectDepth(depthCrop, label, n)

        #Crop point cloud
        pcl = self.pcl.reshape(self.shape[0], self.shape[1], 3)
        pclCrop = self.CropPCL(pcl, self.target_box)

        #select points of object base on depth kmeans result
        pclCrop = pclCrop.reshape(-1, 3)
        pclCrop = pclCrop[(label == n).flatten()]

        #remove nan points in cropped cloud
        pclFilter = self.pclRemoveNan(pclCrop)
        print('trim pcl')
        #downsample clouds of limited height
        pclSelect = self.pclSelect(pclFilter, 1, -0.4, 0.1) # in optical frame y axis represent height

        #cluster selected points base on distance between each other
        c = PointCluster()
        print('cluster pcl')
        c.cluster(pclSelect, 0.5) #points distance within 0.5m will be in one cluster
        sortedClusters = c.ObjList() #get a list of cluster objects, sorted on mean distance from camera

        #select two closest cluster to calculate goal coordinates
        point1 = sortedClusters[0].mean
        point2 = sortedClusters[1].mean
        rospy.loginfo('got target points')
        print(point1, point2)
        #set goal as the middle point of two closest clusters
        #self.sendGoal(point1, point2)

    def sendGoal(self, p1, p2):
        if p1[0] > p2[0]:
            left = p2
            right = p1
        else:
            left = p1
            right = p2

        goal = MoveBaseGoal()
        goalX = (left[1] + right[1]) / 2
        goalY = (left[0] + right[0]) / 2
        yaw = math.atan2((right[1]-left[1]), (right[0] - left[0]))

        goal.target_pose.header.frame_id = 'base_link'
        goal.target_pose.pose.position.x = goalX
        goal.target_pose.pose.position.y = goalY
        q = quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.moveClient.send_goal(goal)
        rospy.loginfo('sent goal pose')
        self.moveClient.wait_for_result()

    def CropDepth(self, dimg, box):
        cropped = dimg[box[1]:box[3], box[0]:box[2]]

        return cropped

    def depthKMeans(self, dimg, K):
        img = np.float32(dimg)
        imgFlat = img.flatten()
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        ret, label, center = cv2.kmeans(imgFlat, K, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
        res = center[label.flatten()]
        res2 = res.reshape(dimg.shape)

        return label, center

    def selectDepth(self, dimg, label, n):
        img_flat = dimg.flatten()
        label_flat = label.flatten()
        ind = np.where(label_flat != n)
        img_flat[ind] = 0

        return img_flat.reshape(dimg.shape)

    def CropPCL(self, pcl, box):
        cropped = pcl[box[1]:box[3], box[0]:box[2], :]

        return cropped

    def pclRemoveNan(self, pcl):
        cloud = pcl.reshape(-1, 3)
        cloud = cloud[~np.isnan(cloud).any(axis=1)]

        return cloud

    def pclSelect(self, pcl, a, min, max):
        '''trim data along which axis, x:0, y:1, z:2'''
        cloud = pcl[np.logical_and(pcl[:, a] > min, pcl[:, a] < max)]

        return cloud


if __name__ == '__main__':
    rospy.init_node('detector')
    d = Detector(test=True)
    rospy.spin()