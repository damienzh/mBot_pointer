#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import TransformStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
import actionlib
import time
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

        rgbTopic = rospy.get_param('ImageTopic', default='/camera/rgb/image_raw')
        depTopic = rospy.get_param('DepthTopic', default='/camera/depth/image_raw')
        pclTopic = rospy.get_param('PointCloudTopic', default='/camera/depth/points')

        rospy.Subscriber(rgbTopic, Image, self.rgb_callback)
        rospy.Subscriber(depTopic, Image, self.dep_callback)
        rospy.Subscriber(pclTopic, PointCloud2, self.pcl_callback)
        self.frame_pub = rospy.Publisher('add_frame', TransformStamped, queue_size=1)

        rospy.Service('/objectDetection', Empty, self.cvDetect)
        rospy.loginfo('call service /objectDetection for object detection')

        if not self.test:
            self.moveClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            self.moveClient.wait_for_server()


    def getData(self):
        if self.pcl_msg != None:
            rospy.loginfo('import point cloud data')
            start = rospy.Time.now().to_sec()
            self.pcl = self.convertPCL(self.pcl_msg)
            end = rospy.Time.now().to_sec()
            rospy.loginfo('time for convert pcl message to numpy array: {}s'.format(end-start))

        self.depth_img = self.bridge.imgmsg_to_cv2(self.depth_img_msg, desired_encoding="passthrough")
        self.color_img = self.bridge.imgmsg_to_cv2(self.color_img_msg, desired_encoding='bgr8')
        self.shape = self.depth_img.shape


    def rgb_callback(self, msg):
        self.color_img_msg = msg


    def dep_callback(self, msg):
        self.depth_img_msg = msg


    def pcl_callback(self, msg):
        self.pcl_msg = msg

    def convertPCL(self, pcl):
        '''read points from PointCloud2 message to numpy array'''
        p = np.array([])
        for pts in point_cloud2.read_points(pcl, skip_nans=False):
            p = np.append(p, pts[0:3])
        return p.reshape(-1, 3)

    def cvDetect(self, req):
        self.found_object = False
        rospy.loginfo('start get images')
        self.getData()
        rospy.loginfo('start detection')
        if self.test:
            t1 = rospy.Time.now().to_sec()
        detcetedImg = self.d.detect(self.color_img)
        self.objects = self.d.objects
        self.object_num = len(self.objects)
        if self.test:
            t2 = rospy.Time.now().to_sec()
            print self.object_num
            rospy.loginfo('Time for object detection: {}'.format(t2-t1))
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
            rospy.loginfo('selected object:{}'.format(self.target_label))

        if self.test:
            cv2.imshow('detected img', detcetedImg)
            k = cv2.waitKey(0)
            rospy.loginfo('press s to save the image')
            if k == ord('s'):
                cv2.imwrite('ObjectDetection{}.png'.format(time.strftime("%Y%m%d-%H%M%S")), detcetedImg)
            cv2.destroyAllWindows()

        if self.found_object:
            rospy.loginfo('object found, getting coordinates')
            t1 = rospy.Time.now().to_sec()
            self.getObjPos()
            t2 = rospy.Time.now().to_sec()
            if self.test:
                rospy.loginfo('Time for getting object position: {}'.format(t2 - t1))

        return EmptyResponse()

    def checkPassage(self):
        windowWidth = int(self.shape[1]/2)
        checkWindow = (self.shape[1]/2 - windowWidth/2, self.shape[1]/2 + windowWidth/2)

    def getObjPos(self):
        # #Crop depth image of target object
        # depthCrop = self.CropDepth(self.depth_img, self.target_box)
        #
        # #Kmeans segment object on cropped depth image 3 clusters: background, object, floor
        # label, center = self.depthKMeans(depthCrop, 3)
        # print('kmeans done')
        # #median label represent pixel indices of object in depth image
        # n = np.argwhere(center == np.median(center))[0][0]
        # targetDepth = self.selectDepth(depthCrop, label, n)

        #Crop point cloud
        pcl = self.pcl.reshape(self.shape[0], self.shape[1], 3)
        pclCrop = self.CropPCL(pcl, self.target_box)

        # #select points of object base on depth kmeans result
        # pclCrop = pclCrop.reshape(-1, 3)
        # pclCrop = pclCrop[(label == n).flatten()]

        #remove nan points in cropped cloud
        pclSelect = self.pclRemoveNan(pclCrop)
        print('trim pcl')
        #downsample clouds of limited height
        #pclSelect = self.pclSelect(pclFilter, 1, -0.4, 0.1) # in optical frame y axis represent height

        #keep planar points only
        pclSelect = np.vstack((pclSelect[:,0], pclSelect[:,2])).transpose()
        #cluster selected points base on distance between each other
        c = PointCluster()
        print('cluster pcl')
        c.cluster(pclSelect, 0.5) #points distance within 0.5m will be in one cluster
        sortedClusters = c.ObjList() #get a list of cluster objects, sorted on mean distance from camera

        #select two closest cluster to calculate goal coordinates
        left_point = sortedClusters[0].left
        right_point = sortedClusters[0].right
        lower_point = sortedClusters[0].bottom
        # select the board face robot
        point1 = lower_point
        if point1[0,1] > 0:
            point2 = left_point
        else:
            point2 = right_point

        rospy.loginfo('got target points')
        print(point1, point2)
        #set goal as the middle point of two closest clusters
        msg = self.genFrame(point1, point2)
        self.frame_pub.publish(msg)

    def genFrame(self, p1, p2):
        '''create ROS frame transform message of the object'''
        central = (p1 + p2) / 2
        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'base_link'
        msg.child_frame_id = 'target'
        msg.transform.translation.x = central[0,1]
        msg.transform.translation.y = -central[0,0]
        yaw = math.atan2(-central[0,0], central[0,1])
        q = quaternion_from_euler(0, 0, yaw)
        msg.transform.rotation.x = q[0]
        msg.transform.rotation.y = q[1]
        msg.transform.rotation.z = q[2]
        msg.transform.rotation.w = q[3]

        return msg

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