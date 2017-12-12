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
        self.shape = depth.shape
        self.found_object = False

    def cvDetect(self):
        d = CvDetector()
        d.detect(self.color_img)
        self.objects = d.objects
        self.object_num = len(self.objects)

        if self.object_num > 0:
            center_w = self.shape[1] / 2
            centerObj = self.objects[0]
            centerObjX = centerObj.center[0]
            for obj in self.objects:
                centerX = obj.center[0]
                if centerX != centerObjX:
                    if abs(centerX - center_w) < abs(centerObjX - center_w):
                        centerObj = obj
            self.target_label = centerObj.label
            self.target_box = centerObj.box
            self.target_center = centerObj.center

            self.found_object = True

            if self.found_object:
                self.getObjPos()

    def checkPassage(self):
        windowWidth = int(self.shape[1]/2)
        checkWindow = (self.shape[1]/2 - windowWidth/2, self.shape[1]/2 + windowWidth/2)

    def getObjPos(self):
        #Crop depth image of target object
        depthCrop = self.CropDepth(self.depth_img, self.target_box)

        #Kmeans segment object on cropped depth image 3 clusters: background, object, floor
        label, center = self.depthKMeans(depthCrop, 3)

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

        #downsample clouds of limited height
        pclSelect = self.pclSelect(pclFilter, 1, -0.4, 0.1) # in optical frame y axis represent height

        #cluster selected points base on distance between each other
        c = PointCluster()
        c.cluster(pclSelect, 0.5) #points distance within 0.5m will be in one cluster
        sortedClusters = c.ObjList() #get a list of cluster objects, sorted on mean distance from camera

        #select two closest cluster to calculate goal coordinates
        point1 = sortedClusters[0].mean
        point2 = sortedClusters[1].mean

        #set goal as the middle point of two closest clusters
        self.goalCoor = (point1 + point2) / 2


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
        cloud = pcl[np.logical_and(pcl[:, a] > min, pcl[:, a] < max)]

        return cloud



class PointCluster:
    def __init__(self):
        self.clusters = []

    def distancePt(self, p1, p2):
        return np.sqrt(sum((p1 - p2) ** 2))

    def distanceAr(self, pt, array):
        if array.size > 2:
            return np.sqrt(np.sum((pt - array)**2, axis=1))
        else:
            return self.distancePt(pt, array)


    def cluster(self, points, thresh):
        self.data = points
        self.thresh = thresh
        self.clusters.append(self.data[0])
        for pt in self.data[1:]:
            self.clusterPoint(pt)

    def clusterPoint(self, point):
        newCluster = True
        ind = 0
        n = len(self.clusters)
        for i in range(n):
            distance = self.distanceAr(point, self.clusters[i])
            if np.min(distance) < self.thresh:
                ind = i
                newCluster = False
                break
        if newCluster:
            self.clusters.append(point)
        else:
            self.clusters[ind] = np.vstack((self.clusters[ind], point))

    def ObjList(self):
        newList = []
        for a in self.clusters:
            newList.append(Cluster(a))

        newList.sort(key=lambda x: x.meanDis)

        return newList


class Cluster:
    def __init__(self, array):
        self.array = array
        self.genAttr()

    def genAttr(self):
        self.distance = np.sqrt(np.sum(self.array**2, axis=1))
        self.meanDis = np.mean(self.distance)
        self.mean = np.mean(self.array, axis=0)

    def calcArea(self):
        maxX = self.array[np.argmax(self.array[:, 0])]
        minX = self.array[np.argmin(self.array[:, 0])]
        maxY = self.array[np.argmax(self.array[:, 1])]
        minY = self.array[np.argmin(self.array[:, 1])]

def getTarget():
    pass


if __name__ == '__main__':
    rospy.init_node('detector')
    rospy.Service('getTarget', Empty, getTarget)
    rospy.spin()