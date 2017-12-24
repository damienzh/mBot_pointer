#! /usr/bin/env python

import numpy as np
from copy import deepcopy

class FIFO:
    '''1D FIFO array'''
    def __init__(self, size):
        self.size = size
        self.data = np.array([])

    def append(self, num):
        self.data = np.append(self.data, num)
        if self.data.size > self.size:
            self.data = self.data[1:]

class PointCluster:
    def __init__(self):
        self.clusters = []

    def distancePt(self, p1, p2):
        return np.sqrt(sum((p1 - p2) ** 2))

    def distanceAr(self, pt, array):
        if array.size > 2:
            return np.sqrt(np.sum((pt - array)**2 ,axis=1))
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


    def clusterDistance(self, c1, c2):
        '''minimum distance between two group of 2D points'''
        mindis = np.inf
        for p in c1:
            dis = np.min(self.distanceAr(p, c2))
            if dis < mindis:
                mindis = dis

        return mindis


class Cluster:
    def __init__(self, array):
        self.array = np.matrix(array)
        self.genAttr()

    def genAttr(self):
        self.distance = np.sqrt(np.sum(np.multiply(self.array, self.array), axis=1))
        self.meanDis = np.mean(self.distance)
        self.mean = np.mean(self.array, axis=0)
        self.maxX = np.max(self.array[:, 0])
        self.maxY = np.max(self.array[:, 1])
        self.minX = np.min(self.array[:, 0])
        self.minY = np.min(self.array[:, 1])
        self.sizeX = self.maxX - self.minX
        self.sizeY = self.maxY - self.minY
        self.centerX = (self.maxX + self.minX) / 2
        self.centerY = (self.maxY + self.minY) / 2
        self.right = self.array[np.argmax(self.array[:, 0])]
        self.left = self.array[np.argmin(self.array[:, 0])]
        self.top = self.array[np.argmax(self.array[:, 1])]
        self.bottom = self.array[np.argmin(self.array[:, 1])]


    def calcArea(self):
        pass