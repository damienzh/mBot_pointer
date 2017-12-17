#! /usr/bin/env python

import numpy as np

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
        self.array = np.matrix(array)
        self.genAttr()

    def genAttr(self):
        self.distance = np.sqrt(np.sum(np.multiply(self.array, self.array), axis=1))
        self.meanDis = np.mean(self.distance)
        self.mean = np.mean(self.array, axis=0)

    def calcArea(self):
        maxX = self.array[np.argmax(self.array[:, 0])]
        minX = self.array[np.argmin(self.array[:, 0])]
        maxY = self.array[np.argmax(self.array[:, 1])]
        minY = self.array[np.argmin(self.array[:, 1])]