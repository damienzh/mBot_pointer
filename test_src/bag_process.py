#! /usr/bin/env python

import os
import rosbag
import numpy as np
import matplotlib.pyplot as plt

count2dis = 0.00035
wheelbase = 0.145


class ImuData:
    def __init__(self, filename):
        _, tail = os.path.split(filename)
        self.name = tail.split('.')[0]

        bag = rosbag.Bag(filename)
        self.data = np.array([])
        for msg in bag.read_messages(['/imu_raw']):
            t = msg.message.header.stamp.to_sec()
            acX = msg.message.linear_acceleration.x
            acY = msg.message.linear_acceleration.y
            acZ = msg.message.linear_acceleration.z
            gyX = msg.message.angular_velocity.x
            gyY = msg.message.angular_velocity.y
            gyZ = msg.message.angular_velocity.z
            self.data = np.append(self.data, [t, acX, acY, acZ, gyX, gyY, gyZ])
        bag.close()

        self.data = self.data.reshape(-1, 7)
        self.count = self.getCount()

    def calcVelocity(self):
        dt = np.diff(self.data[:, 0], axis=0)
        dvx = self.data[1:-1, 1] * dt
        dvy = self.data[1:-1, 2] * dt


    def getCount(self):
        return self.data.shape[0]

    def plotAcc(self):
        plt.figure()
        plt.subplot(311)
        plt.plot(self.data[:,1])
        plt.title('Acc X')

        plt.subplot(312)
        plt.plot(self.data[:,2])
        plt.title('Acc Y')

        plt.subplot(313)
        plt.plot(self.data[:,3])
        plt.title('Acc Z')

    def plotGyro(self):
        plt.figure()
        plt.subplot(311)
        plt.plot(self.data[:,4])
        plt.title('Gyro X')

        plt.subplot(312)
        plt.plot(self.data[:,5])
        plt.title('Gyro Y')

        plt.subplot(313)
        plt.plot(self.data[:,6])
        plt.title('Gyro Z')

    def plotPlanar(self):
        plt.figure()
        plt.subplot(311)
        plt.plot(self.data[:, 1])
        plt.title('Acc X')

        plt.subplot(312)
        plt.plot(self.data[:, 2])
        plt.title('Acc Y')

        plt.subplot(313)
        plt.plot(self.data[:, 6])
        plt.title('Gyro Z')


class EncData:
    def __init__(self, filename):
        _, tail = os.path.split(filename)
        self.name = tail.split('.')[0]

        bag = rosbag.Bag(filename)
        '''data = [timestamp, left count, right count]'''
        self.data_l = np.array([], dtype=np.int8)
        for msg in bag.read_messages(['/encoder_l']):
            t = msg.timestamp.to_sec()
            self.data_l = np.append(self.data_l, [t,msg.message.data])
        self.data_r = np.array([], dtype=np.int8)
        for msg in bag.read_messages(['/encoder_r']):
            self.data_r = np.append(self.data_r, [msg.message.data])
        self.data = np.hstack((self.data_l.reshape([-1,2]),self.data_r.reshape([-1,1])))
        bag.close()

        self.count = self.data.shape[0]
        self.calcVelocity()
        self.calcDistance()

    def calcVelocity(self):
        self.dt = np.diff(self.data[:,0], axis=0)
        vl = self.data[1:, 1] * count2dis / self.dt
        vr = self.data[1:, 2] * count2dis / self.dt
        linear = (vl + vr) / 2
        angular = (vr - vl) / wheelbase
        self.velocity = np.vstack((linear, angular)).transpose()

    def calcDistance(self):
        self.distance = np.sum(self.data[:,1:], axis=0) * count2dis / 2

    def plotVelocity(self):
        plt.figure()
        plt.subplot(211)
        plt.plot(self.velocity[:,0])
        plt.title('Linear Velocity')

        plt.subplot(212)
        plt.plot(self.velocity[:,1])
        plt.title('Angular Velocity')


if __name__ == '__main__':
    pass