#! /usr/bin/env python

import rosbag
import numpy as np
import matplotlib.pyplot as plt

class ImuData():
    def __init__(self, filename):
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
        self.data = self.data.reshape(-1,7)

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

class EncData():
    def __init__(self, filename):
        bag = rosbag.Bag(filename)
        self.data_l = np.array([], dtype=np.int8)
        for msg in bag.read_messages(['/encoder_l']):
            t = msg.timestamp.to_sec()
            self.data_l = np.append(self.data_l, [t,msg.message.data])
        self.data_r = np.array([], dtype=np.int8)
        for msg in bag.read_messages(['/encoder_r']):
            self.data_r = np.append(self.data_r, [msg.message.data])
        self.data = np.hstack((self.data_l.reshape([-1,2]),self.data_r.reshape([-1,1])))

if __name__ == '__main__':
    imu1 = ImuData('../rosbag/pwm50.bag')
    print imu1.getCount()