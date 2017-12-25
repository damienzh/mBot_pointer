#! /usr/bin/env python

import os
from copy import deepcopy
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from src.config import count2dis, wheelbase

bag_path = '/home/k/catkin_ws/src/mbot_pointer/rosbag/'

class ImuData:
    def __init__(self, filename):
        f = bag_path + filename
        self.name = filename.split('.')[0]

        bag = rosbag.Bag(f)
        self.data = np.array([])
        self.topics = bag.get_type_and_topic_info()[1].keys()
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

    def calcRotation(self):
        dt = np.diff(self.data[:, 0])
        self.rotation = self.data[1:,6] * dt
        self.angle_rad = np.sum(self.rotation)
        self.angle_deg = self.angle_rad * 180 / 3.1415926

    def getCount(self):
        return self.data.shape[0]

    def plotAcc(self):
        plt.figure()
        plt.subplot(311)
        plt.plot(self.data[:,1])
        plt.title('Acc X')
        plt.xlabel('Samples')
        plt.ylabel('m/s^2')

        plt.subplot(312)
        plt.plot(self.data[:,2])
        plt.title('Acc Y')
        plt.xlabel('Samples')
        plt.ylabel('m/s^2')

        plt.subplot(313)
        plt.plot(self.data[:,3])
        plt.title('Acc Z')
        plt.xlabel('Samples')
        plt.ylabel('m/s^2')

        plt.tight_layout()

    def plotGyro(self):
        plt.figure()
        plt.subplot(311)
        plt.plot(self.data[:,4])
        plt.title('Gyro X')
        plt.xlabel('Samples')
        plt.ylabel('rad/s')

        plt.subplot(312)
        plt.plot(self.data[:,5])
        plt.title('Gyro Y')
        plt.xlabel('Samples')
        plt.ylabel('rad/s')

        plt.subplot(313)
        plt.plot(self.data[:,6])
        plt.title('Gyro Z')
        plt.xlabel('Samples')
        plt.ylabel('rad/s')

        plt.tight_layout()

    def plotPlanar(self):
        plt.figure()
        plt.subplot(311)
        plt.plot(self.data[:, 1])
        plt.title('Acc X')
        plt.xlabel('Samples')
        plt.ylabel('m/s^2')

        plt.subplot(312)
        plt.plot(self.data[:, 2])
        plt.title('Acc Y')
        plt.xlabel('Samples')
        plt.ylabel('m/s^2')

        plt.subplot(313)
        plt.plot(self.data[:, 6])
        plt.title('Gyro Z')
        plt.xlabel('Samples')
        plt.ylabel('rad/s')

        plt.tight_layout()

    def calcCovariance(self):
        d = self.data[:, 1:]
        acc = d[:, 0:3]
        gy = d[:, 3:6]
        self.acc_cov = np.cov(acc.transpose())
        self.gy_cov = np.cov(gy.transpose())

        return np.cov(d.transpose())


class EncData:
    def __init__(self, filename):
        self.path = bag_path + filename
        self.name = filename.split('.')[0]

        bag = rosbag.Bag(self.path)
        '''data = [timestamp, left count, right count]'''
        self.topics = bag.get_type_and_topic_info()[1].keys()
        self.data = np.array([], dtype=np.int8)
        for msg in bag.read_messages(['/encoder']):
            t = msg.message.header.stamp.to_sec()
            countl = msg.message.left
            countr = msg.message.right
            self.data = np.append(self.data, [t, countl, countr])
        bag.close()
        self.data = self.data.reshape(-1, 3)

        self.count = self.data.shape[0]
        self.calcVelocity()
        self.calcDistance()
        self.calcRotation()

    def calcVelocity(self):
        self.dt = np.diff(self.data[:,0], axis=0)
        vl = self.data[1:, 1] * count2dis / self.dt
        vr = self.data[1:, 2] * count2dis / self.dt
        linear = (vl + vr) / 2
        angular = (vr - vl) / wheelbase
        self.velocity = np.vstack((linear, angular)).transpose()
        #print(wheelbase)

    def calcDistance(self):
        self.distance = np.sum(self.data[:,1:], axis=0) * count2dis
        self.distance_total = np.sum(self.distance) / 2

    def plotVelocity(self):
        plt.figure()
        plt.subplot(211)
        plt.plot(self.velocity[:,0])
        plt.title('Linear Velocity')

        plt.subplot(212)
        plt.plot(self.velocity[:,1])
        plt.title('Angular Velocity')

    def calcRotation(self):
        self.rotation = np.arctan2((self.data[:, 2]-self.data[:, 1])*count2dis, wheelbase)
        self.angle_rad = np.sum(self.rotation)
        self.angle_deg = self.angle_rad * 180 / 3.1415926

    def debug_rpm(self):
        bag = rosbag.Bag(self.path)
        self.rpm_data = np.array([], dtype=np.int16)
        for msg in bag.read_messages(['/motor_rpm']):
            t = msg.message.header.stamp.to_sec()
            rpm_l = msg.message.left
            rpm_r = -msg.message.right
            self.rpm_data = np.append(self.rpm_data, [t, rpm_l, rpm_r])
        bag.close()
        self.rpm_data = self.rpm_data.reshape(-1, 3)
        self.rpm_count_l = self.data[1:, 1] * (60/self.dt) / 360
        self.rpm_count_r = self.data[1:, 2] * (60/self.dt) / 360

        plt.figure()
        plt.subplot(211)
        plt.plot(self.rpm_data[1:, 1])
        plt.plot(self.rpm_count_l)
        plt.plot((self.rpm_count_l - self.rpm_data[1:, 1]))
        plt.legend(['rpm_l', 'count_l', 'diff'])
        plt.subplot(212)
        plt.plot(self.rpm_data[1:, 2])
        plt.plot(self.rpm_count_r)
        plt.plot(self.rpm_count_r - self.rpm_data[1:, 2])
        plt.legend(['rpm_r', 'count_r', 'diff'])

    def load_rpm_cmd(self):
        bag = rosbag.Bag(self.path)
        self.cmd_rpm = np.array([])
        for msg in bag.read_messages(['/cmd_vel_rpm']):
            t = msg.message.header.stamp.to_sec()
            cmd_l = msg.message.left
            cmd_r = msg.message.right
            self.cmd_rpm = np.append(self.cmd_rpm, [t, cmd_l, cmd_r])
        bag.close()
        self.cmd_rpm = self.cmd_rpm.reshape(-1, 3)


class OdomData:
    def __init__(self, filename):
        self.path = bag_path + filename
        self.name = filename.split('.')[0]

        bag = rosbag.Bag(self.path)
        self.poses = np.array([])
        self.vel = np.array([])
        self.orientation = np.array([])
        topics = bag.get_type_and_topic_info()[1].keys()
        topic_name = filter(lambda t: 'odom' in t, topics)
        for msg in bag.read_messages(topic_name):
            t = msg.message.header.stamp.to_sec()
            x = msg.message.pose.pose.position.x
            y = msg.message.pose.pose.position.y
            vx = msg.message.twist.twist.linear.x
            vy = msg.message.twist.twist.linear.y
            w = msg.message.twist.twist.angular.z
            ax = msg.message.pose.pose.orientation.x
            by = msg.message.pose.pose.orientation.y
            cz = msg.message.pose.pose.orientation.z
            dw = msg.message.pose.pose.orientation.w
            self.poses = np.append(self.poses, [t, x, y])
            self.vel = np.append(self.vel, [t, vx, vy, w])
            self.orientation = np.append(self.orientation, [t, ax, by, cz, dw])
        bag.close()
        self.poses = self.poses.reshape(-1, 3)
        self.vel = self.vel.reshape(-1, 4)
        self.orientation = self.orientation.reshape(-1, 5)

    def plotOdom(self):
        x_limit = round(max(self.poses[:,1]))
        plt.figure()
        plt.plot(self.poses[:,1], self.poses[:,2])
        plt.ylim(-3,3)


def matchCmd(data, cmd):
    match_data = np.zeros([data.shape[0], 5])
    i = 0
    cur_cmd = np.array([0, 0])
    for j in range(data.shape[0]):
        match_data[j,:3] = data[j]
        if i < cmd.shape[0]:
            if cmd[i, 0] < data[j,0]:
                cur_cmd = cmd[i, 1:]
                #print cur_cmd
                match_data[j-1, 3:] = deepcopy(cur_cmd)
                i += 1
        match_data[j, 3:] = deepcopy(cur_cmd)

    return match_data


if __name__ == '__main__':
    #f = '../rosbag/encoder.bag'
    #e = EncData(f)
    f1 = '/home/k/catkin_ws/src/mbot_pointer/rosbag/rpm_data.csv'
    f2 = '/home/k/catkin_ws/src/mbot_pointer/rosbag/rpm_cmd_data.csv'
    cmd = np.loadtxt(f2, delimiter=',', skiprows=1, usecols=(0,2,3))
    data = np.loadtxt(f1, delimiter=',', skiprows=1, usecols=(0,4,5))
    match = matchCmd(data, cmd)
    plt.plot(match[:,3])
    plt.show()