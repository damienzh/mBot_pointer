#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from math import atan2

class imu_filter:
    def __init__(self):
        rospy.init_node('imu_filter_test')

        self.gyo_z = 0
        self.acc_x = 0
        self.acc_y = 0

        self.theta_1 = 0
        self.theta_2 = 0
        self.theta_3 = 0
        self.cur_msg_time = 0
        self.pre_msg_time = 0
        self.dt = 0

        rospy.Subscriber('imu_raw', Imu, self.callback)
        self.raw_pub = rospy.Publisher('raw_pub', Float32, queue_size=1)
        self.complementary_pub = rospy.Publisher('complementary_filter', Float32, queue_size=1)
        self.kalman_pub = rospy.Publisher('kalman_filter', Float32, queue_size=1)

    def callback(self, msg):
        self.cur_msg_time = msg.header.stamp.to_sec()
        self.gyo_z = msg.angular_velocity.z
        self.acc_x = msg.linear_acceleration.x
        self.acc_y = msg.linear_acceleration.y
        if self.pre_msg_time != 0:
            self.dt = self.cur_msg_time - self.pre_msg_time
            self.update()
        self.pre_msg_time = self.cur_msg_time

    def raw_output(self):
        msg = Float32()
        self.theta_1 = self.theta_1 + self.gyo_z * self.dt
        msg.data = self.theta_1
        self.raw_pub.publish(msg)

        return msg

    def complementary(self):
        msg = Float32()
        alpha = 0.99
        acc = atan2(self.acc_x, self.acc_y)
        self.theta_2 = alpha * (self.theta_2 + self.gyo_z * self.dt) + (1 - alpha) * acc
        msg.data = self.theta_2
        self.complementary_pub.publish(msg)

        return msg

    def kalman(self):
        msg = Float32()

        msg.data = self.theta_3
        self.kalman_pub.publish(msg)

        return msg

    def update(self):
        self.raw_output()
        self.complementary()
        self.kalman()

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    filter = imu_filter()
    filter.spin()
