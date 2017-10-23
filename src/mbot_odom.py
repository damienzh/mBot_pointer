#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from mbot_pointer.msg import Encoder
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from tf.broadcaster import TransformBroadcaster
import math

from config import wheelbase, count2dis

class MbotOdom:
    def __init__(self):
        '''x,y,theta,dx,dy states in world frame'''
        self.x = 0
        self.y = 0
        self.theta = 0
        self.dx = 0
        self.dy = 0
        '''v,omega,acc, speed and acceleration in robot frame'''
        self.v = 0
        self.omega = 0
        self.acc_x = 0
        self.acc_y = 0
        self.dt = 0

        self.new_msg_time = 0
        self.prev_msg_time = 0
        self.dis_left = 0
        self.dis_right = 0
        self.trv_dis = 0
        self.trv_rot = 0

        self.odom_frame_id = 'odom'
        self.base_frame_id = 'base_link'

        self.odom_pub = rospy.Publisher('mbot_odom', Odometry, queue_size=5)
        self.odomBroadcaster = TransformBroadcaster()

        rospy.Subscriber('/imu_raw', Imu, self.imu_callback)
        rospy.Subscriber('/encoder', Encoder, self.enc_callback)

    def imu_callback(self, msg):
        self.omega = -msg.angular_velocity.z
        self.acc_x = msg.linear_acceleration.x
        self.acc_y = -msg.linear_acceleration.y

    def enc_callback(self, msg):
        self.new_msg_time = msg.header.stamp.to_sec()
        self.dis_left = msg.left * count2dis
        self.dis_right = msg.right * count2dis
        self.trv_dis = (self.dis_left + self.dis_right) / 2

        self.publish()

    def update(self):
        if self.new_msg_time > self.prev_msg_time :
            self.dt = self.new_msg_time - self.prev_msg_time
            self.v = self.trv_dis / self.dt
            self.trv_rot = self.omega * self.dt
            self.dx = self.v * math.cos(self.theta)
            self.dy = self.v * math.sin(self.theta)
            self.x = self.x + self.trv_dis * math.cos(self.trv_rot + self.theta)
            self.y = self.y + self.trv_dis * math.sin(self.trv_rot + self.theta)
            self.complementary()

            self.prev_msg_time = self.new_msg_time

    def publish(self):
        self.update()

        now = rospy.Time.now()
        q = Quaternion()
        q.z = math.sin(self.theta / 2)
        q.w = math.cos(self.theta / 2)

        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation = q
        odom_msg.twist.twist.linear.x = self.dx
        odom_msg.twist.twist.linear.y = self.dy
        odom_msg.twist.twist.angular.z = self.omega

        self.odom_pub.publish(odom_msg)
        self.odomBroadcaster.sendTransform((self.x, self.y, 0),
                                           quaternion_from_euler(0, 0, self.theta),
                                           now, self.base_frame_id, self.odom_frame_id)



    def complementary(self):
        alpha = 0.98
        enc = math.atan2(self.dis_right-self.dis_left, wheelbase)
        self.theta = alpha*(self.theta + self.omega * self.dt) + (1 - alpha)*enc

    def kalman(self):
        pass


if __name__ == '__main__':
    rospy.init_node('mbot_odom')
    odom = MbotOdom()
    rospy.spin()