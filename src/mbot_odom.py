#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from mbot_pointer.msg import Encoder
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from tf.broadcaster import TransformBroadcaster
import tf2_ros
from std_srvs.srv import Empty
import math
from copy import deepcopy

from config import wheelbase, count2dis

class mBotOdom:
    def __init__(self):
        '''x,y,theta,dx,dy states in world frame'''
        self.filter = rospy.get_param('~imu_filter', 'complementary')
        if self.filter == 'kalman':
            rospy.loginfo('use kalman filter')
        else:
            rospy.loginfo('use complementary filter')

        self.x = 0
        self.y = 0
        self.theta = 0
        self.dx = 0
        self.dy = 0
        self.dtheta = 0
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

        self.kalman_p = 0
        self.kalman_k = 0
        self.kalman_q = 0.05**2 + 3.1491719118731052e-06
        self.kalman_r = 3.1491719118731052e-06
        self.kalman_h = 1.0

        self.odom_frame_id = 'odom'
        self.base_frame_id = 'base_link'

        self.odom_pub = rospy.Publisher('mbot_odom', Odometry, queue_size=5)
        self.odomBroadcaster = tf2_ros.TransformBroadcaster()

        rospy.Subscriber('/imu_raw', Imu, self.imu_callback)
        rospy.Subscriber('/encoder', Encoder, self.enc_callback)
        rospy.Service('/reset_odom', Empty, self.reset_odom)

    def imu_callback(self, msg):
        self.gyro_z = -msg.angular_velocity.z
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
            #self.v = self.trv_dis / self.dt
            #self.dx = self.v * math.cos(self.theta)
            #self.dy = self.v * math.sin(self.theta)
            prev_theta = deepcopy(self.theta)
            '''fusion update theta'''
            if self.filter == 'kalman':
                self.kalman()
            else:
                self.complementary()

            self.dx = self.trv_dis * math.cos(self.theta)
            self.dy = self.trv_dis * math.sin(self.theta)
            self.dtheta = self.theta - prev_theta

            self.v = self.trv_dis / self.dt
            self.omega = self.dtheta / self.dt

            self.x = self.x + self.dx
            self.y = self.y + self.dy

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
        odom_msg.twist.twist.linear.x = self.v
        odom_msg.twist.twist.linear.y = 0
        odom_msg.twist.twist.angular.z = self.omega

        self.odom_pub.publish(odom_msg)
        self.broadcast_tf()

    def broadcast_tf(self):
        tf_msg = TransformStamped()
        now = rospy.Time.now()
        q = quaternion_from_euler(0, 0, self.theta)
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = self.odom_frame_id
        tf_msg.child_frame_id = self.base_frame_id
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.rotation.x = q[0]
        tf_msg.transform.rotation.y = q[1]
        tf_msg.transform.rotation.z = q[2]
        tf_msg.transform.rotation.w = q[3]

        self.odomBroadcaster.sendTransform(tf_msg)

    def complementary(self):
        alpha = 0.98
        enc = math.atan2(self.dis_right-self.dis_left, wheelbase)
        self.theta = alpha*(self.theta + self.gyro_z * self.dt) + (1 - alpha)*enc

    def kalman(self):
        enc = math.atan2(self.dis_right - self.dis_left, wheelbase)
        z = self.theta + enc
        '''predict'''
        theta = self.theta + self.gyro_z * self.dt
        p = self.kalman_p + self.kalman_q
        '''update'''
        self.kalman_k = p * self.kalman_h / (p * self.kalman_h**2 + self.kalman_r)
        self.theta = self.theta + self.kalman_k * (z - self.kalman_h * theta)
        self.kalman_p = (1 - self.kalman_p*self.kalman_h) * p

    def reset_odom(self, req):
        rospy.loginfo('reset odom')
        self.x = 0
        self.y = 0
        self.theta = 0
        self.dx = 0
        self.dy = 0
        self.v = 0
        self.omega = 0
        self.new_msg_time =0
        self.prev_msg_time = 0

if __name__ == '__main__':
    rospy.init_node('mbot_odom')
    odom = mBotOdom()
    rospy.spin()