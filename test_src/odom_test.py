#! /usr/bin/env python

import rospy
from tf.broadcaster import TransformBroadcaster
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16
from math import pi, atan2, sin, cos

count2dis = 0.035
wheelbase = 0.145

base_frame_id = '/base_link'
odom_frame_id = 'odom'


class ImuOdom:
    def __init__(self):
        rospy.init_node('imu_odom')

        self.x = 0
        self.y = 0
        self.th = 0
        self.v = 0
        self.w = 0

        self.then = rospy.Time.now()
        self.dt = 0

        rospy.Subscriber('imu_raw', Imu, self.imu_callback)
        self.odom_pub = rospy.Publisher('imu_odom', Odometry, queue_size=5)

    def imu_callback(self,msg):
        
        pass

    def update(self):
        pass

    def spin(self):
        while not rospy.is_shutdown():
            self.update()


class EncOdom:
    def __init__(self):
        rospy.init_node('enc_odom')

        self.x = 0      # X position in planar ground
        self.y = 0      # Y position in planar ground
        self.th = 0     # direction in planar ground
        self.v = 0      # linear velocity
        self.w = 0      # angular velocity

        self.lastTime = rospy.Time.now()
        self.dt = 0

        self.disL = 0
        self.disR = 0
        self.timeL = 0
        self.timeR = 0

        rospy.Subscriber('encoder_l', Int16, self.enc_L_callback)
        rospy.Subscriber('encoder_r', Int16, self.enc_R_callback)
        self.odom_pub = rospy.Publisher('enc_odom', Odometry, queue_size=5)
        self.odomBroadcaster = TransformBroadcaster()

    def enc_L_callback(self,msg):
        self.disL = msg.data * count2dis

    def enc_R_callback(self,msg):
        self.disR = msg.data * count2dis

    def update(self):
        now = rospy.Time.now()
        if now > self.lastTime:
            self.dt = (now - self.lastTime).to_sec()
            trv_dis = (self.disL + self.disR) / 2
            trv_rot = atan2((self.disR - self.disL), wheelbase)
            self.x = self.x + trv_dis * sin(trv_rot + self.th)
            self.y = self.y + trv_dis * cos(trv_rot + self.th)
            self.th = self.th + trv_rot

            self.v = trv_dis / self.dt
            self.w = trv_rot / self.dt

        quaternion = Quaternion()
        quaternion.w = cos(self.th / 2)
        quaternion.z = sin(self.th / 2)

        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = odom_frame_id
        odom_msg.child_frame_id = base_frame_id
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation = quaternion
        odom_msg.twist.twist.linear.x = self.v
        odom_msg.twist.twist.angular.z = self.w

        self.odom_pub.publish(odom_msg)
        self.odomBroadcaster.sendTransform((self.x, self.y, 0),
                                           quaternion_from_euler(0, 0, self.th),
                                           now,
                                           base_frame_id,
                                           odom_frame_id)

        self.lastTime = now

    def spin(self):
        while not rospy.is_shutdown():
            self.update()

if __name__ == '__main__':
    try:
        enc_odom = EncOdom()
        enc_odom.spin()
    except rospy.ROSInterruptException:
        pass
