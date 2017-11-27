#! /usr/bin/env python

import rospy
import tf2_ros
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
import math

base_frame_id = 'base_link'
imu_frame_id = 'imu'
xtion_frame_id = 'camera_link'


if __name__ == '__main__':
    rospy.init_node('static_tf_broadcaster')
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    now = rospy.Time.now()
    imu_static = TransformStamped()
    xtion_static = TransformStamped()
    imu_static.header.stamp = now
    imu_static.header.frame_id = base_frame_id
    imu_static.child_frame_id = imu_frame_id
    q = quaternion_from_euler(math.pi, 0, 0)
    imu_static.transform.rotation.x = q[0]
    imu_static.transform.rotation.y = q[1]
    imu_static.transform.rotation.z = q[2]
    imu_static.transform.rotation.w = q[3]
    broadcaster.sendTransform(imu_static)

    xtion_static.header.stamp = now
    xtion_static.header.frame_id = base_frame_id
    xtion_static.child_frame_id = xtion_frame_id
    xtion_static.transform.translation.x = 0.068
    broadcaster.sendTransform(xtion_static)

    rospy.spin()