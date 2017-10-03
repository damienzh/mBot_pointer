#! /usr/bin/env python

import rospy
import tf
from tf.transformations import quaternion_from_euler
import math

base_frame_id = 'base_link'
imu_frame_id = 'imu'
xtion_frame_id = 'camera_link'


if __name__ == '__main__':
    rospy.init_node('static_tf_broadcaster')
    broadcaster = tf.TransformBroadcaster()
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        broadcaster.sendTransform((0, 0, 0),
                                  quaternion_from_euler(math.pi, 0, 0),
                                  now, imu_frame_id, base_frame_id)
        broadcaster.sendTransform((0.068, 0, 0),
                                  quaternion_from_euler(0, 0, 0),
                                  now, xtion_frame_id, base_frame_id)
        rate.sleep()