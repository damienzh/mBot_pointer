#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
import tf

def callback(msg):
    msg.header.frame_id = 'imu'
    imu_pub.publish(msg)
    tf_pub.sendTransform((0,0,0),
                         (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
                         rospy.Time.now(),'imu','map')

if __name__ == '__main__':
    rospy.init_node('imu_repub')

    tf_pub = tf.TransformBroadcaster()

    imu_pub = rospy.Publisher('/imu/data_repub', Imu, queue_size=1)
    imu_sub = rospy.Subscriber('/imu/data', Imu, callback)

    rospy.spin()

