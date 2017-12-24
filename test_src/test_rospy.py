#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32, String
from mbot_pointer.srv import Twist2cmd
from dynamic_reconfigure.client import Client, DoubleParameter
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from geometry_msgs.msg import TransformStamped
import tf2_ros, tf
import numpy as np
import random

def test_param():
    p = rospy.get_param('~pid_p', default=10.2)
    i = rospy.get_param('~pid_i', default=0.5)
    d = rospy.get_param('~pid_d', default=0.1)

    return p+i+d

def test_main():
    rate = rospy.Rate(30)
    test_pub = rospy.Publisher('test_pub', Float32, queue_size=1)
    t2c_srv = rospy.ServiceProxy('twist2pwm', Twist2cmd)
    t2c_srv.wait_for_service()

    while not rospy.is_shutdown():
        f = Float32()
        f.data = test_param()
        test_pub.publish(f)

        rate.sleep()

def testPCL():
    topic = rospy.get_param('PointCloudTopic')
    rospy.Subscriber(topic, PointCloud2, pcl_callback)

def pcl_callback(msg):
    start = rospy.Time.now()
    p = np.array([])
    for pts in point_cloud2.read_points(msg, skip_nans=False):
        p = np.append(p, pts[0:3])
    p = p.reshape(-1, 3)
    end = rospy.Time.now()
    d = (start - end).to_sec()
    print(d)

def testTF():
    b = tf2_ros.Buffer()
    tf2_ros.TransformListener(b)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            trans = b.lookup_transform('base_link', 'camera_link', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        print(trans)

class tfBrocasters:
    def __init__(self):
        self.br = tf2_ros.TransformBroadcaster()
        self.frames = ['frame1', 'frame2', 'frame3']
        self.rate = rospy.Rate(1)
        rospy.Subscriber('add_frame', String, self.addframe)

    def addframe(self, msg):
        self.frames.append(msg.data)

    def genMsg(self):
        t = TransformStamped()
        for f in self.frames:
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = 'base_link'
            t.child_frame_id = f
            t.transform.translation.x = random.randint(0,10)
            t.transform.rotation.w = 1
            self.br.sendTransform(t)
            self.rate.sleep()

    def update(self):
        while not rospy.is_shutdown():
            self.genMsg()

if __name__ == '__main__':
    rospy.init_node('test_rospy')
    b = tfBrocasters()
    b.update()
    rospy.spin()