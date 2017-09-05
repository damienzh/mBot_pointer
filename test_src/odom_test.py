#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import numpy as np

distance_init = 0
distance_current = 0
imu_data = np.zeros((1, 6))
motor_left = []
motor_right = []

def range_callback(msg, vel):
    global distance_init

    vl = vel[0]
    vr = vel[1]

    if distance_init == 0:
        distance_init = msg.range
        print distance_init
        rospy.sleep(100)
        vel_pub_l.publish(vl)
        vel_pub_r.publish(vr)

    distance_current = msg.range

    if distance_init - distance_current > 2 or distance_current < 0.3:
        vel_pub_l.publish(0)
        vel_pub_r.publish(0)

def rpm_cmd(s):
    pass

def rpm_callback(msg):
    pass

def imu_callback(msg):
    global imu_data

    buff = np.zeros((1, 6))
    buff[0] = msg.linear_acceleration.x
    buff[1] = msg.linear_acceleration.y
    buff[2] = msg.linear_acceleration.z
    buff[3] = msg.angular_velocity.x
    buff[4] = msg.angular_velocity.y
    buff[5] = msg.angular_velocity.z

    imu_data = np.vstack((imu_data, buff))


if __name__ == '__main__':
    s_left = input('target left rpm (0~200):')
    s_right = input('target right rpm (0~200):')

    rospy.init_node('odom_test')
    vel_pub_l = rospy.Publisher('cmd_vel_l', Int16, queue_size=1)
    vel_pub_r = rospy.Publisher('cmd_vel_r', Int16, queue_size=1)
    range_sub = rospy.Subscriber('terarangerone', Range, range_callback, (s_left, s_right))
    encoder_left_sub = rospy.Subscriber('left_rpm', Int16, rpm_callback)
    encoder_right_sub = rospy.Subscriber('right_rpm', Int16, rpm_callback)
    imu_sub = rospy.Subscriber('imu_raw', Imu, imu_callback)


    rospy.spin()