#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
import numpy as np

# rospy.init_node('imu_process', anonymous=True)
#
#
# #buff_ax = 0; buff_ay = 0; buff_az = 0; buff_gx = 0; buff_gy = 0; buff_gz = 0
# buff = np.zeros((1000, 6))
#
# for i in range(1000):
#     imu_data = rospy.wait_for_message('imu_raw', Imu)
#     print 'receive ',i,' message'
#     buff[i, 0] = imu_data.linear_acceleration.x
#     buff[i, 1] = imu_data.linear_acceleration.y
#     buff[i, 2] = imu_data.linear_acceleration.z
#     buff[i, 3] = imu_data.angular_velocity.x
#     buff[i, 4] = imu_data.angular_velocity.y
#     buff[i, 5] = imu_data.angular_velocity.z
#     rospy.sleep(0.2)
#
# np.savetxt('MPU_data.csv', buff, delimiter=",")

#global imu_data

imu_data = np.zeros((1,6))

def imu_process(msg):
    global imu_data
    buff = np.zeros((1,6))
    buff[0,0] = msg.linear_acceleration.x
    buff[0,1] = msg.linear_acceleration.y
    buff[0,2] = msg.linear_acceleration.z
    buff[0,3] = msg.angular_velocity.x
    buff[0,4] = msg.angular_velocity.y
    buff[0,5] = msg.angular_velocity.z

    if np.shape(imu_data)[0] < 1000:
        imu_data = np.vstack((imu_data,buff))
    else:
        imu_data = np.vstack((imu_data,buff))
        np.delete(imu_data,0,0)

    std = np.std(imu_data, axis=0)
    mean = np.mean(imu_data, axis=0)

    new_msg = Imu()
    new_msg.linear_acceleration.x = msg.linear_acceleration.x - mean[0]
    new_msg.linear_acceleration.y = msg.linear_acceleration.y - mean[1]
    new_msg.linear_acceleration.z = msg.linear_acceleration.z - mean[2]
    new_msg.angular_velocity.x = msg.angular_velocity.x - mean[3]
    new_msg.angular_velocity.y = msg.angular_velocity.y - mean[4]
    new_msg.angular_velocity.z = msg.angular_velocity.z - mean[5]

    new_msg.linear_acceleration_covariance[0] = std[0]
    new_msg.linear_acceleration_covariance[4] = std[1]
    new_msg.linear_acceleration_covariance[8] = std[2]
    new_msg.angular_velocity_covariance[0] = std[3]
    new_msg.angular_velocity_covariance[4] = std[4]
    new_msg.angular_velocity_covariance[8] = std[5]

    new_msg.header.stamp = msg.header.stamp
    new_msg.header.frame_id = 'imu'
    imu_pub.publish(new_msg)


if __name__ == '__main__':
    rospy.init_node('imu_process')
    imu_pub = rospy.Publisher('imu_new', Imu, queue_size=1)
    imu_sub = rospy.Subscriber('imu_raw', Imu, imu_process)

    rospy.spin()