#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
import numpy as np

rospy.init_node('imu_process', anonymous=True)


#buff_ax = 0; buff_ay = 0; buff_az = 0; buff_gx = 0; buff_gy = 0; buff_gz = 0
buff = np.zeros((1000, 6))

for i in range(1000):
    imu_data = rospy.wait_for_message('imu_raw', Imu)
    print 'receive ',i,' message'
    buff[i, 0] = imu_data.linear_acceleration.x
    buff[i, 1] = imu_data.linear_acceleration.y
    buff[i, 2] = imu_data.linear_acceleration.z
    buff[i, 3] = imu_data.angular_velocity.x
    buff[i, 4] = imu_data.angular_velocity.y
    buff[i, 5] = imu_data.angular_velocity.z
    rospy.sleep(0.2)




np.savetxt('MPU_data.csv', buff, delimiter=",")