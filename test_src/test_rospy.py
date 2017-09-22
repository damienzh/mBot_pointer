#! /usr/bin/env python

import rospy

rospy.init_node('test_rospy', anonymous=True)

now1 = rospy.get_time()
now2 = rospy.Time.now()
now2 = now2.to_sec()

print 'get_time', now1
print 'Time.now', now2