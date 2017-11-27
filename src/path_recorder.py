#! /usr/bin/env python
import numpy as np
import rospy
import tf2_ros


class PathRecorder:

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.path = np.array([])
        self.tl = tf2_ros.TransformListener(self.tf_buffer)

    def record(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            try:
                trans = self.tf_buffer.lookupTransform('/map', '/base_link', rospy.Time())
                x = trans.transform.tranlation.x
                y = trans.transform.tranlation.y
                self.path = np.append(self.path, [x, y])
            except tf2_ros.LookupExpection:
                continue

            r.sleep()

if __name__ == '__main__':
    rospy.init_node('path_recorder')
    r = PathRecorder()
    r.record()