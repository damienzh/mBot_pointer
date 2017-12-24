#! /usr/bin/env python
import rospy
import tf2_ros
from tf_conversions import posemath
from geometry_msgs.msg import TransformStamped, Pose

class tf_broadcaster:
    def __init__(self):
        self.rate = rospy.Rate(10)
        self.buffer = tf2_ros.Buffer()
        self.br = tf2_ros.TransformBroadcaster()
        tf2_ros.TransformListener(self.buffer)
        self.frames = []
        self.rate.sleep()
        rospy.Subscriber('add_frame', TransformStamped, self.add_frame)

    def add_frame(self, msg):
        t = self.transform('odom', msg)
        self.frames.append(t)

    def transform(self, target_frame, msg):
        frame_id = msg.header.frame_id
        while not self.buffer.can_transform(target_frame, frame_id, rospy.Time()):
            rospy.loginfo('waiting transformation from %s to %s', target_frame, frame_id)
            self.rate.sleep()
        trans = self.buffer.lookup_transform(target_frame, frame_id, rospy.Time())
        f1 = self.from_msg(trans) # transform from target frame to middle frame
        f2 = self.from_msg(msg) # transform from middle frame to object frame
        f = f1 * f2
        t = self.to_msg(f)
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = target_frame
        t.child_frame_id = msg.child_frame_id

        return t

    def from_msg(self, transform_msg):
        p = Pose()
        p.position = transform_msg.transform.translation
        p.orientation = transform_msg.transform.rotation

        return posemath.fromMsg(p)

    def to_msg(self, KDLframe):
        p = posemath.toMsg(KDLframe)
        msg = TransformStamped()
        msg.transform.translation = p.position
        msg.transform.rotation = p.orientation
        return msg


    def pub(self):
        while not rospy.is_shutdown():
            for f in self.frames:
                f.header.stamp = rospy.Time.now()
                self.br.sendTransform(f)
                self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    b = tf_broadcaster()
    b.pub()