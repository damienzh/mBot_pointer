#! /usr/bin/env python

import rospy
import actionlib
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback

class TeleMoveBase:
    def __init__(self):
        self.rate = rospy.Rate(1)
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.move_server = actionlib.SimpleActionServer('teleMoveBase', MoveBaseAction, self.move_base)
        rospy.Subscriber('/mbot_odom', Odometry, self.odom_callback)


    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.theta = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    def move_base(self, goal):
        pass