#! /usr/bin/env python

import math
import rospy
import actionlib
import tf2_ros
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback

class TeleMoveBase:
    def __init__(self):
        self.arrived = False
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
        goalPose = self.extractPose(goal)
        while not self.arrived:
            distance = self.calcDistance((self.x, self.y),(goalPose[0], goalPose[1]))
            angle = goalPose[2] - self.theta

            self.rate.sleep()

    def calcDistance(self, robot, goal):
        return math.sqrt((robot[0]-goal[0])**2 + (robot[1]-goal[1])**2)

    def extractPose(self, goal):
        x = goal.transform.translation.x
        y = goal.transform.translation.y
        yaw = euler_from_quaternion((goal.transform.roation.x, goal.transform.roation.y,
                                    goal.transform.roation.z, goal.transform.roation.w))[2]

        return (x,y,yaw)
