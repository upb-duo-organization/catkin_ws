#!/usr/bin/env python

import rospy
import math
import tf

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


class OdomNode:

    def __init__(self):

        rospy.init_node("odom_node")

        self.wheel_base = rospy.get_param("~wheel_base", 0.18)
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.0325)

        self.left_rps = 0.0
        self.right_rps = 0.0

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = rospy.Time.now()

        rospy.Subscriber(
            "/wheel_left",
            Float32,
            self.left_callback
        )

        rospy.Subscriber(
            "/wheel_right",
            Float32,
            self.right_callback
        )

        self.odom_pub = rospy.Publisher(
            "/odom",
            Odometry,
            queue_size=20
        )

        self.br = tf.TransformBroadcaster()

        rospy.loginfo("Odometry node started")

    def left_callback(self, msg):
        self.left_rps = msg.data

    def right_callback(self, msg):
        self.right_rps = msg.data

    def update(self):

        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        self.last_time = now

        # convert RPS -> m/s
        left_v = 2 * math.pi * self.wheel_radius * self.left_rps
        right_v = 2 * math.pi * self.wheel_radius * self.right_rps

        v = (left_v + right_v) / 2.0
        w = (right_v - left_v) / self.wheel_base

        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt

        q = tf.transformations.quaternion_from_euler(
            0,
            0,
            self.theta
        )

        self.br.sendTransform(
            (self.x, self.y, 0),
            q,
            now,
            "base_link",
            "odom"
        )

        msg = Odometry()
        msg.header.stamp = now
        msg.header.frame_id = "odom"

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation = Quaternion(*q)

        msg.child_frame_id = "base_link"
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w

        self.odom_pub.publish(msg)


if __name__ == "__main__":
    node = OdomNode()
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        node.update()
        rate.sleep()
