#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf


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

        rospy.Subscriber("/wheel_left", Float32, self.left_callback)
        rospy.Subscriber("/wheel_right", Float32, self.right_callback)

        # IMPORTANT: publish as sensor odometry (not final odom)
        self.odom_pub = rospy.Publisher(
            "/wheel_odom",
            Odometry,
            queue_size=20
        )

        rospy.loginfo("Wheel odometry node started (EKF-ready)")

    def left_callback(self, msg):
        self.left_rps = msg.data

    def right_callback(self, msg):
        self.right_rps = msg.data

    def update(self):
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        self.last_time = now

        if dt <= 0.0:
            return

        # wheel velocities (m/s)
        left_v = 2.0 * math.pi * self.wheel_radius * self.left_rps
        right_v = 2.0 * math.pi * self.wheel_radius * self.right_rps

        v = (left_v + right_v) / 2.0
        w = (right_v - left_v) / self.wheel_base

        # deadband (removes jitter drift)
        if abs(v) < 1e-4:
            v = 0.0
        if abs(w) < 1e-4:
            w = 0.0

        # integrate pose
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt

        q = tf.transformations.quaternion_from_euler(0, 0, self.theta)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = Quaternion(*q)

        # velocity
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        # IMPORTANT: covariance (needed for EKF)
        odom.pose.covariance = [
            0.05, 0,    0,    0,    0,    0,
            0,    0.05, 0,    0,    0,    0,
            0,    0,    99999,0,    0,    0,
            0,    0,    0,    99999,0,    0,
            0,    0,    0,    0,    99999,0,
            0,    0,    0,    0,    0,    0.2
        ]

        odom.twist.covariance = [
            0.02, 0,    0,    0,    0,    0,
            0,    0.02, 0,    0,    0,    0,
            0,    0,    99999,0,    0,    0,
            0,    0,    0,    99999,0,    0,
            0,    0,    0,    0,    99999,0,
            0,    0,    0,    0,    0,    0.1
        ]

        self.odom_pub.publish(odom)


if __name__ == "__main__":
    node = OdomNode()
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        node.update()
        rate.sleep()
