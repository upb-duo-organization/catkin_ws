#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


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

        rospy.Subscriber("/wheel_left", Float32, self.left_cb, queue_size=1)
        rospy.Subscriber("/wheel_right", Float32, self.right_cb, queue_size=1)

        self.pub = rospy.Publisher("/wheel_odom", Odometry, queue_size=10)

        rospy.loginfo("Wheel odometry node started (EKF-ready)")

    def left_cb(self, msg):
        self.left_rps = msg.data

    def right_cb(self, msg):
        self.right_rps = msg.data

    def update(self):
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        self.last_time = now

        if dt <= 0.0 or dt > 0.2:
            return

        # RPS -> rad/s -> m/s
        left_v = 2.0 * math.pi * self.wheel_radius * self.left_rps
        right_v = 2.0 * math.pi * self.wheel_radius * self.right_rps

        # integrate distance (more stable than velocity integration)
        dl = left_v * dt
        dr = right_v * dt

        d_center = (dr + dl) / 2.0
        d_theta = (dr - dl) / self.wheel_base

        theta_mid = self.theta + d_theta * 0.5

        self.x += d_center * math.cos(theta_mid)
        self.y += d_center * math.sin(theta_mid)
        self.theta = normalize_angle(self.theta + d_theta)

        q = tf.transformations.quaternion_from_euler(0, 0, self.theta)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = Quaternion(*q)

        odom.twist.twist.linear.x = d_center / dt
        odom.twist.twist.angular.z = d_theta / dt

        # IMPORTANT: realistic covariance (this is critical for EKF stability)
        odom.pose.covariance = [
            0.02, 0,    0,    0, 0, 0,
            0,    0.02, 0,    0, 0, 0,
            0,    0,    99999,0, 0, 0,
            0,    0,    0,    99999,0, 0,
            0,    0,    0,    0, 99999,0,
            0,    0,    0,    0, 0, 0.05
        ]

        odom.twist.covariance = [
            0.05, 0,    0,    0, 0, 0,
            0,    0.05, 0,    0, 0, 0,
            0,    0,    99999,0, 0, 0,
            0,    0,    0,    99999,0, 0,
            0,    0,    0,    0, 99999,0,
            0,    0,    0,    0, 0, 0.2
        ]

        self.pub.publish(odom)


if __name__ == "__main__":
    node = OdomNode()
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        node.update()
        rate.sleep()
