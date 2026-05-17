#!/usr/bin/env python

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

        rospy.Subscriber("/wheel_left", Float32, self.left_callback)
        rospy.Subscriber("/wheel_right", Float32, self.right_callback)

        self.odom_pub = rospy.Publisher(
            "/wheel_odom",
            Odometry,
            queue_size=20
        )

        rospy.loginfo("Wheel odometry node started")

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

        # Convert RPS → linear wheel velocity (m/s)
        left_v = 2.0 * math.pi * self.wheel_radius * self.left_rps
        right_v = 2.0 * math.pi * self.wheel_radius * self.right_rps

        # Convert velocity → distance per timestep (IMPORTANT FIX)
        dl = left_v * dt
        dr = right_v * dt

        # Optional mild smoothing (distance domain, not velocity domain)
        if not hasattr(self, "dl_f"):
            self.dl_f = 0.0
            self.dr_f = 0.0

        alpha = 0.3
        self.dl_f = alpha * dl + (1 - alpha) * self.dl_f
        self.dr_f = alpha * dr + (1 - alpha) * self.dr_f

        # Differential drive model
        d_center = (self.dr_f + self.dl_f) / 2.0
        d_theta = (self.dr_f - self.dl_f) / self.wheel_base

        # midpoint integration (much more stable)
        theta_mid = self.theta + d_theta / 2.0

        self.x += d_center * math.cos(theta_mid)
        self.y += d_center * math.sin(theta_mid)
        self.theta = normalize_angle(self.theta + d_theta)

        q = tf.transformations.quaternion_from_euler(0, 0, self.theta)

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = Quaternion(*q)

        # velocity estimate (still useful for debugging / EKF)
        odom.twist.twist.linear.x = d_center / dt
        odom.twist.twist.angular.z = d_theta / dt

        # Pose covariance (reasonable trust in x,y, not in orientation)
        odom.pose.covariance = [
            0.05, 0,    0,    0, 0, 0,
            0,    0.05, 0,    0, 0, 0,
            0,    0,    99999,0, 0, 0,
            0,    0,    0,    99999,0, 0,
            0,    0,    0,    0, 99999,0,
            0,    0,    0,    0, 0, 0.2
        ]

        # Twist covariance
        odom.twist.covariance = [
            0.02, 0,    0,    0, 0, 0,
            0,    0.02, 0,    0, 0, 0,
            0,    0,    99999,0, 0, 0,
            0,    0,    0,    99999,0, 0,
            0,    0,    0,    0, 99999,0,
            0,    0,    0,    0, 0, 0.1
        ]

        self.odom_pub.publish(odom)


if __name__ == "__main__":
    node = OdomNode()
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        node.update()
        rate.sleep()
