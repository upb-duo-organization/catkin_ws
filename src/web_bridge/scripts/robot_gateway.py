#!/usr/bin/env python3

import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Twist


class RobotGateway:

    def __init__(self):

        rospy.init_node("robot_gateway_node")

        self.pub_twist = rospy.Publisher(
            "/cmd_vel",
            Twist,
            queue_size=10
        )

        self.sub_web = rospy.Subscriber(
            "/web_cmd",
            String,
            self.web_callback
        )

        self.linear_speed = 0.5
        self.angular_speed = 1.0

        self.current_cmd = "stop"

        self.last_msg_time = rospy.Time.now()

        self.timeout = rospy.Duration(0.3)

        rospy.loginfo("Robot gateway ready")

        self.run()

    def web_callback(self, msg):

        self.current_cmd = msg.data.lower()

        self.last_msg_time = rospy.Time.now()

    def build_twist(self):

        move = Twist()

        if self.current_cmd == "w":
            move.linear.x = self.linear_speed

        elif self.current_cmd == "s":
            move.linear.x = -self.linear_speed

        elif self.current_cmd == "a":
            move.angular.z = self.angular_speed

        elif self.current_cmd == "d":
            move.angular.z = -self.angular_speed

        return move

    def run(self):

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():

            elapsed = rospy.Time.now() - self.last_msg_time

            # If timeout or explicit stop -> don't publish
            if elapsed > self.timeout or self.current_cmd == "stop":
                rate.sleep()
                continue

            twist = self.build_twist()
            self.pub_twist.publish(twist)

            rate.sleep()


if __name__ == "__main__":

    try:
        RobotGateway()

    except rospy.ROSInterruptException:
        pass
