#!/usr/bin/env python3

import rospy
import serial
import struct

from geometry_msgs.msg import Twist

class UARTBridge:

    def __init__(self):

        port = rospy.get_param("~port", "/dev/ttyTHS1")
        baud = rospy.get_param("~baud", 115200)

        self.ser = serial.Serial(
            port,
            baud,
            timeout=0.01
        )

        rospy.Subscriber(
            "/cmd_vel",
            Twist,
            self.cmd_callback
        )

        rospy.loginfo("UART bridge started")

    def cmd_callback(self, msg):

        linear = msg.linear.x
        angular = msg.angular.z

        rospy.loginfo(
            f"linear={linear} angular={angular}"
        )

        packet = struct.pack(
            "<BBff",
            0xAA,
            0x01,
            linear,
            angular
        )

        self.ser.write(packet)


if __name__ == "__main__":

    rospy.init_node("uart_bridge")

    bridge = UARTBridge()

    rospy.spin()
