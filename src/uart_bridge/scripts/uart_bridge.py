#!/usr/bin/env python3

import rospy
import serial
import struct
import threading

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu


class UARTBridge:

    RX_PACKET_SIZE = 34

    def __init__(self):

        port = rospy.get_param("~port", "/dev/ttyTHS1")
        baud = rospy.get_param("~baud", 115200)

        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=0.1
        )

        rospy.Subscriber(
            "/cmd_vel",
            Twist,
            self.cmd_callback
        )

        # publishers
        self.left_pub = rospy.Publisher(
            "/wheel_left",
            Float32,
            queue_size=10
        )

        self.right_pub = rospy.Publisher(
            "/wheel_right",
            Float32,
            queue_size=10
        )

        self.imu_pub = rospy.Publisher(
            "/imu",
            Imu,
            queue_size=10
        )

        self.rx_thread = threading.Thread(
            target=self.rx_loop,
            daemon=True
        )
        self.rx_thread.start()

        rospy.loginfo("UART bridge started")

    def cmd_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        packet = struct.pack(
            "<BBff",
            0xAA,
            0x01,
            linear,
            angular
        )

        self.ser.write(packet)

    def publish_imu(self, accel, gyro):
        msg = Imu()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "imu_link"

        # linear acceleration
        msg.linear_acceleration.x = accel[0]
        msg.linear_acceleration.y = accel[1]
        msg.linear_acceleration.z = accel[2]

        # angular velocity
        msg.angular_velocity.x = gyro[0]
        msg.angular_velocity.y = gyro[1]
        msg.angular_velocity.z = gyro[2]

        # orientation unavailable
        msg.orientation_covariance[0] = -1

        self.imu_pub.publish(msg)

    def rx_loop(self):

        while not rospy.is_shutdown():
            try:
                header = self.ser.read(2)

                if len(header) != 2:
                    continue

                if header != b'\xAA\x55':
                    rospy.logwarn(
                        f"Bad header: {header.hex()}"
                    )
                    continue

                payload = self.ser.read(32)

                if len(payload) != 32:
                    rospy.logwarn(
                        f"Incomplete packet: {len(payload)}"
                    )
                    continue

                (
                    left,
                    right,
                    ax, ay, az,
                    gx, gy, gz
                ) = struct.unpack(
                    "<ffffffff",
                    payload
                )

                # publish encoders
                self.left_pub.publish(left)
                self.right_pub.publish(right)

                # publish imu
                self.publish_imu(
                    (ax, ay, az),
                    (gx, gy, gz)
                )

                rospy.loginfo(
                    f"RX left={left:.3f} "
                    f"right={right:.3f}"
                )

            except serial.SerialException as e:
                rospy.logerr(f"UART error: {e}")


if __name__ == "__main__":
    rospy.init_node("uart_bridge")
    bridge = UARTBridge()
    rospy.spin()