#!/usr/bin/env python3
"""
uart_bridge.py — UART bridge between ROS and MCU.

RX packet (MCU → ROS), 38 bytes:
  [0xAA][0x55][left f32][right f32][accel xyz f32x3][gyro xyz f32x3][sr04 f32]

TX packet (ROS → MCU), 10 bytes:
  [0xAA][0x01][linear f32][angular f32]  — cmd_vel
  [0xAA][0x02][mode u8][0x00 x7]         — mode (1=teleop, 0=autonomous)
"""

import threading
import struct
import rospy
import serial
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Float32, String


class UARTBridge:

    RX_HEADER    = b'\xAA\x55'
    RX_PAYLOAD_S = 36    # 9 floats: left, right, ax, ay, az, gx, gy, gz, sr04
    TX_PACKET_S  = 10

    def __init__(self):
        port = rospy.get_param("~port", "/dev/ttyTHS1")
        baud = rospy.get_param("~baud", 115200)

        self._ser      = serial.Serial(port=port, baudrate=baud, timeout=0.1)
        self._tx_lock  = threading.Lock()

        # publishers
        self._left_pub  = rospy.Publisher("/wheel_left",   Float32, queue_size=10)
        self._right_pub = rospy.Publisher("/wheel_right",  Float32, queue_size=10)
        self._imu_pub   = rospy.Publisher("/imu/data_raw", Imu,     queue_size=10)
        self._range_pub = rospy.Publisher("/sr04/range",   Range,   queue_size=10)

        # subscribers
        rospy.Subscriber("/cmd_vel",    Twist,  self._cmd_cb)
        rospy.Subscriber("/robot/mode", String, self._mode_cb)

        # RX thread
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        rospy.loginfo("UART bridge started on %s @ %d", port, baud)

    # ── TX ────────────────────────────────────────────────────────────────────

    def _write(self, packet):
        with self._tx_lock:
            self._ser.write(packet)

    def _cmd_cb(self, msg):
        packet = struct.pack(
            "<BBff4x",
            0xAA, 0x01,
            msg.linear.x,
            msg.angular.z,
        )
        self._write(packet)

    def _mode_cb(self, msg):
        is_connected = 1 if msg.data.strip().lower() == "teleop" else 0
        packet = struct.pack(
            "<BBB7x",
            0xAA, 0x02,
            is_connected,
        )
        self._write(packet)
        rospy.loginfo("UART: mode=%s IS_CONNECTED=%d", msg.data, is_connected)

    # ── RX ────────────────────────────────────────────────────────────────────

    def _rx_loop(self):
        while not rospy.is_shutdown():
            try:
                header = self._ser.read(2)
                if len(header) != 2:
                    continue
                if header != self.RX_HEADER:
                    rospy.logwarn_throttle(5.0, "UART RX: bad header %s", header.hex())
                    continue

                payload = self._ser.read(self.RX_PAYLOAD_S)
                if len(payload) != self.RX_PAYLOAD_S:
                    rospy.logwarn_throttle(5.0, "UART RX: short payload %d", len(payload))
                    continue

                self._parse_rx(payload)

            except serial.SerialException as e:
                rospy.logerr_throttle(5.0, "UART error: %s", e)

    def _parse_rx(self, payload):
        try:
            left, right, ax, ay, az, gx, gy, gz, sr04_cm = struct.unpack(
                "<fffffffff",
                payload,
            )
        except struct.error as e:
            rospy.logwarn("UART RX: unpack error %s", e)
            return

        now = rospy.Time.now()

        # wheel encoders
        self._left_pub.publish(Float32(data=left))
        self._right_pub.publish(Float32(data=right))

        # IMU
        imu = Imu()
        imu.header.stamp    = now
        imu.header.frame_id = "imu_link"

        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz
        imu.angular_velocity_covariance = [
            0.01, 0.0,  0.0,
            0.0,  0.01, 0.0,
            0.0,  0.0,  0.01,
        ]

        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        imu.linear_acceleration_covariance = [
            0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1,
        ]

        # -1 in [0,0] signals no orientation available
        imu.orientation_covariance = [
            -1.0, 0.0, 0.0,
            0.0,  0.0, 0.0,
            0.0,  0.0, 0.0,
        ]

        self._imu_pub.publish(imu)

        # SR04 — skip on timeout (-1.0)
        if sr04_cm > 0.0:
            r = Range()
            r.header.stamp    = now
            r.header.frame_id = "base_link"
            r.radiation_type  = Range.ULTRASOUND
            r.field_of_view   = 0.26
            r.min_range       = 0.02
            r.max_range       = 4.0
            r.range           = sr04_cm / 100.0  # cm → metres
            self._range_pub.publish(r)

        rospy.loginfo(
            "RX left=%.3f right=%.3f sr04=%.1fcm",
            left, right, sr04_cm,
        )


if __name__ == "__main__":
    rospy.init_node("uart_bridge")
    UARTBridge()
    rospy.spin()
