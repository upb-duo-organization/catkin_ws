#!/usr/bin/env python
import rospy
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RobotGateway:
    def __init__(self):
        rospy.init_node('robot_gateway_node')
        
        # --- CONTROL SECTION ---
        self.pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub_web = rospy.Subscriber('/web_cmd', String, self.web_callback)
        
        # Movement Settings
        self.linear_speed = 0.5   # m/s
        self.angular_speed = 1.0  # rad/s

        # --- CAMERA SECTION (UDP) ---
        # This pipeline listens for an incoming UDP stream on port 5000
        self.udp_pipeline = "udpsrc port=5000 ! application/x-rtp,payload=96 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink"
        self.cap = cv2.VideoCapture(self.udp_pipeline, cv2.CAP_GSTREAMER)
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)
        self.bridge = CvBridge()

        rospy.loginfo("Gateway Node Started. Waiting for WASD on /web_cmd...")

    def web_callback(self, msg):
        command = msg.data.lower()
        move = Twist()

        if command == 'w':
            move.linear.x = self.linear_speed
        elif command == 's':
            move.linear.x = -self.linear_speed
        elif command == 'a':
            move.angular.z = self.angular_speed
        elif command == 'd':
            move.angular.z = -self.angular_speed
        elif command == 'stop':
            move.linear.x = 0
            move.angular.z = 0
        
        self.pub_twist.publish(move)

    def run(self):
        rate = rospy.Rate(20) # 20 FPS for the camera loop
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.image_pub.publish(img_msg)
            rate.sleep()
        self.cap.release()

if __name__ == '__main__':
    try:
        node = RobotGateway()
        node.run()
    except rospy.ROSInterruptException:
        pass
