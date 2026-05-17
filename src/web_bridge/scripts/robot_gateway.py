#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class RobotGateway:
    def __init__(self):
        rospy.init_node('robot_gateway_node')

        self.pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub_web = rospy.Subscriber('/web_cmd', String, self.web_callback)

        self.linear_speed = 1.0
        self.angular_speed = 16.0

        rospy.loginfo("Gateway ready")

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

if __name__ == '__main__':
    try:
        RobotGateway()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
