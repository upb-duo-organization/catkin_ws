cmake_minimum_required(VERSION 3.0.2)
project(robot_yolo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/person_detector_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
