cmake_minimum_required(VERSION 2.8.3)

project(simple_arm)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  rospy
)

catkin_python_setup()

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS sensor_msgs rospy
)

catkin_install_python(
  PROGRAMS nodes/simple_arm
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)