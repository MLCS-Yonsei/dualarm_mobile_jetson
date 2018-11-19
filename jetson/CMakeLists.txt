cmake_minimum_required(VERSION 2.8.3)
project(jetson)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  rospy
)

catkin_package(
  LIBRARIES jetson
  CATKIN_DEPENDS roscpp rospy
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
