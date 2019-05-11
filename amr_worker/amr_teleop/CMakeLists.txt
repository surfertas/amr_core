cmake_minimum_required(VERSION 2.8.3)
project(amr_teleop)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

