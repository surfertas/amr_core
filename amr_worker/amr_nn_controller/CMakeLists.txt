cmake_minimum_required(VERSION 2.8.3)
project(amr_nn_controller)

# add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  amr_controller
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

add_service_files(
  FILES
  PredictCommand.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  amr_controller
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES amr_nn_controller
   CATKIN_DEPENDS
     amr_controller
     geometry_msgs
     roscpp
     rospy 
     sensor_msgs
     std_msgs
#  DEPENDS system_lib
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

