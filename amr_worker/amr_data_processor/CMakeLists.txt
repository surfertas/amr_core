cmake_minimum_required(VERSION 2.8.3)
project(amr_data_processor)

find_package(catkin REQUIRED COMPONENTS
  amr_controller
  geometry_msgs
  message_filters
  sensor_msgs
)

find_package(OpenCV)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES amr_data_processor
   CATKIN_DEPENDS geometry_msgs message_filters sensor_msgs amr_controller
#  DEPENDS system_lib
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

