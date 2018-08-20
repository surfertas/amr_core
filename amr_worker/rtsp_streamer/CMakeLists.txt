cmake_minimum_required(VERSION 2.8.3)
project(rtsp_streamer)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  camera_info_manager
  roscpp
  rospy
  sensor_msgs
)

find_package(OpenCV REQUIRED
  NO_MODULE
  PATHS /usr/local
  NO_DEFAULT_PATH
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES streamer
  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs camera_info_manager
  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(streamer
   src/streamer.cpp
)

target_link_libraries(streamer
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node
  src/rtsp_streamer_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   streamer
)
