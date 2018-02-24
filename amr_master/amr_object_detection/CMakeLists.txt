cmake_minimum_required(VERSION 2.8.3)
project(amr_object_detection)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

find_package(OpenCV REQUIRED
NO_MODULE
PATHS /usr/local
NO_DEFAULT_PATH)

add_service_files(DIRECTORY srv
  FILES
  Yolo2.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES object_detection
  CATKIN_DEPENDS
    cv_bridge
    image_transport 
    roscpp 
    rospy 
    sensor_msgs
    std_msgs
  DEPENDS OpenCV
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

add_library(object_detection
   src/object_detection.cpp
)

target_link_libraries(object_detection
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES} 
)

add_executable(object_detection_node
  src/object_detection_node.cpp
)


target_link_libraries(object_detection_node
   ${catkin_LIBRARIES}
   object_detection
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_object_detection
    test/test_object_detection_launch.test
    src/test/test_object_detection.cpp
  )
  target_link_libraries(test_object_detection
    ${catkin_LIBRARIES}
    object_detection
  )
endif()


