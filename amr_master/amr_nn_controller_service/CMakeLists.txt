cmake_minimum_required(VERSION 2.8.3)
project(amr_nn_controller_service)

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


add_message_files(DIRECTORY msg
   FILES
   Command2D.msg
)

add_service_files(DIRECTORY srv
   FILES
   PredictCommand.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    cv_bridge
    image_transport 
    roscpp 
    rospy 
    sensor_msgs
    std_msgs
  DEPENDS OpenCV
)
