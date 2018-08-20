// @auther Tasuku Miura

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <ros/ros.h>
#include <rtsp_streamer/streamer.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_streamer");
  ros::NodeHandle nh;
  int frame_width;
  int frame_height;
  std::string camera_name;
  std::string camera_info_url;
  std::string user;
  std::string pw;
  std::string ip;
  std::string port;

  nh.getParam("camera_name", camera_name);
  nh.getParam("camera_info_url", camera_info_url);
  nh.getParam("/ros_streamer/width", frame_width);
  nh.getParam("/ros_streamer/height", frame_height);
  nh.getParam("/user", user);
  nh.getParam("/pw", pw);
  nh.getParam("/ip", ip);
  nh.getParam("/port", port);

  rtsps::RtspStreamer streamer(nh, camera_name, camera_info_url, frame_width, frame_height);

  auto source = "rtsp://"+user+":"+pw+"@"+ip+":"+port+"/cam/realmonitor?channel=1&subtype=0";
  std::cout << source << '\n';
  streamer.publishStream(source);

  ros::spin();
  return 0;
}
