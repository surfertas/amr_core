/*
 * @author Tasuku Miura
 */

#include <ros/ros.h>
#include <amr_object_detection/object_detection.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_detection");
  ros::NodeHandle nh;
  object_detection::ObjectDetect object_detector(nh);
  ros::spin();
  return 0;
}
 
    
