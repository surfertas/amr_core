/*
 * @author Tasuku Miura
 */

#ifndef OBJECT_DETECTION_H
#define OBJECT_DETECTION_H

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/cudaobjdetect.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>

#include "amr_object_detection/Yolo2.h"

namespace object_detection
{
class ObjectDetect
{
public:
  ObjectDetect(ros::NodeHandle& nh);

  virtual ~ObjectDetect();

  void registerServiceClient();

  void registerPublisher();

  void registerSubscriber();

  /**
   *@brief Callback for cam_img_sub_ to convert ROS msg to CvImage
   *
   *@param nh the nodehandle
   */ 
  void convertImageCB(const sensor_msgs::ImageConstPtr& nh);


private:

 /**
   *@brief Node handle for object detection.
   */ 
  ros::NodeHandle nh_;

 /**
   *@brief Service client for object detection service.
   */ 
  ros::ServiceClient detect_client_;

 /**
   *@brief ROS Image transport.
   */ 
  image_transport::ImageTransport it_;

 /**
   *@brief Image subscriber to subscribe to raw image topic.
   */ 
  image_transport::Subscriber cam_img_sub_;

 /**
   *@brief Image publisher that publishes image with bounding boxes.
   */ 
  image_transport::Publisher detected_objects_pub_;

 /**
   *@brief Bridge uses to convert ROS images to CV images.
   */
  cv_bridge::CvImage bridge_;
  
 /**
   *@brief Stores latest image.
   */
  cv::Mat cur_img_;
};
}
#endif //OBJECT_DETECTION_H

