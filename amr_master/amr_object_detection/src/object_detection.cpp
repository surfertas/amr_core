/*
 * @author Tasuku Miura
 * @brief Object detection package with cuda enabled.
 */

#include "amr_object_detection/object_detection.h"

namespace object_detection {

ObjectDetect::ObjectDetect(ros::NodeHandle& nh) :
  nh_(nh),
  it_(nh_)
{
  registerServiceClient();
  registerPublisher();
  registerSubscriber();
}


ObjectDetect::~ObjectDetect() {}


void ObjectDetect::registerServiceClient()
{
  detect_client_ =  nh_.serviceClient<amr_object_detection::Yolo2>("object_detection_service");
}


void ObjectDetect::registerPublisher()
{
  detected_objects_pub_ = it_.advertise("/object_detect/detected_objects", 0);
  std::cout << "Publisher initialized.." << std::endl;
}


void ObjectDetect::registerSubscriber() 
{
  cam_img_sub_ = it_.subscribe(
    "/webcam/image_raw", 10, &ObjectDetect::convertImageCB, this, image_transport::TransportHints("compressed"));
  std::cout << "Subscriber initialized.." << std::endl;
}


void ObjectDetect::convertImageCB(const sensor_msgs::ImageConstPtr& img)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e) {
    std::cerr << "cv_bridge exception: " << e.what();
    return;
  }

  cur_img_ = cv_ptr->image;
  cv::cuda::GpuMat img_in(cur_img_);
  cv::cuda::GpuMat img_out;

  // Resize image to size accepted by the Yolo2 model and convert from BGR to RGB.
  cv::cuda::resize(img_in, img_out, cv::Size(416,416), CV_INTER_AREA);
  cv::cuda::cvtColor(img_out, img_out, 4);
  cv::Mat img_mat;
  img_out.download(img_mat);

  sensor_msgs::ImagePtr msg;
  try {
    msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", img_mat).toImageMsg();
  }
  catch (cv_bridge::Exception& e) {
    std::cerr << "cv_bridge exception: " << e.what();
    return;
  }

  amr_object_detection::Yolo2 srv;
  srv.request.img_req = *msg;
  detect_client_.waitForExistence();

  if (detect_client_.call(srv)) {
    detected_objects_pub_.publish(srv.response.img_res);
  } else {
    std::cout << "Unable to reach object detection service.\n";
  }
}
} // object_detect
