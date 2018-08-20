// @auther Tasuku Miura

#include "rtsp_streamer/streamer.h"

namespace rtsps {
  RtspStreamer::RtspStreamer(
      ros::NodeHandle& nh,
      std::string camera_name,
      std::string camera_info_url,
      int width,
      int height) :
    nh_(nh),
    it_(nh_),
    camera_name_(camera_name),
    camera_info_url_(camera_info_url),
    frame_width_(width),
    frame_height_(height)
  {
      std::cout << "Initialized" << '\n';
      registerPublishers();
  }
  RtspStreamer::~RtspStreamer() {};

  void RtspStreamer::registerPublishers()
  {
    rtsp_pub_ = it_.advertiseCamera("/rtsp_streamer/stream", 0);
    std::cout << "Registered publishers" << '\n';
  }

  sensor_msgs::CameraInfo RtspStreamer::get_default_camera_info_from_image(sensor_msgs::ImagePtr img)
  {
    sensor_msgs::CameraInfo cam_info_msg;
    cam_info_msg.header.frame_id = img->header.frame_id;
    // Fill image size
    cam_info_msg.height = img->height;
    cam_info_msg.width = img->width;

    ROS_INFO_STREAM("The image width is: " << img->width);
    ROS_INFO_STREAM("The image height is: " << img->height);
    // Add the most common distortion model as sensor_msgs/CameraInfo says
    cam_info_msg.distortion_model = "plumb_bob";
    // Don't let distorsion matrix be empty
    cam_info_msg.D.resize(5, 0.0);
    // Give a reasonable default intrinsic camera matrix
    cam_info_msg.K = boost::assign::list_of(1.0) (0.0) (img->width/2.0)
            (0.0) (1.0) (img->height/2.0)
            (0.0) (0.0) (1.0);
    // Give a reasonable default rectification matrix
    cam_info_msg.R = boost::assign::list_of (1.0) (0.0) (0.0)
            (0.0) (1.0) (0.0)
            (0.0) (0.0) (1.0);
    // Give a reasonable default projection matrix
    cam_info_msg.P = boost::assign::list_of (1.0) (0.0) (img->width/2.0) (0.0)
            (0.0) (1.0) (img->height/2.0) (0.0)
            (0.0) (0.0) (1.0) (0.0);
    return cam_info_msg;
  }

  void RtspStreamer::publishStream(const std::string source)
  {
    cv::VideoCapture vcap;
    cv::Mat frame;
    sensor_msgs::ImagePtr msg;
    sensor_msgs::CameraInfo cam_info_msg;
    camera_info_manager::CameraInfoManager cam_info_manager(
        nh_, camera_name_, camera_info_url_);
    std_msgs::Header header;

    //open the video stream and make sure it's opened
    if(!vcap.open(source)) {
        std::cout << "Error opening video stream or file" << std::endl;
    }

    for(;;) {
        if(!vcap.read(frame)) {
            std::cout << "No frame" << std::endl;
            cv::waitKey();
        }
        // CV_INTER_AREA apparently good when shrinking
        cv::resize(frame, frame, cv::Size(640, 360), 0, 0, CV_INTER_AREA);
        msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();

        cam_info_msg = get_default_camera_info_from_image(msg);
        cam_info_manager.setCameraInfo(cam_info_msg);

        rtsp_pub_.publish(*msg, cam_info_msg, ros::Time::now());

        if(cv::waitKey(1) >= 0) break;
    }
  }


}
