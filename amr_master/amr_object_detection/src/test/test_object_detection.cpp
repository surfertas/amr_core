/*
 * @author Tasuku Miura
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <thread>
#include <object_detection/object_detection.h>


using namespace object_detection;

TEST(ObjectDetect, testDummy)
{
  ASSERT_EQ(true, true);
}

TEST(ObjectDetect, initDetector)
{
  ros::NodeHandle nh;
  ASSERT_NO_THROW(ObjectDetect object_detector(nh););
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_detection");
  testing::InitGoogleTest(&argc, argv);
  std::thread t([]{while(ros::ok()) ros::spin();});
  auto result = RUN_ALL_TESTS();
  ros::shutdown();
  return result;
}
