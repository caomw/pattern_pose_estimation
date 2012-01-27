#include <gtest/gtest.h>
#include <ros/package.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_calibration_parsers/parse.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

#include "marker_detector_server.h"
#include "pattern_pose_estimation/DetectMarker.h"

sensor_msgs::Image loadImage(const std::string& filename)
{
  cv::Mat image = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
  if(image.empty()) throw std::runtime_error("image load failed");
  sensor_msgs::Image image_msg;
  cv_bridge::CvImage cv_image;
  cv_image.image = image;
  cv_image.encoding = sensor_msgs::image_encodings::BGR8;
  cv_image.toImageMsg(image_msg);
  return image_msg;
}

sensor_msgs::CameraInfo loadCameraInfo(const std::string& filename)
{
  std::string camera_name;
  sensor_msgs::CameraInfo camera_info;
  bool ok = camera_calibration_parsers::readCalibration(filename, camera_name, camera_info);
  if (!ok) throw std::runtime_error("camera info load failed");
  return camera_info;
}

TEST(MarkerDetectorServiceTest, detectionTest)
{
  // load data
  std::string pkg_path = ros::package::getPath(ROS_PACKAGE_NAME);

  sensor_msgs::Image image_with_marker_msg = loadImage(pkg_path + "/test/data/marker.jpg");
  sensor_msgs::Image image_without_marker_msg = loadImage(pkg_path + "/test/data/no_marker.jpg");

  sensor_msgs::CameraInfo camera_info = loadCameraInfo(pkg_path + "/test/data/camera_info.yaml");

  ros::NodeHandle nh;
  ros::ServiceClient client = 
    nh.serviceClient<pattern_pose_estimation::DetectMarker>("marker_detector_server/detect_marker");
  bool service_found = client.waitForExistence(ros::Duration(5));
  ASSERT_TRUE(service_found);

  pattern_pose_estimation::DetectMarker service_msg;

  service_msg.request.image = image_with_marker_msg;
  service_msg.request.camera_info = camera_info;
  service_msg.request.rectified = true;

  EXPECT_TRUE(client.call(service_msg));

  EXPECT_EQ(service_msg.response.markers.markers.size(), 8);

  pattern_pose_estimation::DetectMarker service_msg_2;

  service_msg.request.image = image_without_marker_msg;
  service_msg.request.camera_info = camera_info;
  service_msg.request.rectified = true;

  EXPECT_TRUE(client.call(service_msg));

  EXPECT_EQ(service_msg.response.markers.markers.size(), 0);
}
  
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "marker_detector_service_test", ros::init_options::AnonymousName);
  return RUN_ALL_TESTS();
}

