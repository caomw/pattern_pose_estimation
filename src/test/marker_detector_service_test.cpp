#include <gtest/gtest.h>
#include <ros/package.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_calibration_parsers/parse.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

#include "marker_detector_server.h"
#include "pattern_pose_estimation/DetectMarker.h"

struct TestData
{
  std::string image_file;
  std::string camera_info_file;
  bool rectified;
  int expected_num_markers;

  TestData(const std::string& img_file, const std::string& caminfo_file,
      bool rect, int num_markers) :
    image_file(img_file),
    camera_info_file(caminfo_file),
    rectified(rect),
    expected_num_markers(num_markers)
  {}
};

std::vector<TestData> getDataSets()
{
  std::vector<TestData> data_sets;
  data_sets.push_back(TestData("markers.jpg", "ueye.yaml", true, 8));
  data_sets.push_back(TestData("no_markers.jpg", "ueye.yaml", true, 0));
  return data_sets;
}

sensor_msgs::Image loadImage(const std::string& filename)
{
  std::string pkg_path = ros::package::getPath(ROS_PACKAGE_NAME);
  std::string data_path = pkg_path + "/test/data/";
  cv::Mat image = cv::imread(data_path + filename, CV_LOAD_IMAGE_COLOR);
  if(image.empty()) throw std::runtime_error("image load failed: " + data_path + filename);
  sensor_msgs::Image image_msg;
  cv_bridge::CvImage cv_image;
  cv_image.image = image;
  cv_image.encoding = sensor_msgs::image_encodings::BGR8;
  cv_image.toImageMsg(image_msg);
  return image_msg;
}

sensor_msgs::CameraInfo loadCameraInfo(const std::string& filename)
{
  std::string pkg_path = ros::package::getPath(ROS_PACKAGE_NAME);
  std::string data_path = pkg_path + "/test/data/";
  std::string camera_name;
  sensor_msgs::CameraInfo camera_info;
  bool ok = camera_calibration_parsers::readCalibration(data_path + filename, camera_name, camera_info);
  if (!ok) throw std::runtime_error("camera info load failed");
  return camera_info;
}

class MarkerDetectorServiceTest : public ::testing::TestWithParam<TestData>
{};


TEST_P(MarkerDetectorServiceTest, detectionTest)
{
  sensor_msgs::Image image_msg = loadImage(GetParam().image_file);
  sensor_msgs::CameraInfo camera_info = loadCameraInfo(GetParam().camera_info_file);

  ros::NodeHandle nh;
  ros::ServiceClient client = 
    nh.serviceClient<pattern_pose_estimation::DetectMarker>(
        "marker_detector_server/detect_marker");
  bool service_found = client.waitForExistence(ros::Duration(5));
  ASSERT_TRUE(service_found);

  pattern_pose_estimation::DetectMarker service_msg;

  service_msg.request.image = image_msg;
  service_msg.request.camera_info = camera_info;
  service_msg.request.rectified = GetParam().rectified;

  EXPECT_TRUE(client.call(service_msg));

  EXPECT_EQ(service_msg.response.markers.markers.size(), 
      GetParam().expected_num_markers);
}
  
INSTANTIATE_TEST_CASE_P(MarkerDetectorServiceTests,
    MarkerDetectorServiceTest,
    ::testing::ValuesIn(getDataSets()));
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "marker_detector_service_test", ros::init_options::AnonymousName);
  return RUN_ALL_TESTS();
}

