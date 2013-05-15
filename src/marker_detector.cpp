#include <ros/package.h>

#include <cv_bridge/cv_bridge.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/image_encodings.h>
#include <AR/ar.h>

#include <opencv2/highgui/highgui.hpp>

#include "marker_detector.h"

static const double ROS_TO_AR = 1000.0;
static const double AR_TO_ROS = 0.001;

static const char* DEBUG_WINDOW_NAME = "MarkerDetector";

bool pattern_pose_estimation::MarkerDetector::has_instance_ = false;


void paintMarker(cv::Mat& canvas, const ARMarkerInfo& marker_info)
{
  cv::Point2d p1, p2, p3, p4;
  p1.x = marker_info.vertex[0][0]; p1.y = marker_info.vertex[0][1];
  p2.x = marker_info.vertex[1][0]; p2.y = marker_info.vertex[1][1];
  p3.x = marker_info.vertex[2][0]; p3.y = marker_info.vertex[2][1];
  p4.x = marker_info.vertex[3][0]; p4.y = marker_info.vertex[3][1];
  cv::line(canvas, p1, p2, cv::Scalar(255, 0, 255), 2);
  cv::line(canvas, p2, p3, cv::Scalar(255, 0, 255), 2);
  cv::line(canvas, p3, p4, cv::Scalar(255, 0, 255), 2);
  cv::line(canvas, p4, p1, cv::Scalar(255, 0, 255), 2);
}

pattern_pose_estimation::MarkerDetector::MarkerDetector()
  : threshold_(DEFAULT_THRESHOLD), camera_initialized_(false)
{
  if (has_instance_)
  {
    throw MarkerDetectorException(
        "There can only be one instance of MarkerDetector");
  }
  has_instance_ = true;
}

pattern_pose_estimation::MarkerDetector::~MarkerDetector()
{
  for (size_t i = 0; i < markers_.size(); ++i)
  {
    arFreePatt(markers_[i].pattern_id);
  }
  if (show_debug_image_)
  {
    cv::destroyWindow(DEBUG_WINDOW_NAME);
  }
}

void pattern_pose_estimation::MarkerDetector::loadSettings(ros::NodeHandle& nh)
{
  int threshold;
  nh.param("threshold", threshold, DEFAULT_THRESHOLD);
  setThreshold(threshold);

  nh.param("show_debug_image", show_debug_image_, false);
  if (show_debug_image_)
  {
    cv::namedWindow(DEBUG_WINDOW_NAME, 0);
  }

  XmlRpc::XmlRpcValue marker_list;
  if (!nh.getParam("markers", marker_list))
  {
    throw MarkerDetectorException(
      "No markers configured. Set the markers parameter!");
  }
  ROS_ASSERT(marker_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int i = 0; i < marker_list.size(); ++i)
  {
    ROS_ASSERT_MSG(marker_list[i].getType() == XmlRpc::XmlRpcValue::TypeArray,
                   "Marker list corrupt, one list item is no list");
    ROS_ASSERT_MSG(marker_list[i].size() == 5,
                   "Marker list corrupt, marker description has not 5 values");
    ROS_ASSERT(marker_list[i][0].getType() == XmlRpc::XmlRpcValue::TypeInt);
    ROS_ASSERT(marker_list[i][1].getType() == XmlRpc::XmlRpcValue::TypeString);
    ROS_ASSERT(marker_list[i][2].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    ROS_ASSERT(marker_list[i][3].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    ROS_ASSERT(marker_list[i][4].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    int marker_id           = marker_list[i][0];
    std::string pattern_url = marker_list[i][1];
    double marker_width     = marker_list[i][2];
    double marker_center_x  = marker_list[i][3];
    double marker_center_y  = marker_list[i][4];
    std::string pattern_filename = resolveURL(pattern_url);
    loadMarker(marker_id, pattern_filename,
               marker_width, marker_center_x, marker_center_y);
  }
}

std::string pattern_pose_estimation::MarkerDetector::resolveURL(const std::string& url) const
{
  std::string prefix = "file://";
  if (url.substr(0, prefix.length()) == prefix)
  {
    return url.substr(prefix.length());
  }
  // Scan URL from after "package://" until next '/' and extract
  prefix = "package://";
  if (url.substr(0, prefix.length()) == prefix)
  {
    size_t rest = url.find('/', prefix.length());
    std::string package(url.substr(prefix.length(), rest - prefix.length()));
    // Look up the ROS package path name.
    std::string pkg_path(ros::package::getPath(package));
    if (pkg_path.empty())                    // package not found?
    {
      ROS_ERROR_STREAM("unknown package: " << pkg_path);
      return url;
    }
    else
    {
      // Construct file name from package location and remainder of URL.
      return pkg_path + url.substr(rest);
    }
  }
  throw MarkerDetectorException("Could not resolve URL " + url);
  return url;
}

void pattern_pose_estimation::MarkerDetector::loadMarker(
    int id,
    const std::string& pattern_file_name,
    double width,
    double x,
    double y)
{
  Marker marker;
  marker.pattern_id = arLoadPatt(pattern_file_name.c_str());
  if (marker.pattern_id < 0)
  {
    throw MarkerDetectorException("Could not load pattern " + pattern_file_name);
  }
  marker.id = id;
  marker.width = width * ROS_TO_AR;
  marker.center[0] = x * ROS_TO_AR;
  marker.center[1] = y * ROS_TO_AR;
  for (size_t i = 0; i < markers_.size(); ++i)
  {
    if (markers_[i].id == marker.id)
    {
      throw MarkerDetectorException("Multiple markers have same ID");
    }
  }
  marker.detection_flag = NOT_DETECTED;
  markers_.push_back(marker);
}

void pattern_pose_estimation::MarkerDetector::setCameraInfo(
    const sensor_msgs::CameraInfo& camera_info_msg,
    bool rectified)
{
  ARParam cam_param;
  cam_param.xsize = camera_info_msg.width;
  cam_param.ysize = camera_info_msg.height;
  
  cam_param.mat[0][0] = camera_info_msg.P[0];
  cam_param.mat[1][0] = camera_info_msg.P[4];
  cam_param.mat[2][0] = camera_info_msg.P[8];
  cam_param.mat[0][1] = camera_info_msg.P[1];
  cam_param.mat[1][1] = camera_info_msg.P[5];
  cam_param.mat[2][1] = camera_info_msg.P[9];
  cam_param.mat[0][2] = camera_info_msg.P[2];
  cam_param.mat[1][2] = camera_info_msg.P[6];
  cam_param.mat[2][2] = camera_info_msg.P[10];
  cam_param.mat[0][3] = camera_info_msg.P[3];
  cam_param.mat[1][3] = camera_info_msg.P[7];
  cam_param.mat[2][3] = camera_info_msg.P[11];
  cam_param.dist_factor[0] = camera_info_msg.K[2];       // x0 = cX from openCV calibration
  cam_param.dist_factor[1] = camera_info_msg.K[5];       // y0 = cY from openCV calibration
  cam_param.dist_factor[3] = 1.0;
  
  if (rectified)
  {
    // no distortion
    cam_param.dist_factor[2] = 0.0;
  }
  else
  {
    // (ARToolKit does not support more sophisticated distortion models)
    cam_param.dist_factor[2] = -100*camera_info_msg.D[0];  // f = -100*k1 from CV. Note, we had to do mm^2 to m^2, hence 10^8->10^2
  }
  arInitCparam(&cam_param);
  camera_initialized_ = true;
}

void pattern_pose_estimation::MarkerDetector::detect(
    const sensor_msgs::Image& image,
    ar_pose::ARMarkers& markers_msg)
{
  detectImpl(image, markers_msg, false);
}

void pattern_pose_estimation::MarkerDetector::redetect(
    const sensor_msgs::Image& image,
    ar_pose::ARMarkers& markers_msg)
{
  detectImpl(image, markers_msg, true);
}

void pattern_pose_estimation::MarkerDetector::detectImpl(
    const sensor_msgs::Image& image,
    ar_pose::ARMarkers& markers_msg, bool use_cache)
{
  if (!camera_initialized_)
  {
    throw MarkerDetectorException("detect called with uninitialized camera");
  }

  if (markers_.size() == 0)
  {
    throw MarkerDetectorException("detect called without any loaded markers");
  }

  markers_msg.header.stamp = image.header.stamp;
  markers_msg.header.frame_id = image.header.frame_id;

  /* 
  * NOTE: the dataPtr format is BGR because the ARToolKit library was
  * build with V4L, dataPtr format change according to the 
  * ARToolKit configure option (see config.h).*/
  ARUint8* dataPtr;
  cv_bridge::CvImageConstPtr cv_ptr;
  
  if (image.encoding == sensor_msgs::image_encodings::BGR8)
  {
    dataPtr = (ARUint8*)&image.data.front();
  }
  else
  {
    cv_ptr = cv_bridge::toCvCopy(image, "bgr8");
    dataPtr = (ARUint8*)cv_ptr->image.data;
  }

  ARMarkerInfo *detected_markers;
  int num_detected_markers;
  // we use arDetectMarkerLite here instead of arDetectMarker
  // as the latter uses a history for smoothing
  if (arDetectMarkerLite(dataPtr, threshold_, 
        &detected_markers, &num_detected_markers) < 0)
  {
    throw MarkerDetectorException("arDetectMarker failed");
  }
  ROS_DEBUG("Detected %i markers.", num_detected_markers);

  if (show_debug_image_)
  {
    if (!cv_ptr)
    {
      cv_ptr = cv_bridge::toCvCopy(image, "bgr8");
    }
    cv::Mat canvas = cv_ptr->image.clone();
    for (int i = 0; i < num_detected_markers; ++i)
    {
      paintMarker(canvas, detected_markers[i]);
    }
    cv::imshow(DEBUG_WINDOW_NAME, canvas);
    cv::waitKey(5);
  }

  // identify markers (check for known patterns)
  for (int i = 0; i < num_detected_markers; ++i)
  {
    for (size_t m = 0; m < markers_.size(); ++m)
    {
      if (detected_markers[i].id == markers_[m].pattern_id)
      {
        ROS_DEBUG("Found pattern: %i with confidence %f", 
          markers_[m].pattern_id, detected_markers[i].cf);
      
        if (use_cache && markers_[m].detection_flag == LAST_DETECTED)
        {
          arGetTransMatCont(&detected_markers[i], 
              markers_[m].transformation,
              markers_[m].center,
              markers_[m].width, 
              markers_[m].transformation);
        }
        else
        {
          arGetTransMat(&detected_markers[i], 
              markers_[m].center, 
              markers_[m].width, 
              markers_[m].transformation);
        }
        // hack for negative z detection (bad pose calculation)
        if (markers_[m].transformation[2][3] < 0)
        {
          continue;
        }
        markers_[m].detection_flag = DETECTED;
        ar_pose::ARMarker marker_msg;
        marker_msg.header.frame_id = image.header.frame_id;
        marker_msg.header.stamp = image.header.stamp;
        marker_msg.id = markers_[m].id;
        int confidence = static_cast<int>(100000.0 * detected_markers[i].area / (image.width * image.height) *
          detected_markers[i].cf);
        marker_msg.confidence = confidence;
        arTransformationToPose(
            markers_[m].transformation, marker_msg.pose.pose);
        markers_msg.markers.push_back(marker_msg);
      }
    }
  }
  for (size_t m = 0; m < markers_.size(); ++m)
  {
    if (markers_[m].detection_flag == DETECTED)
    {
      markers_[m].detection_flag = LAST_DETECTED;
    }
    else
    {
      markers_[m].detection_flag = NOT_DETECTED;
    }
  }
}

void pattern_pose_estimation::MarkerDetector::arTransformationToPose(
    double ar_transformation[3][4], geometry_msgs::Pose& pose)
{
  tf::Vector3 translation(ar_transformation[0][3] * AR_TO_ROS, 
                          ar_transformation[1][3] * AR_TO_ROS, 
                          ar_transformation[2][3] * AR_TO_ROS);
  tf::Matrix3x3 rot_mat(ar_transformation[0][0], ar_transformation[0][1], ar_transformation[0][2],
                       ar_transformation[1][0], ar_transformation[1][1], ar_transformation[1][2],
                       ar_transformation[2][0], ar_transformation[2][1], ar_transformation[2][2]);
  tf::Quaternion q;
  rot_mat.getRotation(q);
  tf::Transform tf_transform(q, translation);
  tf::poseTFToMsg(tf_transform, pose);
}

