#include "marker_detector.h"

#include <cv_bridge/cv_bridge.h>
#include <tf/transform_datatypes.h>

#include <AR/ar.h>

static const double ROS_TO_AR = 1000.0;
static const double AR_TO_ROS = 0.001;
pattern_pose_estimation::MarkerDetector::MarkerDetector()
  : threshold_(DEFAULT_THRESHOLD)
{
}

pattern_pose_estimation::MarkerDetector::~MarkerDetector()
{
  for (size_t i = 0; i < markers_.size(); ++i)
  {
    arFreePatt(markers_[i].pattern_id);
  }
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
  markers_.push_back(marker);
}

void pattern_pose_estimation::MarkerDetector::setCameraInfo(
    const sensor_msgs::CameraInfoConstPtr& camera_info_msg,
    bool rectified)
{
  ARParam cam_param;
  cam_param.xsize = camera_info_msg->width;
  cam_param.ysize = camera_info_msg->height;
  
  cam_param.mat[0][0] = camera_info_msg->P[0];
  cam_param.mat[1][0] = camera_info_msg->P[4];
  cam_param.mat[2][0] = camera_info_msg->P[8];
  cam_param.mat[0][1] = camera_info_msg->P[1];
  cam_param.mat[1][1] = camera_info_msg->P[5];
  cam_param.mat[2][1] = camera_info_msg->P[9];
  cam_param.mat[0][2] = camera_info_msg->P[2];
  cam_param.mat[1][2] = camera_info_msg->P[6];
  cam_param.mat[2][2] = camera_info_msg->P[10];
  cam_param.mat[0][3] = camera_info_msg->P[3];
  cam_param.mat[1][3] = camera_info_msg->P[7];
  cam_param.mat[2][3] = camera_info_msg->P[11];
  cam_param.dist_factor[0] = camera_info_msg->K[2];       // x0 = cX from openCV calibration
  cam_param.dist_factor[1] = camera_info_msg->K[5];       // y0 = cY from openCV calibration
  cam_param.dist_factor[3] = 1.0;
  
  if (rectified)
  {
    // no distortion
    cam_param.dist_factor[2] = 0.0;
  }
  else
  {
    // (ARToolKit does not support more sophisticated distortion models)
    cam_param.dist_factor[2] = -100*camera_info_msg->D[0];  // f = -100*k1 from CV. Note, we had to do mm^2 to m^2, hence 10^8->10^2
  }
  arInitCparam(&cam_param);
}

void pattern_pose_estimation::MarkerDetector::detect(
    const sensor_msgs::ImageConstPtr& image,
    ar_pose::ARMarkersPtr& markers_msg)
{
  detectImpl(image, markers_msg, false);
}

void pattern_pose_estimation::MarkerDetector::redetect(
    const sensor_msgs::ImageConstPtr& image,
    ar_pose::ARMarkersPtr& markers_msg)
{
  detectImpl(image, markers_msg, true);
}

void pattern_pose_estimation::MarkerDetector::detectImpl(
    const sensor_msgs::ImageConstPtr& image,
    ar_pose::ARMarkersPtr& markers_msg, bool use_cache)
{
  markers_msg->header.stamp = image->header.stamp;
  markers_msg->header.frame_id = image->header.frame_id;

  cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image, "bgr8");

  /* 
  * NOTE: the dataPtr format is BGR because the ARToolKit library was
  * build with V4L, dataPtr format change according to the 
  * ARToolKit configure option (see config.h).*/
  ARUint8* dataPtr = (ARUint8*)cv_ptr->image.data;

  ARMarkerInfo *detected_markers;
  int num_detected_markers;
  if (arDetectMarker(dataPtr, threshold_, 
        &detected_markers, &num_detected_markers) < 0)
  {
    throw MarkerDetectorException("arDetectMarker failed");
  }
  ROS_DEBUG("Detected %i markers.", num_detected_markers);

  // identify markers (check for known patterns)
  for (int i = 0; i < num_detected_markers; ++i)
  {
    for (size_t m = 0; m < markers_.size(); ++m)
    {
      if (detected_markers[i].id == markers_[m].pattern_id)
      {
        ROS_DEBUG("Found pattern: %i with confidence %f", 
          markers_[m].pattern_id, detected_markers[i].cf);
      
        if (use_cache)
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
        ar_pose::ARMarker marker_msg;
        marker_msg.header.frame_id = image->header.frame_id;
        marker_msg.header.stamp = image->header.stamp;
        marker_msg.id = markers_[m].id;
        marker_msg.confidence = detected_markers[i].cf * 100;
        arTransformationToPose(
            markers_[m].transformation, marker_msg.pose.pose);
        markers_msg->markers.push_back(marker_msg);
      }
    }
  }
}

void pattern_pose_estimation::MarkerDetector::arTransformationToPose(
    double ar_transformation[3][4], geometry_msgs::Pose& pose)
{
  double arQuat[4], arPos[3];
  arUtilMat2QuatPos(ar_transformation, arQuat, arPos);
  tf::Vector3 translation(arPos[0] * AR_TO_ROS, 
                          arPos[1] * AR_TO_ROS, 
                          arPos[2] * AR_TO_ROS);
  tf::Quaternion rotation(-arQuat[0], -arQuat[1], -arQuat[2], arQuat[3]);
  tf::Transform tf_transform(rotation, translation);
  tf::poseTFToMsg(tf_transform, pose);
}

