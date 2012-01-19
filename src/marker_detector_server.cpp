#include "marker_detector_server.h"

pattern_pose_estimation::MarkerDetectorServer::MarkerDetectorServer(
  ros::NodeHandle& nh, ros::NodeHandle& nh_private) :
  nh_(nh), nh_private_(nh_private)
{
  detector_.loadSettings(nh_private_);

  service_server_ = nh_private_.advertiseService("detect_marker", 
      &MarkerDetectorServer::serviceCallback, this);
  ROS_INFO("Service \"detect_marker\" advertised.");
}

bool pattern_pose_estimation::MarkerDetectorServer::serviceCallback(
    DetectMarkerRequest& req,
    DetectMarkerResponse& res)
{
  detector_.setCameraInfo(req.camera_info, req.rectified);
  detector_.detect(req.image, res.markers);
  return true;
}

