#ifndef PATTERN_POSE_ESTIMATION_MARKER_DETECTOR_SERVER
#define PATTERN_POSE_ESTIMATION_MARKER_DETECTOR_SERVER

#include "marker_detector.h"
#include "pattern_pose_estimation/DetectMarker.h"

namespace pattern_pose_estimation
{

class MarkerDetectorServer
{
public:

  MarkerDetectorServer(ros::NodeHandle& nh,
      ros::NodeHandle& nh_private);

private:

  bool serviceCallback(DetectMarkerRequest& req, DetectMarkerResponse& res);

  ros::NodeHandle nh_, nh_private_;
  MarkerDetector detector_;
};

} // end of namespace


#endif

