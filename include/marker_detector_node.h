#ifndef PATTERN_POSE_ESTIMATION_MARKER_DETECTOR_NODE
#define PATTERN_POSE_ESTIMATION_MARKER_DETECTOR_NODE

#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>

#include "marker_detector.h"

namespace pattern_pose_estimation
{

class MarkerDetectorNode
{
public:

  MarkerDetectorNode(ros::NodeHandle& nh,
      ros::NodeHandle& nh_private);

private:


  void connectCallback(const ros::SingleSubscriberPublisher&);

  void imageCallback(const sensor_msgs::ImageConstPtr& image_msg,
      const sensor_msgs::CameraInfoConstPtr& camera_info);

  void loadMarkers();

  static std::string resolveURL(const std::string& url);

  ros::NodeHandle nh_, nh_private_;
  tf::TransformBroadcaster broadcaster_;
  ros::Publisher markers_pub_;

  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber camera_sub_;

  MarkerDetector detector_;

  std::string marker_frame_;
  bool publish_tf_;
  bool use_history_;

  bool cache_camera_info_;
  bool previous_marker_detected_;
  bool rectified_;
  bool lazy_;

};

} // end of namespace


#endif

