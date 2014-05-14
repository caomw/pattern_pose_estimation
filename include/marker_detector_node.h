#ifndef PATTERN_POSE_ESTIMATION_MARKER_DETECTOR_NODE
#define PATTERN_POSE_ESTIMATION_MARKER_DETECTOR_NODE

#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <std_srvs/Empty.h>

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

  bool startDetection(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  bool stopDetection(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  void imageCallback(const sensor_msgs::ImageConstPtr& image_msg,
      const sensor_msgs::CameraInfoConstPtr& camera_info);

  inline void timerCallback() { update_thres_ = true; }
  inline static bool sortByConfidence(const std::pair<double,int> p1, const std::pair<double,int> p2)
  {
    return (p1.first > p2.first);
  }

  ros::NodeHandle nh_, nh_private_;
  tf::TransformBroadcaster broadcaster_;
  ros::Publisher markers_pub_;

  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber camera_sub_;

  MarkerDetector detector_;

  std::string marker_frame_;
  bool publish_tf_;
  bool use_history_;
  bool listen_services_;

  bool cache_camera_info_;
  bool previous_marker_detected_;
  bool rectified_;
  bool lazy_;

  ros::Timer dynamic_thres_timer_;
  int max_bin_thres_;
  int min_bin_thres_;
  int step_bin_thres_;
  bool update_thres_;
  int bin_thres_;
};

} // end of namespace


#endif

