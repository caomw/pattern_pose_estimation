#include "marker_detector_node.h"

int main (int argc, char **argv)
{
  ros::init (argc, argv, "marker_detector");
  ros::NodeHandle nh, nh_private("~");
  pattern_pose_estimation::MarkerDetectorNode detector_node(nh, nh_private);
  ros::spin();
  return 0;
}

