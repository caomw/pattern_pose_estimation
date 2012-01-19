#include "marker_detector_server.h"

int main (int argc, char **argv)
{
  ros::init (argc, argv, "marker_detector_server");
  ros::NodeHandle nh, nh_private("~");
  pattern_pose_estimation::MarkerDetectorServer detector_server(nh, nh_private);
  ros::spin();
  return 0;
}

