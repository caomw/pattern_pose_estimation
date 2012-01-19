#include <nodelet/nodelet.h>

#include "marker_detector_node.h"

namespace pattern_pose_estimation
{
class MarkerDetectorNodelet : public nodelet::Nodelet
{

public:
  MarkerDetectorNodelet()
  {}

private:

  void onInit()
  {
    detector_node_.reset(new MarkerDetectorNode(getNodeHandle(), 
        getPrivateNodeHandle()));
  }

  boost::shared_ptr<MarkerDetectorNode> detector_node_;
};

}

// this should really be in the implementation (.cpp file)
#include <pluginlib/class_list_macros.h>

// watch the capitalization carefully
PLUGINLIB_DECLARE_CLASS(pattern_pose_estimation, MarkerDetector, 
    pattern_pose_estimation::MarkerDetectorNodelet, nodelet::Nodelet) 

