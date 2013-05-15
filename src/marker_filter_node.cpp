#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <ar_pose/ARMarkers.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace pattern_pose_estimation
{

class MarkerFilterNode
{

private:

  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  std::string frame_id_;
  bool publish_tf_;

  ros::Subscriber markers_sub_;
  ros::Publisher pose_pub_;

  tf::TransformBroadcaster broadcaster_;

public:

  MarkerFilterNode() :
    nh_(), nh_priv_("~")
  {
    init();
  }

  void init()
  {
    nh_priv_.param("publish_tf", publish_tf_, true);
    ROS_INFO ("\tPublish transforms: %d", publish_tf_);

    nh_priv_.param("frame_id", frame_id_, std::string("marker"));
    ROS_INFO ("\tFrame id: %s", frame_id_.c_str());
    
    ros::SubscriberStatusCallback connect_cb = boost::bind(&MarkerFilterNode::connectCallback, this, _1);
    pose_pub_ = nh_priv_.advertise<geometry_msgs::PoseStamped>("marker_pose", 0, connect_cb, connect_cb);

  }

  void connectCallback(const ros::SingleSubscriberPublisher&)
  {
    ROS_DEBUG("Connection callback");
    bool has_subscribers = pose_pub_.getNumSubscribers() > 0;
    if (has_subscribers)
    {
      ROS_INFO("Subscribing to marker msg.");
      markers_sub_ = nh_.subscribe("ar_pose_markers", 0, &MarkerFilterNode::markersCallback, this);
    }
    else
    {
      ROS_INFO("Unsubscribing from marker msg.");
      markers_sub_.shutdown();
    }
  }

  void markersCallback(const ar_pose::ARMarkersConstPtr& markers_msg)
  {
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header = markers_msg->header;
    if (calculateFilteredPose(*markers_msg, pose.pose))
    {
      pose_pub_.publish(pose);
      if (publish_tf_)
      {
        tf::Transform transform;
        tf::poseMsgToTF(pose.pose.pose, transform);
        tf::StampedTransform camToMarker(transform, markers_msg->header.stamp, markers_msg->header.frame_id, frame_id_);
        broadcaster_.sendTransform(camToMarker);
      }
    }
    else
    {
      ROS_INFO("No confident marker pose found");
    }
  }

  bool calculateFilteredPose(const ar_pose::ARMarkers& markers_msg, 
      geometry_msgs::PoseWithCovariance& pose_msg)
  {
    if (markers_msg.markers.size() == 0)
    {
      return false;
    }
    // simply take the marker pose with highest confidence,
    // this could be replaced by some weighted mean or median
    // to gain more stability
    double best_confidence = 0;
    bool confident_marker_found = false;
    for (size_t i = 0; i < markers_msg.markers.size(); ++i)
    {
      if (markers_msg.markers[i].confidence > best_confidence)
      {
        best_confidence = markers_msg.markers[i].confidence;
        pose_msg = markers_msg.markers[i].pose;
        confident_marker_found = true;
      }
    }
    return confident_marker_found;
  }
};

}

int main(int argc, char* argv[])
{
  ros::init(argc,argv,"marker_filter");
  pattern_pose_estimation::MarkerFilterNode mfn;
  ros::spin();
}

