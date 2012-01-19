#include <AR/param.h>
#include <AR/ar.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/TransformStamped.h>

#include <ar_pose/ARMarkers.h> // marker msg

#include "marker_detector_node.h"

pattern_pose_estimation::MarkerDetectorNode::MarkerDetectorNode(
  ros::NodeHandle& nh, ros::NodeHandle& nh_private) :
  nh_(nh), nh_private_(nh_private), it_(nh_), previous_marker_detected_(false)
{
  detector_.loadSettings(nh_private_);

  nh_private_.param("publish_tf", publish_tf_, true);
  ROS_INFO ("\tPublish transforms: %d", publish_tf_);

  nh_private_.param("marker_frame", marker_frame_, std::string("ar_marker"));
  ROS_INFO ("\tMarker frame: %s", marker_frame_.c_str());

  nh_private_.param("use_history", use_history_, true);
  ROS_INFO("\tUse history: %d", use_history_);

  nh_private_.param ("rectified", rectified_, false);
  ROS_INFO_STREAM ("\trectified? " << (rectified_ ? "true" : "false"));

  nh_private_.param ("lazy", lazy_, false);
  ROS_INFO_STREAM ("\tLazy? " << (lazy_ ? "true" : "false"));

  nh_private_.param ("cache_camera_info", cache_camera_info_, true);
  ROS_INFO_STREAM ("\tCache camera info? " << (cache_camera_info_ ? "true" : "false"));

  // lazy subscription
  ros::SubscriberStatusCallback connect_cb;
  if (lazy_)
  {
    connect_cb = boost::bind(&MarkerDetectorNode::connectCallback, this, _1);
  }
  else
  {
    ROS_INFO("Subscribing to camera");
    camera_sub_ = it_.subscribeCamera("image", 0, &MarkerDetectorNode::imageCallback, this);
  }
  // advertise
  markers_pub_ = nh_.advertise<ar_pose::ARMarkers>("ar_pose_markers", 1, connect_cb, connect_cb);
}

void pattern_pose_estimation::MarkerDetectorNode::connectCallback(
    const ros::SingleSubscriberPublisher&)
{
  ROS_DEBUG("Connection callback");
  bool has_subscribers = markers_pub_.getNumSubscribers() > 0;
  if (has_subscribers)
  {
    ROS_INFO("Subscribing to camera.");
    camera_sub_ = it_.subscribeCamera("image", 0, &MarkerDetectorNode::imageCallback, this);
  }
  else
  {
    ROS_INFO("Unsubscribing from camera.");
    camera_sub_.shutdown();
  }
}

void pattern_pose_estimation::MarkerDetectorNode::imageCallback(const sensor_msgs::ImageConstPtr& image_msg,
                   const sensor_msgs::CameraInfoConstPtr& camera_info_msg)
{
  // initialize ar camera parameters if needed
  static bool cparam_initialized = false;
  if (!cparam_initialized || !cache_camera_info_)
  {
    detector_.setCameraInfo(*camera_info_msg, rectified_);
    cparam_initialized = true;
  }

  ar_pose::ARMarkersPtr markers_msg(new ar_pose::ARMarkers());
  if (!use_history_ || !previous_marker_detected_)
    detector_.detect(*image_msg, *markers_msg);
  else
    detector_.redetect(*image_msg, *markers_msg);

  if (markers_msg->markers.size() > 0)
  {
    previous_marker_detected_ = true;
    if (publish_tf_)
    {
      for (size_t i = 0; i < markers_msg->markers.size(); ++i)
      {
        std::ostringstream frame_id;
        frame_id << marker_frame_ << "_" << markers_msg->markers[i].id;
        tf::Transform transform;
        tf::poseMsgToTF(markers_msg->markers[i].pose.pose, transform);
        tf::StampedTransform camToMarker(transform, image_msg->header.stamp, image_msg->header.frame_id, frame_id.str());
        broadcaster_.sendTransform(camToMarker);
      }
    }
  }
  else
  {
    previous_marker_detected_ = false;
  }
  markers_pub_.publish(markers_msg);
}

