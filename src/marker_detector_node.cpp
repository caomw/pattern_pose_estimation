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
  ROS_INFO_STREAM ("\tPublish transforms? " << (publish_tf_ ? "true" : "false"));

  nh_private_.param("marker_frame", marker_frame_, std::string("ar_marker"));
  ROS_INFO ("\tMarker frame: %s", marker_frame_.c_str());

  nh_private_.param("use_history", use_history_, true);
  ROS_INFO_STREAM ("\tUse history? " << (use_history_ ? "true" : "false"));

  nh_private_.param ("rectified", rectified_, false);
  ROS_INFO_STREAM ("\trectified? " << (rectified_ ? "true" : "false"));

  nh_private_.param ("lazy", lazy_, false);
  ROS_INFO_STREAM ("\tLazy? " << (lazy_ ? "true" : "false"));

  nh_private_.param ("cache_camera_info", cache_camera_info_, true);
  ROS_INFO_STREAM ("\tCache camera info? " << (cache_camera_info_ ? "true" : "false"));

  std::string marker_msg;
  nh_private_.param("marker_msg", marker_msg, std::string("ar_pose_markers"));
  ROS_INFO ("\tMarker Message: %s", marker_msg.c_str());

  double dynamic_thres_rate;
  nh_private_.param ("dynamic_thres_rate", dynamic_thres_rate, 0.0);
  ROS_INFO_STREAM ("\tDynamic binary threshold rate: " << dynamic_thres_rate );

  nh_private_.param("max_bin_thres", max_bin_thres_, 140);
  ROS_INFO_STREAM ("\t\tMaximum binary threshold: " << max_bin_thres_);

  nh_private_.param("min_bin_thres", min_bin_thres_, 70);
  ROS_INFO_STREAM ("\t\tMinimum binary threshold: " << min_bin_thres_);

  nh_private_.param("step_bin_thres", step_bin_thres_, 5);
  ROS_INFO_STREAM ("\t\tStep for dynamic binary threshold: " << step_bin_thres_);

  // Create the timer for dynamic threshold update
  update_thres_ = false;
  if (dynamic_thres_rate > 0.0)
  {
    ros::TimerCallback cb = boost::bind(&MarkerDetectorNode::timerCallback, this);
    dynamic_thres_timer_ = nh_private_.createTimer(ros::Duration(1.0 / dynamic_thres_rate), cb);
    dynamic_thres_timer_.start();
  }

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
  markers_pub_ = nh_.advertise<ar_pose::ARMarkers>(marker_msg, 1, connect_cb, connect_cb);
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

  // Detect the markers
  ar_pose::ARMarkersPtr markers_msg(new ar_pose::ARMarkers());
  if (!use_history_ || !previous_marker_detected_)
    detector_.detect(*image_msg, *markers_msg);
  else
    detector_.redetect(*image_msg, *markers_msg);

  // If markers -> publish
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
  else // No markers found...
    previous_marker_detected_ = false;

  // Publish the marker message
  markers_pub_.publish(markers_msg);

  // Update the binary threshold if required
  if (update_thres_)
  {
    std::vector< std::pair<double,int> > confidences;
    for (int i=min_bin_thres_; i<=max_bin_thres_; i=i+step_bin_thres_)
    {
      // Set the threshold
      detector_.setThreshold(i);

      // Detect the marker
      ar_pose::ARMarkersPtr markers_msg(new ar_pose::ARMarkers());
      detector_.detect(*image_msg, *markers_msg);

      // Get the confidence
      double conf = 0.0;
      if (markers_msg->markers.size() > 0)
      {
        for (size_t j = 0; j < markers_msg->markers.size(); ++j)
          conf += (double)markers_msg->markers[j].confidence;
        conf /= markers_msg->markers.size();
      }
      confidences.push_back(std::make_pair(conf, i));
    }

    // Sort the confidences
    sort(confidences.begin(), confidences.end(), &MarkerDetectorNode::sortByConfidence);

    // Set the best binary threshold found
    detector_.setThreshold(confidences[0].second);

    update_thres_ = false;
  }
}