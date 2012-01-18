/*
 *  Single Marker Pose Estimation using ARToolkit
 *  Copyright (C) 2010, CCNY Robotics Lab
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *  William Morris <morris@ee.ccny.cuny.edu>
 *  Gautier Dumonteil <gautier.dumonteil@gmail.com>
 *  http://robotics.ccny.cuny.edu
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AR/param.h>
#include <AR/ar.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>

#include <ar_pose/ARMarker.h> // marker msg

static const double AR_TO_ROS = 0.001;
static const double ROS_TO_AR = 1000.0;

namespace ar_pose
{

class ARSinglePublisher
{

public:
  ARSinglePublisher() : nh_(), nh_private_("~"), it_(nh_), previous_marker_detected_(false)
  {
    // **** get parameters
    nh_private_.param("publish_tf", publishTf_, true);
    ROS_INFO ("\tPublish transforms: %d", publishTf_);

    nh_private_.param("threshold", threshold_, 100);
    ROS_INFO ("\tThreshold: %d", threshold_);

    nh_private_.param("marker_width", markerWidth_, 0.08);
    ROS_INFO ("\tMarker Width [m]: %.1f", markerWidth_);
    markerWidth_ *= ROS_TO_AR;

    nh_private_.param("reverse_transform", reverse_transform_, false);
    ROS_INFO("\tReverse Transform: %d", reverse_transform_);

    nh_private_.param("marker_frame", markerFrame_, std::string("ar_marker"));
    ROS_INFO ("\tMarker frame: %s", markerFrame_.c_str());

    // If mode=0, we use arGetTransMat instead of arGetTransMatCont
    // The arGetTransMatCont function uses information from the previous image
    // frame to reduce the jittering of the marker
    nh_private_.param("use_history", useHistory_, true);
    ROS_INFO("\tUse history: %d", useHistory_);

    std::string pattern_filename;
	nh_private_.param("marker_pattern", pattern_filename, 
	    ros::package::getPath("ar_pose") + "/data/patt.hiro");
    // load pattern file
    ROS_INFO_STREAM("Loading pattern " << pattern_filename);
    patt_id_ = arLoadPatt(pattern_filename.c_str());
    if (patt_id_ < 0)
    {
      ROS_ERROR_STREAM("Pattern " << pattern_filename << " could not be loaded.");
      ROS_BREAK();
    }

    nh_private_.param ("marker_center_x", marker_center_[0], 0.0);
    nh_private_.param ("marker_center_y", marker_center_[1], 0.0);
    ROS_INFO ("\tMarker Center [m]: (%.1f,%.1f)", marker_center_[0], marker_center_[1]);
    marker_center_[0] *= ROS_TO_AR;
    marker_center_[1] *= ROS_TO_AR;

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
      connect_cb = boost::bind(&ARSinglePublisher::connectCb, this);
    }
    else
    {
      ROS_INFO("Subscribing to camera");
      camera_sub_ = it_.subscribeCamera("image", 0, &ARSinglePublisher::imageCallback, this);
    }
    // **** advertise 
    arMarkerPub_ = nh_.advertise<ar_pose::ARMarker>("ar_pose_marker", 1, connect_cb, connect_cb);
  }

  ~ARSinglePublisher() 
  { 
    arFreePatt(patt_id_);
  }


private:

  void connectCb()
  {
    ROS_DEBUG("Connection callback");
    bool has_subscribers = arMarkerPub_.getNumSubscribers() > 0;
    if (has_subscribers)
    {
      ROS_INFO("Subscribing to camera.");
      camera_sub_ = it_.subscribeCamera("image", 0, &ARSinglePublisher::imageCallback, this);
    }
    else 
    {
      ROS_INFO("Unsubscribing from camera.");
      camera_sub_.shutdown();
    }
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& image_msg,
                     const sensor_msgs::CameraInfoConstPtr& camera_info_msg)
  {
    // initialize ar camera parameters if needed
    static bool cparam_initialized = false;
    if (!cparam_initialized || !cache_camera_info_)
    {
      initAR(camera_info_msg);
      cparam_initialized = true;
    }

    // convert ROS image to OpenCV
    try
    {
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image_msg, "bgr8");

      /* 
      * NOTE: the dataPtr format is BGR because the ARToolKit library was
      * build with V4L, dataPtr format change according to the 
      * ARToolKit configure option (see config.h).*/
      ARUint8* dataPtr = (ARUint8 *)cv_ptr->image.data;

      // detect the markers in the video frame 
      ARMarkerInfo *marker_info;
      int num_markers;
      if (arDetectMarker(dataPtr, threshold_, &marker_info, &num_markers) < 0)
      {
        ROS_ERROR("arDetectMarker failed");
        return;
      }
      ROS_DEBUG("Detected %i markers.", num_markers);
      // check for known patterns
      bool pattern_found = false;
      int best_marker_index = -1;
      for(int i = 0; i < num_markers; i++)
      {
        if (marker_info[i].id == patt_id_)
        {
          pattern_found = true;
          ROS_DEBUG ("Found pattern: %i with confidence %f", 
              patt_id_, marker_info[i].cf);
          if (best_marker_index < 0)
          {
            best_marker_index = i;
          }
          else
          {
            // make sure you have the best pattern (highest confidence factor)
            if (marker_info[i].cf > marker_info[best_marker_index].cf)
            {
              best_marker_index = i;
            }
          }
        }
      }
      if (pattern_found)
      {
        // **** get the transformation between the marker and the real camera
        if (!useHistory_ || !previous_marker_detected_)
          arGetTransMat (&marker_info[best_marker_index], marker_center_, markerWidth_, marker_trans_);
        else
          arGetTransMatCont (&marker_info[best_marker_index], marker_trans_, marker_center_, markerWidth_, marker_trans_);

        previous_marker_detected_ = true;
        double arQuat[4], arPos[3];
        arUtilMat2QuatPos (marker_trans_, arQuat, arPos);
        tf::Vector3 translation(arPos[0] * AR_TO_ROS, 
                                arPos[1] * AR_TO_ROS, 
                                arPos[2] * AR_TO_ROS);

        tf::Quaternion rotation(-arQuat[0], -arQuat[1], -arQuat[2], arQuat[3]);
        tf::Transform transform(rotation, translation);
        // **** publish the marker msg
        ar_pose::ARMarker marker_msg;
        marker_msg.header.frame_id = image_msg->header.frame_id;
        marker_msg.header.stamp    = image_msg->header.stamp;
        marker_msg.id              = marker_info[best_marker_index].id;
        tf::poseTFToMsg(transform, marker_msg.pose.pose);

        marker_msg.confidence = marker_info[best_marker_index].cf;
        arMarkerPub_.publish(marker_msg);
        
        if (publishTf_)
        {
          // **** publish transform between camera and marker
          if (reverse_transform_)
          {
            tf::StampedTransform markerToCam (transform.inverse(), image_msg->header.stamp, markerFrame_.c_str(), image_msg->header.frame_id);
            broadcaster_.sendTransform(markerToCam);
          } else {
            tf::StampedTransform camToMarker (transform, image_msg->header.stamp, image_msg->header.frame_id, markerFrame_.c_str());
            broadcaster_.sendTransform(camToMarker);
          }
        }
      }
      else
      {
        previous_marker_detected_ = false;
        ROS_DEBUG("Failed to locate marker pattern.");
      }
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }

  void initAR(const sensor_msgs::CameraInfoConstPtr& camera_info_msg)
  {
    ARParam cam_param;
    cam_param.xsize = camera_info_msg->width;
    cam_param.ysize = camera_info_msg->height;
    
    cam_param.mat[0][0] = camera_info_msg->P[0];
    cam_param.mat[1][0] = camera_info_msg->P[4];
    cam_param.mat[2][0] = camera_info_msg->P[8];
    cam_param.mat[0][1] = camera_info_msg->P[1];
    cam_param.mat[1][1] = camera_info_msg->P[5];
    cam_param.mat[2][1] = camera_info_msg->P[9];
    cam_param.mat[0][2] = camera_info_msg->P[2];
    cam_param.mat[1][2] = camera_info_msg->P[6];
    cam_param.mat[2][2] = camera_info_msg->P[10];
    cam_param.mat[0][3] = camera_info_msg->P[3];
    cam_param.mat[1][3] = camera_info_msg->P[7];
    cam_param.mat[2][3] = camera_info_msg->P[11];
    cam_param.dist_factor[0] = camera_info_msg->K[2];       // x0 = cX from openCV calibration
    cam_param.dist_factor[1] = camera_info_msg->K[5];       // y0 = cY from openCV calibration
    cam_param.dist_factor[3] = 1.0;
    
    if (rectified_)
    {
      // no distortion
      cam_param.dist_factor[2] = 0.0;
    }
    else
    {
      // (ARToolKit does not support more sophisticated distortion models)
      cam_param.dist_factor[2] = -100*camera_info_msg->D[0];  // f = -100*k1 from CV. Note, we had to do mm^2 to m^2, hence 10^8->10^2
    }

    ROS_DEBUG("Initializing AR camera parameters");
    arInitCparam(&cam_param);
  }

  ros::NodeHandle nh_, nh_private_;
  tf::TransformBroadcaster broadcaster_;
  ros::Publisher arMarkerPub_;

  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber camera_sub_;

  // **** parameters
  std::string markerFrame_;
  bool publishTf_;
  bool useHistory_;
  int threshold_;
  double markerWidth_;        // Size of the AR Marker in mm

  int patt_id_;               // AR Marker Pattern
  bool reverse_transform_;    // Reverse direction of transform marker -> cam

  double marker_trans_[3][4]; // Marker Transform cache
  double marker_center_[2];   // Physical Center of the Marker

  bool cache_camera_info_;
  bool previous_marker_detected_;
  bool rectified_;
  bool lazy_;

};

} // end namespace ar_pose


int main (int argc, char **argv)
{
  ros::init (argc, argv, "marker_detector");
  ar_pose::ARSinglePublisher arSingle;
  ros::spin();
  return 0;
}

