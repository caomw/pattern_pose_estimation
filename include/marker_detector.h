#ifndef PATTERN_POSE_ESTIMATION_MARKER_DETECTOR
#define PATTERN_POSE_ESTIMATION_MARKER_DETECTOR

#include <ros/ros.h>

#include <stdexcept>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <ar_pose/ARMarkers.h>
#include <image_transport/image_transport.h>

#define DEFAULT_THRESHOLD 100

namespace pattern_pose_estimation
{

class MarkerDetectorException : public std::runtime_error
{
public:
  MarkerDetectorException(const std::string& what) : std::runtime_error(what) {};
};

class MarkerDetector
{
public:

  //static const int DEFAULT_THRESHOLD = 100;

  /**
  * Creates a detector with default values
  */
  MarkerDetector();

  /**
  * unloads the markers
  */
  ~MarkerDetector();

  /**
  * loads all settings using given node handle. will load threshold and markers
  */
  void loadSettings(ros::NodeHandle& nh);

  /**
  * Loads a marker for the detector. Every marker that has to be detected has
  * to be loaded before using this method.
  * \param marker_id id of the marker (which will be returned on detection)
  * \param pattern_file_name file name of the pattern file that describes the marker
  * \param width the width of the printed marker in meters
  * \param x x-shift of marker center in meters
  * \param y y-shift of marker center in meters
  * \throws MarkerDetectorException if loading fails.
  */
  void loadMarker(int marker_id, const std::string& pattern_file_name, double width,
      double x = 0.0, double y = 0.0);

  /**
  * \param threshold new threshold to use for binarization, must be between 0 and 255
  */
  inline void setThreshold(int threshold)
  {
    ROS_ASSERT(threshold >= 0 && threshold <= 255);
    threshold_ = threshold;
  }

  void setCameraInfo(const sensor_msgs::CameraInfo& camera_info, bool rectified);
  void detect(const sensor_msgs::Image& image, ar_pose::ARMarkers& markers);
  void redetect(const sensor_msgs::Image& image, ar_pose::ARMarkers& markers);

private:

  std::string resolveURL(const std::string& url) const;

  void detectImpl(const sensor_msgs::Image& image,
      ar_pose::ARMarkers& markers, bool use_cache);

  void arTransformationToPose(double ar_transformation[3][4],
      geometry_msgs::Pose& pose);

  enum DetectionFlag
  {
    NOT_DETECTED, // marker was not detected in this and last run
    DETECTED,     // marker is detected in current run
    LAST_DETECTED // marker was detected in last run
  };

  struct Marker
  {
    int id;
    double width;
    double center[2];
    int pattern_id;
    double transformation[3][4]; // cache for last transformation
    DetectionFlag detection_flag;
  };

  int threshold_;
  bool show_debug_image_;
  bool camera_initialized_;
  std::vector<Marker> markers_;

  image_transport::Publisher debug_img_pub_;

  static bool has_instance_;


};

} // end of namespace

#endif

