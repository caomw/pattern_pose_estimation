#ifndef PATTERN_POSE_ESTIMATION_MARKER_DETECTOR
#define PATTERN_POSE_ESTIMATION_MARKER_DETECTOR

#include <stdexcept>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <ar_pose/ARMarkers.h>

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

  static const int DEFAULT_THRESHOLD = 100;

  /**
  * Creates a detector with default values
  */
  MarkerDetector();

  /**
  * unloads the markers
  */
  ~MarkerDetector();

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

  void setCameraInfo(const sensor_msgs::CameraInfoConstPtr& camera_info, bool rectified);
  void detect(const sensor_msgs::ImageConstPtr& image, ar_pose::ARMarkersPtr& markers);
  void redetect(const sensor_msgs::ImageConstPtr& image, ar_pose::ARMarkersPtr& markers);

private:

  void detectImpl(const sensor_msgs::ImageConstPtr& image, 
      ar_pose::ARMarkersPtr& markers, bool use_cache);

  void arTransformationToPose(double ar_transformation[3][4], 
      geometry_msgs::Pose& pose);

  struct Marker
  {
    int id;
    double width;
    double center[2];
    int pattern_id;
    double transformation[3][4]; // cache for last transformation
  };

  int threshold_;
  std::vector<Marker> markers_;

};

} // end of namespace

#endif

