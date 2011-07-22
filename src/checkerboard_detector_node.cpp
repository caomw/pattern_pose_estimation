#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <iostream>



class CheckerboardDetector
{
private:
  
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber camera_sub_;
  ros::Publisher pose_pub_;
  tf::TransformBroadcaster tf_broadcaster_;
    
  std::vector<cv::Point3f> points3d_; // known 3D points
  bool calibrated_; // is input image calibrated ?
  // pattern params
  int cols_, rows_;
  double square_size_;

  bool show_detection_; // draw detection on screen?
  
public:

  CheckerboardDetector()
  : nh_(), nh_priv_("~"), it_(nh_)
  {
      init();
  }

  void init()
  {
    nh_priv_.param("cols", cols_, 8);
    nh_priv_.param("rows", rows_, 6);
    nh_priv_.param("size", square_size_, 0.06);
    nh_priv_.param("calibrated", calibrated_, false);
    nh_priv_.param("show_detection", show_detection_, false);
    ROS_INFO_STREAM("Calibrated is set to " << calibrated_);
    ROS_INFO_STREAM("Calibration pattern parameters: " << rows_ << "x" << cols_ << ", size is " << square_size_);
    for (int i=0; i<rows_; i++)
      for(int j=0; j<cols_; j++)
        points3d_.push_back(cv::Point3f(j*square_size_,i*square_size_,0.0));

    ROS_INFO_STREAM("Subscribing to image ropic " << nh_.resolveName("image"));
    camera_sub_ = it_.subscribeCamera("image", 1, &CheckerboardDetector::detect, this);

    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("checkerboard_pose", 1);
  }
  
  void report(const cv::Mat& t_vec, const cv::Mat& r_vec, const cv::Mat& r_mat)
  {
    std::cout << "Current checkerboard pose is:\n";
    std::cout << "Rv = [ " << r_vec.at<double>(0,0) << " " << r_vec.at<double>(1,0) << " " << r_vec.at<double>(2,0) << " ]\n";
    std::cout << "R  = [ " << r_mat.at<double>(0,0) << " " << r_mat.at<double>(0,1) << " " << r_mat.at<double>(0,2) << " ]\n"; 
    std::cout << "     [ " << r_mat.at<double>(1,0) << " " << r_mat.at<double>(1,1) << " " << r_mat.at<double>(1,2) << " ]\n";
    std::cout << "     [ " << r_mat.at<double>(2,0) << " " << r_mat.at<double>(2,1) << " " << r_mat.at<double>(2,2) << " ]\n"; 
    std::cout << "T  = [ " << t_vec.at<double>(0,0) << " " << t_vec.at<double>(1,0) << " " << t_vec.at<double>(2,0) << " ]\n";
  }
  
  void sendMessageAndTransform(const cv::Mat& t_vec, const cv::Mat& r_vec, const ros::Time& stamp, const std::string& camera_frame_id)
  {
    tf::Vector3 axis(r_vec.at<double>(0, 0), r_vec.at<double>(1, 0), r_vec.at<double>(2, 0));
    double angle = cv::norm(r_vec);
    tf::Quaternion quaternion(axis, angle);
    
    tf::Vector3 translation(t_vec.at<double>(0, 0), t_vec.at<double>(1, 0), t_vec.at<double>(2, 0));

    tf::Transform transform(quaternion, translation);
    tf::StampedTransform stamped_transform(transform, stamp, camera_frame_id, "checkerboard");
    tf_broadcaster_.sendTransform(stamped_transform);

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = camera_frame_id;
    tf::poseTFToMsg(transform, pose_msg.pose);

    pose_pub_.publish(pose_msg);
  }
  
  void detect(const sensor_msgs::ImageConstPtr& image, 
          const sensor_msgs::CameraInfoConstPtr& cam_info)
  { 
    namespace enc = sensor_msgs::image_encodings;
    const std::string& encoding = image->encoding;
    int image_type = CV_8UC1;
    if (encoding == enc::BGR8 || encoding == enc::RGB8)
      image_type = CV_8UC3;
    const cv::Mat mat(image->height, image->width, image_type,
                      const_cast<uint8_t*>(&image->data[0]), image->step);
    std::vector<cv::Point2f> corners;
    bool success = cv::findChessboardCorners(mat, cv::Size(cols_, rows_), corners);
    if (!success)
    {
      ROS_WARN_STREAM("Checkerboard not detected");
      return;
    }
    cv::cornerSubPix(mat, corners, cv::Size(5,5), cv::Size(-1,-1), 
            cv::TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
    const cv::Mat K(3,3, CV_64FC1, const_cast<double*>(cam_info->K.data()));
    const cv::Mat D(4,1, CV_64FC1, const_cast<double*>(cam_info->D.data()));
    cv::Mat t_vec(3,1,CV_64FC1);
    cv::Mat r_vec(3,1,CV_64FC1);
    cv::Mat r_mat(3,3,CV_64FC1);
    cv::Mat points_mat(points3d_);
    cv::Mat corners_mat(corners);
    if (calibrated_)
      cv::solvePnP(points_mat, corners_mat, K, cv::Mat(), r_vec, t_vec);
    else
      cv::solvePnP(points_mat, corners_mat, K, D, r_vec, t_vec);
    cv::Rodrigues(r_vec, r_mat);

    report(t_vec, r_vec, r_mat);
    ros::Time stamp = image->header.stamp;
    if (stamp.toSec()==0.0)
      stamp = ros::Time::now();
    sendMessageAndTransform(t_vec, r_vec, stamp, image->header.frame_id);

    if (show_detection_)
    {
        cv::Mat draw;
        cv::cvtColor(mat, draw, CV_GRAY2BGR);
        cv::drawChessboardCorners(draw, cv::Size(cols_,rows_), corners, true);
        cv::imshow("checkerboard", draw);
        cv::waitKey(5);
    }
  }
};

int main(int argc, char* argv[])
{
  ros::init(argc,argv,"checkerboard_detector");
  CheckerboardDetector cbd;
  ros::spin();
}
