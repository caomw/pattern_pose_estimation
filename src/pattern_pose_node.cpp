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



class ChessPoseSensor
{
private:
  
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber camera_sub_;
  ros::Publisher pose_pub_;
  tf::TransformBroadcaster tfbr_;
    
  std::vector<cv::Point3f> points3d_;
  bool calibrated_;
  int cols_, rows_, num_points_;
  double square_size_;
  
public:

  ChessPoseSensor()
  : nh_(), nh_priv_("~"), it_(nh_)
  {}

  void init()
  {
    nh_priv_.param("cols",cols_,8);
    nh_priv_.param("rows",rows_,6);
    nh_priv_.param("size",square_size_,0.06);
    nh_priv_.param("calibrated",calibrated_,false);
    num_points_ = rows_*cols_;
    for (int i=0; i<rows_; i++)
      for(int j=0; j<cols_; j++)
        points3d_.push_back(cv::Point3f(j*square_size_,i*square_size_,0.0));

    camera_sub_ = it_.subscribeCamera("image", 1, &ChessPoseSensor::detect, this);

    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pattern_pose", 1);
  }
  
  void report(const cv::Mat& t_vec, const cv::Mat& r_vec, 
          const cv::Mat& r_qua, const cv::Mat& r_mat)
  {
    std::cout << "Current chess board pose is:\n";
    std::cout << "Rv = [ " << r_vec.at<double>(0,0) << " " << r_vec.at<double>(1,0) << " " << r_vec.at<double>(2,0) << " ]\n";
    std::cout << "Rq = [ " << r_qua.at<double>(0,0) << " " << r_qua.at<double>(1,0) << " " << r_qua.at<double>(2,0) << r_qua.at<double>(3,0) << " ]\n";
    std::cout << "R  = [ " << r_mat.at<double>(0,0) << " " << r_mat.at<double>(0,1) << " " << r_mat.at<double>(0,2) << " ]\n"; 
    std::cout << "     [ " << r_mat.at<double>(1,0) << " " << r_mat.at<double>(1,1) << " " << r_mat.at<double>(1,2) << " ]\n";
    std::cout << "     [ " << r_mat.at<double>(2,0) << " " << r_mat.at<double>(2,1) << " " << r_mat.at<double>(2,2) << " ]\n"; 
    std::cout << "T  = [ " << t_vec.at<double>(0,0) << " " << t_vec.at<double>(1,0) << " " << t_vec.at<double>(2,0) << " ]\n";
  }
  
  void sendMessageAndTransform(const cv::Mat& t_vec, const cv::Mat& r_quat, const ros::Time& stamp)
  {
    geometry_msgs::Quaternion quat;
    quat.w = r_quat.at<double>(0,0);
    quat.x = r_quat.at<double>(1,0);
    quat.y = r_quat.at<double>(2,0);
    quat.z = r_quat.at<double>(3,0);

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = "camera";
    pose_msg.pose.position.x = t_vec.at<double>(0,0);
    pose_msg.pose.position.y = t_vec.at<double>(1,0);
    pose_msg.pose.position.z = t_vec.at<double>(2,0);
    pose_msg.pose.orientation = quat;
    
    pose_pub_.publish(pose_msg);

    /*
    tf_.header.stamp = stamp;
    tf_.header.frame_id = "camera";
    tf_.child_frame_id = "chessboard";
    tf_.setOrigin( tf::Vector3(t_vec.at<double>(0,0), t_vec.at<double>(1,0), t_vec.at<double>(2,0) ) );
    tf_.setRotation(quat);
    
    tfbr_.sendTransform(tf_);
    */
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
      ROS_WARN_STREAM("Chessboard not detected");
      return;
    }
    cv::cornerSubPix(mat, corners, cv::Size(5,5), cv::Size(-1,-1), 
            cv::TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
    cv::Mat K(3,3, CV_64FC1, cam_info->P[0]);
    cv::Mat D(4,1, CV_64FC1, cam_info->D[0]);
    cv::Mat t_vec(3,1,CV_64FC1);
    cv::Mat r_vec(3,1,CV_64FC1);
    cv::Mat r_mat(3,3,CV_64FC1);
    cv::Mat r_qua(3,1,CV_64FC1);
    cv::Mat points_mat(points3d_);
    cv::Mat corners_mat(corners);
    if (calibrated_)
      cv::solvePnP(points_mat, corners_mat, K, D, t_vec, r_vec);
    else
      cv::solvePnP(points_mat, corners_mat, K, cv::Mat(), t_vec, r_vec);
    cv::Rodrigues(r_vec, r_mat);
    cv::Mat r_quat = cv::Mat::zeros(4, 1, CV_64F);
    report(t_vec, r_vec, r_quat, r_mat);
    ros::Time stamp = image->header.stamp;
    if (stamp.toSec()==0.0)
      stamp = ros::Time::now();
    sendMessageAndTransform(t_vec, r_quat, stamp);
    cv::Mat draw = mat.clone();
    cv::drawChessboardCorners(draw, cv::Size(cols_,rows_), corners, true);
    cv::imshow("chessboard", draw);
    cv::waitKey(5);
  }
};

int main(int argc, char* argv[])
{
  ros::init(argc,argv,"pattern_pose_node");
  ChessPoseSensor cps;
  ros::spin();
}
