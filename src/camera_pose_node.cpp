#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>



class ChessPoseSensor
{
private:
  
  ros::NodeHandle nh_;
  ros::NodeHandle priv_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber subs_;
  ros::Publisher publ_;
  tf::TransformBroadcaster tfbr_;
  geometry_msgs::PoseStamped pose_msg_;
  geometry_msgs::TransformStamped pose_tf_;
    
  std::vector<cv::Point3f> points3d_;
  bool calibrated_;
  int cols_, rows_, num_points_;
  double square_size_;
  
public:

  ChessPoseSensor()
  : nh_(), priv_("~"), it_(nh_)
  {}
  
  void setParams()
  {
    priv_.param("cols",cols_,8);
    priv_.param("rows",rows_,6);
    priv_.param("size",square_size_,0.06);
    priv_.param("calibrated",calibrated_,false);
    num_points_ = rows_*cols_;
    for (int i=0; i<rows_; i++)
      for(int j=0; j<cols_; j++)
        points3d_.push_back(cv::Point3f(j*square_size_,i*square_size_,0.0));
  }
  
  void advertisePoseTopic(const std::string& pose_topic)
  {
    publ_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic,1);
  }
  
  void subscribeImageTopic(const std::string& image_topic)
  {
    subs_ = it_.subscribeCamera(image_topic,1,&ChessPoseSensor::detect_cb, this);
  }
    
  void report(cv::Mat t_vec, cv::Mat r_vec, cv::Mat r_qua, cv::Mat r_mat)
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
    
    pose_msg_.header.stamp = stamp;
    pose_msg_.header.frame_id = "camera";
    pose_msg_.position.x = t_vec.at<double>(0,0);
    pose_msg_.position.y = t_vec.at<double>(1,0);
    pose_msg_.position.z = t_vec.at<double>(2,0);
    pose_msg_.orientation = quat;
    
    tf_.header.stamp = stamp;
    tf_.header.frame_id = "camera";
    tf_.child_frame_id = "chessboard";
    tf_.setOrigin( tf::Vector3(t_vec.at<double>(0,0), t_vec.at<double>(1,0), t_vec.at<double>(2,0) ) );
    tf_.setRotation(quat);
    
    publ_.publish(pose_msg_);
    tfbr_.sendTransform(tf_);
  }
  
  void detect_cb(const sensor_msgs::Image& image, const sensor_msgs::CameraInfo& cam_info)
  { 
    const std::string& encoding = image->encoding;
    int image_type = CV_8UC1;
    if (encoding == enc::BGR8 || encoding == enc::RGB8)
      image_type = CV_8UC3;
    const cv::Mat mat(image.height, image.width, image_type,
                      const_cast<uint8_t*>(&image->data[0]), image->step);
    std::vector<cv::Point2f> corners;
    bool success = findChessboardCorners(mat,cv::Size(cols,rows),corners,cv::CV_CALIB_CB_ADAPTIVE_THRESH | cv::CV_CALIB_CB_NORMALIZE_IMAGE);
    if (!success)
    {
      ROS_WARN_STREAM("Chessboard not detected");
      return;
    }
    corners = findCornerSubpix(mat,corners, num_points_, cv::Size(5,5), cv::Size(-1,-1), 
                               ( cv.CV_TERMCRIT_EPS+cv.CV_TERMCRIT_ITER, 30, 0.1 ));
    cv::Mat K(3,3, CV_64FC1, cam_info.camera_matrix);
    cv::Mat D(4,1, CV_64FC1, cam_info.dist_coeffs);
    cv::Mat t_vec(3,1,CV_64FC1);
    cv::Mat r_vec(3,1,CV_64FC1);
    cv::Mat r_mat(3,3,CV_64FC1);
    cv::Mat r_qua(3,1,CV_64FC1);
    if (calibrated)
      findExtrinsicParameters(corners,points3d,K,D,&t_vec,&r_vec);
    else
      findExtrinsicParameters(corners,points3d,K,0,&t_vec,&r_vec);
    cv::Rodrigues2(r_vec,r_mat);
    report(t_vec,r_vec,r_mat);
    ros::Time stamp = image.header.stamp;
    if (stamp.toSec()==0.0)
      stamp = ros::Time::now();
    sendMessageAndTransform(t_vec,r_quat,stamp);
    cv::Mat draw = mat.clone();
    cv::DrawChessboardCorners(draw, cv::Size(cols,rows), corners);
    cv::imShow(draw,"chessboard");
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc,argv,"checkerboard_pose_node");
  ChessPoseSensor cps;
  cps.setParams();
  cps.advertisePoseTopic("checkerboard_pose");
  cps.subscribeImageTopic("image");
  ros::spin();
}
