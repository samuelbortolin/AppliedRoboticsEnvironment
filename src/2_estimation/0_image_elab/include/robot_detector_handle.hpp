#pragma once

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <image_elab/PlaneTransform.h>

#include <vector>

#include "utilsIP.hpp"

namespace image_proc {

class RobotDetectorHandle {

 public:
  // Constructor
  explicit RobotDetectorHandle();
    
  void onInit(ros::NodeHandle &nodeHandle);

private:
  // Methods
  void loadParameters();  
  void publishToTopics();
  void subscribeToTopic(); // when a subscruiber want the rectify img
  
  // ROS node handle
  ros::NodeHandle nh_;
  bool initialized_; // check if node handle is initialized
  int queue_size_;
  bool default_implementation_;
  std::string config_folder_;
  
  // ROS communication  
  ros::Subscriber sub_image_, sub_transf_;
  ros::Publisher  pub_robot_;
  ros::Publisher  pub_gps_odom_, pub_gps_loc_, pub_dt_;
  
  // TOPICS  
  std::string frame_id_;
  std::string sub_image_topic_name_, sub_transf_topic_name_;
  std::string pub_gps_odom_topic_name_, pub_robot_topic_name_, pub_gps_loc_topic_name_, pub_dt_topic_name_;

  // CALIBRATION MATRIX  
  bool has_transform_;
  cv::Mat transform_;  
  double scale_;

  Polygon triangle_;
  double x_, y_, theta_;

  std::string robot_ns_;

  // Callback
  void imageCb(const sensor_msgs::ImageConstPtr& image_msg);
  void transformCb(const image_elab::PlaneTransform& transf);
};
}
