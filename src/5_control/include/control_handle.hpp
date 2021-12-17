#pragma once

#include <ros/ros.h>
#include <vector>

#include "planning/WaypointList.h"
#include "nav_msgs/Odometry.h"
#include "utils.hpp"
#include "actionlib/server/simple_action_server.h"
#include "control/FollowPathAction.h"
#include "planning/ComputePlan.h"

namespace control {


class ControlHandle {

public:
  // Constructor
  explicit ControlHandle();
    
  void onInit(ros::NodeHandle &nodeHandle);
  ~ControlHandle();
private:
  // Methods
  void loadParameters();  
  void publishToTopics();
  void subscribeToTopic(); 
  void initService();

  // ROS node handle
  ros::NodeHandle nh_;
  bool initialized_;
  int queue_size_;
  //bool default_implementation_;
  std::string config_folder_;
  
  // ROS communication  
  actionlib::SimpleActionServer<FollowPathAction> *as_;
  ros::Subscriber sub_robot_;
  ros::Subscriber sub_robot0_, sub_robot1_, sub_robot2_;
  ros::Publisher  pub_control_, pub_dt_, pub_plan_rviz_, pub_frenet_rviz_;
  std::vector<ros::Publisher>  pub_controls_, pub_plans_rviz_, pub_frenets_rviz_;
  ros::ServiceClient srv_plan_;

  bool has_robot_;

  // TOPICS  
  std::string as_topic_name_;
  std::string srv_plan_topic_name_;
  std::string sub_robot_topic_name_;
  std::string pub_control_topic_name_, pub_dt_topic_name_, pub_plan_rviz_topic_name_, pub_frenet_rviz_topic_name_;
  
  std::vector<double> x_, y_, theta_, v_, omega_; // ROBOT STATE
  double v_ref_;
  std::vector<Path> path_;

  // Callback
  void robotCb0(const nav_msgs::OdometryPtr robot_state);
  void robotCb1(const nav_msgs::OdometryPtr robot_state);
  void robotCb2(const nav_msgs::OdometryPtr robot_state);
  void executeCB(const FollowPathGoalConstPtr &goal);
  
  FollowPathFeedback feedback_;
  FollowPathResult result_;
};

}
