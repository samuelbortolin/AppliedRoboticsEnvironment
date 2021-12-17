#pragma once

#include <ros/ros.h>
#include <vector>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PolygonStamped.h"
#include "jsk_recognition_msgs/PolygonArray.h"
#include "planning/ComputePlan.h"
#include "utils.hpp"

namespace planning {


class PlanningHandle {

public:
  // Constructor
  explicit PlanningHandle();
    
  void onInit(ros::NodeHandle &nodeHandle);

private:
  // Methods
  void loadParameters();  
  void publishToTopics();
  void subscribeToTopic(); 
  void initServices();
  
  // ROS node handle
  ros::NodeHandle nh_;
  bool initialized_; // check if node handle is initialized
  int queue_size_;
  bool default_implementation_;
  std::string config_folder_;
  
  // ROS communication  
  ros::Subscriber sub_victims_, sub_obstacles_, sub_gate_, sub_robot_;
  ros::Subscriber sub_robot0_, sub_robot1_, sub_robot2_;
  ros::Publisher  pub_plan_, pub_dt_, pub_plan_rviz_;
  ros::Publisher  pub_plan0_, pub_plan1_, pub_plan2_, pub_plan0_rviz_, pub_plan1_rviz_, pub_plan2_rviz_;
  ros::ServiceServer srv_plan_;
  bool has_victims_, has_obstacles_, has_gate_, has_robot_;

  // TOPICS  
  std::string sub_victims_topic_name_, sub_obstacles_topic_name_, sub_gate_topic_name_, sub_robot_topic_name_;
  std::string pub_plan_topic_name_, pub_dt_topic_name_, pub_plan_rviz_topic_name_;
  std::string srv_plan_topic_name_;

  std::vector<Polygon> obstacle_list_, gate_list_;
  //std::vector<std::pair<int,Polygon>> victim_list_;
  Polygon gate_, borders_;
  std::vector<float> x_, y_, theta_;
  double x0_, x1_, x2_, y0_, y1_, y2_, theta0_, theta1_, theta2_;
  std::vector<Path> path_;

  std_msgs::Header header_;

  // Callback
  void robotCb0(const geometry_msgs::PoseStampedPtr robot_pose);
  void robotCb1(const geometry_msgs::PoseStampedPtr robot_pose);
  void robotCb2(const geometry_msgs::PoseStampedPtr robot_pose);
  void victimsCb(const jsk_recognition_msgs::PolygonArrayPtr victims);
  void obstaclesCb(const jsk_recognition_msgs::PolygonArrayPtr obstacles);
  void gateCb(const jsk_recognition_msgs::PolygonArrayPtr gate);
  void gatesCb(const jsk_recognition_msgs::PolygonArrayPtr gates);

  // Service
  bool computePlanSrv(ComputePlan::Request& req, ComputePlan::Response& res);

};

}
