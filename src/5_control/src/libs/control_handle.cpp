#include "control_handle.hpp"

#include "std_msgs/Float32.h" // used to publish dt
#include "nav_msgs/Path.h"
#include "planning/WaypointList.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/Twist.h"
#include "visualization_msgs/Marker.h"
#include "path_following.hpp"

#include <assert.h>
#include <sstream>


const std::string kPringName = "control_handle.hpp";

namespace control {

    nav_msgs::Path createRVIZPath(const planning::WaypointList& path){
        nav_msgs::Path res;
        res.header = path.header;
        for (const auto& pt: path.wp) {
            geometry_msgs::PoseStamped wp;
            wp.header = path.header;
            wp.pose.position.x = pt.x;
            wp.pose.position.y = pt.y;
            wp.pose.orientation = tf::createQuaternionMsgFromYaw(pt.theta);
            res.poses.push_back(wp);
        }
        return res;   
    }

    Path createPath(const planning::WaypointList& path){
        Path res;
        for (const auto& pt: path.wp) {
            res.points.emplace_back(pt.s, pt.x, pt.y, pt.theta, pt.kappa);
        }
        return res;   
    }

    // Constructor
    ControlHandle::ControlHandle(){
        ROS_DEBUG_NAMED(kPringName, "Constructor");
        initialized_   = false;          
        has_robot_     = false;
        as_ = nullptr;
    }

    ControlHandle::~ControlHandle(){
        if(as_){
            delete as_;
        }
    }

    void ControlHandle::onInit(ros::NodeHandle &nodeHandle){
        nh_ = nodeHandle;
        initialized_ = true;

        loadParameters();    
        publishToTopics();
        subscribeToTopic();  
        initService();

        as_ = new actionlib::SimpleActionServer<FollowPathAction>(nh_, as_topic_name_,boost::bind(&ControlHandle::executeCB, this, _1), false );

        as_->start();
    }

    template <class T>
    void loadVariable(ros::NodeHandle &nh, std::string variable_name, T* ret_val){
        T val;    
        if (!nh.getParam(variable_name, *ret_val)) 
        {          
            std::stringstream ss;
            
            ROS_ERROR_STREAM("Did not load " << variable_name);
            throw std::logic_error("Did not load " + variable_name);
        }
    }


    // Methods
    void ControlHandle::loadParameters() {
        ROS_DEBUG_NAMED(kPringName, "Loading Params");

        //loadVariable<bool>(nh_,"/default_implementation/control", &default_implementation_);
        
        queue_size_ = 1;

        srv_plan_topic_name_      = "/planning/compute_plan";
        sub_robot_topic_name_     = "/ideal/odom"; // "/ideal/odom";"" /estimation/map"
        as_topic_name_            = "/control/action";
        pub_control_topic_name_   = "/control/cmd_vel";
        pub_dt_topic_name_        = "/process_time/control";
        pub_plan_rviz_topic_name_   = "/control/plan_rviz";
        pub_frenet_rviz_topic_name_ = "/control/frenet_rviz";

        v_ref_ = 0.1;

    }

    void ControlHandle::publishToTopics() {
        ROS_DEBUG_NAMED(kPringName, "Init publishers");
        assert (initialized_);
        
        x_.resize(3);
        y_.resize(3);
        theta_.resize(3);
        v_.resize(3);
        omega_.resize(3);
        path_.resize(3);
        pub_plans_rviz_.resize(3);
        pub_controls_.resize(3);
        pub_frenets_rviz_.resize(3);

        //pub_plan_rviz_ = nh_.advertise<nav_msgs::Path>(pub_plan_rviz_topic_name_, 1, true);
        pub_plans_rviz_[0] = nh_.advertise<nav_msgs::Path>("/my_robot_0" + pub_plan_rviz_topic_name_, 1, true);
        pub_plans_rviz_[1] = nh_.advertise<nav_msgs::Path>("/my_robot_1" + pub_plan_rviz_topic_name_, 1, true);
        pub_plans_rviz_[2] = nh_.advertise<nav_msgs::Path>("/my_robot_2" + pub_plan_rviz_topic_name_, 1, true);
        pub_dt_ = nh_.advertise<std_msgs::Float32>(pub_dt_topic_name_, 1, true);
        //pub_control_ = nh_.advertise<geometry_msgs::Twist>(pub_control_topic_name_, 1, false);
        pub_controls_[0] = nh_.advertise<geometry_msgs::Twist>("/my_robot_0" + pub_control_topic_name_, 1, false);
        pub_controls_[1] = nh_.advertise<geometry_msgs::Twist>("/my_robot_1" + pub_control_topic_name_, 1, false);
        pub_controls_[2] = nh_.advertise<geometry_msgs::Twist>("/my_robot_2" + pub_control_topic_name_, 1, false);
        //pub_frenet_rviz_ = nh_.advertise<visualization_msgs::Marker>(pub_frenet_rviz_topic_name_, 1, false);
        pub_frenets_rviz_[0] = nh_.advertise<visualization_msgs::Marker>("/my_robot_0" + pub_frenet_rviz_topic_name_, 1, false);
        pub_frenets_rviz_[1] = nh_.advertise<visualization_msgs::Marker>("/my_robot_1" + pub_frenet_rviz_topic_name_, 1, false);
        pub_frenets_rviz_[2] = nh_.advertise<visualization_msgs::Marker>("/my_robot_2" + pub_frenet_rviz_topic_name_, 1, false);
    }

    void ControlHandle::subscribeToTopic() {
        ROS_DEBUG_NAMED(kPringName, "Init subscribers");
        assert (initialized_);
         
        //sub_robot_ = nh_.subscribe(sub_robot_topic_name_, queue_size_, &ControlHandle::robotCb, this);
        sub_robot0_ = nh_.subscribe("/my_robot_0" + sub_robot_topic_name_, queue_size_, &ControlHandle::robotCb0, this);
        sub_robot1_ = nh_.subscribe("/my_robot_1" + sub_robot_topic_name_, queue_size_, &ControlHandle::robotCb1, this);
        sub_robot2_ = nh_.subscribe("/my_robot_2" + sub_robot_topic_name_, queue_size_, &ControlHandle::robotCb2, this);
    }

    void ControlHandle::initService() {
        srv_plan_ = nh_.serviceClient<planning::ComputePlan>(srv_plan_topic_name_);
    }

    void ControlHandle::executeCB(const FollowPathGoalConstPtr &goal){
        bool ok = false;

        // WAIT FOR PATH SERVICE
        {
            feedback_.status = "waiting_path";
            feedback_.s_done = 0.f;
            feedback_.s_tot0 = 0.f;
            feedback_.s_tot1 = 0.f;
            feedback_.s_tot2 = 0.f;

            ros::Rate r(5);

            planning::ComputePlan srv;
            while (!ok && (!as_->isPreemptRequested() && ros::ok())) {

                if (srv_plan_.call(srv))
                {
                    ok = srv.response.status == 0;
                    if (!ok) {
                        ROS_WARN_STREAM("Planning returned " << srv.response.status);
                    }
                    else {
                        path_[0] = createPath(srv.response.wp0);
                        path_[1] = createPath(srv.response.wp1);
                        path_[2] = createPath(srv.response.wp2);
                        break;
                    }
                }

                as_->publishFeedback(feedback_);
                r.sleep();
            }
        }

        if (!ok) {
            ROS_INFO("%s: Preempted", as_topic_name_.c_str());
                // set the action state to preempted
            as_->setPreempted();
            return;
        }

        feedback_.status = "navigation";
        feedback_.s_done = 0.f;
        if (!path_[0].points.empty()) feedback_.s_tot0 = path_[0].points.back().s;
        if (!path_[1].points.empty()) feedback_.s_tot1 = path_[1].points.back().s;
        if (!path_[2].points.empty()) feedback_.s_tot2 = path_[2].points.back().s;

        std::vector<PathFollowing> pf(3);
        pf[0].setPath(path_[0]);
        pf[1].setPath(path_[1]);
        pf[2].setPath(path_[2]);
        
        ros::Rate r(20);
        bool success = true;
        const auto start_time = ros::Time::now();
        geometry_msgs::Twist cmd_vel;

        std::vector<bool> finito = {false, false, false};
        bool loopa = true;
        while (loopa) {
            if (as_->isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", as_topic_name_.c_str());
                // set the action state to preempted
                as_->setPreempted();
                success = false;
                
                break;
            }

            for (int robot=0; robot<3; robot++) {
                if (path_[robot].points.empty()) {
                    finito[robot] = true;
                    continue;
                }

                // CALL AL CONTROLLO
                double fp_s, fp_x, fp_y;

                const auto start_control = ros::Time::now();
                const float rho = pf[robot].computeControl(x_[robot], y_[robot], theta_[robot], fp_s, fp_x, fp_y);
                std_msgs::Float32 dt_msg;
                dt_msg.data = (ros::Time::now()-start_control).toSec()/3;
                pub_dt_.publish(dt_msg);

                //std::cerr << "STATE: " << rho << " " << x_ << " " << y_ << " " << theta_ << std::endl;
                        
                cmd_vel.linear.x  =   v_ref_;
                cmd_vel.angular.z = - v_ref_*rho;
                pub_controls_[robot].publish(cmd_vel);

                // Publish marker
                visualization_msgs::Marker frenet_point;
                frenet_point.header.frame_id = "map";
                frenet_point.header.stamp = ros::Time();
                frenet_point.ns = "control";
                frenet_point.id = 0;
                frenet_point.type = visualization_msgs::Marker::SPHERE;
                frenet_point.action = visualization_msgs::Marker::ADD;
                frenet_point.pose.position.x = fp_x;
                frenet_point.pose.position.y = fp_y;
                frenet_point.pose.position.z = 0.;
                frenet_point.pose.orientation.x = 0.0;
                frenet_point.pose.orientation.y = 0.0;
                frenet_point.pose.orientation.z = 0.0;
                frenet_point.pose.orientation.w = 1.0;
                frenet_point.scale.x = 0.02;
                frenet_point.scale.y = 0.02;
                frenet_point.scale.z = 0.02;
                frenet_point.color.a = 1.0;
                frenet_point.color.r = 1.0;
                frenet_point.color.g = 0.0;
                frenet_point.color.b = 0.0;            
                pub_frenets_rviz_[robot].publish(frenet_point);

                // publish the feedback
                feedback_.s_done = fp_s;
                as_->publishFeedback(feedback_);
                r.sleep();

                if (pf[robot].endOfPath()) {
                    cmd_vel.linear.x = 0.f;
                    cmd_vel.angular.z = 0.f;
                    pub_controls_[robot].publish(cmd_vel);
                    finito[robot] = true;
                }
            }

            if (std::find(begin(finito), std::end(finito), false) == std::end(finito)) {
                loopa = false;
            }
        }


        if(success){
            result_.travel_time = (ros::Time::now()-start_time).toSec();
            ROS_INFO("%s: Succeeded", as_topic_name_.c_str());
            // set the action state to succeeded
            as_->setSucceeded(result_);      
            
            //SEND STOP AL CONTROLLO
            for (int robot=0; robot<3; robot++) {
              cmd_vel.linear.x = 0.f;
              cmd_vel.angular.z = 0.f;
              for (int i=0; i<10; ++i) {
                  pub_controls_[robot].publish(cmd_vel);
                  r.sleep();
              }
            }
        }
        return;
    }

    void ControlHandle::robotCb0(const nav_msgs::OdometryPtr robot_state){
        x_[0] = robot_state->pose.pose.position.x;
        y_[0] = robot_state->pose.pose.position.y;
        tf::Quaternion q(robot_state->pose.pose.orientation.x, 
            robot_state->pose.pose.orientation.y, 
            robot_state->pose.pose.orientation.z, 
            robot_state->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        theta_[0] = yaw + M_PI;

        v_[0] = robot_state->twist.twist.linear.x;
        omega_[0] = robot_state->twist.twist.angular.z;

        has_robot_ = true;
    }

    void ControlHandle::robotCb1(const nav_msgs::OdometryPtr robot_state){
        x_[1] = robot_state->pose.pose.position.x;
        y_[1] = robot_state->pose.pose.position.y;
        tf::Quaternion q(robot_state->pose.pose.orientation.x, 
            robot_state->pose.pose.orientation.y, 
            robot_state->pose.pose.orientation.z, 
            robot_state->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        theta_[1] = yaw + M_PI;

        v_[1] = robot_state->twist.twist.linear.x;
        omega_[1] = robot_state->twist.twist.angular.z;

        has_robot_ = true;
    }

    void ControlHandle::robotCb2(const nav_msgs::OdometryPtr robot_state){
        x_[2] = robot_state->pose.pose.position.x;
        y_[2] = robot_state->pose.pose.position.y;
        tf::Quaternion q(robot_state->pose.pose.orientation.x, 
            robot_state->pose.pose.orientation.y, 
            robot_state->pose.pose.orientation.z, 
            robot_state->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        theta_[2] = yaw + M_PI;

        v_[2] = robot_state->twist.twist.linear.x;
        omega_[2] = robot_state->twist.twist.angular.z;

        has_robot_ = true;
    }
    
}

