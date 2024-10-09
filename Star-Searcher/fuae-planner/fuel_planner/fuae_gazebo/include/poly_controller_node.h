#include <thread>
#include <chrono>
#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <std_srvs/Empty.h>

#include "rotors_control/common.h"
#include "rotors_control/lee_position_controller.h"


class LeePositionControllerNode {
 public:
  LeePositionControllerNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
  ~LeePositionControllerNode();

  void InitializeParams();
  void publishRotorVelocities(const mav_msgs::EigenTrajectoryPoint& command_point);
  void publishLeeCmd(const mav_msgs::EigenTrajectoryPoint& command_point);
  void xBounce();
  void lemniscate();
  
  bool is_get_odom;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  rotors_control::LeePositionController lee_position_controller_;

  std::string namespace_;
  nav_msgs::Odometry curr_odom_;
  Eigen::VectorXd curr_rotor_velocity_;

  // subscribers
  ros::Subscriber odometry_sub_;
  ros::Publisher motor_velocity_reference_pub_, pos_cmd_pub_, vel_cmd_pub_, acc_cmd_pub_;
  
  void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
};
