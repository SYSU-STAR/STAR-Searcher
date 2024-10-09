/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
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
  void init();
  void publishRotorVelocities(const mav_msgs::EigenTrajectoryPoint& command_point);
  void publishLeeCmd(const mav_msgs::EigenTrajectoryPoint& command_point);
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

