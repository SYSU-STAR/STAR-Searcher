#ifndef _FAST_EXPLORATION_FSM_H_
#define _FAST_EXPLORATION_FSM_H_

#include <Eigen/Eigen>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using Eigen::Vector3d;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
class FastPlannerManager;
class FastExplorationManager;
class PlanningVisualization;
class PerceptionUtils;
struct FSMParam;
struct FSMData;

enum EXPL_STATE {
  INIT,
  WAIT_TRIGGER,
  PLAN_TRAJ,
  PUB_TRAJ,
  EXEC_TRAJ,
  FINISH,
  SPIRAL
};

class FastExplorationFSM {
private:
  /* planning utils */
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<FastExplorationManager> expl_manager_;
  shared_ptr<PlanningVisualization> visualization_;
  shared_ptr<PerceptionUtils> percep_utils_;

  shared_ptr<FSMParam> fp_;
  shared_ptr<FSMData> fd_;
  EXPL_STATE state_;

  bool classic_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
  ros::Subscriber trigger_sub_, odom_sub_;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_, spiral_pub_;
  ros::Publisher start_flag_pub; //发布比赛开始相关标志，及是否接收到trigger

  /* helper functions */
  int callExplorationPlanner();
  void transitState(EXPL_STATE new_state, string pos_call);

  /* ROS functions */
  void FSMCallback(const ros::TimerEvent &e);
  void safetyCallback(const ros::TimerEvent &e);
  void frontierCallback(const ros::TimerEvent &e);
  void triggerCallback(const nav_msgs::PathConstPtr &msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
  void visualize();
  void clearVisMarker();

public:
  FastExplorationFSM(/* args */) {}
  ~FastExplorationFSM() {}

  void init(ros::NodeHandle &nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace fast_planner

#endif