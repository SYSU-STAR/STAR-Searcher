#ifndef _ASTAR2_H
#define _ASTAR2_H

#include "plan_env/edt_environment.h"
#include <Eigen/Eigen>
#include <boost/functional/hash.hpp>
#include <iostream>
#include <map>
#include <path_searching/matrix_hash.h>
#include <queue>
#include <ros/console.h>
#include <ros/ros.h>
#include <string>
#include <unordered_map>
#include <visualization_msgs/MarkerArray.h>

namespace fast_planner {
class Node {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef shared_ptr<Node> Ptr;
  typedef shared_ptr<const Node> ConstPtr;

  Eigen::Vector3i index;
  Eigen::Vector3d position;
  double g_score, f_score;
  Node *parent;

  /* -------------------- */
  Node() { parent = NULL; }
  ~Node(){};
};
typedef Node *NodePtr;

class NodeComparator0 {
public:
  bool operator()(NodePtr node1, NodePtr node2) { return node1->f_score > node2->f_score; }
};

class Astar {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef shared_ptr<Astar> Ptr;
  typedef shared_ptr<const Astar> ConstPtr;

  Astar();
  ~Astar();
  enum { REACH_END = 1, NO_PATH = 2 };

  void init(ros::NodeHandle &nh, const EDTEnvironment::Ptr &env);
  void reset();
  int search(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt);
  int searchUnknown(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt);
  int searchUnknownBBox(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt,
                        const Eigen::Vector3d &bbox_min, const Eigen::Vector3d &bbox_max);
  void setResolution(const double &res);
  void setMaxSearchTime(const double &t);
  static double pathLength(const vector<Eigen::Vector3d> &path);

  std::vector<Eigen::Vector3d> getPath();
  std::vector<Eigen::Vector3d> getVisited();
  double getEarlyTerminateCost();

  double lambda_heu_;
  double max_search_time_;
  double max_search_time_unknown_;

private:
  void backtrack(const NodePtr &end_node, const Eigen::Vector3d &end);
  void posToIndex(const Eigen::Vector3d &pt, Eigen::Vector3i &idx);
  double getDiagHeu(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2);
  double getEuclHeu(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2);
  double getManhHeu(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2);

  // Debug visualization
  void publishNode(NodePtr node);

  // main data structure
  vector<NodePtr> path_node_pool_;
  int use_node_num_, iter_num_;
  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> open_set_;
  std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash<Eigen::Vector3i>> open_set_map_;
  std::unordered_map<Eigen::Vector3i, int, matrix_hash<Eigen::Vector3i>> close_set_map_;
  std::vector<Eigen::Vector3d> path_nodes_;
  double early_terminate_cost_;

  EDTEnvironment::Ptr edt_env_;

  // parameter
  // resolution: larger -> faster, less optimal
  // lambda_heu: larger -> more straight line, less optimal
  double margin_;
  int allocate_num_;
  double tie_breaker_;
  double resolution_, inv_resolution_;
  double epsilon_;
  Eigen::Vector3d map_size_3d_, origin_;

  ros::Publisher astar_pub_;
};

} // namespace fast_planner

#endif