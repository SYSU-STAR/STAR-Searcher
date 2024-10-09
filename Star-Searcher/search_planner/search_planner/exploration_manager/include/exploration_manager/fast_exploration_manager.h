#ifndef _EXPLORATION_MANAGER_H_
#define _EXPLORATION_MANAGER_H_

#include <Eigen/Eigen>
#include <memory>
#include <ros/ros.h>
#include <string>
#include <vector>

using Eigen::Vector3d;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
class EDTEnvironment;
class SDFMap;
class FastPlannerManager;
class FrontierFinder;
class PlanningVisualization;
struct ExplorationParam;
struct ExplorationData;
struct FrontierCluster;
struct checkPoint;

enum EXPL_RESULT { NO_GRID, FAIL, SUCCEED, NO_FRONTIER };

class FastExplorationManager {
public:
  FastExplorationManager();
  ~FastExplorationManager();

  void initialize(ros::NodeHandle &nh);
  int planExploreMotionCluster(const Vector3d &pos, const Vector3d &vel,
                               const Vector3d &acc, const Vector3d &yaw);
  int updateFrontierStruct(const Eigen::Vector3d &pos);
  // int planTrajToViewInfo(const Vector3d &pos, const Vector3d &vel,
  //                        const Vector3d &acc, const Vector3d &yaw,
  //                        const Vector3d &next_pos, const double &next_yaw);
  // Benchmark method, classic frontier and rapid frontier
  int classicFrontier(const Vector3d &pos, const double &yaw);
  int rapidFrontier(const Vector3d &pos, const Vector3d &vel, const double &yaw,
                    bool &classic);

 
  shared_ptr<ExplorationData> ed_;
  shared_ptr<ExplorationParam> ep_;
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<FrontierFinder> frontier_finder_;
  shared_ptr<SDFMap> sdf_map_;
  shared_ptr<PlanningVisualization> visualization_;
  // unique_ptr<ViewFinder> view_finder_;

private:
  struct TSPConfig {
    int dimension_;
    string problem_name_;

    bool skip_first_ = false;
    bool skip_last_ = false;
    int result_id_offset_ = 1;
  };

  shared_ptr<EDTEnvironment> edt_environment_;

  // Find optimal tour for coarse viewpoints of all frontiers
  void findGlobalTour(const Vector3d &cur_pos, const Vector3d &cur_vel,
                      const Vector3d cur_yaw, vector<int> &indices);

  void findLocalTour(const Vector3d &cur_pos, const Vector3d &cur_vel,
                     const Vector3d cur_yaw, const Vector3d &next_cluster_pos,
                     vector<int> &indices);

  void findNextCluster(const Vector3d &cur_pos, const Vector3d &cur_vel,
                       const Vector3d &cur_yaw, vector<checkPoint> &check_tour,
                       Eigen::Vector3d &next_cluster_pos,const bool neighbor);

  // Refine local tour for next few frontiers, using more diverse viewpoints
  void refineLocalTour(const Vector3d &cur_pos, const Vector3d &cur_vel,
                       const Vector3d &cur_yaw,
                       const vector<vector<Vector3d>> &n_points,
                       const vector<vector<double>> &n_yaws,
                       vector<Vector3d> &refined_pts,
                       vector<double> &refined_yaws);
  void updateInertialTour(const Vector3d cur_pos,
                          const vector<Vector3d> &last_tour,
                          const vector<Vector3d> &cluster_centers,
                          const Eigen::MatrixXd &cluster_cost_matrix,
                          vector<int> &indices, double &inertia_cost);

  void shortenPath(vector<Vector3d> &path);
  void solveTSP(const Eigen::MatrixXd &cost_matrix, const TSPConfig &config,
                vector<int> &result_indices, double &total_cost);
  // bool findActiveLoop(const Vector3d &cur_pos, const Vector3d &cur_vel,
  //                     const Vector3d &cur_yaw, Vector3d &alc_pos,
  //                     double &alc_yaw, ros::Time &alc_timestamp,
  //                     vector<Vector3d> &alc_pts, vector<double> &alc_yaws,
  //                     double &utility);
  void clearExplorationData();
  double hausdorffDistance(const vector<Vector3d> &set1,
                           const vector<Vector3d> &set2, vector<int> &indices);

public:
  typedef shared_ptr<FastExplorationManager> Ptr;
};

} // namespace fast_planner

#endif