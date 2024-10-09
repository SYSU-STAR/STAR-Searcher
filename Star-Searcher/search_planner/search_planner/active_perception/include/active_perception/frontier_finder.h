#ifndef _FRONTIER_FINDER_H_
#define _FRONTIER_FINDER_H_

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <list>
#include <memory>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <queue>
#include <ros/ros.h>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <visualization_msgs/Marker.h>
using Eigen::Vector3d;
using std::list;
using std::pair;
using std::priority_queue;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::unordered_map;
using std::vector;

class RayCaster;

namespace fast_planner {
static int num_PCA = 0;
class EDTEnvironment;
class PerceptionUtils;
// Viewpoint to cover a frontier cluster
struct Viewpoint {
  // Position and heading
  Vector3d pos_;
  double yaw_;
  // Fraction of the cluster that can be covered
  // double fraction_;
  int visib_num_;
};

// A frontier cluster, the viewpoints to cover it
struct Frontier {
  // Complete voxels belonging to the cluster
  vector<Vector3d> cells_;
  // down-sampled voxels filtered by voxel grid filter
  vector<Vector3d> filtered_cells_;
  // Average position of all voxels
  Vector3d average_;
  // Average normal of all voxels
  Vector3d normal_;
  // Idx of frontier
  int id_;
  // Idx of the cluster it belongs to
  int clu_id_;
  int type_;
  bool divided;
  // Viewpoints that can cover the cluster
  vector<Viewpoint> viewpoints_;
  // Bounding box of cluster, center & 1/2 side length
  Vector3d box_min_, box_max_;
  // Path and cost from this cluster to other clusters
  vector<vector<Vector3d>> paths_;
  vector<double> costs_;
};

struct FrontierCluster {
  vector<Frontier> frts_;
  Eigen::Vector3d center_;
  vector<double> costs_;
  vector<bool> merged_;
};

struct checkPoint {
  Eigen::Vector3d pos_;
  vector<double> yaws_;
  vector<double> costs_;
};

struct CompareCost {
  bool operator()(const pair<double, int> &a, const pair<double, int> &b) {
    return a.first < b.first;
  }
};

class FrontierFinder {
public:
  enum FrontierTYPE { SURFACEFTR, UNKNOWNFTR };
  FrontierFinder(const shared_ptr<EDTEnvironment> &edt, ros::NodeHandle &nh);
  ~FrontierFinder();

  void searchFrontiers(Eigen::Vector3d cur_pos);
  void computeFrontiersToVisit(Eigen::Vector3d cur_pos);
  void computeNormal(const vector<Vector3d> &point_cloud,
                     vector<Vector3d> &normals, Vector3d center);
  void getFrontiers(vector<vector<Vector3d>> &clusters);
  void getFrontierDivision(vector<vector<Eigen::Vector3d>> &division);
  void getDormantFrontiers(vector<vector<Vector3d>> &clusters);
  void getFrontierBoxes(vector<pair<Vector3d, Vector3d>> &boxes);
  // Get viewpoint with highest coverage for each frontier
  void getTopViewpointsInfo(const Vector3d &cur_pos, vector<Vector3d> &points,
                            vector<double> &yaws, vector<Vector3d> &averages);
  void getTop5ViewPoints(vector<vector<Eigen::Vector3d>> &clusters);
  // Get several viewpoints for a subset of frontiers
  void getViewpointsInfo(const Vector3d &cur_pos, const vector<int> &ids,
                         const int &view_num, const double &max_decay,
                         vector<vector<Vector3d>> &points,
                         vector<vector<double>> &yaws);
  void updateFrontierCostMatrix();
  void clusterFrontiers(const Eigen::Vector3d &cur_pos, bool &neighbor);
  void getFullCostMatrix(const Vector3d &cur_pos, const Vector3d &cur_vel,
                         const Vector3d cur_yaw, Eigen::MatrixXd &mat);
  void getCheckTourCostMatrix(const Vector3d &cur_pos, const Vector3d &cur_vel,
                              const Vector3d cur_yaw,
                              const Vector3d &next_cluster_pos,
                              Eigen::MatrixXd &mat);
  void getSingleCellCostMatrix(const Vector3d &cur_pos, const Vector3d &cur_vel,
                               const Vector3d &cur_yaw,
                               const vector<int> &ftr_ids,
                               const Vector3d &next_grid_pos,
                               Eigen::MatrixXd &mat);
  void getPathForTour(const Vector3d &pos, const vector<int> &tsp_ids,
                      vector<Vector3d> &path);

  void getCheckTour(const int cluster_id, vector<checkPoint> &check_tour);
  void setNextFrontier(const int &id);
  bool isFrontierCovered();
  int getFrontierClusterNum();
  void wrapYaw(double &yaw);
  void getClusterMatrix(const Vector3d &cur_pos, const Vector3d &cur_vel,
                        const Vector3d cur_yaw, Eigen::MatrixXd &cost_mat);
  void getClusterTour(const vector<int> indices, vector<Vector3d> &path);
  void getClusterCenter(vector<Vector3d> &centers);
  shared_ptr<PerceptionUtils> percep_utils_;

private:
  void splitLargeFrontiers(vector<Frontier> &frontiers);
  bool splitIn3D(const Frontier &frontier, vector<Frontier> &splits);
  bool splitHorizontally(const Frontier &frontier, vector<Frontier> &splits);
  void mergeFrontiers(Frontier &ftr1, const Frontier &ftr2);
  bool isFrontierChanged(const Frontier &ft);
  bool haveOverlap(const Vector3d &min1, const Vector3d &max1,
                   const Vector3d &min2, const Vector3d &max2);
  void computeFrontierInfo(Frontier &frontier);
  void downsample(const vector<Vector3d> &cluster_in,
                  vector<Vector3d> &cluster_out);
  void sampleViewpoints(Frontier &frontier);
  int countVisibleCells(const Vector3d &pos, const double &yaw,
                        const vector<Vector3d> &cluster, const int ftr_type,
                        bool draw);
  bool isNearObstacle(const Vector3d &pos);

  vector<Eigen::Vector3i> sixNeighbors(const Eigen::Vector3i &voxel);
  vector<Eigen::Vector3i> tenNeighbors(const Eigen::Vector3i &voxel);
  vector<Eigen::Vector3i> allNeighbors(const Eigen::Vector3i &voxel);
  bool isNeighborUnknown(const Eigen::Vector3i &voxel);
  bool isNeighborUnderObserved(const Eigen::Vector3i &voxel);
  void expandFrontier(const Eigen::Vector3i &
                          first /* , const int& depth, const int& parent_id */);
  void findTopNCost(const vector<double> cost, int N,
                    priority_queue<pair<double, int>, vector<pair<double, int>>,
                                   CompareCost> &result);
  void fillBasicInfo(visualization_msgs::Marker &mk,
                     const Eigen::Vector3d &scale, const Eigen::Vector4d &color,
                     const string &ns, const int &id, const int &shape);
  void drawBox(const Eigen::Vector3d &center, const Eigen::Vector3d &scale,
               const Eigen::Vector4d &color, const string &ns, const int &id);
  void drawText(const Eigen::Vector3d &pos, const string &text,
                const double &scale, const Eigen::Vector4d &color,
                const string &ns, const int &id);
  // Wrapper of sdf map
  int toadr(const Eigen::Vector3i &idx);
  bool knownfree(const Eigen::Vector3i &idx);
  bool inmap(const Eigen::Vector3i &idx);

  // Deprecated
  Eigen::Vector3i searchClearVoxel(const Eigen::Vector3i &pt);
  bool isInBoxes(const vector<pair<Vector3d, Vector3d>> &boxes,
                 const Eigen::Vector3i &idx);
  bool canBeMerged(const Frontier &ftr1, const Frontier &ftr2);
  void findViewpoints(const Vector3d &sample, const Vector3d &ftr_avg,
                      vector<Viewpoint> &vps);
  Eigen::Vector4d getColor(const double &h, double alpha);
  // Data
  vector<char> frontier_flag_;
  vector<Frontier> frontiers_, dormant_frontiers_, tmp_frontiers_;
  vector<int> removed_ids_;
  vector<int> removed_cluster_ids_;
  int first_new_frt_;
  int frts_num_after_remove_;
  Frontier next_frontier_;
  vector<FrontierCluster> frontier_clusters_;
  vector<checkPoint> check_tour;

  // Params
  int cluster_min_;
  double cluster_size_xy_, cluster_size_z_;
  double candidate_rmax_, candidate_rmin_, candidate_dphi_, min_candidate_dist_,
      min_candidate_clearance_;
  int down_sample_;
  double min_view_finish_fraction_, resolution_;
  int min_visib_num_, candidate_rnum_;
  double division_ratio_;
  string tsp_dir_;
  double frt_cluster_radius_;

  // Utils
  ros::Publisher debug_pts_, debug_marker_, debug_pts1_;
  shared_ptr<EDTEnvironment> edt_env_;
  unique_ptr<RayCaster> raycaster_;
};

} // namespace fast_planner
#endif