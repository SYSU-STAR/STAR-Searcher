#include <active_perception/frontier_finder.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>
// #include <path_searching/astar2.h>

#include <active_perception/graph_node.h>
#include <active_perception/perception_utils.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <plan_env/edt_environment.h>

// use PCL region growing segmentation
// #include <pcl/point_types.h>
// #include <pcl/search/search.h>
// #include <pcl/search/kdtree.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/segmentation/region_growing.h>
#include <Eigen/Eigenvalues>
#include <pcl/filters/voxel_grid.h>

namespace fast_planner {
FrontierFinder::FrontierFinder(const EDTEnvironment::Ptr &edt,
                               ros::NodeHandle &nh) {
  this->edt_env_ = edt;
  int voxel_num = edt->sdf_map_->getVoxelNum();
  frontier_flag_ = vector<char>(voxel_num, 0);
  fill(frontier_flag_.begin(), frontier_flag_.end(), 0);

  nh.param("frontier/cluster_min", cluster_min_, -1);
  nh.param("frontier/cluster_size_xy", cluster_size_xy_, -1.0);
  nh.param("frontier/cluster_size_z", cluster_size_z_, -1.0);
  nh.param("frontier/min_candidate_dist", min_candidate_dist_, -1.0);
  nh.param("frontier/min_candidate_clearance", min_candidate_clearance_, -1.0);
  nh.param("frontier/candidate_dphi", candidate_dphi_, -1.0);
  nh.param("frontier/candidate_rmax", candidate_rmax_, -1.0);
  nh.param("frontier/candidate_rmin", candidate_rmin_, -1.0);
  nh.param("frontier/candidate_rnum", candidate_rnum_, -1);
  nh.param("frontier/down_sample", down_sample_, -1);
  nh.param("frontier/min_visib_num", min_visib_num_, -1);
  nh.param("frontier/min_view_finish_fraction", min_view_finish_fraction_,
           -1.0);
  nh.param("frontier/tsp_dir_", tsp_dir_, string("null"));
  nh.param("frontier/frt_cluster_radius", frt_cluster_radius_, -1.0);

  raycaster_.reset(new RayCaster);
  resolution_ = edt_env_->sdf_map_->getResolution();
  Eigen::Vector3d origin, size;
  edt_env_->sdf_map_->getRegion(origin, size);
  raycaster_->setParams(resolution_, origin);
  debug_pts_ = nh.advertise<sensor_msgs::PointCloud2>(
      "/frontier_debug/debug_before", 10);
  debug_pts1_ =
      nh.advertise<sensor_msgs::PointCloud2>("/frontier_debug/debug_after", 10);
  debug_marker_ =
      nh.advertise<visualization_msgs::Marker>("/frontier_finder/marker", 1000);
  percep_utils_.reset(new PerceptionUtils(nh));
  first_new_frt_ = -1;
}

FrontierFinder::~FrontierFinder() {}

void FrontierFinder::searchFrontiers(Eigen::Vector3d cur_pos) {
  ros::Time t1 = ros::Time::now();
  tmp_frontiers_.clear();

  // Bounding box of updated region
  Vector3d update_min, update_max;
  edt_env_->sdf_map_->getUpdatedBox(update_min, update_max, true);

  // Removed changed frontiers in updated map
  auto resetFlag = [&](int iter, vector<Frontier> &frontiers) {
    // erase frontier
    Eigen::Vector3i idx;
    for (auto cell : frontiers[iter].cells_) {
      edt_env_->sdf_map_->posToIndex(cell, idx);
      frontier_flag_[toadr(idx)] = 0;
    }
    frontiers.erase(frontiers.begin() + iter);

    // erase cluster it belongs to
  };

  // std::cout << "Before remove: " << frontiers_.size() << std::endl;

  removed_ids_.clear();
  removed_cluster_ids_.clear();
  int rmv_idx = 0;
  for (int iter = 0; iter < frontiers_.size();) {
    if (haveOverlap(frontiers_[iter].box_min_, frontiers_[iter].box_max_,
                    update_min, update_max) &&
        isFrontierChanged(frontiers_[iter])) {

      resetFlag(iter, frontiers_);
      removed_ids_.push_back(rmv_idx);
    } else {
      ++rmv_idx;
      ++iter;
    }
  }
  frts_num_after_remove_ = frontiers_.size();
  // std::cout << "After remove: " << frontiers_.size() << std::endl;
  for (int iter = 0; iter < dormant_frontiers_.size();) {
    if (haveOverlap(dormant_frontiers_[iter].box_min_,
                    dormant_frontiers_[iter].box_max_, update_min,
                    update_max) &&
        isFrontierChanged(dormant_frontiers_[iter]))
      resetFlag(iter, dormant_frontiers_);
    else
      ++iter;
  }

  // Search new frontier within box slightly inflated from updated box
  Vector3d search_min = update_min - Vector3d(0.3, 0.3, 0.5);
  Vector3d search_max = update_max + Vector3d(0.3, 0.3, 0.5);

  Vector3d box_min, box_max;
  edt_env_->sdf_map_->getBox(box_min, box_max);
  for (int k = 0; k < 3; ++k) {
    search_min[k] = max(search_min[k], box_min[k]);
    search_max[k] = min(search_max[k], box_max[k]);
  }
  // drawBox((search_min + search_max) / 2, search_max - search_min,
  //         Eigen::Vector4d(0, 0, 1, 0.3), "bbox", 0);
  Eigen::Vector3i min_id, max_id;
  edt_env_->sdf_map_->posToIndex(search_min, min_id);
  edt_env_->sdf_map_->posToIndex(search_max, max_id);

  ros::Time t2 = ros::Time::now();
  int frts_pts_num = 0;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z) {
        // Scanning the updated region to find seeds of frontiers
        Eigen::Vector3i cur(x, y, z);
        Eigen::Vector3d pos;
        edt_env_->sdf_map_->indexToPos(cur, pos);
        if (frontier_flag_[toadr(cur)] == 0 &&
            ((knownfree(cur) &&
              (isNeighborUnderObserved(cur) || isNeighborUnknown(cur))))) {
          // Expand from the seed cell to find a complete frontier cluster
          expandFrontier(cur);
        }
      }

  // double frontier_expand_time_ = (ros::Time::now() - t2).toSec();
  // ROS_WARN("Frontier expand Time: %lf", frontier_expand_time_);
  splitLargeFrontiers(tmp_frontiers_);
  double frontier_search_time_ = (ros::Time::now() - t1).toSec();
  // ROS_WARN("Frontier Search Time: %lf", frontier_search_time_);
}

void FrontierFinder::expandFrontier(
    const Eigen::Vector3i
        &first /* , const int& depth, const int& parent_id */) {
  // std::cout << "depth: " << depth << std::endl;
  auto t1 = ros::Time::now();

  // Data for clustering
  queue<Eigen::Vector3i> cell_queue;
  vector<Eigen::Vector3d> expanded;
  Vector3d pos;

  int ftr_type;
  if (isNeighborUnderObserved(first)) {
    ftr_type = SURFACEFTR;
  } else {
    ftr_type = UNKNOWNFTR;
    // return;
  }

  edt_env_->sdf_map_->indexToPos(first, pos);
  expanded.push_back(pos);
  cell_queue.push(first);
  frontier_flag_[toadr(first)] = 1;
  auto addCell = [&](int adr, Eigen::Vector3i nbr, Eigen::Vector3d pos) {
    edt_env_->sdf_map_->indexToPos(nbr, pos);
    if (pos[2] < 0.5 || pos[2] > 6.5)
      return;
    expanded.push_back(pos);
    cell_queue.push(nbr);
    frontier_flag_[adr] = 1;
  };

  while (!cell_queue.empty()) {
    auto cur = cell_queue.front();
    cell_queue.pop();
    auto nbrs = allNeighbors(cur);
    for (auto nbr : nbrs) {
      int adr = toadr(nbr);
      if (frontier_flag_[adr] == 1 || !edt_env_->sdf_map_->isInBox(nbr))
        continue;
      // if (ftr_type == SURFACEFTR && knownfree(nbr) &&
      //     isNeighborUnderObserved(nbr)) {
      //   addCell(adr, nbr, pos);
      // }
      // if (ftr_type == UNKNOWNFTR && knownfree(nbr) && isNeighborUnknown(nbr))
      // {
      //   addCell(adr, nbr, pos);
      // }
      if (knownfree(nbr) &&
          (isNeighborUnderObserved(nbr) || isNeighborUnknown(nbr))) {
        addCell(adr, nbr, pos);
      }
    }
  }
  if (expanded.size() > cluster_min_) {
    // Compute detailed info
    Frontier frontier;
    frontier.type_ = ftr_type;
    frontier.cells_ = expanded;
    computeFrontierInfo(frontier);
    double z_range = frontier.box_max_[2] - frontier.box_min_[2];
    if (z_range < .5 || frontier.average_[2] < 0.5 ||
        frontier.average_[2] > 6.0)
      return;
    tmp_frontiers_.push_back(frontier);
  } else {
    for (auto cell : expanded) {
      Eigen::Vector3i idx_;
      edt_env_->sdf_map_->posToIndex(cell, idx_);
      frontier_flag_[toadr(idx_)] = 0;
    }
  }
}

void FrontierFinder::splitLargeFrontiers(vector<Frontier> &frontiers) {
  vector<Frontier> splits, tmps;
  for (auto it = frontiers.begin(); it != frontiers.end(); ++it) {
    // Check if each frontier needs to be split horizontally
    if (splitIn3D(*it, splits)) {
      tmps.insert(tmps.end(), splits.begin(), splits.end());
      splits.clear();
    } else
      tmps.push_back(*it);
  }
  frontiers = tmps;
}

bool FrontierFinder::splitIn3D(const Frontier &frontier,
                               vector<Frontier> &splits) {
  auto mean = frontier.average_;
  bool need_split = false;
  for (auto cell : frontier.filtered_cells_) {
    if ((cell - mean).norm() > cluster_size_xy_) {
      need_split = true;
      break;
    }
  }
  if (!need_split)
    return false;

  // Compute covariance matrix of cells
  Eigen::Matrix3d cov;
  cov.setZero();
  for (auto cell : frontier.filtered_cells_) {
    Eigen::Vector3d diff = cell - mean;
    cov += diff * diff.transpose();
  }
  cov /= double(frontier.filtered_cells_.size());

  // Find eigenvector corresponds to maximal eigenvalue
  Eigen::EigenSolver<Eigen::Matrix3d> es(cov);
  auto values = es.eigenvalues().real();
  auto vectors = es.eigenvectors().real();
  int max_idx;
  double max_eigenvalue = -1000000;
  for (int i = 0; i < values.rows(); ++i) {
    if (values[i] > max_eigenvalue) {
      max_idx = i;
      max_eigenvalue = values[i];
    }
  }
  Eigen::Vector3d first_pc = vectors.col(max_idx);

  // Split the frontier into two groups along the first PC
  Frontier ftr1, ftr2;
  for (auto cell : frontier.cells_) {
    if ((cell - mean).dot(first_pc) >= 0)
      ftr1.cells_.push_back(cell);
    else
      ftr2.cells_.push_back(cell);
  }
  computeFrontierInfo(ftr1);
  computeFrontierInfo(ftr2);

  // Recursive call to split frontier that is still too large
  vector<Frontier> splits2;
  if (splitIn3D(ftr1, splits2)) {
    splits.insert(splits.end(), splits2.begin(), splits2.end());
    splits2.clear();
  } else
    splits.push_back(ftr1);

  if (splitIn3D(ftr2, splits2))
    splits.insert(splits.end(), splits2.begin(), splits2.end());
  else
    splits.push_back(ftr2);

  return true;
}

bool FrontierFinder::splitHorizontally(const Frontier &frontier,
                                       vector<Frontier> &splits) {
  bool need_split = false;
  // Split a frontier into small piece if it is too large
  auto mean = frontier.average_.head<2>();
  for (auto cell : frontier.filtered_cells_) {
    if ((cell.head<2>() - mean).norm() > cluster_size_xy_) {
      need_split = true;
      break;
    }
  }
  if (!need_split)
    return false;

  // Compute principal component
  // Covariance matrix of cells
  Eigen::Matrix2d cov;
  cov.setZero();
  for (auto cell : frontier.filtered_cells_) {
    Eigen::Vector2d diff = cell.head<2>() - mean;
    cov += diff * diff.transpose();
  }
  cov /= double(frontier.filtered_cells_.size());

  // Find eigenvector corresponds to maximal eigenvector
  Eigen::EigenSolver<Eigen::Matrix2d> es(cov);
  auto values = es.eigenvalues().real();
  auto vectors = es.eigenvectors().real();
  int max_idx;
  double max_eigenvalue = -1000000;
  for (int i = 0; i < values.rows(); ++i) {
    if (values[i] > max_eigenvalue) {
      max_idx = i;
      max_eigenvalue = values[i];
    }
  }
  Eigen::Vector2d first_pc = vectors.col(max_idx).normalized();
  // std::cout << "max idx: " << max_idx << std::endl;
  // std::cout << "mean: " << mean.transpose()
  //           << ", first pc: " << first_pc.transpose() << std::endl;

  // Split the frontier into two groups along the first PC
  Frontier ftr1, ftr2;
  for (auto cell : frontier.cells_) {
    if ((cell.head<2>() - mean).dot(first_pc) >= 0)
      ftr1.cells_.push_back(cell);
    else
      ftr2.cells_.push_back(cell);
  }
  computeFrontierInfo(ftr1);
  computeFrontierInfo(ftr2);
  ftr1.type_ = frontier.type_;
  ftr2.type_ = frontier.type_;
  // Recursive call to split frontier that is still too large
  vector<Frontier> splits2;
  if (splitHorizontally(ftr1, splits2)) {
    splits.insert(splits.end(), splits2.begin(), splits2.end());
    splits2.clear();
  } else
    splits.push_back(ftr1);

  if (splitHorizontally(ftr2, splits2))
    splits.insert(splits.end(), splits2.begin(), splits2.end());
  else
    splits.push_back(ftr2);

  return true;
}

bool FrontierFinder::isInBoxes(const vector<pair<Vector3d, Vector3d>> &boxes,
                               const Eigen::Vector3i &idx) {
  Vector3d pt;
  edt_env_->sdf_map_->indexToPos(idx, pt);
  for (auto box : boxes) {
    // Check if contained by a box
    bool inbox = true;
    for (int i = 0; i < 3; ++i) {
      inbox = inbox && pt[i] > box.first[i] && pt[i] < box.second[i];
      if (!inbox)
        break;
    }
    if (inbox)
      return true;
  }
  return false;
}

void FrontierFinder::updateFrontierCostMatrix() {
  auto updateCost = [](Frontier &ft1, Frontier &ft2) {
    // Search path from old cluster's top viewpoint to new cluster'
    Viewpoint &vui = ft1.viewpoints_.front();
    Viewpoint &vuj = ft2.viewpoints_.front();
    vector<Vector3d> path_ij;
    Eigen::Vector3d zero_;
    double cost_ij =
        ViewNode::computeCostPos(vui.pos_, vuj.pos_, zero_, path_ij);
    // Insert item for both old and new clusters
    ft1.costs_.push_back(cost_ij);
    ft1.paths_.push_back(path_ij);
    reverse(path_ij.begin(), path_ij.end());
    ft2.costs_.push_back(cost_ij);
    ft2.paths_.push_back(path_ij);
  };

  auto updateCostbetweenNewFtrs = [=, &updateCost](vector<Frontier> &frontiers,
                                                   int first_new_idx) {
    for (int it1 = first_new_idx; it1 < frontiers.size(); ++it1)
      for (int it2 = it1; it2 < frontiers.size(); ++it2) {
        if (it1 == it2) {
          frontiers[it1].costs_.push_back(0);
          frontiers[it1].paths_.push_back({});
        } else
          updateCost(frontiers[it1], frontiers[it2]);
      }
  };

  if (!removed_ids_.empty()) {
    // Delete path and cost for removed clusters
    for (int it = 0; it < frts_num_after_remove_; ++it) {
      for (int i = 0; i < removed_ids_.size(); ++i) {
        int iter_idx = 0;
        auto cost_iter = frontiers_[it].costs_.begin();
        auto path_iter = frontiers_[it].paths_.begin();
        // Step iterator to the item to be removed
        while (iter_idx < removed_ids_[i]) {
          ++cost_iter;
          ++path_iter;
          ++iter_idx;
        }
        frontiers_[it].costs_.erase(cost_iter);
        frontiers_[it].paths_.erase(path_iter);
      }
      std::cout << "(" << frontiers_[it].costs_.size() << ","
                << frontiers_[it].paths_.size() << "), ";
    }
    removed_ids_.clear();
  }
  if (first_new_frt_ == -1) // not add any new frontier in this calculation
    return;

  std::cout << "cost mat add: " << std::endl;
  // Compute path and cost between old and new clusters
  for (int it1 = 0; it1 < first_new_frt_; ++it1)
    for (int it2 = first_new_frt_; it2 < frontiers_.size(); ++it2) {
      updateCost(frontiers_[it1], frontiers_[it2]);
    }
  // Compute path and cost between new clusters
  updateCostbetweenNewFtrs(frontiers_, first_new_frt_);
}

void FrontierFinder::clusterFrontiers(const Eigen::Vector3d &cur_pos,
                                      bool &neighbor) {
  ros::Time t1 = ros::Time::now();
  // Cluster frontiers into groups
  frontier_clusters_.clear();
  bool merge;
  neighbor = false; //If the drone is in a viewpoint cluster, give this cluster the highest priority

  vector<int> frt_ids;
  vector<int> left_ids;
  vector<double> vp_dists;

  // cluster frontiers neighbor cur_pos
  for (int i = 0; i < frontiers_.size(); i++) {
    frt_ids.push_back(i);
    vp_dists.push_back(
        (frontiers_[i].viewpoints_.front().pos_ - cur_pos).norm());
  }

  auto dist_sort = [=](int id1, int id2) {
    return vp_dists[id1] < vp_dists[id2];
  };
  sort(frt_ids.begin(), frt_ids.end(), dist_sort);

  FrontierCluster first_tmp_clu;
  //check between the drone and the frontier
  for (auto frt_id : frt_ids) {
    double d_ = (frontiers_[frt_id].viewpoints_.front().pos_ - cur_pos).norm();
    if (d_ > frt_cluster_radius_/2)
      break;
    merge = true;
    Vector3i idx;
    raycaster_->input(frontiers_[frt_id].viewpoints_.front().pos_, cur_pos);
    while (raycaster_->nextId(idx)) {
      // Hit obstacle, stop the ray
      if (edt_env_->sdf_map_->getOccupancy(idx) != SDFMap::FREE ||
          !edt_env_->sdf_map_->isInBox(idx)) {
        merge = false;
        break;
      }
    }
    if (!merge)
      continue;

    //check between the frontier and the frontier
    for (auto ftr : first_tmp_clu.frts_) {
      raycaster_->input(frontiers_[frt_id].viewpoints_.front().pos_,
                        ftr.viewpoints_.front().pos_);
      while (raycaster_->nextId(idx)) {
        // Hit obstacle, stop the ray
        if (edt_env_->sdf_map_->getOccupancy(idx) != SDFMap::FREE ||
            !edt_env_->sdf_map_->isInBox(idx)) {
          merge = false;
          break;
        }
      }
      if (!merge)
        break;
    }
    if (merge) {
      frontiers_[frt_id].divided = true;
      first_tmp_clu.center_ =
          (double(first_tmp_clu.frts_.size()) * first_tmp_clu.center_ +
           frontiers_[frt_id].viewpoints_.front().pos_) /
          double(first_tmp_clu.frts_.size() + 1);
      first_tmp_clu.frts_.emplace_back(frontiers_[frt_id]);
      neighbor = true;
    }
  }
  if (neighbor)
    frontier_clusters_.emplace_back(first_tmp_clu);

  // cluster between frontiers
  for (int i = 0; i < frontiers_.size(); i++) {
    if (frontiers_[i].divided)
      continue;
    FrontierCluster tmp_clu_;
    tmp_clu_.frts_.push_back(frontiers_[i]);
    tmp_clu_.center_ = frontiers_[i].viewpoints_.front().pos_;
    frontiers_[i].divided = true;

    frt_ids.clear();  // frt_ids is the index of frontiers_, like [2,4,7,9]
    left_ids.clear(); // left_ids is the index of frt_ids, like [0,1,2,3]
    vp_dists.clear();

    int idx = 0;
    for (int id = 0; id < frontiers_.size(); id++) {
      if (frontiers_[id].divided)
        continue;
      frt_ids.push_back(id);
      left_ids.push_back(idx++);
      vp_dists.push_back(
          (frontiers_[id].viewpoints_.front().pos_ - tmp_clu_.center_).norm());
    }
    sort(left_ids.begin(), left_ids.end(), dist_sort);

    int j = 0;
    while (j < frt_ids.size()) {
      int frt_id = frt_ids[left_ids[j]];
      double d_ = (frontiers_[frt_id].viewpoints_.front().pos_-tmp_clu_.center_).norm();

      if (d_ > frt_cluster_radius_) {
        break; //early stop because no neighbor cluster left
      }

      merge = true;
      for (auto ftr : tmp_clu_.frts_) {
        Vector3i idx;
        raycaster_->input(frontiers_[frt_id].viewpoints_.front().pos_,
                          ftr.viewpoints_.front().pos_);
        while (raycaster_->nextId(idx)) {
          // Hit obstacle, stop the ray
          if (edt_env_->sdf_map_->getOccupancy(idx) != SDFMap::FREE ||
              !edt_env_->sdf_map_->isInBox(idx)) {
            merge = false;
            break;
          }
        }
        if (!merge)
          break;
      }

      if (merge) {
        frontiers_[frt_id].divided = true;
        tmp_clu_.center_ = (double(tmp_clu_.frts_.size()) * tmp_clu_.center_ +
                            frontiers_[frt_id].viewpoints_.front().pos_) /
                           double(tmp_clu_.frts_.size() + 1);
        tmp_clu_.frts_.emplace_back(frontiers_[frt_id]);

        // frt_ids.clear();
        // left_ids.clear();
        // vp_dists.clear();
        // idx = 0;
        // for (int id = 0; id < frontiers_.size(); id++) {
        //   if (frontiers_[id].divided)
        //     continue;
        //   frt_ids.push_back(id);
        //   left_ids.push_back(idx++);
        //   vp_dists.push_back(
        //       (frontiers_[id].viewpoints_.front().pos_ - tmp_clu_.center_)
        //           .norm());
        // }
        // sort(left_ids.begin(), left_ids.end(), dist_sort);
        // j = 0;
        j++;
      } else {
        j++;
      }
    }
    frontier_clusters_.emplace_back(tmp_clu_);
  }

  // ros::Time t3 = ros::Time::now();
  // int test = 0;
  // for (int cnt = 0; cnt < frontier_clusters_.size(); cnt++) {
  //   for (int idx = 0; idx < frontier_clusters_[cnt].frts_.size(); idx++) {
  //     visualization_msgs::Marker mk;
  //     fillBasicInfo(mk, Eigen::Vector3d(0.5, 0.5, 0.5),
  //                   Eigen::Vector4d(0, 255, 127, 1), "frontier_clusters_",
  //                   test++, visualization_msgs::Marker::TEXT_VIEW_FACING);
  //     mk.action = visualization_msgs::Marker::DELETE;
  //     debug_marker_.publish(mk);
  //   }
  // }

  // test = 0;
  // for (int cnt = 0; cnt < frontier_clusters_.size(); cnt++) {
  //   for (int idx = 0; idx < frontier_clusters_[cnt].frts_.size(); idx++) {
  //     drawText(frontier_clusters_[cnt].frts_[idx].viewpoints_[0].pos_,
  //              std::to_string(cnt), 0.5, Eigen::Vector4d(0, 255, 127, 1),
  //              "frontier_clusters_", test++);
  //   }
  // }
  // double cal_t2 = (ros::Time::now() - t3).toSec();
  // ROS_WARN("[Frontier Cluster] Debug Draw Time: %f", cal_t2);

  auto updateCost = [](FrontierCluster &clu1, FrontierCluster &clu2) {
    // Search path from old cluster's top viewpoint to new cluster'

    vector<Vector3d> path_ij;
    Eigen::Vector3d zero_;
    double cost_ij =
        ViewNode::computeCostPos(clu1.center_, clu2.center_, zero_, path_ij);
    // Insert item for both old and new clusters
    clu1.costs_.push_back(cost_ij);
    clu2.costs_.push_back(cost_ij);
  };

  ros::Time t2 = ros::Time::now();
  for (int it1 = 0; it1 < frontier_clusters_.size(); it1++) {
    for (int it2 = it1; it2 < frontier_clusters_.size(); it2++) {
      if (it1 == it2) {
        frontier_clusters_[it1].costs_.push_back(0.0);
      } else {
        updateCost(frontier_clusters_[it1], frontier_clusters_[it2]);
      }
    }
  }

  // double cal_t1 = (ros::Time::now() - t2).toSec();
  // ROS_WARN("[Frontier Cluster]Cost Calculation Time: %f", cal_t1);

  for (auto &frt : frontiers_) {
    frt.divided = false;
  }
  double t = (ros::Time::now() - t1).toSec();
  // ROS_WARN("[Frontier Cluster]Cluster Time: %f, Cluster num:%d", t,
  //          frontier_clusters_.size());
}

void FrontierFinder::mergeFrontiers(Frontier &ftr1, const Frontier &ftr2) {
  // Merge ftr2 into ftr1
  ftr1.average_ = (ftr1.average_ * double(ftr1.cells_.size()) +
                   ftr2.average_ * double(ftr2.cells_.size())) /
                  (double(ftr1.cells_.size() + ftr2.cells_.size()));
  ftr1.cells_.insert(ftr1.cells_.end(), ftr2.cells_.begin(), ftr2.cells_.end());
  computeFrontierInfo(ftr1);
}

bool FrontierFinder::canBeMerged(const Frontier &ftr1, const Frontier &ftr2) {
  Vector3d merged_avg = (ftr1.average_ * double(ftr1.cells_.size()) +
                         ftr2.average_ * double(ftr2.cells_.size())) /
                        (double(ftr1.cells_.size() + ftr2.cells_.size()));
  // Check if it can merge two frontier without exceeding size limit
  for (auto c1 : ftr1.cells_) {
    auto diff = c1 - merged_avg;
    if (diff.head<2>().norm() > cluster_size_xy_ || diff[2] > cluster_size_z_)
      return false;
  }
  for (auto c2 : ftr2.cells_) {
    auto diff = c2 - merged_avg;
    if (diff.head<2>().norm() > cluster_size_xy_ || diff[2] > cluster_size_z_)
      return false;
  }
  return true;
}

bool FrontierFinder::haveOverlap(const Vector3d &min1, const Vector3d &max1,
                                 const Vector3d &min2, const Vector3d &max2) {
  // Check if two box have overlap part
  Vector3d bmin, bmax;
  for (int i = 0; i < 3; ++i) {
    bmin[i] = max(min1[i], min2[i]);
    bmax[i] = min(max1[i], max2[i]);
    if (bmin[i] > bmax[i] + 1e-3)
      return false;
  }
  return true;
}

bool FrontierFinder::isFrontierChanged(const Frontier &ft) {
  int change_num = 0;
  int thresh = 0;
  // if (ft.type_ == UNKNOWNFTR) {
  //   thresh = 0;
  // } else {
  //   thresh = ft.cells_.size() * 0.08;
  // }

  thresh = ft.cells_.size() * 0.05;

  for (auto cell : ft.cells_) {
    Eigen::Vector3i idx;
    edt_env_->sdf_map_->posToIndex(cell, idx);
    // if (ft.type_ == UNKNOWNFTR && !(knownfree(idx) &&
    // isNeighborUnknown(idx))) {
    //   change_num++;
    // }
    // if (ft.type_ == SURFACEFTR &&
    //     !(knownfree(idx) && isNeighborUnderObserved(idx))) {
    //   change_num++;
    // }
    if (!(knownfree(idx) && isNeighborUnderObserved(idx)) &&
        !(knownfree(idx) && isNeighborUnknown(idx))) {
      change_num++;
    }

    if (change_num > thresh)
      return true;
  }
  return false;
}

void FrontierFinder::computeFrontierInfo(Frontier &ftr) {
  if (ftr.cells_.size() == 0)
    return;
  // Compute average position and bounding box of cluster
  ftr.average_.setZero();
  ftr.normal_.setZero();
  ftr.box_max_ = ftr.cells_.front();
  ftr.box_min_ = ftr.cells_.front();

  for (auto cell : ftr.cells_) {
    ftr.average_ += cell;
    for (int i = 0; i < 3; ++i) {
      ftr.box_min_[i] = min(ftr.box_min_[i], cell[i]);
      ftr.box_max_[i] = max(ftr.box_max_[i], cell[i]);
    }
  }
  ftr.average_ /= double(ftr.cells_.size());
  ftr.divided = false;
  // Compute downsampled cluster
  downsample(ftr.cells_, ftr.filtered_cells_);

  vector<Vector3d> normals;
  computeNormal(ftr.cells_, normals, ftr.average_);
  ftr.normal_ = normals[0];
  for (size_t i = 0; i < normals.size(); i++) {
    if (ftr.normal_.dot(normals[i]) < 0) {
      normals[i] = -normals[i];
    }
    ftr.normal_ += normals[i];
    ftr.normal_ = ftr.normal_.normalized();
  }
}

void FrontierFinder::computeNormal(const vector<Vector3d> &point_cloud,
                                   vector<Vector3d> &normals, Vector3d center) {
  normals.clear();
  vector<Vector3d> vis_avg;
  vector<Vector3d> vis_yaw;
  vis_avg.clear();
  vis_yaw.clear();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (auto cell : point_cloud)
    cloud->points.emplace_back(cell[0], cell[1], cell[2]);
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setNumberOfThreads(8);
  ne.setInputCloud(cloud);
  // ne.setViewPoint(center.x(), center.y(), center.z());
  ne.setViewPoint(0.0, 0.0, 0.0);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
      new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch(1.0);
  ne.compute(*cloud_normals);
  normals.resize(cloud_normals->size());
  for (size_t i = 0; i < cloud_normals->size(); ++i) {

    normals[i] = Eigen::Vector3d(cloud_normals->points[i].normal_x,
                                 cloud_normals->points[i].normal_y,
                                 cloud_normals->points[i].normal_z);
    normals[i].normalized();
    // if (i % 5 == 0) {
    //   double phi = atan2(normals[i][1], normals[i][0]);
    //   vis_avg.push_back(point_cloud[i]);
    //   vis_yaw.push_back(point_cloud[i] + 1.0 * Vector3d(cos(phi), sin(phi),
    //   0));
    // }
  }

  // drawLines(vis_avg, vis_yaw, 0.05, Eigen::Vector4d(0, 255, 127, 1),
  //           "normal_view", 0);
}

void FrontierFinder::computeFrontiersToVisit(Eigen::Vector3d cur_pos) {
  ros::Time t1 = ros::Time::now();
  bool insert_frontier = false;
  int new_num = 0;
  int new_dormant_num = 0;
  first_new_frt_ = -1;
  // Try find viewpoints for each cluster and categorize them according to
  // viewpoint number
  for (auto &tmp_ftr : tmp_frontiers_) {
    // Search viewpoints around frontier
    sampleViewpoints(tmp_ftr);
    if (!tmp_ftr.viewpoints_.empty()) {
      ++new_num;
      vector<Frontier>::iterator inserted =
          frontiers_.insert(frontiers_.end(), tmp_ftr);
      // Sort the viewpoints by coverage fraction, best view in front

      auto compare = [=](const Viewpoint &v1, const Viewpoint &v2) {
        Eigen::Vector3d pos_dir = (cur_pos - inserted->average_).normalized();
        Eigen::Vector3d v1_dir = (v1.pos_ - inserted->average_).normalized();
        Eigen::Vector3d v2_dir = (v2.pos_ - inserted->average_).normalized();
        double score_v1 = abs(inserted->normal_.dot(v1_dir)) *
                          pos_dir.dot(v1_dir) * v1.visib_num_;
        double score_v2 = abs(inserted->normal_.dot(v2_dir)) *
                          pos_dir.dot(v2_dir) * v2.visib_num_;
        // double score_v1 = pos_dir.dot(v1_dir) * v1.visib_num_;
        // double score_v2 = pos_dir.dot(v2_dir) * v2.visib_num_;
        return score_v1 > score_v2;
      };
      sort(inserted->viewpoints_.begin(), inserted->viewpoints_.end(), compare);

      if (!insert_frontier) {
        first_new_frt_ = frontiers_.size() - 1;
        insert_frontier = true;
      }
    } else {
      for (auto cell : tmp_ftr.cells_) {
        Eigen::Vector3i idx_;
        edt_env_->sdf_map_->posToIndex(cell, idx_);
        frontier_flag_[toadr(idx_)] = 0;
      }
      // Find no viewpoint, move cluster to dormant vector
      dormant_frontiers_.push_back(tmp_ftr);
      ++new_dormant_num;
    }
  }

  auto compare_frontier = [=](const Frontier &f1, const Frontier &f2) {
    return f1.viewpoints_.front().pos_(0) < f2.viewpoints_.front().pos_(0);
  };
  sort(frontiers_.begin(), frontiers_.end(), compare_frontier);

  // Reset indices of frontiers
  int idx = 0;
  for (auto &ft : frontiers_) {
    ft.id_ = idx++;
    std::cout << ft.id_ << ", ";
  }
  std::cout << "\nnew num: " << new_num << ", new dormant: " << new_dormant_num
            << std::endl;
  std::cout << "to visit: " << frontiers_.size()
            << ", dormant: " << dormant_frontiers_.size() << std::endl;
  double frontier_visit_time_ = (ros::Time::now() - t1).toSec();
  // ROS_WARN("Frontier Visit Time: %lf", frontier_visit_time_);
}

void FrontierFinder::getTopViewpointsInfo(const Vector3d &cur_pos,
                                          vector<Eigen::Vector3d> &points,
                                          vector<double> &yaws,
                                          vector<Eigen::Vector3d> &averages) {
  points.clear();
  yaws.clear();
  averages.clear();
  for (auto frontier : frontiers_) {
    bool no_view = true;
    for (auto view : frontier.viewpoints_) {
      // Retrieve the first viewpoint that is far enough and has highest
      // coverage
      if ((view.pos_ - cur_pos).norm() < min_candidate_dist_)
        continue;
      points.push_back(view.pos_);
      yaws.push_back(view.yaw_);
      averages.push_back(frontier.average_);
      no_view = false;
      break;
    }
    if (no_view) {
      // All viewpoints are very close, just use the first one (with highest
      // coverage).
      auto view = frontier.viewpoints_.front();
      points.push_back(view.pos_);
      yaws.push_back(view.yaw_);
      averages.push_back(frontier.average_);
    }
  }
}

void FrontierFinder::getViewpointsInfo(const Vector3d &cur_pos,
                                       const vector<int> &ids,
                                       const int &view_num,
                                       const double &max_decay,
                                       vector<vector<Eigen::Vector3d>> &points,
                                       vector<vector<double>> &yaws) {
  points.clear();
  yaws.clear();
  for (auto id : ids) {
    // Scan all frontiers to find one with the same id
    for (auto frontier : frontiers_) {
      if (frontier.id_ == id) {
        // Get several top viewpoints that are far enough
        vector<Eigen::Vector3d> pts;
        vector<double> ys;
        int visib_thresh = frontier.viewpoints_.front().visib_num_ * max_decay;
        for (auto view : frontier.viewpoints_) {
          if (pts.size() >= view_num || view.visib_num_ <= visib_thresh)
            break;
          if ((view.pos_ - cur_pos).norm() < min_candidate_dist_)
            continue;
          pts.push_back(view.pos_);
          ys.push_back(view.yaw_);
        }
        if (pts.empty()) {
          // All viewpoints are very close, ignore the distance limit
          for (auto view : frontier.viewpoints_) {
            if (pts.size() >= view_num || view.visib_num_ <= visib_thresh)
              break;
            pts.push_back(view.pos_);
            ys.push_back(view.yaw_);
          }
        }
        points.push_back(pts);
        yaws.push_back(ys);
      }
    }
  }
}

void FrontierFinder::getSingleCellCostMatrix(const Vector3d &cur_pos,
                                             const Vector3d &cur_vel,
                                             const Vector3d &cur_yaw,
                                             const vector<int> &ftr_ids,
                                             const Vector3d &next_grid_pos,
                                             Eigen::MatrixXd &mat) {
  // Current position + frontiers + next grid position
  int dimension = ftr_ids.size() + 2;
  mat = Eigen::MatrixXd::Zero(dimension, dimension);

  // ROS_WARN("Begin get SingleCellCostMatrix!");
  // Costs from drone to frontiers
  for (int i = 0; i < ftr_ids.size(); i++) {
    int frontier_id = ftr_ids[i];
    auto frontier_it = frontiers_.begin();
    std::advance(frontier_it, frontier_id);
    Viewpoint vj = frontier_it->viewpoints_.front();
    vector<Vector3d> path;
    mat(0, i + 1) = ViewNode::computeCost(cur_pos, vj.pos_, cur_yaw[0], vj.yaw_,
                                          cur_vel, 0.0, path);
    // Not go back from frontiers
    mat(i + 1, 0) = 1000;
  }

  // Among frontiers
  for (int i = 0; i < ftr_ids.size(); i++) {
    int frontier_id = ftr_ids[i];
    auto frontier_it = frontiers_.begin();
    std::advance(frontier_it, frontier_id);
    for (int j = i; j < ftr_ids.size(); j++) {
      if (i == j) {
        mat(i + 1, j + 1) = 0;
        continue;
      }
      int frontier_id2 = ftr_ids[j];
      auto frontier_it2 = frontiers_.begin();
      std::advance(frontier_it2, frontier_id2);
      auto cost_iter = frontier_it->costs_.begin();
      std::advance(cost_iter, frontier_id2);
      mat(i + 1, j + 1) = *cost_iter;
      mat(j + 1, i + 1) = mat(i + 1, j + 1);
    }
  }

  // Ensure next grid position will be the last one
  // From drone to next grid
  vector<Vector3d> path;
  mat(0, dimension - 1) = 1000;
  mat(dimension - 1, 0) = 0;

  // From frontiers to next grid
  for (int i = 0; i < ftr_ids.size(); i++) {
    int frontier_id = ftr_ids[i];
    auto frontier_it = frontiers_.begin();
    std::advance(frontier_it, frontier_id);
    Viewpoint vj = frontier_it->viewpoints_.front();

    // All frontiers can go to next grid but next grid cannot go to frontiers
    mat(i + 1, dimension - 1) = ViewNode::computeCost(
        vj.pos_, next_grid_pos, 0.0, 0.0, Eigen::Vector3d::Zero(), 0.0, path);
    mat(dimension - 1, i + 1) = 1000;
  }
}

void FrontierFinder::getFrontiers(vector<vector<Eigen::Vector3d>> &clusters) {
  clusters.clear();
  for (auto frontier : frontiers_) {
    // if (frontier.type_ == SURFACEFTR)
    { clusters.push_back(frontier.cells_); }
  }

  // clusters.push_back(frontier.filtered_cells_);
}

void FrontierFinder::getFrontierDivision(
    vector<vector<Eigen::Vector3d>> &frt_division) {
  frt_division.clear();
  for (auto clu : frontier_clusters_) {
    vector<Eigen::Vector3d> tmp;
    for (auto frt : clu.frts_) {
      tmp.push_back(frt.viewpoints_.front().pos_);
    }
    frt_division.push_back(tmp);
  }
}

void FrontierFinder::getTop5ViewPoints(
    vector<vector<Eigen::Vector3d>> &clusters) {
  clusters.clear();
  for (auto frontier : frontiers_) {
    vector<Eigen::Vector3d> vps_;
    for (int i = 0; i < frontier.viewpoints_.size(); i++) {
      if (i > 5)
        break;
      vps_.push_back(frontier.viewpoints_[i].pos_);
    }
    clusters.push_back(vps_);
  }

  // clusters.push_back(frontier.filtered_cells_);
}

void FrontierFinder::getDormantFrontiers(vector<vector<Vector3d>> &clusters) {
  clusters.clear();
  for (auto ft : dormant_frontiers_)
    clusters.push_back(ft.cells_);
}

void FrontierFinder::getFrontierBoxes(
    vector<pair<Eigen::Vector3d, Eigen::Vector3d>> &boxes) {
  boxes.clear();
  for (auto frontier : frontiers_) {
    Vector3d center = (frontier.box_max_ + frontier.box_min_) * 0.5;
    Vector3d scale = frontier.box_max_ - frontier.box_min_;
    boxes.push_back(make_pair(center, scale));
  }
}

void FrontierFinder::getCheckTour(const int cluster_id,
                                  vector<checkPoint> &local_tour) {
  check_tour.clear();
  FrontierCluster next_clu = frontier_clusters_[cluster_id];
  for (int i = 0; i < next_clu.frts_.size(); i++) {
    checkPoint cp;
    Eigen::Vector3d cp_pos = next_clu.frts_[i].viewpoints_.front().pos_;
    cp.yaws_.push_back(next_clu.frts_[i].viewpoints_.front().yaw_);
    cp.pos_ = cp_pos;
    check_tour.push_back(cp);
  }

  auto updateCost = [](checkPoint &clu1, checkPoint &clu2) {
    // Search path from old cluster's top viewpoint to new cluster'
    double pos_cost_ij = (clu1.pos_ - clu2.pos_).norm() / ViewNode::vm_;
    double yaw_cost_ij =
        (clu1.yaws_.front() - clu2.yaws_.front()) / ViewNode::yd_;
    double cost_ij = max(pos_cost_ij, yaw_cost_ij);
    // Insert item for both old and new clusters
    clu1.costs_.push_back(cost_ij);
    clu2.costs_.push_back(cost_ij);
  };

  for (int it1 = 0; it1 < check_tour.size(); it1++) {
    for (int it2 = it1; it2 < check_tour.size(); it2++) {
      if (it1 == it2) {
        check_tour[it1].costs_.push_back(0.0);
      } else {
        updateCost(check_tour[it1], check_tour[it2]);
      }
    }
  }
  local_tour = check_tour;
}

void FrontierFinder::getPathForTour(const Vector3d &pos,
                                    const vector<int> &tsp_ids,
                                    vector<Vector3d> &path) {
  // Compute the path from current pos to the first frontier
  vector<Vector3d> segment;
  ViewNode::searchPath(pos, frontiers_[tsp_ids[0]].viewpoints_.front().pos_,
                       segment);
  path.insert(path.end(), segment.begin(), segment.end());
  // Get paths of tour passing all clusters
  for (int i = 0; i < tsp_ids.size() - 1; i++) {
    // Move to path to next cluster
    auto path_iter = frontiers_[tsp_ids[i]].paths_.begin();
    int next_idx = tsp_ids[i + 1];
    for (int j = 0; j < next_idx; j++)
      ++path_iter;
    path.insert(path.end(), path_iter->begin(), path_iter->end());
  }
}

void FrontierFinder::getFullCostMatrix(const Vector3d &cur_pos,
                                       const Vector3d &cur_vel,
                                       const Vector3d cur_yaw,
                                       Eigen::MatrixXd &mat) {

  // Use Asymmetric TSP
  int dimen = frontiers_.size();
  mat = Eigen::MatrixXd::Zero(dimen + 1, dimen + 1);
  // std::endl; Fill block for clusters
  int i = 1, j = 1;
  for (auto frt : frontiers_) {
    for (auto cs : frt.costs_) {
      mat(i, j++) = cs;
    }
    ++i;
    j = 1;
  }

  // Fill block from current state to clusters
  mat.leftCols<1>().setZero();
  j = 1;
  for (auto ftr : frontiers_) {
    Viewpoint vj = ftr.viewpoints_.front();
    vector<Vector3d> path;
    mat(0, j++) = ViewNode::computeCostPos(cur_pos, vj.pos_, cur_vel, path);
  }
  // std::cout << mat << std::endl;
} // namespace fast_planner

void FrontierFinder::getCheckTourCostMatrix(const Vector3d &cur_pos,
                                            const Vector3d &cur_vel,
                                            const Vector3d cur_yaw,
                                            const Vector3d &next_cluster_pos,
                                            Eigen::MatrixXd &mat) {
  // Use Asymmetric TSP
  int dimen = check_tour.size();
  mat = Eigen::MatrixXd::Zero(
      dimen + 2, dimen + 2); // the last one is the center of next cluster
  // std::endl; Fill block for clusters
  int i = 1, j = 1;
  for (auto cp : check_tour) {
    for (auto cs : cp.costs_) {
      mat(i, j++) = cs;
    }
    ++i;
    j = 1;
  }

  mat.leftCols<1>().setZero();
  j = 1;
  for (auto cp : check_tour) {
    vector<Vector3d> path;
    // fill block about current pos to frontiers
    mat(0, j) = ViewNode::computeCost(cur_pos, cp.pos_, cur_yaw[0],
                                      cp.yaws_.front(), cur_vel, 0.0, path);
    mat(j, 0) = 65536;
    // fill block about frontiers to next cluster
    mat(j, dimen + 1) = 
        ViewNode::computeCost(cp.pos_, next_cluster_pos, 0.0, 0.0,
                              Eigen::Vector3d::Zero(), 0.0, path);
    mat(dimen + 1, j) = 65536;
    j++;
  }
  mat(dimen + 1, 0) = 0;
  mat(0, dimen + 1) = 65536;
}

void FrontierFinder::getClusterMatrix(const Vector3d &cur_pos,
                                      const Vector3d &cur_vel,
                                      const Vector3d cur_yaw,
                                      Eigen::MatrixXd &cost_mat) {
  // Find the next cluster to visit
  int dimen = frontier_clusters_.size();
  cost_mat = Eigen::MatrixXd::Zero(dimen + 1, dimen + 1);
  int i = 1, j = 1;
  for (auto clu : frontier_clusters_) {
    for (auto cs : clu.costs_) {
      cost_mat(i, j++) = cs;
    }
    ++i;
    j = 1;
  }
  // Fill block from current state to clusters
  cost_mat.leftCols<1>().setZero();
  j = 1;
  for (auto clu : frontier_clusters_) {
    vector<Vector3d> path;
    cost_mat(0, j++) = ViewNode::computeCost(cur_pos, clu.center_, 0.0, 0.0,
                                             cur_vel, 0.0, path);
  }
}

void FrontierFinder::getClusterTour(const vector<int> indices,
                                    vector<Vector3d> &path) {
  path.clear();
  for (int i = 0; i < indices.size(); i++) {
    path.push_back(frontier_clusters_[indices[i]].center_);
  }
}

void FrontierFinder::getClusterCenter(vector<Vector3d> &centers) {
  centers.clear();
  for (auto clu : frontier_clusters_) {
    centers.push_back(clu.center_);
  }
}

void FrontierFinder::findViewpoints(const Vector3d &sample,
                                    const Vector3d &ftr_avg,
                                    vector<Viewpoint> &vps) {
  if (!edt_env_->sdf_map_->isInBox(sample) ||
      edt_env_->sdf_map_->getInflateOccupancy(sample) == 1 ||
      isNearObstacle(sample))
    return;

  double left_angle_, right_angle_, vertical_angle_, ray_length_;

  // Central yaw is determined by frontier's average position and sample
  auto dir = ftr_avg - sample;
  double hc = atan2(dir[1], dir[0]);

  vector<int> slice_gains;
  // Evaluate info gain of different slices
  for (double phi_h = -M_PI_2; phi_h <= M_PI_2 + 1e-3; phi_h += M_PI / 18) {
    // Compute gain of one slice
    int gain = 0;
    for (double phi_v = -vertical_angle_; phi_v <= vertical_angle_;
         phi_v += vertical_angle_ / 3) {
      // Find endpoint of a ray
      Vector3d end;
      end[0] = sample[0] + ray_length_ * cos(phi_v) * cos(hc + phi_h);
      end[1] = sample[1] + ray_length_ * cos(phi_v) * sin(hc + phi_h);
      end[2] = sample[2] + ray_length_ * sin(phi_v);

      // Do raycasting to check info gain
      Vector3i idx;
      raycaster_->input(sample, end);
      while (raycaster_->nextId(idx)) {
        // Hit obstacle, stop the ray
        if (edt_env_->sdf_map_->getOccupancy(idx) != SDFMap::FREE ||
            !edt_env_->sdf_map_->isInBox(idx))
          break;
        // Count number of unknown cells
        if (edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN)
          ++gain;
      }
    }
    slice_gains.push_back(gain);
  }

  // Sum up slices' gain to get different yaw's gain
  vector<pair<double, int>> yaw_gains;
  for (int i = 0; i < 6; ++i) // [-90,-10]-> [10,90], delta_yaw = 20, 6 groups
  {
    double yaw = hc - M_PI_2 + M_PI / 9.0 * i + right_angle_;
    int gain = 0;
    for (int j = 2 * i; j < 2 * i + 9; ++j) // 80 degree hFOV, 9 slices
      gain += slice_gains[j];
    yaw_gains.push_back(make_pair(yaw, gain));
  }

  // Get several yaws with highest gain
  vps.clear();
  sort(yaw_gains.begin(), yaw_gains.end(),
       [](const pair<double, int> &p1, const pair<double, int> &p2) {
         return p1.second > p2.second;
       });
  for (int i = 0; i < 3; ++i) {
    if (yaw_gains[i].second < min_visib_num_)
      break;
    Viewpoint vp = {sample, yaw_gains[i].first, yaw_gains[i].second};
    while (vp.yaw_ < -M_PI)
      vp.yaw_ += 2 * M_PI;
    while (vp.yaw_ > M_PI)
      vp.yaw_ -= 2 * M_PI;
    vps.push_back(vp);
  }
}

// Sample viewpoints around frontier's average position, check coverage to the
// frontier cells
void FrontierFinder::sampleViewpoints(Frontier &frontier) {
  // Evaluate sample viewpoints on circles, find ones that cover most cells
  for (double rc = candidate_rmin_,
              dr = (candidate_rmax_ - candidate_rmin_) / candidate_rnum_;
       rc <= candidate_rmax_ + 1e-3; rc += dr)
    for (double phi = -M_PI; phi < M_PI; phi += candidate_dphi_) {
      const Vector3d sample_pos =
          frontier.average_ + rc * Vector3d(cos(phi), sin(phi), 0);

      // Qualified viewpoint is in bounding box and in safe region
      if (!edt_env_->sdf_map_->isInBox(sample_pos) ||
          edt_env_->sdf_map_->getInflateOccupancy(sample_pos) == 1 ||
          edt_env_->sdf_map_->getOccupancy(sample_pos) != SDFMap::FREE ||
          isNearObstacle(sample_pos))
        continue;

      // Compute average yaw
      auto &cells = frontier.cells_;
      Eigen::Vector3d ref_dir = (cells.front() - sample_pos).normalized();
      double avg_yaw = 0.0;
      for (int i = 1; i < cells.size(); ++i) {
        Eigen::Vector3d dir = (cells[i] - sample_pos).normalized();
        double yaw = acos(dir.dot(ref_dir));
        if (ref_dir.cross(dir)[2] < 0)
          yaw = -yaw;
        avg_yaw += yaw;
      }
      avg_yaw = avg_yaw / cells.size() + atan2(ref_dir[1], ref_dir[0]);
      wrapYaw(avg_yaw);

      // Compute the fraction of covered and visible cells
      int visib_num =
          countVisibleCells(sample_pos, avg_yaw, cells, frontier.type_, false);
      if (visib_num > min_visib_num_) {
        Viewpoint vp = {sample_pos, avg_yaw, visib_num};
        frontier.viewpoints_.push_back(vp);
        // int gain = findMaxGainYaw(sample_pos, frontier, sample_yaw);
      }
      // }
    }
}

int FrontierFinder::getFrontierClusterNum() {
  return frontier_clusters_.size();
}

bool FrontierFinder::isFrontierCovered() {
  Vector3d update_min, update_max;
  edt_env_->sdf_map_->getUpdatedBox(update_min, update_max, false);

  auto checkChanges = [&](const vector<Frontier> &frontiers) {
    for (auto ftr : frontiers) {
      if (!haveOverlap(ftr.box_min_, ftr.box_max_, update_min, update_max))
        continue;
      const int change_thresh = min_view_finish_fraction_ * ftr.cells_.size();
      int change_num = 0;
      for (auto cell : ftr.cells_) {
        Eigen::Vector3i idx;
        edt_env_->sdf_map_->posToIndex(cell, idx);
        if (!(knownfree(idx) &&
              (isNeighborUnknown(idx) || isNeighborUnderObserved(idx))) &&
            ++change_num >= change_thresh)
          // if (!(knownfree(idx) && (isNeighborUnknown(idx))) &&
          //     ++change_num >= change_thresh)
          // if (!(knownfree(idx) && isNeighborUnderObserved(idx)) &&
          //     ++change_num >= change_thresh)
          return true;
      }
    }
    return false;
  };

  if (checkChanges(frontiers_))
    return true;

  return false;
}

bool FrontierFinder::isNearObstacle(const Eigen::Vector3d &pos) {
  const int vox_num = floor(min_candidate_clearance_ / resolution_);
  for (int x = -vox_num; x <= vox_num; ++x)
    for (int y = -vox_num; y <= vox_num; ++y)
      for (int z = -1; z <= 1; ++z) {
        Eigen::Vector3d vox;
        vox << pos[0] + x * resolution_, pos[1] + y * resolution_,
            pos[2] + z * resolution_;
        if (edt_env_->sdf_map_->getOccupancy(vox) == SDFMap::UNKNOWN ||
            edt_env_->sdf_map_->getOccupancy(vox) == SDFMap::OCCUPIED ||
            edt_env_->sdf_map_->getOccupancy(vox) == SDFMap::UNDEROBSERVED)
          return true;
      }
  return false;
}

int FrontierFinder::countVisibleCells(const Eigen::Vector3d &pos,
                                      const double &yaw,
                                      const vector<Eigen::Vector3d> &cluster,
                                      const int ftr_type, bool draw) {
  percep_utils_->setPose(pos, yaw);
  int visib_num = 0;
  Eigen::Vector3i idx;
  for (auto cell : cluster) {
    // Check if frontier cell is inside FOV
    if (!percep_utils_->insideFOV(cell))
      continue;

    // Check if frontier cell is visible (not occulded by obstacles)
    raycaster_->input(cell, pos);
    bool visib = true;
    while (raycaster_->nextId(idx)) {
      if (edt_env_->sdf_map_->getOccupancy(idx) != SDFMap::FREE ||
          !edt_env_->sdf_map_->isInBox(idx)) {
        visib = false;
        break;
      }
    }

    if (visib) {
      double dist = (cell - pos).norm();
      if (dist < edt_env_->sdf_map_->getBeliefDist()) {
        visib_num += 1;
      }
    }
  }
  return visib_num;
}

void FrontierFinder::downsample(const vector<Eigen::Vector3d> &cluster_in,
                                vector<Eigen::Vector3d> &cluster_out) {
  // downsamping cluster
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudf(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (auto cell : cluster_in)
    cloud->points.emplace_back(cell[0], cell[1], cell[2]);

  const double leaf_size = edt_env_->sdf_map_->getResolution() * down_sample_;
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(leaf_size, leaf_size, leaf_size);
  sor.filter(*cloudf);

  cluster_out.clear();
  for (auto pt : cloudf->points)
    cluster_out.emplace_back(pt.x, pt.y, pt.z);
}

void FrontierFinder::wrapYaw(double &yaw) {
  while (yaw < -M_PI)
    yaw += 2 * M_PI;
  while (yaw > M_PI)
    yaw -= 2 * M_PI;
}

Eigen::Vector3i FrontierFinder::searchClearVoxel(const Eigen::Vector3i &pt) {
  queue<Eigen::Vector3i> init_que;
  vector<Eigen::Vector3i> nbrs;
  Eigen::Vector3i cur, start_idx;
  init_que.push(pt);
  // visited_flag_[toadr(pt)] = 1;

  while (!init_que.empty()) {
    cur = init_que.front();
    init_que.pop();
    if (knownfree(cur)) {
      start_idx = cur;
      break;
    }

    nbrs = sixNeighbors(cur);
    for (auto nbr : nbrs) {
      int adr = toadr(nbr);
      // if (visited_flag_[adr] == 0)
      // {
      //   init_que.push(nbr);
      //   visited_flag_[adr] = 1;
      // }
    }
  }
  return start_idx;
}

void FrontierFinder::findTopNCost(
    const vector<double> cost, int N,
    priority_queue<pair<double, int>, vector<pair<double, int>>, CompareCost>
        &result) {
  for (auto it = cost.begin(); it != cost.end(); it++) {
    result.push(std::make_pair(*it, std::distance(cost.begin(), it)));
    if (result.size() > N) {
      result.pop();
    }
  }
}

inline vector<Eigen::Vector3i>
FrontierFinder::sixNeighbors(const Eigen::Vector3i &voxel) {
  vector<Eigen::Vector3i> neighbors(6);
  Eigen::Vector3i tmp;

  tmp = voxel - Eigen::Vector3i(1, 0, 0);
  neighbors[0] = tmp;
  tmp = voxel + Eigen::Vector3i(1, 0, 0);
  neighbors[1] = tmp;
  tmp = voxel - Eigen::Vector3i(0, 1, 0);
  neighbors[2] = tmp;
  tmp = voxel + Eigen::Vector3i(0, 1, 0);
  neighbors[3] = tmp;
  tmp = voxel - Eigen::Vector3i(0, 0, 1);
  neighbors[4] = tmp;
  tmp = voxel + Eigen::Vector3i(0, 0, 1);
  neighbors[5] = tmp;

  return neighbors;
}

inline vector<Eigen::Vector3i>
FrontierFinder::tenNeighbors(const Eigen::Vector3i &voxel) {
  vector<Eigen::Vector3i> neighbors(10);
  Eigen::Vector3i tmp;
  int count = 0;

  for (int x = -1; x <= 1; ++x) {
    for (int y = -1; y <= 1; ++y) {
      if (x == 0 && y == 0)
        continue;
      tmp = voxel + Eigen::Vector3i(x, y, 0);
      neighbors[count++] = tmp;
    }
  }
  neighbors[count++] = tmp - Eigen::Vector3i(0, 0, 1);
  neighbors[count++] = tmp + Eigen::Vector3i(0, 0, 1);
  return neighbors;
}

inline vector<Eigen::Vector3i>
FrontierFinder::allNeighbors(const Eigen::Vector3i &voxel) {
  vector<Eigen::Vector3i> neighbors(26);
  Eigen::Vector3i tmp;
  int count = 0;
  for (int x = -1; x <= 1; ++x)
    for (int y = -1; y <= 1; ++y)
      for (int z = -1; z <= 1; ++z) {
        if (x == 0 && y == 0 && z == 0)
          continue;
        tmp = voxel + Eigen::Vector3i(x, y, z);
        neighbors[count++] = tmp;
      }
  return neighbors;
}

inline bool FrontierFinder::isNeighborUnknown(const Eigen::Vector3i &voxel) {
  // At least one neighbor is unknown
  auto nbrs = sixNeighbors(voxel);
  for (auto nbr : nbrs) {
    if (edt_env_->sdf_map_->getOccupancy(nbr) == SDFMap::UNKNOWN)
      return true;
  }
  return false;
}

inline bool
FrontierFinder::isNeighborUnderObserved(const Eigen::Vector3i &voxel) {
  // At least one neighbor is unknown
  auto nbrs = sixNeighbors(voxel);
  // auto nbrs = tenNeighbors(voxel);
  for (auto nbr : nbrs) {
    if (edt_env_->sdf_map_->getOccupancy(nbr) == SDFMap::UNDEROBSERVED)
      return true;
  }
  return false;
}

inline int FrontierFinder::toadr(const Eigen::Vector3i &idx) {
  return edt_env_->sdf_map_->toAddress(idx);
}

inline bool FrontierFinder::knownfree(const Eigen::Vector3i &idx) {
  return edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::FREE;
}

inline bool FrontierFinder::inmap(const Eigen::Vector3i &idx) {
  return edt_env_->sdf_map_->isInMap(idx);
}

void FrontierFinder::fillBasicInfo(visualization_msgs::Marker &mk,
                                   const Eigen::Vector3d &scale,
                                   const Eigen::Vector4d &color,
                                   const string &ns, const int &id,
                                   const int &shape) {
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = id;
  mk.ns = ns;
  mk.type = shape;

  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = scale[0];
  mk.scale.y = scale[1];
  mk.scale.z = scale[2];
}

void FrontierFinder::drawText(const Eigen::Vector3d &pos, const string &text,
                              const double &scale, const Eigen::Vector4d &color,
                              const string &ns, const int &id) {
  visualization_msgs::Marker mk;
  fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id,
                visualization_msgs::Marker::TEXT_VIEW_FACING);

  mk.action = visualization_msgs::Marker::DELETE;
  debug_marker_.publish(mk);
  mk.text = text;
  mk.pose.position.x = pos[0];
  mk.pose.position.y = pos[1];
  mk.pose.position.z = pos[2];
  mk.action = visualization_msgs::Marker::ADD;
  debug_marker_.publish(mk);
  ros::Duration(0.0005).sleep();
}

Eigen::Vector4d FrontierFinder::getColor(const double &h, double alpha) {
  double h1 = h;
  if (h1 < 0.0) {
    h1 = 0.0;
  }

  double lambda;
  Eigen::Vector4d color1, color2;
  if (h1 >= -1e-4 && h1 < 1.0 / 6) {
    lambda = (h1 - 0.0) * 6;
    color1 = Eigen::Vector4d(1, 0, 0, 1);
    color2 = Eigen::Vector4d(1, 0, 1, 1);
  } else if (h1 >= 1.0 / 6 && h1 < 2.0 / 6) {
    lambda = (h1 - 1.0 / 6) * 6;
    color1 = Eigen::Vector4d(1, 0, 1, 1);
    color2 = Eigen::Vector4d(0, 0, 1, 1);
  } else if (h1 >= 2.0 / 6 && h1 < 3.0 / 6) {
    lambda = (h1 - 2.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 0, 1, 1);
    color2 = Eigen::Vector4d(0, 1, 1, 1);
  } else if (h1 >= 3.0 / 6 && h1 < 4.0 / 6) {
    lambda = (h1 - 3.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 1, 1, 1);
    color2 = Eigen::Vector4d(0, 1, 0, 1);
  } else if (h1 >= 4.0 / 6 && h1 < 5.0 / 6) {
    lambda = (h1 - 4.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 1, 0, 1);
    color2 = Eigen::Vector4d(1, 1, 0, 1);
  } else if (h1 >= 5.0 / 6 && h1 <= 1.0 + 1e-4) {
    lambda = (h1 - 5.0 / 6) * 6;
    color1 = Eigen::Vector4d(1, 1, 0, 1);
    color2 = Eigen::Vector4d(1, 0, 0, 1);
  }

  Eigen::Vector4d fcolor = (1 - lambda) * color1 + lambda * color2;
  fcolor(3) = alpha;

  return fcolor;
}
} // namespace fast_planner