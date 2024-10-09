#include <path_searching/astar2.h>
#include <plan_env/sdf_map.h>
#include <sstream>

using namespace std;
using namespace Eigen;

namespace fast_planner {
Astar::Astar() {}

Astar::~Astar() {
  for (int i = 0; i < (int)allocate_num_; i++)
    delete path_node_pool_[i];
}

void Astar::init(ros::NodeHandle &nh, const EDTEnvironment::Ptr &env) {
  nh.param("astar/resolution_astar", resolution_, -1.0);
  nh.param("astar/lambda_heu", lambda_heu_, -1.0);
  nh.param("astar/max_search_time", max_search_time_, -1.0);
  nh.param("astar/max_search_time_unknown", max_search_time_unknown_, -1.0);
  nh.param("astar/allocate_num", allocate_num_, -1);
  nh.param("astar/epsilon", epsilon_, 1e-4);

  astar_pub_ = nh.advertise<visualization_msgs::MarkerArray>("astar_vis", 1);

  tie_breaker_ = 1.0 + 1.0 / 1000;

  this->edt_env_ = env;

  /* ---------- map params ---------- */
  this->inv_resolution_ = 1.0 / resolution_;
  edt_env_->sdf_map_->getRegion(origin_, map_size_3d_);
  // cout << "init astar" << endl;
  // cout << "origin_: " << origin_.transpose() << endl;
  // cout << "map size: " << map_size_3d_.transpose() << endl;

  path_node_pool_.resize(allocate_num_);
  for (int i = 0; i < (int)allocate_num_; i++) {
    path_node_pool_[i] = new Node;
  }
  use_node_num_ = 0;
  iter_num_ = 0;
  early_terminate_cost_ = 0.0;
}

// Dynamically change resolution to adapt to different environment
void Astar::setResolution(const double &res) {
  resolution_ = res;
  this->inv_resolution_ = 1.0 / resolution_;
}
void Astar::setMaxSearchTime(const double &t) { max_search_time_ = t; }
int Astar::search(const Eigen::Vector3d &start_pt,
                  const Eigen::Vector3d &end_pt) {
  NodePtr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->position = start_pt;
  posToIndex(start_pt, cur_node->index);
  cur_node->g_score = 0.0;
  cur_node->f_score = lambda_heu_ * getDiagHeu(cur_node->position, end_pt);

  Eigen::Vector3i end_index;
  posToIndex(end_pt, end_index);

  open_set_.push(cur_node);
  open_set_map_.insert(make_pair(cur_node->index, cur_node));
  use_node_num_ += 1;

  const auto t1 = ros::Time::now();

  /* ---------- search loop ---------- */
  while (!open_set_.empty()) {
    cur_node = open_set_.top();

    bool reach_end = cur_node->index(0) == end_index(0) &&
                     cur_node->index(1) == end_index(1) &&
                     cur_node->index(2) == end_index(2);

    if (reach_end) {
      backtrack(cur_node, end_pt);
      return REACH_END;
    }

    if (abs(cur_node->index(0) - end_index(0)) <= 1 &&
        abs(cur_node->index(1) - end_index(1)) <= 1 &&
        abs(cur_node->index(2) - end_index(2)) <= 1) {
      bool safe = true;
      Vector3d dir = end_pt - cur_node->position;
      double len = dir.norm();
      dir.normalize();
      for (double l = 0.1; l < len; l += 0.1) {
        Vector3d ckpt = cur_node->position + l * dir;
        if (edt_env_->sdf_map_->getInflateOccupancy(ckpt) == 1 ||
            edt_env_->sdf_map_->getOccupancy(ckpt) == SDFMap::UNKNOWN) {
          safe = false;
          break;
        }
      }
      if (safe) {
        backtrack(cur_node, end_pt);
        return REACH_END;
      }
    }

    // Early termination if time up
    if ((ros::Time::now() - t1).toSec() > max_search_time_) {
      // ROS_WARN("[AStar] Early terminated, duration: %f", (ros::Time::now() -
      // t1).toSec());
      early_terminate_cost_ =
          cur_node->g_score + getDiagHeu(cur_node->position, end_pt);
      return NO_PATH;
    }

    open_set_.pop();
    open_set_map_.erase(cur_node->index);
    close_set_map_.insert(make_pair(cur_node->index, 1));
    iter_num_ += 1;

    Eigen::Vector3d cur_pos = cur_node->position;
    Eigen::Vector3d nbr_pos;
    Eigen::Vector3d step;

    for (double dx = -resolution_ + epsilon_; dx <= resolution_ + epsilon_;
         dx += resolution_)
      for (double dy = -resolution_ + epsilon_; dy <= resolution_ + epsilon_;
           dy += resolution_)
        for (double dz = -resolution_ + epsilon_; dz <= resolution_ + epsilon_;
             dz += resolution_) {
          step << dx, dy, dz;
          if (step.norm() < epsilon_)
            continue;
          nbr_pos = cur_pos + step;
          // Check safety
          if (!edt_env_->sdf_map_->isInBox(nbr_pos))
            continue;

          if (edt_env_->sdf_map_->getInflateOccupancy(nbr_pos) == 1 ||
              edt_env_->sdf_map_->getOccupancy(nbr_pos) == SDFMap::UNKNOWN)
            continue;

          bool safe = true;
          Vector3d dir = nbr_pos - cur_pos;
          double len = dir.norm();
          dir.normalize();
          for (double l = 0.1; l < len; l += 0.1) {
            Vector3d ckpt = cur_pos + l * dir;
            if (edt_env_->sdf_map_->getInflateOccupancy(ckpt) == 1 ||
                edt_env_->sdf_map_->getOccupancy(ckpt) == SDFMap::UNKNOWN) {
              safe = false;
              break;
            }
          }
          if (!safe) {
            continue;
          }

          // Check not in close set
          Eigen::Vector3i nbr_idx;
          posToIndex(nbr_pos, nbr_idx);
          if (close_set_map_.find(nbr_idx) != close_set_map_.end()) {
            continue;
          }

          NodePtr neighbor;
          double tmp_g_score = step.norm() + cur_node->g_score;
          auto node_iter = open_set_map_.find(nbr_idx);
          if (node_iter == open_set_map_.end()) {
            neighbor = path_node_pool_[use_node_num_];
            use_node_num_ += 1;
            if (use_node_num_ == allocate_num_) {
              ROS_WARN("[AStar] Run out of node pool. Duration: %f",
                       (ros::Time::now() - t1).toSec());
              return NO_PATH;
            }
            neighbor->index = nbr_idx;
            neighbor->position = nbr_pos;
          } else if (tmp_g_score < node_iter->second->g_score) {
            neighbor = node_iter->second;
          } else
            continue;

          neighbor->parent = cur_node;
          neighbor->g_score = tmp_g_score;
          neighbor->f_score =
              tmp_g_score + lambda_heu_ * getDiagHeu(nbr_pos, end_pt);
          open_set_.push(neighbor);
          open_set_map_[nbr_idx] = neighbor;
        }
  }
  // cout << "open set empty, no path!" << endl;
  // cout << "use node num: " << use_node_num_ << endl;
  // cout << "iter num: " << iter_num_ << endl;
  return NO_PATH;
}

int Astar::searchUnknown(const Eigen::Vector3d &start_pt,
                         const Eigen::Vector3d &end_pt) {
  NodePtr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->position = start_pt;
  posToIndex(start_pt, cur_node->index);
  cur_node->g_score = 0.0;
  cur_node->f_score = lambda_heu_ * getDiagHeu(cur_node->position, end_pt);

  Eigen::Vector3i end_index;
  posToIndex(end_pt, end_index);

  open_set_.push(cur_node);
  open_set_map_.insert(make_pair(cur_node->index, cur_node));
  use_node_num_ += 1;

  const auto t1 = ros::Time::now();

  /* ---------- search loop ---------- */
  while (!open_set_.empty()) {
    cur_node = open_set_.top();

    bool reach_end = cur_node->index(0) == end_index(0) &&
                     cur_node->index(1) == end_index(1) &&
                     cur_node->index(2) == end_index(2);

    if (reach_end) {
      backtrack(cur_node, end_pt);
      return REACH_END;
    }

    if (abs(cur_node->index(0) - end_index(0)) <= 1 &&
        abs(cur_node->index(1) - end_index(1)) <= 1 &&
        abs(cur_node->index(2) - end_index(2)) <= 1) {
      bool safe = true;
      Vector3d dir = end_pt - cur_node->position;
      double len = dir.norm();
      dir.normalize();
      for (double l = 0.1; l < len; l += 0.1) {
        Vector3d ckpt = cur_node->position + l * dir;
        if (edt_env_->sdf_map_->getInflateOccupancy(ckpt) == 1) {
          safe = false;
          break;
        }
      }
      if (safe) {
        backtrack(cur_node, end_pt);
        return REACH_END;
      }
    }

    // Early termination if time up
    if ((ros::Time::now() - t1).toSec() > max_search_time_) {
      // ROS_WARN("[AStar] Early terminated, duration: %f", (ros::Time::now() -
      // t1).toSec());
      early_terminate_cost_ =
          cur_node->g_score + getDiagHeu(cur_node->position, end_pt);
      return NO_PATH;
    }

    open_set_.pop();
    open_set_map_.erase(cur_node->index);
    close_set_map_.insert(make_pair(cur_node->index, 1));
    iter_num_ += 1;

    Eigen::Vector3d cur_pos = cur_node->position;
    Eigen::Vector3d nbr_pos;
    Eigen::Vector3d step;

    for (double dx = -resolution_ + epsilon_; dx <= resolution_ + epsilon_;
         dx += resolution_)
      for (double dy = -resolution_ + epsilon_; dy <= resolution_ + epsilon_;
           dy += resolution_)
        for (double dz = -resolution_ + epsilon_; dz <= resolution_ + epsilon_;
             dz += resolution_) {
          step << dx, dy, dz;
          if (step.norm() < epsilon_)
            continue;
          nbr_pos = cur_pos + step;
          // Check safety
          if (!edt_env_->sdf_map_->isInBox(nbr_pos)) {
            continue;
          }

          if (edt_env_->sdf_map_->getInflateOccupancy(nbr_pos) == 1) {
            continue;
          }

          bool safe = true;
          Vector3d dir = nbr_pos - cur_pos;
          double len = dir.norm();
          dir.normalize();
          for (double l = 0.1; l < len; l += 0.1) {
            Vector3d ckpt = cur_pos + l * dir;
            if (edt_env_->sdf_map_->getInflateOccupancy(ckpt) == 1) {
              safe = false;
              break;
            }
          }
          if (!safe) {
            continue;
          }

          // Check not in close set
          Eigen::Vector3i nbr_idx;
          posToIndex(nbr_pos, nbr_idx);
          if (close_set_map_.find(nbr_idx) != close_set_map_.end()) {
            continue;
          }

          NodePtr neighbor;
          double tmp_g_score = step.norm() + cur_node->g_score;
          auto node_iter = open_set_map_.find(nbr_idx);
          if (node_iter == open_set_map_.end()) {
            neighbor = path_node_pool_[use_node_num_];
            use_node_num_ += 1;
            if (use_node_num_ == allocate_num_) {
              ROS_WARN("[AStar] Run out of node pool. Duration: %f",
                       (ros::Time::now() - t1).toSec());
              return NO_PATH;
            }
            neighbor->index = nbr_idx;
            neighbor->position = nbr_pos;
          } else if (tmp_g_score < node_iter->second->g_score) {
            neighbor = node_iter->second;
          } else
            continue;

          neighbor->parent = cur_node;
          neighbor->g_score = tmp_g_score;
          neighbor->f_score =
              tmp_g_score + lambda_heu_ * getDiagHeu(nbr_pos, end_pt);
          open_set_.push(neighbor);
          open_set_map_[nbr_idx] = neighbor;
        }
  }
  // cout << "open set empty, no path!" << endl;
  // cout << "use node num: " << use_node_num_ << endl;
  // cout << "iter num: " << iter_num_ << endl;
  return NO_PATH;
}

int Astar::searchUnknownBBox(const Eigen::Vector3d &start_pt,
                             const Eigen::Vector3d &end_pt,
                             const Eigen::Vector3d &bbox_min,
                             const Eigen::Vector3d &bbox_max) {
  NodePtr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->position = start_pt;
  posToIndex(start_pt, cur_node->index);
  cur_node->g_score = 0.0;
  cur_node->f_score = lambda_heu_ * getDiagHeu(cur_node->position, end_pt);

  Eigen::Vector3i end_index;
  posToIndex(end_pt, end_index);

  open_set_.push(cur_node);
  open_set_map_.insert(make_pair(cur_node->index, cur_node));
  use_node_num_ += 1;

  const auto t1 = ros::Time::now();

  /* ---------- search loop ---------- */
  while (!open_set_.empty()) {
    cur_node = open_set_.top();

    bool reach_end = cur_node->index(0) == end_index(0) &&
                     cur_node->index(1) == end_index(1) &&
                     cur_node->index(2) == end_index(2);

    if (reach_end) {
      backtrack(cur_node, end_pt);
      return REACH_END;
    }

    if (abs(cur_node->index(0) - end_index(0)) <= 1 &&
        abs(cur_node->index(1) - end_index(1)) <= 1 &&
        abs(cur_node->index(2) - end_index(2)) <= 1) {
      bool safe = true;
      Vector3d dir = end_pt - cur_node->position;
      double len = dir.norm();
      dir.normalize();
      for (double l = 0.1; l < len; l += 0.1) {
        Vector3d ckpt = cur_node->position + l * dir;
        if (edt_env_->sdf_map_->getInflateOccupancy(ckpt) == 1) {
          safe = false;
          break;
        }
      }
      if (safe) {
        backtrack(cur_node, end_pt);
        return REACH_END;
      }
    }

    // Early termination if time up
    if ((ros::Time::now() - t1).toSec() > max_search_time_) {
      // ROS_WARN("[AStar] Early terminated, duration: %f", (ros::Time::now() -
      // t1).toSec());
      early_terminate_cost_ =
          cur_node->g_score + getDiagHeu(cur_node->position, end_pt);
      return NO_PATH;
    }

    open_set_.pop();
    open_set_map_.erase(cur_node->index);
    close_set_map_.insert(make_pair(cur_node->index, 1));
    iter_num_ += 1;

    Eigen::Vector3d cur_pos = cur_node->position;
    Eigen::Vector3d nbr_pos;
    Eigen::Vector3d step;

    for (double dx = -resolution_ + epsilon_; dx <= resolution_ + epsilon_;
         dx += resolution_)
      for (double dy = -resolution_ + epsilon_; dy <= resolution_ + epsilon_;
           dy += resolution_)
        for (double dz = -resolution_ + epsilon_; dz <= resolution_ + epsilon_;
             dz += resolution_) {
          step << dx, dy, dz;
          if (step.norm() < epsilon_)
            continue;
          nbr_pos = cur_pos + step;
          // Check safety
          auto isInBox = [&](const Eigen::Vector3d &pos) {
            return pos(0) >= bbox_min(0) && pos(0) <= bbox_max(0) &&
                   pos(1) >= bbox_min(1) && pos(1) <= bbox_max(1) &&
                   pos(2) >= bbox_min(2) && pos(2) <= bbox_max(2);
          };
          if (!isInBox(nbr_pos)) {
            continue;
          }

          if (edt_env_->sdf_map_->getInflateOccupancy(nbr_pos) == 1) {
            continue;
          }

          bool safe = true;
          Vector3d dir = nbr_pos - cur_pos;
          double len = dir.norm();
          dir.normalize();
          for (double l = 0.1; l < len; l += 0.1) {
            Vector3d ckpt = cur_pos + l * dir;
            if (edt_env_->sdf_map_->getInflateOccupancy(ckpt) == 1) {
              safe = false;
              break;
            }
          }
          if (!safe) {
            continue;
          }

          // Check not in close set
          Eigen::Vector3i nbr_idx;
          posToIndex(nbr_pos, nbr_idx);
          if (close_set_map_.find(nbr_idx) != close_set_map_.end()) {
            continue;
          }

          NodePtr neighbor;
          double tmp_g_score = step.norm() + cur_node->g_score;
          auto node_iter = open_set_map_.find(nbr_idx);
          if (node_iter == open_set_map_.end()) {
            neighbor = path_node_pool_[use_node_num_];
            use_node_num_ += 1;
            if (use_node_num_ == allocate_num_) {
              ROS_WARN("[AStar] Run out of node pool. Duration: %f",
                       (ros::Time::now() - t1).toSec());
              return NO_PATH;
            }
            neighbor->index = nbr_idx;
            neighbor->position = nbr_pos;
          } else if (tmp_g_score < node_iter->second->g_score) {
            neighbor = node_iter->second;
          } else
            continue;

          neighbor->parent = cur_node;
          neighbor->g_score = tmp_g_score;
          neighbor->f_score =
              tmp_g_score + lambda_heu_ * getDiagHeu(nbr_pos, end_pt);
          open_set_.push(neighbor);
          open_set_map_[nbr_idx] = neighbor;
        }
  }
  // cout << "open set empty, no path!" << endl;
  // cout << "use node num: " << use_node_num_ << endl;
  // cout << "iter num: " << iter_num_ << endl;
  return NO_PATH;
}

double Astar::getEarlyTerminateCost() { return early_terminate_cost_; }

void Astar::reset() {
  open_set_map_.clear();
  close_set_map_.clear();
  path_nodes_.clear();

  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0>
      empty_queue;
  open_set_.swap(empty_queue);
  for (int i = 0; i < (int)use_node_num_; i++) {
    path_node_pool_[i]->parent = NULL;
  }
  use_node_num_ = 0;
  iter_num_ = 0;
}

double Astar::pathLength(const vector<Eigen::Vector3d> &path) {
  double length = 0.0;
  if ((int)path.size() < 2)
    return length;
  for (int i = 0; i < (int)path.size() - 1; ++i)
    length += (path[i + 1] - path[i]).norm();
  return length;
}

void Astar::backtrack(const NodePtr &end_node, const Eigen::Vector3d &end) {
  path_nodes_.push_back(end);
  path_nodes_.push_back(end_node->position);
  NodePtr cur_node = end_node;
  while (cur_node->parent != NULL) {
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node->position);
  }
  reverse(path_nodes_.begin(), path_nodes_.end());
}

std::vector<Eigen::Vector3d> Astar::getPath() { return path_nodes_; }

double Astar::getDiagHeu(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2) {
  double dx = fabs(x1(0) - x2(0));
  double dy = fabs(x1(1) - x2(1));
  double dz = fabs(x1(2) - x2(2));
  double h = 0.0;
  double diag = min(min(dx, dy), dz);
  dx -= diag;
  dy -= diag;
  dz -= diag;

  if (dx < epsilon_) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
  }
  if (dy < epsilon_) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
  }
  if (dz < epsilon_) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
  }
  return tie_breaker_ * h;
}

double Astar::getManhHeu(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2) {
  double dx = fabs(x1(0) - x2(0));
  double dy = fabs(x1(1) - x2(1));
  double dz = fabs(x1(2) - x2(2));
  return tie_breaker_ * (dx + dy + dz);
}

double Astar::getEuclHeu(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2) {
  return tie_breaker_ * (x2 - x1).norm();
}

std::vector<Eigen::Vector3d> Astar::getVisited() {
  vector<Eigen::Vector3d> visited;
  for (int i = 0; i < (int)use_node_num_; ++i)
    visited.push_back(path_node_pool_[i]->position);
  return visited;
}

void Astar::posToIndex(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) {
  idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
}

void Astar::publishNode(const NodePtr node) {
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "node";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = node->position(0);
  marker.pose.position.y = node->position(1);
  marker.pose.position.z = node->position(2);
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 0.5;

  marker_array.markers.push_back(marker);
  astar_pub_.publish(marker_array);
}

} // namespace fast_planner