#include <active_perception/graph_node.h>
#include <path_searching/astar2.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>

namespace fast_planner {
// Static data
double ViewNode::vm_;
double ViewNode::am_;
double ViewNode::yd_;
double ViewNode::ydd_;
double ViewNode::w_dir_;
shared_ptr<Astar> ViewNode::astar_;
shared_ptr<RayCaster> ViewNode::caster_;
shared_ptr<SDFMap> ViewNode::map_;

// Graph node for viewpoints planning
ViewNode::ViewNode(const Vector3d &p, const double &y) {
  pos_ = p;
  yaw_ = y;
  parent_ = nullptr;
  vel_.setZero(); // vel is zero by default, should be set explicitly
}

double ViewNode::costTo(const ViewNode::Ptr &node) {
  vector<Vector3d> path;
  double c = ViewNode::computeCost(pos_, node->pos_, yaw_, node->yaw_, vel_,
                                   yaw_dot_, path);
  // std::cout << "cost from " << id_ << " to " << node->id_ << " is: " << c <<
  // std::endl;
  return c;
}

double ViewNode::searchPath(const Vector3d &p1, const Vector3d &p2,
                            vector<Vector3d> &path) {
  // Try connect two points with straight line
  bool safe = true;
  Vector3i idx;
  caster_->input(p1, p2);
  while (caster_->nextId(idx)) {
    // if (map_->getInflateOccupancy(idx) == 1 || map_->getOccupancy(idx) ==
    // SDFMap::UNKNOWN ||
    //     !map_->isInBox(idx))
    if (map_->getInflateOccupancy(idx) == 1 ||
        !map_->isInBox(idx)) // Treat UNKNOWN as FREE in coverage path finding.
    {
      safe = false;
      break;
    }
  }
  if (safe) {
    path = {p1, p2};
    return (p1 - p2).norm();
  }
  // Search a path using decreasing resolution
  // vector<double> res = {0.2};
  // for (int k = 0; k < res.size(); ++k) {
  astar_->reset();
  // astar_->setResolution(res[k]);
  if (astar_->search(p1, p2) == Astar::REACH_END) {
    path = astar_->getPath();
    return astar_->pathLength(path);
  }
  // Use Astar early termination cost as an estimate
  path = {p1, p2};
  return 1000.0 + (p1 - p2).norm();
}

double ViewNode::searchPathUnknown(const Vector3d &p1, const Vector3d &p2,
                                   vector<Vector3d> &path) {
  // Try connect two points with straight line
  bool safe = true;
  Vector3i idx;
  caster_->input(p1, p2);
  while (caster_->nextId(idx)) {
    if (map_->getInflateOccupancy(idx) == 1 || !map_->isInBox(idx)) {
      safe = false;
      break;
    }
  }
  if (safe) {
    path = {p1, p2};
    return (p1 - p2).norm();
  }
  // Search a path using decreasing resolution
  // vector<double> res = {0.2};
  // for (int k = 0; k < res.size(); ++k) {
  astar_->reset();
  // astar_->setResolution(res[k]);
  if (astar_->searchUnknown(p1, p2) == Astar::REACH_END) {
    path = astar_->getPath();
    return astar_->pathLength(path);
  }
  // }
  // Use Astar early termination cost as an estimate
  path = {p1, p2};
  return 1000.0 + (p1 - p2).norm();
  // return (p1 - p2).norm() * 10.0;
}

double ViewNode::searchPathUnknownBBox(const Vector3d &p1, const Vector3d &p2,
                                       const Vector3d &bbox_min,
                                       const Vector3d &bbox_max,
                                       vector<Vector3d> &path) {
  // Try connect two points with straight line
  auto isInBox = [&](const Vector3d &p) {
    return p.x() > bbox_min.x() && p.x() < bbox_max.x() &&
           p.y() > bbox_min.y() && p.y() < bbox_max.y() &&
           p.z() > bbox_min.z() && p.z() < bbox_max.z();
  };

  bool safe = true;
  Vector3i idx;
  caster_->input(p1, p2);
  while (caster_->nextId(idx)) {
    if (map_->getInflateOccupancy(idx) == 1 || !map_->isInBox(idx)) {
      safe = false;
      break;
    }
  }
  if (safe) {
    path = {p1, p2};
    return (p1 - p2).norm();
  }
  // Search a path using decreasing resolution
  // vector<double> res = {0.2};
  // for (int k = 0; k < res.size(); ++k) {
  astar_->reset();
  // astar_->setResolution(res[k]);
  if (astar_->searchUnknownBBox(p1, p2, bbox_min, bbox_max) ==
      Astar::REACH_END) {
    path = astar_->getPath();
    return astar_->pathLength(path);
  }
  // }
  // Use Astar early termination cost as an estimate
  path = {p1, p2};
  return 1000.0 + (p1 - p2).norm();
  // return (p1 - p2).norm() * 10.0;
}

double ViewNode::computeCost(const Vector3d &p1, const Vector3d &p2,
                             const double &y1, const double &y2,
                             const Vector3d &v1, const double &yd1,
                             vector<Vector3d> &path) {
  // Cost of position change
  path.clear();
  double pos_dist = ViewNode::searchPath(p1, p2, path);

  double a = am_ / sqrt(2.0);
  Vector3d dir = (p2 - p1).normalized();
  double v_horizontal = v1.dot(dir);
  double v_vertical = sqrt(v1.norm()*v1.norm() - v_horizontal * v_horizontal);
  double t_vertical = 2.0 * v_vertical / a;
  double dist_thresh = (vm_ * vm_ - v_horizontal * v_horizontal) / (2.0 * a);
  double t_horizontal =
      dist_thresh < pos_dist
          ? (sqrt(v_horizontal * v_horizontal + 2.0 * a * pos_dist) -
             v_horizontal) /
                a
          : (vm_ - v_horizontal) / a + (pos_dist - dist_thresh) / vm_;

  double pos_cost = max(t_horizontal, t_vertical);

  // // Consider velocity change
  // if (v1.norm() > 1e-3) {
  //   // Vector3d dir = (p2 - p1).normalized();
  //   // Vector3d vdir = v1.normalized();
  //   // double diff = acos(vdir.dot(dir));
  //   // pos_cost += w_dir_ * diff;
  //   // double vc = v1.dot(dir);
  //   // pos_cost += w_dir_ * pow(vm_ - fabs(vc), 2) / (2 * vm_ * am_);
  //   // if (vc < 0)
  //   //   pos_cost += w_dir_ * 2 * fabs(vc) / am_;

  //   double vc = v1.dot(dir);
  //   double t = w_dir_ * (vm_ - vc) / am_;
  //   pos_cost += t;
  // }

  // Cost of yaw change
  double diff = fabs(y2 - y1);
  diff = min(diff, 2 * M_PI - diff);
  double yaw_cost = diff / yd_;
  return max(pos_cost, yaw_cost);
}

double ViewNode::computeCostPos(const Vector3d &p1, const Vector3d &p2,
                                const Vector3d &v1, vector<Vector3d> &path) {
  // Cost of position change
  double pos_dist = ViewNode::searchPath(p1, p2, path);
  double pos_cost;
  // if (v1.norm() == 0.0)
  //   pos_cost = pos_dist / vm_;
  // else {
  //   Eigen::Vector3d next_dir_ = (p2 - p1).normalized();
  //   Eigen::Vector3d last_dir = v1.normalized();
  //   double cos_dir_ = next_dir_.dot(last_dir);
  //   double v1_x_ = v1.norm() * cos_dir_;
  //   double v1_y_ = v1.norm() * sqrt(1 - cos_dir_ * cos_dir_);
  //   double t_x_ =
  //       sqrt(v1.norm() * v1.norm() + 2 * am_ * pos_dist - v1.norm()) / am_;
  //   double t_y_ = 2 * v1_y_ / am_;
  //   pos_cost = max(t_x_, t_y_);
  // }
  pos_cost = pos_dist / vm_;

  return pos_cost;
}

double ViewNode::computeCostUnknown(const Vector3d &p1, const Vector3d &p2,
                                    const double &y1, const double &y2,
                                    const Vector3d &v1, const double &yd1,
                                    vector<Vector3d> &path) {
  // Cost of position change
  double pos_cost = ViewNode::searchPathUnknown(p1, p2, path) / vm_;
  // Consider velocity change
  if (v1.norm() > 1e-3) {
    // Vector3d dir = (p2 - p1).normalized();
    // Vector3d vdir = v1.normalized();
    // double diff = acos(vdir.dot(dir));
    // pos_cost += w_dir_ * diff;

    // double vc = v1.dot(dir);
    // pos_cost += w_dir_ * pow(vm_ - fabs(vc), 2) / (2 * vm_ * am_);
    // if (vc < 0)
    //   pos_cost += w_dir_ * 2 * fabs(vc) / am_;

    Vector3d dir = (p2 - p1).normalized();
    double vc = v1.dot(dir);
    double t = w_dir_ * (vm_ - vc) / am_;
    pos_cost += t;
  }
  // Cost of yaw change
  double diff = fabs(y2 - y1);
  diff = min(diff, 2 * M_PI - diff);
  double yaw_cost = diff / yd_;
  return max(pos_cost, yaw_cost);
}

double ViewNode::computeCostUnknownBBox(const Vector3d &p1, const Vector3d &p2,
                                        const double &y1, const double &y2,
                                        const Vector3d &v1, const double &yd1,
                                        const Vector3d &bbox_min,
                                        const Vector3d &bbox_max,
                                        vector<Vector3d> &path) {
  // Cost of position change
  double pos_cost =
      ViewNode::searchPathUnknownBBox(p1, p2, bbox_min, bbox_max, path) / vm_;

  // Consider velocity change
  if (v1.norm() > 1e-3) {
    Vector3d dir = (p2 - p1).normalized();
    Vector3d vdir = v1.normalized();
    double diff = acos(vdir.dot(dir));
    pos_cost += w_dir_ * diff;
    // double vc = v1.dot(dir);
    // pos_cost += w_dir_ * pow(vm_ - fabs(vc), 2) / (2 * vm_ * am_);
    // if (vc < 0)
    //   pos_cost += w_dir_ * 2 * fabs(vc) / am_;
  }

  // Cost of yaw change
  double diff = fabs(y2 - y1);
  diff = min(diff, 2 * M_PI - diff);
  double yaw_cost = diff / yd_;
  return max(pos_cost, yaw_cost);
}

} // namespace fast_planner