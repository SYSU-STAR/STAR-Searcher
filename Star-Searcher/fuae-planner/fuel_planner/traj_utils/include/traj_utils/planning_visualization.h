#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <Eigen/Eigen>
#include <active_perception/traj_visibility.h>
#include <algorithm>
#include <bspline/non_uniform_bspline.h>
#include <iostream>
#include <path_searching/topo_prm.h>
#include <plan_env/obj_predictor.h>
#include <poly_traj/polynomial_traj.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <vector>
#include <visualization_msgs/Marker.h>
using std::string;
using std::vector;
namespace fast_planner {
class PlanningVisualization {
private:
  enum TRAJECTORY_PLANNING_ID {
    GOAL = 1,
    PATH = 200,
    BSPLINE = 300,
    BSPLINE_CTRL_PT = 400,
    POLY_TRAJ = 500
  };

  enum TOPOLOGICAL_PATH_PLANNING_ID {
    GRAPH_NODE = 1,
    GRAPH_EDGE = 100,
    RAW_PATH = 200,
    FILTERED_PATH = 300,
    SELECT_PATH = 400
  };

  /* data */
  /* visib_pub is seperated from previous ones for different info */
  ros::NodeHandle node;
  ros::Publisher traj_pub_;      // 0
  ros::Publisher topo_pub_;      // 1
  ros::Publisher predict_pub_;   // 2
  ros::Publisher visib_pub_;     // 3, visibility constraints
  ros::Publisher frontier_pub_;  // 4, frontier searching
  ros::Publisher yaw_pub_;       // 5, yaw trajectory
  ros::Publisher viewpoint_pub_; // 6, viewpoint planning
  ros::Publisher hgrid_pub_;     // 7, hierarchical grid
  ros::Publisher debug_pub_;     // 8, debug
  ros::Publisher markerarray_pub_;
  vector<ros::Publisher> pubs_; //

  int last_topo_path1_num_;
  int last_topo_path2_num_;
  int last_bspline_phase1_num_;
  int last_bspline_phase2_num_;
  int last_frontier_num_;

public:
  PlanningVisualization(/* args */) {}
  ~PlanningVisualization() {}
  PlanningVisualization(ros::NodeHandle &nh);

  ros::Publisher frts_pub_;
  vector<Eigen::Vector3d> arrow_start;
  vector<Eigen::Vector3d> arrow_end;

  enum PUBLISHER {
    TRAJECTORY = 0,
    TOPOLOGICAL_PATH = 1,
    PREDICTION = 2,
    VISIBILITY = 3,
    FRONTIER = 4,
    YAW_TRAJ = 5,
    VIEWPOINT = 6,
    HGRID = 7,
    DEBUG = 8,
    MarkerArray = 9
  };

  struct Color {
    double r_;
    double g_;
    double b_;
    double a_;

    Color() : r_(0), g_(0), b_(0), a_(1) {}
    Color(double r, double g, double b) : Color(r, g, b, 1.) {}
    Color(double r, double g, double b, double a)
        : r_(r), g_(g), b_(b), a_(a) {}
    Color(int r, int g, int b) {
      r_ = static_cast<double>(r) / 255.;
      g_ = static_cast<double>(g) / 255.;
      b_ = static_cast<double>(b) / 255.;
      a_ = 1.;
    }
    Color(int r, int g, int b, int a) {
      r_ = static_cast<double>(r) / 255.;
      g_ = static_cast<double>(g) / 255.;
      b_ = static_cast<double>(b) / 255.;
      a_ = static_cast<double>(a) / 255.;
    }

    static Eigen::Vector4d toEigen(const Color &color) {
      return Eigen::Vector4d(color.r_, color.g_, color.b_, color.a_);
    }

    static const Eigen::Vector4d White() {
      return toEigen(Color(255, 255, 255));
    }
    static const Eigen::Vector4d Black() { return toEigen(Color(0, 0, 0)); }
    static const Eigen::Vector4d Gray() {
      return toEigen(Color(127, 127, 127));
    }
    static const Eigen::Vector4d Red() { return toEigen(Color(255, 0, 0)); }
    static const Eigen::Vector4d DeepRed() { return toEigen(Color(127, 0, 0)); }
    static const Eigen::Vector4d Green() { return toEigen(Color(0, 255, 0)); }
    static const Eigen::Vector4d DeepGreen() {
      return toEigen(Color(0, 127, 0));
    }
    static const Eigen::Vector4d SpringGreen() {
      return toEigen(Color(0, 255, 127));
    }
    static const Eigen::Vector4d Blue() { return toEigen(Color(0, 0, 255)); }
    static const Eigen::Vector4d DeepBlue() {
      return toEigen(Color(0, 0, 127));
    }
    static const Eigen::Vector4d Yellow() {
      return toEigen(Color(255, 255, 0));
    }
    static const Eigen::Vector4d Orange() {
      return toEigen(Color(255, 127, 0));
    }
    static const Eigen::Vector4d Purple() {
      return toEigen(Color(127, 0, 255));
    }
    static const Eigen::Vector4d Teal() { return toEigen(Color(0, 255, 255)); }
    static const Eigen::Vector4d TealTransparent() {
      return toEigen(Color(0, 255, 255, 200));
    }
    static const Eigen::Vector4d Pink() { return toEigen(Color(255, 0, 127)); }
    static const Eigen::Vector4d Magenta() {
      return toEigen(Color(255, 0, 255));
    }
  };

  // new interface
  void fillBasicInfo(visualization_msgs::Marker &mk,
                     const Eigen::Vector3d &scale, const Eigen::Vector4d &color,
                     const string &ns, const int &id, const int &shape);
  void fillGeometryInfo(visualization_msgs::Marker &mk,
                        const vector<Eigen::Vector3d> &list);
  void fillGeometryInfo(visualization_msgs::Marker &mk,
                        const vector<Eigen::Vector3d> &list1,
                        const vector<Eigen::Vector3d> &list2);

  void drawSpheres(const vector<Eigen::Vector3d> &list, const double &scale,
                   const Eigen::Vector4d &color, const string &ns,
                   const int &id, const int &pub_id);
  void drawCubes(const vector<Eigen::Vector3d> &list, const double &scale,
                 const Eigen::Vector4d &color, const string &ns, const int &id,
                 const int &pub_id);
  void drawLines(const vector<Eigen::Vector3d> &list1,
                 const vector<Eigen::Vector3d> &list2, const double &scale,
                 const Eigen::Vector4d &color, const string &ns, const int &id,
                 const int &pub_id);
  void drawLines(const vector<Eigen::Vector3d> &list, const double &scale,
                 const Eigen::Vector4d &color, const string &ns, const int &id,
                 const int &pub_id);
  void drawBox(const Eigen::Vector3d &center, const Eigen::Vector3d &scale,
               const Eigen::Vector4d &color, const string &ns, const int &id,
               const int &pub_id);
  void drawRGBpt(const Eigen::Vector3d &xyz, const Eigen::Vector4d &color,
                 const int &pub_id);
  void drawText(const Eigen::Vector3d &pos, const string &text,
                const double &scale, const Eigen::Vector4d &color,
                const string &ns, const int &id, const int &pub_id);
  void drawArrow(const vector<Eigen::Vector3d> &list,
                 const Eigen::Vector3d &scale, const Eigen::Vector4d &color,
                 const string &ns, const int &id, const int &pub_id);
  void drawArrow(const vector<Eigen::Vector3d> &list_start,
                 const vector<Eigen::Vector3d> &list_end, const int action,
                 const Eigen::Vector3d &scale, const Eigen::Vector4d &color,
                 const string &ns, const int &id, const int &pub_id);
  void removeText(const string &ns, const int &id, const int &pub_id);

  // Deprecated
  // draw basic shapes
  void displaySphereList(const vector<Eigen::Vector3d> &list, double resolution,
                         const Eigen::Vector4d &color, int id, int pub_id = 0);
  void displayCubeList(const vector<Eigen::Vector3d> &list, double resolution,
                       const Eigen::Vector4d &color, int id, int pub_id = 0);
  void displayLineList(const vector<Eigen::Vector3d> &list1,
                       const vector<Eigen::Vector3d> &list2, double line_width,
                       const Eigen::Vector4d &color, int id, int pub_id = 0);
  // draw a piece-wise straight line path
  void drawGeometricPath(const vector<Eigen::Vector3d> &path, double resolution,
                         const Eigen::Vector4d &color, int id = 0);
  // draw a polynomial trajectory
  void drawPolynomialTraj(PolynomialTraj poly_traj, double resolution,
                          const Eigen::Vector4d &color, int id = 0);
  // draw a bspline trajectory
  void drawBspline(NonUniformBspline &bspline, double size,
                   const Eigen::Vector4d &color, bool show_ctrl_pts = false,
                   double size2 = 0.1,
                   const Eigen::Vector4d &color2 = Eigen::Vector4d(1, 1, 0, 1),
                   int id = 0);
  // draw a set of bspline trajectories generated in different phases
  void drawBsplinesPhase1(vector<NonUniformBspline> &bsplines, double size);
  void drawBsplinesPhase2(vector<NonUniformBspline> &bsplines, double size);
  // draw topological graph and paths
  void drawTopoGraph(list<GraphNode::Ptr> &graph, double point_size,
                     double line_width, const Eigen::Vector4d &color1,
                     const Eigen::Vector4d &color2,
                     const Eigen::Vector4d &color3, int id = 0);
  void drawTopoPathsPhase1(vector<vector<Eigen::Vector3d>> &paths,
                           double line_width);
  void drawTopoPathsPhase2(vector<vector<Eigen::Vector3d>> &paths,
                           double line_width);

  void drawGoal(Eigen::Vector3d goal, double resolution,
                const Eigen::Vector4d &color, int id = 0);
  void drawPrediction(ObjPrediction pred, double resolution,
                      const Eigen::Vector4d &color, int id = 0);

  Eigen::Vector4d getColor(const double &h, double alpha = 1.0);

  typedef std::shared_ptr<PlanningVisualization> Ptr;

  // SECTION developing
  void drawVisibConstraint(const Eigen::MatrixXd &ctrl_pts,
                           const vector<Eigen::Vector3d> &block_pts);
  void drawVisibConstraint(const Eigen::MatrixXd &pts,
                           const vector<VisiblePair> &pairs);
  void drawViewConstraint(const ViewConstraint &vc);
  void drawFrontier(const vector<vector<Eigen::Vector3d>> &frontiers);
  void drawYawTraj(NonUniformBspline &pos, NonUniformBspline &yaw,
                   const double &dt);
  void drawYawPath(NonUniformBspline &pos, const vector<double> &yaw,
                   const double &dt);
  void drawFOV(const vector<Eigen::Vector3d> &list1,
          const vector<Eigen::Vector3d> &list2, const string &ns, const int &id,
          const int &pub_id);
};
} // namespace fast_planner
#endif