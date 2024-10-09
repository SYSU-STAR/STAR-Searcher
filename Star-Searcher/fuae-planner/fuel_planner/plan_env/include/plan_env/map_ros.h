#ifndef _MAP_ROS_H
#define _MAP_ROS_H

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <memory>
#include <random>
#include <tf/transform_listener.h>

using std::default_random_engine;
using std::normal_distribution;
using std::shared_ptr;

namespace fast_planner {
class SDFMap;

class MapROS {
public:
  MapROS();
  ~MapROS();
  void setMap(SDFMap *map);
  void init();

private:
  void depthPoseCallback(const sensor_msgs::ImageConstPtr &img,
                         const geometry_msgs::PoseStampedConstPtr &pose);
  void cloudPoseCallback(const sensor_msgs::PointCloud2ConstPtr &msg,
                         const geometry_msgs::PoseStampedConstPtr &pose);
  void cameralidarCallback(const sensor_msgs::PointCloud2ConstPtr &cloud,
                           const sensor_msgs::ImageConstPtr &image_rect,
                           const nav_msgs::OdometryConstPtr &pose);
  void updateESDFCallback(const ros::TimerEvent & /*event*/);
  void visCallback(const ros::TimerEvent & /*event*/);
  void resultCallback(const ros::TimerEvent &e);
  void publishMapAll();
  void publishMapLocal();
  void publishESDF();
  void publishUpdateRange();
  void publishUnknown();
  void publishDepth();
  void publishMinObservedDist();
  void publishUnderObserved();

  void proessDepthImage();
  void processFusionCloud();

  SDFMap *map_;
  // may use ExactTime?
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, geometry_msgs::PoseStamped>
      SyncPolicyImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>>
      SynchronizerImagePose;
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2, geometry_msgs::PoseStamped>
      SyncPolicyCloudPose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyCloudPose>>
      SynchronizerCloudPose;
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2, sensor_msgs::Image, nav_msgs::Odometry>
      SyncPolicyLidarCamera;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyLidarCamera>>
      SynchronizerLidarCamera;

  ros::NodeHandle node_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
  shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> lidar_sub_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> image_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
  SynchronizerImagePose sync_image_pose_;
  SynchronizerCloudPose sync_cloud_pose_;
  SynchronizerLidarCamera sync_camera_lidar_;

  ros::Publisher map_local_pub_, map_local_inflate_pub_, esdf_pub_,
      map_all_pub_, unknown_pub_, update_range_pub_, depth_pub_,
      map_object_pub_, under_observed_pub_, debug_pub_, test_pub_;
  ros::Timer esdf_timer_, vis_timer_, result_timer_;

  // Camera, Lidar Fusion
  std::string ptcloud_topic;
  std::string image_topic;
  std::string lidar_tf_target_frame;
  std::string lidar_tf_source_frame;
  Eigen::Quaterniond lidar2cam_rotation;
  Eigen::Vector3d lidar2cam_trans;
  Eigen::Quaterniond lidar2world_rotation;
  Eigen::Vector3d lidar2world_translation;
  double observe_update_dist;
  double cl_fx_, cl_fy_, cl_cx_, cl_cy_;
  cv_bridge::CvImagePtr cv_image_;
  tf::StampedTransform transform_;
  std::unique_ptr<pcl::PointCloud<pcl::PointXYZ>> temp_cloud;
  Eigen::Matrix<double, 3, Eigen::Dynamic> world_pts_in_lidar, pixel_uv,
      lidar_pts;

  // params, depth projection
  double cx_, cy_, fx_, fy_;
  Eigen::Matrix3d K;
  double depth_filter_maxdist_, depth_filter_mindist_;
  int depth_filter_margin_;
  double k_depth_scaling_factor_;
  int skip_pixel_;
  std::string frame_id_;

  // msg publication
  double esdf_slice_height_;
  double visualization_truncate_height_, visualization_truncate_low_;
  bool show_esdf_time_, show_occ_time_;
  bool show_all_map_;

  // data
  // flags of map state
  bool local_updated_, esdf_need_update_;
  // input
  Eigen::Vector3d camera_pos_;
  Eigen::Quaterniond camera_q_;
  std::unique_ptr<cv::Mat> depth_image_;
  std::vector<Eigen::Vector3d> proj_points_;
  int proj_points_cnt;
  double fuse_time_, esdf_time_, max_fuse_time_, max_esdf_time_;
  int fuse_num_, esdf_num_;
  pcl::PointCloud<pcl::PointXYZ> point_cloud_;

  normal_distribution<double> rand_noise_;
  default_random_engine eng_;

  ros::Time map_start_time_;

  friend SDFMap;
};
} // namespace fast_planner

#endif