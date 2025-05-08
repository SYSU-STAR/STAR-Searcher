#include <plan_env/map_ros.h>
#include <plan_env/sdf_map.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

#include <fstream>

namespace fast_planner {
MapROS::MapROS() {}

MapROS::~MapROS() {}

void MapROS::setMap(SDFMap *map) { this->map_ = map; }

void MapROS::init() {
  node_.param("map_ros/fx", fx_, -1.0);
  node_.param("map_ros/fy", fy_, -1.0);
  node_.param("map_ros/cx", cx_, -1.0);
  node_.param("map_ros/cy", cy_, -1.0);
  node_.param("map_ros/depth_filter_maxdist", depth_filter_maxdist_, -1.0);
  node_.param("map_ros/depth_filter_mindist", depth_filter_mindist_, -1.0);
  node_.param("map_ros/depth_filter_margin", depth_filter_margin_, -1);
  node_.param("map_ros/k_depth_scaling_factor", k_depth_scaling_factor_, -1.0);
  node_.param("map_ros/skip_pixel", skip_pixel_, -1);

  node_.param("map_ros/esdf_slice_height", esdf_slice_height_, -0.1);
  node_.param("map_ros/visualization_truncate_height",
              visualization_truncate_height_, -0.1);
  node_.param("map_ros/visualization_truncate_low", visualization_truncate_low_,
              -0.1);
  node_.param("map_ros/show_occ_time", show_occ_time_, false);
  node_.param("map_ros/show_esdf_time", show_esdf_time_, false);
  node_.param("map_ros/show_all_map", show_all_map_, false);
  node_.param("map_ros/frame_id", frame_id_, string("world"));

  node_.getParam("map_ros/camera_lidar_fusion/ptcloud_topic", ptcloud_topic);
  node_.getParam("map_ros/camera_lidar_fusion/image_topic", image_topic);
  node_.getParam("map_ros/camera_lidar_fusion/lidar_tf_target_frame",
                 lidar_tf_target_frame);
  node_.getParam("map_ros/camera_lidar_fusion/lidar_tf_source_frame",
                 lidar_tf_source_frame);
  node_.getParam("map_ros/camera_lidar_fusion/observe_update_dist",
                 observe_update_dist);
  node_.getParam("map_ros/camera_lidar_fusion/fx", cl_fx_);
  node_.getParam("map_ros/camera_lidar_fusion/fy", cl_fy_);
  node_.getParam("map_ros/camera_lidar_fusion/cx", cl_cx_);
  node_.getParam("map_ros/camera_lidar_fusion/cy", cl_cy_);
  lidar2cam_rotation = Eigen::Quaterniond(0.500, 0.500, -0.500, 0.500);
  lidar2cam_trans << 0.033, -0.010, -0.100;
  K << cl_fx_, 0.0, cl_cx_, 0.0, cl_fy_, cl_cy_, 0.0, 0.0, 1.0;

  // * LiDAR Air Projection
  node_.param("map_ros/h_fov", h_fov_, 360.0);
  node_.param("map_ros/v_fov_min", v_fov_min_, 0.0);
  node_.param("map_ros/v_fov_max", v_fov_max_, 0.0);
  node_.param("map_ros/inf_fov", inf_fov_, 0.0);
  node_.param("map_ros/h_res", h_res_, 0.2);
  node_.param("map_ros/v_res", v_res_, 0.2);
  node_.param("sdf_map/max_ray_length", mapping_visible_dist_, 0.0);
  node_.param("map_ros/dil_pixels", dil_pixels_, 0);

  proj_points_.resize(640 * 480 / (skip_pixel_ * skip_pixel_));
  point_cloud_.points.resize(640 * 480 / (skip_pixel_ * skip_pixel_));
  // proj_points_.reserve(640 * 480 / map_->mp_->skip_pixel_ /
  // map_->mp_->skip_pixel_);
  proj_points_cnt = 0;

  local_updated_ = false;
  esdf_need_update_ = false;
  fuse_time_ = 0.0;
  esdf_time_ = 0.0;
  max_fuse_time_ = 0.0;
  max_esdf_time_ = 0.0;
  fuse_num_ = 0;
  esdf_num_ = 0;
  depth_image_.reset(new cv::Mat);
  temp_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

  rand_noise_ = normal_distribution<double>(0, 0.1);
  random_device rd;
  eng_ = default_random_engine(rd());

  esdf_timer_ =
      node_.createTimer(ros::Duration(0.05), &MapROS::updateESDFCallback, this);
  vis_timer_ =
      node_.createTimer(ros::Duration(0.05), &MapROS::visCallback, this);
  result_timer_ =
      node_.createTimer(ros::Duration(0.5), &MapROS::resultCallback, this);

  map_all_pub_ =
      node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_all", 10);
  map_local_pub_ =
      node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_local", 10);
  map_local_inflate_pub_ = node_.advertise<sensor_msgs::PointCloud2>(
      "/sdf_map/occupancy_local_inflate", 10);
  unknown_pub_ =
      node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/unknown", 10);
  esdf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/esdf", 10);
  update_range_pub_ =
      node_.advertise<visualization_msgs::Marker>("/sdf_map/update_range", 10);
  depth_pub_ =
      node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/depth_cloud", 10);
  map_object_pub_ = node_.advertise<sensor_msgs::PointCloud2>(
      "/cam_lidar/min_observed_dist", 10);
  under_observed_pub_ = node_.advertise<sensor_msgs::PointCloud2>(
      "/cam_lidar/under_observed", 10);
  debug_pub_ =
      node_.advertise<sensor_msgs::PointCloud2>("/cam_lidar/debug", 10);
  test_pub_ = node_.advertise<std_msgs::Float32>("/result/voxel_ratio", 10);

  depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(
      node_, "/map_ros/depth", 50));
  cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(
      node_, "/map_ros/cloud", 50));
  pose_sub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(
      node_, "/map_ros/pose", 25));
  lidar_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(
      node_, ptcloud_topic, 50));
  image_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(
      node_, image_topic, 50));
  odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(
      node_, "/ardrone/ground_truth/odometry", 25));

  sync_image_pose_.reset(
      new message_filters::Synchronizer<MapROS::SyncPolicyImagePose>(
          MapROS::SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
  sync_image_pose_->registerCallback(
      boost::bind(&MapROS::depthPoseCallback, this, _1, _2));
  sync_cloud_pose_.reset(
      new message_filters::Synchronizer<MapROS::SyncPolicyCloudPose>(
          MapROS::SyncPolicyCloudPose(100), *cloud_sub_, *pose_sub_));
  sync_cloud_pose_->registerCallback(
      boost::bind(&MapROS::cloudPoseCallback, this, _1, _2));
  sync_camera_lidar_.reset(
      new message_filters::Synchronizer<MapROS::SyncPolicyLidarCamera>(
          MapROS::SyncPolicyLidarCamera(100), *lidar_sub_, *image_sub_,
          *odom_sub_));
  sync_camera_lidar_->registerCallback(
      boost::bind(&MapROS::cameralidarCallback, this, _1, _2, _3));

  map_start_time_ = ros::Time::now();
}

void MapROS::visCallback(const ros::TimerEvent &e) {
  publishMapLocal();
  if (show_all_map_) {
    // Limit the frequency of all map
    static double tpass = 0.0;
    tpass += (e.current_real - e.last_real).toSec();
    if (tpass > 0.1) {
      publishMapAll();
      tpass = 0.0;
    }
  }
  publishMinObservedDist();
  // publishUnderObserved();
  // publishUnknown();
  // publishESDF();

  // publishUpdateRange();
  // publishDepth();
}

void MapROS::resultCallback(const ros::TimerEvent &e) {
  double discoverd_ = 0.0, total_ = 0.0;
  for (int x = map_->mp_->box_min_(0) /* + 1 */; x < map_->mp_->box_max_(0);
       ++x)
    for (int y = map_->mp_->box_min_(1) /* + 1 */; y < map_->mp_->box_max_(1);
         ++y)
      for (int z = map_->mp_->box_min_(2) /* + 1 */; z < map_->mp_->box_max_(2);
           ++z) {
        if (map_->getOccupancy(Eigen::Vector3i(x, y, z)) == SDFMap::FREE ||
            map_->getOccupancy(Eigen::Vector3i(x, y, z)) == SDFMap::OCCUPIED) {
          discoverd_ += 1.0;
        }
        total_ += 1.0;
      }
  std_msgs::Float32 msg;
  msg.data = discoverd_ / total_;
  test_pub_.publish(msg);
}

void MapROS::updateESDFCallback(const ros::TimerEvent & /*event*/) {
  if (!esdf_need_update_)
    return;
  auto t1 = ros::Time::now();

  map_->updateESDF3d();
  esdf_need_update_ = false;

  auto t2 = ros::Time::now();
  esdf_time_ += (t2 - t1).toSec();
  max_esdf_time_ = max(max_esdf_time_, (t2 - t1).toSec());
  esdf_num_++;
  if (show_esdf_time_)
    ROS_WARN("ESDF t: cur: %lf, avg: %lf, max: %lf", (t2 - t1).toSec(),
             esdf_time_ / esdf_num_, max_esdf_time_);
}

void MapROS::depthPoseCallback(const sensor_msgs::ImageConstPtr &img,
                               const geometry_msgs::PoseStampedConstPtr &pose) {
  camera_pos_(0) = pose->pose.position.x;
  camera_pos_(1) = pose->pose.position.y;
  camera_pos_(2) = pose->pose.position.z;
  if (!map_->isInMap(camera_pos_)) // exceed mapped region
    return;

  camera_q_ =
      Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                         pose->pose.orientation.y, pose->pose.orientation.z);
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, k_depth_scaling_factor_);
  cv_ptr->image.copyTo(*depth_image_);

  auto t1 = ros::Time::now();

  // generate point cloud, update map
  proessDepthImage();
  map_->inputPointCloud(point_cloud_, proj_points_cnt, camera_pos_, false);
  if (local_updated_) {
    map_->clearAndInflateLocalMap();
    esdf_need_update_ = true;
    local_updated_ = false;
  }

  auto t2 = ros::Time::now();
  fuse_time_ += (t2 - t1).toSec();
  max_fuse_time_ = max(max_fuse_time_, (t2 - t1).toSec());
  fuse_num_ += 1;
  if (show_occ_time_)
    ROS_WARN("Fusion t: cur: %lf, avg: %lf, max: %lf", (t2 - t1).toSec(),
             fuse_time_ / fuse_num_, max_fuse_time_);
}

void MapROS::cameralidarCallback(const sensor_msgs::PointCloud2ConstPtr &cloud,
                                 const sensor_msgs::ImageConstPtr &image_rect,
                                 const nav_msgs::OdometryConstPtr &pose) {
  auto t1 = ros::Time::now();
  lidar2world_rotation = Eigen::Quaterniond(
      pose->pose.pose.orientation.w, pose->pose.pose.orientation.x,
      pose->pose.pose.orientation.y, pose->pose.pose.orientation.z);
  lidar2world_translation << pose->pose.pose.position.x,
      pose->pose.pose.position.y, pose->pose.pose.position.z;
  Eigen::Vector3d lidar_pos = lidar2world_translation;

  cv_image_ = cv_bridge::toCvCopy(image_rect, image_rect->encoding);

  pcl::fromROSMsg(*cloud, *temp_cloud);
  processFusionCloud();
  map_->inputCamLidarPointCloud(world_pts_in_lidar, pixel_uv, lidar_pts,
                                lidar_pos);

  // pcl::PointXYZ pt;
  // pcl::PointCloud<pcl::PointXYZ> cloud1;
  // for (int i = 0; i < world_pts_in_lidar.cols(); i++) {
  //   // if (!(pixel_uv(0, i) < cl_cx_ * 2 && pixel_uv(1, i) < cl_cy_ * 2 &&
  //   //       pixel_uv(0, i) > 0 && pixel_uv(1, i) > 0 &&
  //   //       lidar_pts(0, i) > 0.0)) {
  //   //   continue;
  //   // }
  //   pt.x = world_pts_in_lidar(0, i);
  //   pt.y = world_pts_in_lidar(1, i);
  //   pt.z = world_pts_in_lidar(2, i);
  //   cloud1.push_back(pt);
  // }
  // cloud1.width = cloud1.points.size();
  // cloud1.height = 1;
  // cloud1.is_dense = true;
  // cloud1.header.frame_id = "world";
  // sensor_msgs::PointCloud2 cloud_msg;
  // pcl::toROSMsg(cloud1, cloud_msg);
  // under_observed_pub_.publish(cloud_msg);

  if (local_updated_) {
    map_->clearAndInflateLocalMap();
    esdf_need_update_ = true;
    local_updated_ = false;
  }
  auto t2 = ros::Time::now();
  float fusion_time = (t2 - t1).toSec();
  // ROS_INFO("Cam_Lidar_Fusion Time: %lf", fusion_time);
}

void MapROS::processFusionCloud() {
  Eigen::Matrix<double, 3, Eigen::Dynamic> cloud_tmp;
  for (int i = 0; i < temp_cloud->points.size(); i++) {
    if (isnan(temp_cloud->points[i].x) || isnan(temp_cloud->points[i].y) ||
        isnan(temp_cloud->points[i].z)) {
      continue;
    }

    Eigen::MatrixXd newColumn(3, 1);
    newColumn << temp_cloud->points[i].x, temp_cloud->points[i].y,
        temp_cloud->points[i].z;
    cloud_tmp.conservativeResize(Eigen::NoChange, cloud_tmp.cols() + 1);
    cloud_tmp.rightCols<1>() = newColumn;
  }

  Eigen::Matrix3d lidar2world_R = lidar2world_rotation.toRotationMatrix();

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.rotate(lidar2world_rotation);
  T.pretranslate(lidar2world_translation);

  lidar_pts = cloud_tmp;
  world_pts_in_lidar = T * lidar_pts;
  // map_->inputCamLidarPointCloud(object_map, lidar_pos);

  Eigen::Matrix3d rotation_matrix = lidar2cam_rotation.toRotationMatrix();
  T.setIdentity();
  T.rotate(rotation_matrix);
  T.pretranslate(lidar2cam_trans);
  Eigen::MatrixXd cam_permute = T * lidar_pts;

  for (int i = 0; i < cam_permute.cols(); i++) {
    cam_permute.col(i) =
        cam_permute.col(i) / cam_permute(cam_permute.rows() - 1, i);
  }
  pixel_uv = K * cam_permute;
}

void MapROS::cloudPoseCallback(const sensor_msgs::PointCloud2ConstPtr &msg,
                               const geometry_msgs::PoseStampedConstPtr &pose) {
  camera_pos_(0) = pose->pose.position.x;
  camera_pos_(1) = pose->pose.position.y;
  camera_pos_(2) = pose->pose.position.z;
  camera_q_ =
      Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                         pose->pose.orientation.y, pose->pose.orientation.z);

  tf::Quaternion tfQuat_body;
  tf::quaternionMsgToTF(pose->pose.orientation, tfQuat_body);
  tf::Matrix3x3 m_body(tfQuat_body);
  double roll_body, pitch_body, yaw_body;
  m_body.getRPY(roll_body, pitch_body, yaw_body);

  Eigen::Matrix3d body2world_R_matrix;

  body2world_R_matrix = Eigen::AngleAxisd(yaw_body, Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(pitch_body, Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(roll_body, Eigen::Vector3d::UnitX());
  Eigen::Vector3d body2world_t_vector = camera_pos_;

  Eigen::Matrix3d sensor2world_R_matrix;
  Eigen::Vector3d sensor2world_t_vector;

  sensor2world_R_matrix = body2world_R_matrix;
  sensor2world_t_vector = body2world_t_vector;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);
  int num = cloud.points.size();

  pcl::PointCloud<pcl::PointXYZ> air_cloud = generateAirPointCloud(cloud, sensor2world_R_matrix, sensor2world_t_vector);
  int air_num = air_cloud.points.size();

  map_->inputPointCloud(air_cloud, air_num, camera_pos_, true);
  map_->inputPointCloud(cloud, num, camera_pos_, false);

  if (local_updated_) {
    map_->clearAndInflateLocalMap();
    esdf_need_update_ = true;
    local_updated_ = false;
  }
}

pcl::PointCloud<pcl::PointXYZ> MapROS::generateAirPointCloud(
  const pcl::PointCloud<pcl::PointXYZ>& input_cloud, 
  const Eigen::Matrix3d& R_sensor2world, 
  const Eigen::Vector3d& t_sensor2world)
{
  pcl::PointCloud<pcl::PointXYZ> air_inf_cloud;

  const double horizontal_fov = this->h_fov_;
  const double vertical_fov_min = this->v_fov_min_ + this->inf_fov_;
  const double vertical_fov_max = this->v_fov_max_ - this->inf_fov_;
  const double horizontal_resolution = this->h_res_;
  const double vertical_resolution = this->v_res_;
  const double max_depth = this->mapping_visible_dist_ + 0.5;
  const double min_depth = 0.5;

  int cols = static_cast<int>(horizontal_fov / horizontal_resolution);
  int rows = static_cast<int>((vertical_fov_max - vertical_fov_min) / vertical_resolution);

  Eigen::MatrixXd depth_map = Eigen::MatrixXd::Constant(rows, cols, max_depth);
  double horizontal_fov_min = -horizontal_fov / 2.0;

  for (const auto& pt : input_cloud.points) 
  {
    Eigen::Vector3d pt_world(pt.x, pt.y, pt.z);
    Eigen::Vector3d pt_sensor = R_sensor2world.transpose() * (pt_world - t_sensor2world);

    
    double horizontal_angle = std::atan2(pt_sensor.y(), pt_sensor.x()) * 180.0 / M_PI;
    double vertical_angle = std::atan2(pt_sensor.z(), std::sqrt(pt_sensor.x() * pt_sensor.x() + pt_sensor.y() * pt_sensor.y())) * 180.0 / M_PI;

    if (horizontal_angle < horizontal_fov_min || horizontal_angle > -horizontal_fov_min ||
        vertical_angle < vertical_fov_min || vertical_angle > vertical_fov_max) continue;

    int u = static_cast<int>((horizontal_angle - horizontal_fov_min) / horizontal_resolution);
    int v = static_cast<int>((vertical_angle - vertical_fov_min) / vertical_resolution);

    if (u >= 0 && u < cols && v >= 0 && v < rows) 
    {
      double depth = pt_sensor.norm();
      depth_map(v, u) = std::min(depth_map(v, u), depth);
    }
  }

  int dilation_radius = this->dil_pixels_;
  Eigen::MatrixXd depth_map_dilated = depth_map;

  for (int v = 0; v < rows; v++) 
  {
    for (int u = 0; u < cols; u++) 
    {
      if (depth_map(v, u) < max_depth && depth_map(v, u) >= min_depth) 
      {
        for (int dv = -dilation_radius; dv <= dilation_radius; dv++) 
        {
          for (int du = -dilation_radius; du <= dilation_radius; du++) 
          {
            int u_neighbor = u + du;
            int v_neighbor = v + dv;

            if (u_neighbor >= 0 && u_neighbor < cols && v_neighbor >= 0 && v_neighbor < rows) 
            {
              if (depth_map(v_neighbor, u_neighbor) < max_depth && depth_map(v_neighbor, u_neighbor) >= min_depth) continue;
              depth_map_dilated(v_neighbor, u_neighbor) = 0.9*min_depth;
            }
          }
        }
      }
    }
  }

  for (int v = 0; v < rows; v++) 
  {
    for (int u = 0; u < cols; u++) 
    {
      double depth = depth_map_dilated(v, u);

      if (depth < max_depth) continue;

      if (depth >= max_depth) depth = max_depth;

      double horizontal_angle = horizontal_fov_min + u * horizontal_resolution;
      double vertical_angle = vertical_fov_min + v * vertical_resolution;

      Eigen::Vector3d direction;
      direction.x() = std::cos(horizontal_angle * M_PI / 180.0) * std::cos(vertical_angle * M_PI / 180.0);
      direction.y() = std::sin(horizontal_angle * M_PI / 180.0) * std::cos(vertical_angle * M_PI / 180.0);
      direction.z() = std::sin(vertical_angle * M_PI / 180.0);

      Eigen::Vector3d pt_sensor = direction * depth;

      Eigen::Vector3d pt_world = R_sensor2world * pt_sensor + t_sensor2world;

      pcl::PointXYZ pt;
      pt.x = pt_world.x();
      pt.y = pt_world.y();
      pt.z = pt_world.z();
      air_inf_cloud.points.push_back(pt);
    }
  }

  air_inf_cloud.width = air_inf_cloud.points.size();
  air_inf_cloud.height = 1;
  air_inf_cloud.is_dense = true;

  return air_inf_cloud;
}

void MapROS::proessDepthImage() {
  proj_points_cnt = 0;

  uint16_t *row_ptr;
  int cols = depth_image_->cols;
  int rows = depth_image_->rows;
  double depth;
  Eigen::Matrix3d camera_r = camera_q_.toRotationMatrix();
  Eigen::Vector3d pt_cur, pt_world;
  const double inv_factor = 1.0 / k_depth_scaling_factor_;

  for (int v = depth_filter_margin_; v < rows - depth_filter_margin_;
       v += skip_pixel_) {
    row_ptr = depth_image_->ptr<uint16_t>(v) + depth_filter_margin_;
    for (int u = depth_filter_margin_; u < cols - depth_filter_margin_;
         u += skip_pixel_) {
      depth = (*row_ptr) * inv_factor;
      row_ptr = row_ptr + skip_pixel_;

      // // filter depth
      // if (depth > 0.01)
      //   depth += rand_noise_(eng_);

      // TODO: simplify the logic here
      if (*row_ptr == 0 || depth > depth_filter_maxdist_)
        depth = depth_filter_maxdist_;
      else if (depth < depth_filter_mindist_)
        continue;

      pt_cur(0) = (u - cx_) * depth / fx_;
      pt_cur(1) = (v - cy_) * depth / fy_;
      pt_cur(2) = depth;
      pt_world = camera_r * pt_cur + camera_pos_;
      auto &pt = point_cloud_.points[proj_points_cnt++];
      pt.x = pt_world[0];
      pt.y = pt_world[1];
      pt.z = pt_world[2];
    }
  }

  publishDepth();
}

void MapROS::publishMapAll() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
  for (int x = map_->mp_->box_min_(0) /* + 1 */; x < map_->mp_->box_max_(0);
       ++x)
    for (int y = map_->mp_->box_min_(1) /* + 1 */; y < map_->mp_->box_max_(1);
         ++y)
      for (int z = map_->mp_->box_min_(2) /* + 1 */; z < map_->mp_->box_max_(2);
           ++z) {
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] >
            map_->mp_->min_occupancy_log_) {
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > visualization_truncate_height_)
            continue;
          if (pos(2) < visualization_truncate_low_)
            continue;
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud1.push_back(pt);
        }
      }
  cloud1.width = cloud1.points.size();
  cloud1.height = 1;
  cloud1.is_dense = true;
  cloud1.header.frame_id = frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud1, cloud_msg);
  map_all_pub_.publish(cloud_msg);

  // Output time and known volumn
  double time_now = (ros::Time::now() - map_start_time_).toSec();
  double known_volumn = 0;

  for (int x = map_->mp_->box_min_(0) /* + 1 */; x < map_->mp_->box_max_(0);
       ++x)
    for (int y = map_->mp_->box_min_(1) /* + 1 */; y < map_->mp_->box_max_(1);
         ++y)
      for (int z = map_->mp_->box_min_(2) /* + 1 */; z < map_->mp_->box_max_(2);
           ++z) {
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] >
            map_->mp_->clamp_min_log_ - 1e-3)
          known_volumn += 0.1 * 0.1 * 0.1;
      }

  // ofstream
  // file("/home/boboyu/workspaces/plan_ws/src/fast_planner/exploration_manager/resource/"
  //               "curve1.txt",
  //               ios::app);
  // file << "time:" << time_now << ",vol:" << known_volumn << std::endl;
}

void MapROS::publishMapLocal() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ> cloud2;
  Eigen::Vector3i min_cut = map_->md_->local_bound_min_;
  Eigen::Vector3i max_cut = map_->md_->local_bound_max_;
  map_->boundIndex(min_cut);
  map_->boundIndex(max_cut);

  // for (int z = min_cut(2); z <= max_cut(2); ++z)
  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = map_->mp_->box_min_(2); z < map_->mp_->box_max_(2); ++z) {
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] >
            map_->mp_->min_occupancy_log_) {
          // Occupied cells
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > visualization_truncate_height_)
            continue;
          if (pos(2) < visualization_truncate_low_)
            continue;

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
        // else if (map_->md_->occupancy_buffer_inflate_[map_->toAddress(x, y,
        // z)] == 1)
        // {
        //   // Inflated occupied cells
        //   Eigen::Vector3d pos;
        //   map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
        //   if (pos(2) > visualization_truncate_height_)
        //     continue;
        //   if (pos(2) < visualization_truncate_low_)
        //     continue;

        //   pt.x = pos(0);
        //   pt.y = pos(1);
        //   pt.z = pos(2);
        //   cloud2.push_back(pt);
        // }
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  cloud2.width = cloud2.points.size();
  cloud2.height = 1;
  cloud2.is_dense = true;
  cloud2.header.frame_id = frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_local_pub_.publish(cloud_msg);
  pcl::toROSMsg(cloud2, cloud_msg);
  map_local_inflate_pub_.publish(cloud_msg);
}

void MapROS::publishUnknown() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  Eigen::Vector3i min_cut = map_->md_->local_bound_min_;
  Eigen::Vector3i max_cut = map_->md_->local_bound_max_;
  map_->boundIndex(max_cut);
  map_->boundIndex(min_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] <
            map_->mp_->clamp_min_log_ - 1e-3) {
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > visualization_truncate_height_)
            continue;
          if (pos(2) < visualization_truncate_low_)
            continue;
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  unknown_pub_.publish(cloud_msg);
}

void MapROS::publishDepth() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int i = 0; i < proj_points_cnt; ++i) {
    cloud.push_back(point_cloud_.points[i]);
  }
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  depth_pub_.publish(cloud_msg);
}

void MapROS::publishUpdateRange() {
  Eigen::Vector3d esdf_min_pos, esdf_max_pos, cube_pos, cube_scale;
  visualization_msgs::Marker mk;
  map_->indexToPos(map_->md_->local_bound_min_, esdf_min_pos);
  map_->indexToPos(map_->md_->local_bound_max_, esdf_max_pos);

  cube_pos = 0.5 * (esdf_min_pos + esdf_max_pos);
  cube_scale = esdf_max_pos - esdf_min_pos;
  mk.header.frame_id = frame_id_;
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::CUBE;
  mk.action = visualization_msgs::Marker::ADD;
  mk.id = 0;
  mk.pose.position.x = cube_pos(0);
  mk.pose.position.y = cube_pos(1);
  mk.pose.position.z = cube_pos(2);
  mk.scale.x = cube_scale(0);
  mk.scale.y = cube_scale(1);
  mk.scale.z = cube_scale(2);
  mk.color.a = 0.3;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;

  update_range_pub_.publish(mk);
}

void MapROS::publishESDF() {
  double dist;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;

  const double min_dist = 0.0;
  const double max_dist = 3.0;

  Eigen::Vector3i min_cut = map_->md_->local_bound_min_ -
                            Eigen::Vector3i(map_->mp_->local_map_margin_,
                                            map_->mp_->local_map_margin_,
                                            map_->mp_->local_map_margin_);
  Eigen::Vector3i max_cut = map_->md_->local_bound_max_ +
                            Eigen::Vector3i(map_->mp_->local_map_margin_,
                                            map_->mp_->local_map_margin_,
                                            map_->mp_->local_map_margin_);
  map_->boundIndex(min_cut);
  map_->boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y) {
      Eigen::Vector3d pos;
      map_->indexToPos(Eigen::Vector3i(x, y, 1), pos);
      pos(2) = esdf_slice_height_;
      dist = map_->getDistance(pos);
      dist = min(dist, max_dist);
      dist = max(dist, min_dist);
      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = -0.2;
      pt.intensity = (dist - min_dist) / (max_dist - min_dist);
      cloud.push_back(pt);
    }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  esdf_pub_.publish(cloud_msg);

  // ROS_INFO("pub esdf");
}

void MapROS::publishMinObservedDist() {
  pcl::PointXYZRGB pt;
  pcl::PointCloud<pcl::PointXYZRGB> cloud1;
  for (int x = map_->mp_->box_min_(0) /* + 1 */; x < map_->mp_->box_max_(0);
       ++x)
    for (int y = map_->mp_->box_min_(1) /* + 1 */; y < map_->mp_->box_max_(1);
         ++y)
      for (int z = map_->mp_->box_min_(2) /* + 1 */; z < map_->mp_->box_max_(2);
           ++z) {
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] >
            map_->mp_->min_occupancy_log_) {
          Eigen::Vector3d pos;
          int color = 
          floor((map_->md_->min_observed_dist_[map_->toAddress(x, y, z)] /
                 observe_update_dist) *
                510);
          // map_->md_->min_observed_dist_[map_->toAddress(x, y, z)] <
          //                     map_->getBeliefDist()
          //                 ? 255
          //                 : 0;
          
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          pt.r = color < 255 ? color : 255;
          pt.g = color > 255 ? 510 - color : 255;
          pt.b = 0;
          // pt.r = 255 - color;
          // pt.g = color;
          // pt.b = 0;
          cloud1.push_back(pt);
        }
      }
  cloud1.width = cloud1.points.size();
  cloud1.height = 1;
  cloud1.is_dense = true;
  cloud1.header.frame_id = "world";
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud1, cloud_msg);
  map_object_pub_.publish(cloud_msg);
};
void MapROS::publishUnderObserved() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int x = map_->mp_->box_min_(0) /* + 1 */; x < map_->mp_->box_max_(0);
       ++x)
    for (int y = map_->mp_->box_min_(1) /* + 1 */; y < map_->mp_->box_max_(1);
         ++y)
      for (int z = map_->mp_->box_min_(2) /* + 1 */; z < map_->mp_->box_max_(2);
           ++z) {
        Eigen::Vector3i id(x, y, z);

        if (map_->getOccupancy(id) == map_->UNDEROBSERVED) {
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > visualization_truncate_height_)
            continue;
          if (pos(2) < visualization_truncate_low_)
            continue;
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  // under_observed_pub_.publish(cloud_msg);
};
} // namespace fast_planner