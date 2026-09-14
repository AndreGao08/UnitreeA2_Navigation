#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace
{
using PointT = pcl::PointXYZI;
using Cloud = pcl::PointCloud<PointT>;

Eigen::Matrix4d poseToMatrix(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond quaternion(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  if (quaternion.norm() < 1.0e-9) {
    quaternion = Eigen::Quaterniond::Identity();
  } else {
    quaternion.normalize();
  }

  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
  transform.block<3, 1>(0, 3) = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);
  return transform;
}

geometry_msgs::msg::Pose matrixToPose(const Eigen::Matrix4d & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform(0, 3);
  pose.position.y = transform(1, 3);
  pose.position.z = transform(2, 3);
  Eigen::Quaterniond quaternion(transform.block<3, 3>(0, 0));
  quaternion.normalize();
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
}

Eigen::Matrix4d xyzrpyToMatrix(const std::vector<double> & values)
{
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  if (values.size() != 6) {
    return transform;
  }
  const Eigen::AngleAxisd roll(values[3], Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch(values[4], Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw(values[5], Eigen::Vector3d::UnitZ());
  transform.block<3, 3>(0, 0) = (yaw * pitch * roll).toRotationMatrix();
  transform.block<3, 1>(0, 3) = Eigen::Vector3d(values[0], values[1], values[2]);
  return transform;
}

double rotationAngle(const Eigen::Matrix3d & rotation)
{
  const double cosine = std::clamp((rotation.trace() - 1.0) * 0.5, -1.0, 1.0);
  return std::acos(cosine);
}

Cloud::Ptr voxelized(const Cloud::ConstPtr & input, double leaf_size)
{
  Cloud::Ptr output(new Cloud());
  if (leaf_size <= 0.0) {
    *output = *input;
    return output;
  }
  pcl::VoxelGrid<PointT> filter;
  filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  filter.setInputCloud(input);
  filter.filter(*output);
  return output;
}
}  // namespace

class MapLocalizer : public rclcpp::Node
{
public:
  MapLocalizer()
  : Node("map_localizer"), tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    declareParameters();
    readParameters();

    const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(map_topic_, map_qos);
    aligned_scan_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/a2/relocalization/aligned_scan", rclcpp::SensorDataQoS());
    global_odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(global_odom_topic_, 20);
    map_to_odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      "/a2/relocalization/map_to_odom", 10);
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/a2/relocalization/status", map_qos);
    rmse_publisher_ = create_publisher<std_msgs::msg::Float64>(
      "/a2/relocalization/rmse", 10);
    overlap_publisher_ = create_publisher<std_msgs::msg::Float64>(
      "/a2/relocalization/overlap", 10);

    scan_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MapLocalizer::scanCallback, this, std::placeholders::_1));
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 30, std::bind(&MapLocalizer::odomCallback, this, std::placeholders::_1));
    initial_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initial_pose_topic_, 10,
      std::bind(&MapLocalizer::initialPoseCallback, this, std::placeholders::_1));

    relocalize_service_ = create_service<std_srvs::srv::Trigger>(
      "/a2/relocalize",
      std::bind(
        &MapLocalizer::relocalizeCallback, this, std::placeholders::_1,
        std::placeholders::_2));
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/a2/relocalization/reset",
      std::bind(
        &MapLocalizer::resetCallback, this, std::placeholders::_1,
        std::placeholders::_2));

    if (!loadMap()) {
      publishStatus("ERROR: failed to load map");
      return;
    }

    if (auto_initialize_) {
      initial_map_to_base_ = xyzrpyToMatrix(initial_pose_xyzrpy_);
      have_initial_pose_ = true;
      localization_requested_ = true;
      publishStatus("WAITING_FOR_SCAN_AND_ODOMETRY: automatic initial pose loaded");
    } else {
      publishStatus("WAITING_FOR_INITIAL_POSE: publish /initialpose in map frame");
    }

    const auto localization_period = std::chrono::duration<double>(1.0 / localization_frequency_);
    localization_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(localization_period),
      std::bind(&MapLocalizer::localizationTimer, this));
    const auto map_period = std::chrono::duration<double>(1.0 / map_publish_frequency_);
    map_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(map_period),
      std::bind(&MapLocalizer::publishMap, this));

    publishMap();
    RCLCPP_INFO(
      get_logger(), "Loaded map '%s': raw=%zu, fine=%zu, coarse=%zu points",
      map_path_.c_str(), map_raw_->size(), map_fine_->size(), map_coarse_->size());
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("map_path", "/tmp/a2_fastlio_map.pcd");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("scan_topic", "/cloud_registered");
    declare_parameter<std::string>("odometry_topic", "/a2/odometry");
    declare_parameter<std::string>("initial_pose_topic", "/initialpose");
    declare_parameter<std::string>("map_topic", "/a2/map");
    declare_parameter<std::string>("global_odometry_topic", "/a2/localization");
    declare_parameter<double>("localization_frequency", 0.5);
    declare_parameter<double>("map_publish_frequency", 0.2);
    declare_parameter<double>("map_voxel_size", 0.25);
    declare_parameter<double>("scan_voxel_size", 0.15);
    declare_parameter<double>("coarse_voxel_size", 0.50);
    declare_parameter<double>("submap_radius", 30.0);
    declare_parameter<double>("coarse_max_correspondence", 2.5);
    declare_parameter<double>("fine_max_correspondence", 0.75);
    declare_parameter<int>("coarse_iterations", 40);
    declare_parameter<int>("fine_iterations", 30);
    declare_parameter<int>("minimum_scan_points", 100);
    declare_parameter<int>("minimum_submap_points", 300);
    declare_parameter<double>("minimum_overlap", 0.35);
    declare_parameter<double>("maximum_rmse", 0.35);
    declare_parameter<double>("maximum_update_translation", 0.30);
    declare_parameter<double>("maximum_update_rotation_deg", 8.0);
    declare_parameter<double>("correction_alpha", 0.15);
    declare_parameter<bool>("auto_initialize", false);
    declare_parameter<std::vector<double>>(
      "initial_pose_xyzrpy", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  }

  void readParameters()
  {
    map_path_ = get_parameter("map_path").as_string();
    map_frame_ = get_parameter("map_frame").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    scan_topic_ = get_parameter("scan_topic").as_string();
    odom_topic_ = get_parameter("odometry_topic").as_string();
    initial_pose_topic_ = get_parameter("initial_pose_topic").as_string();
    map_topic_ = get_parameter("map_topic").as_string();
    global_odom_topic_ = get_parameter("global_odometry_topic").as_string();
    localization_frequency_ = std::max(get_parameter("localization_frequency").as_double(), 0.05);
    map_publish_frequency_ = std::max(get_parameter("map_publish_frequency").as_double(), 0.02);
    map_voxel_size_ = get_parameter("map_voxel_size").as_double();
    scan_voxel_size_ = get_parameter("scan_voxel_size").as_double();
    coarse_voxel_size_ = get_parameter("coarse_voxel_size").as_double();
    submap_radius_ = get_parameter("submap_radius").as_double();
    coarse_max_correspondence_ = get_parameter("coarse_max_correspondence").as_double();
    fine_max_correspondence_ = get_parameter("fine_max_correspondence").as_double();
    coarse_iterations_ = get_parameter("coarse_iterations").as_int();
    fine_iterations_ = get_parameter("fine_iterations").as_int();
    minimum_scan_points_ = get_parameter("minimum_scan_points").as_int();
    minimum_submap_points_ = get_parameter("minimum_submap_points").as_int();
    minimum_overlap_ = get_parameter("minimum_overlap").as_double();
    maximum_rmse_ = get_parameter("maximum_rmse").as_double();
    maximum_update_translation_ = get_parameter("maximum_update_translation").as_double();
    maximum_update_rotation_ =
      get_parameter("maximum_update_rotation_deg").as_double() * M_PI / 180.0;
    correction_alpha_ = std::clamp(get_parameter("correction_alpha").as_double(), 0.0, 1.0);
    auto_initialize_ = get_parameter("auto_initialize").as_bool();
    initial_pose_xyzrpy_ = get_parameter("initial_pose_xyzrpy").as_double_array();
    if (initial_pose_xyzrpy_.size() != 6) {
      throw std::runtime_error("initial_pose_xyzrpy must contain x y z roll pitch yaw");
    }
  }

  bool loadMap()
  {
    map_raw_.reset(new Cloud());
    if (pcl::io::loadPCDFile<PointT>(map_path_, *map_raw_) < 0 || map_raw_->empty()) {
      RCLCPP_ERROR(get_logger(), "Cannot load a non-empty PCD map from '%s'", map_path_.c_str());
      return false;
    }
    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(*map_raw_, *map_raw_, valid_indices);
    map_fine_ = voxelized(map_raw_, map_voxel_size_);
    map_coarse_ = voxelized(map_raw_, coarse_voxel_size_);
    return !map_fine_->empty() && !map_coarse_->empty();
  }

  void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    Cloud::Ptr cloud(new Cloud());
    pcl::fromROSMsg(*message, *cloud);
    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, valid_indices);
    latest_scan_ = voxelized(cloud, scan_voxel_size_);
    latest_scan_stamp_ = message->header.stamp;
    ++scan_sequence_;
    if (message->header.frame_id != odom_frame_ && !warned_scan_frame_) {
      warned_scan_frame_ = true;
      RCLCPP_WARN(
        get_logger(), "Expected scan frame '%s', received '%s'; ICP assumes scan points are in odom",
        odom_frame_.c_str(), message->header.frame_id.c_str());
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    latest_odom_ = message;
    if (!localized_) {
      return;
    }

    const Eigen::Matrix4d map_to_base = map_to_odom_ * poseToMatrix(message->pose.pose);
    nav_msgs::msg::Odometry output = *message;
    output.header.frame_id = map_frame_;
    output.child_frame_id = base_frame_;
    output.pose.pose = matrixToPose(map_to_base);
    global_odom_publisher_->publish(output);
    publishTransformAndCorrection(message->header.stamp);
  }

  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
  {
    if (!message->header.frame_id.empty() && message->header.frame_id != map_frame_) {
      RCLCPP_ERROR(
        get_logger(), "Rejecting initial pose in frame '%s'; expected '%s'",
        message->header.frame_id.c_str(), map_frame_.c_str());
      publishStatus("REJECTED_INITIAL_POSE: wrong frame");
      return;
    }
    initial_map_to_base_ = poseToMatrix(message->pose.pose);
    have_initial_pose_ = true;
    localization_requested_ = true;
    localized_ = false;
    publishStatus("INITIALIZING: initial pose accepted, waiting for a fresh scan");
  }

  void localizationTimer()
  {
    if (!map_fine_ || !latest_scan_ || !latest_odom_) {
      return;
    }
    if (latest_scan_->size() < static_cast<std::size_t>(minimum_scan_points_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for enough scan points: %zu/%d",
        latest_scan_->size(), minimum_scan_points_);
      return;
    }
    if (!localization_requested_ && !localized_) {
      return;
    }
    if (scan_sequence_ == last_processed_scan_sequence_) {
      return;
    }
    last_processed_scan_sequence_ = scan_sequence_;

    Eigen::Matrix4d initial_guess = map_to_odom_;
    if (!localized_) {
      if (!have_initial_pose_) {
        return;
      }
      initial_guess = initial_map_to_base_ * poseToMatrix(latest_odom_->pose.pose).inverse();
    }

    const bool initial_registration = !localized_;
    Eigen::Matrix4d candidate;
    double rmse = std::numeric_limits<double>::infinity();
    double overlap = 0.0;
    if (!registerScan(initial_guess, initial_registration, candidate, rmse, overlap)) {
      publishMetrics(rmse, overlap);
      if (initial_registration) {
        publishStatus("INITIALIZATION_REJECTED: adjust /initialpose and retry");
      } else {
        RCLCPP_WARN(
          get_logger(),
          "Periodic correction rejected by quality gate; retaining the current map -> odom");
      }
      return;
    }

    if (localized_) {
      const Eigen::Matrix4d delta = map_to_odom_.inverse() * candidate;
      const double translation_jump = delta.block<3, 1>(0, 3).norm();
      const double rotation_jump = rotationAngle(delta.block<3, 3>(0, 0));
      if (translation_jump > maximum_update_translation_ || rotation_jump > maximum_update_rotation_) {
        publishMetrics(rmse, overlap);
        RCLCPP_WARN(
          get_logger(),
          "Periodic correction rejected: jump %.3f m, %.2f deg exceeds limits; retaining map -> odom",
          translation_jump, rotation_jump * 180.0 / M_PI);
        return;
      }
      map_to_odom_ = smoothedTransform(map_to_odom_, candidate, correction_alpha_);
    } else {
      map_to_odom_ = candidate;
      localized_ = true;
      localization_requested_ = false;
      RCLCPP_INFO(get_logger(), "Relocalization succeeded: RMSE=%.3f m overlap=%.1f%%", rmse, overlap * 100.0);
    }

    publishMetrics(rmse, overlap);
    publishStatus("LOCALIZED");
    publishAlignedScan();
    publishTransformAndCorrection(latest_scan_stamp_);
  }

  bool registerScan(
    const Eigen::Matrix4d & initial_guess, bool run_coarse_stage, Eigen::Matrix4d & result,
    double & rmse, double & overlap)
  {
    const Eigen::Matrix4d map_to_base = initial_guess * poseToMatrix(latest_odom_->pose.pose);
    const Eigen::Vector3d center = map_to_base.block<3, 1>(0, 3);
    Cloud::Ptr fine_submap = cropMap(map_fine_, center, submap_radius_);
    Cloud::Ptr coarse_submap = cropMap(map_coarse_, center, submap_radius_);
    if (fine_submap->size() < static_cast<std::size_t>(minimum_submap_points_) ||
      (run_coarse_stage &&
      coarse_submap->size() < static_cast<std::size_t>(minimum_submap_points_)))
    {
      RCLCPP_WARN(
        get_logger(), "Local map is too small around the initial pose: fine=%zu coarse=%zu",
        fine_submap->size(), coarse_submap->size());
      return false;
    }

    Eigen::Matrix4f fine_guess = initial_guess.cast<float>();
    if (run_coarse_stage) {
      Cloud::Ptr coarse_scan = voxelized(latest_scan_, coarse_voxel_size_);
      pcl::IterativeClosestPoint<PointT, PointT> coarse_icp;
      configureIcp(coarse_icp, coarse_max_correspondence_, coarse_iterations_);
      coarse_icp.setInputSource(coarse_scan);
      coarse_icp.setInputTarget(coarse_submap);
      Cloud coarse_aligned;
      coarse_icp.align(coarse_aligned, fine_guess);
      if (!coarse_icp.hasConverged()) {
        RCLCPP_WARN(get_logger(), "Coarse ICP did not converge");
        return false;
      }
      fine_guess = coarse_icp.getFinalTransformation();
    }

    pcl::IterativeClosestPoint<PointT, PointT> fine_icp;
    configureIcp(fine_icp, fine_max_correspondence_, fine_iterations_);
    fine_icp.setInputSource(latest_scan_);
    fine_icp.setInputTarget(fine_submap);
    Cloud fine_aligned;
    fine_icp.align(fine_aligned, fine_guess);
    if (!fine_icp.hasConverged()) {
      RCLCPP_WARN(get_logger(), "Fine ICP did not converge");
      return false;
    }

    result = fine_icp.getFinalTransformation().cast<double>();
    evaluateAlignment(
      latest_scan_, fine_submap, result, fine_max_correspondence_, rmse, overlap);
    RCLCPP_INFO(
      get_logger(), "Scan-to-map candidate: RMSE=%.3f m overlap=%.1f%% source=%zu target=%zu",
      rmse, overlap * 100.0, latest_scan_->size(), fine_submap->size());
    return std::isfinite(rmse) && rmse <= maximum_rmse_ && overlap >= minimum_overlap_;
  }

  static void configureIcp(
    pcl::IterativeClosestPoint<PointT, PointT> & icp, double correspondence, int iterations)
  {
    icp.setMaxCorrespondenceDistance(correspondence);
    icp.setMaximumIterations(iterations);
    icp.setTransformationEpsilon(1.0e-8);
    icp.setEuclideanFitnessEpsilon(1.0e-6);
    icp.setRANSACOutlierRejectionThreshold(correspondence);
  }

  static Cloud::Ptr cropMap(
    const Cloud::ConstPtr & map, const Eigen::Vector3d & center, double radius)
  {
    Cloud::Ptr output(new Cloud());
    output->reserve(map->size());
    const double radius_squared = radius * radius;
    for (const auto & point : map->points) {
      const double dx = static_cast<double>(point.x) - center.x();
      const double dy = static_cast<double>(point.y) - center.y();
      const double dz = static_cast<double>(point.z) - center.z();
      if (dx * dx + dy * dy + dz * dz <= radius_squared) {
        output->push_back(point);
      }
    }
    return output;
  }

  static void evaluateAlignment(
    const Cloud::ConstPtr & source, const Cloud::ConstPtr & target,
    const Eigen::Matrix4d & transform, double evaluation_distance,
    double & rmse, double & overlap)
  {
    Cloud transformed;
    pcl::transformPointCloud(*source, transformed, transform.cast<float>());
    pcl::KdTreeFLANN<PointT> tree;
    tree.setInputCloud(target);
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    std::size_t matched = 0;
    double squared_error = 0.0;
    const double evaluation_distance_squared = evaluation_distance * evaluation_distance;
    for (const auto & point : transformed.points) {
      if (tree.nearestKSearch(point, 1, indices, distances) == 1 &&
        distances[0] <= evaluation_distance_squared)
      {
        ++matched;
        squared_error += distances[0];
      }
    }
    overlap = transformed.empty() ? 0.0 :
      static_cast<double>(matched) / static_cast<double>(transformed.size());
    rmse = matched == 0 ? std::numeric_limits<double>::infinity() :
      std::sqrt(squared_error / static_cast<double>(matched));
  }

  static Eigen::Matrix4d smoothedTransform(
    const Eigen::Matrix4d & previous, const Eigen::Matrix4d & candidate, double alpha)
  {
    Eigen::Matrix4d output = Eigen::Matrix4d::Identity();
    output.block<3, 1>(0, 3) =
      (1.0 - alpha) * previous.block<3, 1>(0, 3) + alpha * candidate.block<3, 1>(0, 3);
    Eigen::Quaterniond previous_q(previous.block<3, 3>(0, 0));
    Eigen::Quaterniond candidate_q(candidate.block<3, 3>(0, 0));
    output.block<3, 3>(0, 0) = previous_q.normalized().slerp(alpha, candidate_q.normalized()).toRotationMatrix();
    return output;
  }

  void publishMap()
  {
    if (!map_fine_ || map_fine_->empty()) {
      return;
    }
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(*map_fine_, message);
    message.header.frame_id = map_frame_;
    message.header.stamp = now();
    map_publisher_->publish(message);
  }

  void publishAlignedScan()
  {
    if (!latest_scan_) {
      return;
    }
    Cloud aligned;
    pcl::transformPointCloud(*latest_scan_, aligned, map_to_odom_.cast<float>());
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(aligned, message);
    message.header.frame_id = map_frame_;
    message.header.stamp = latest_scan_stamp_;
    aligned_scan_publisher_->publish(message);
  }

  void publishTransformAndCorrection(const builtin_interfaces::msg::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    const auto pose = matrixToPose(map_to_odom_);
    transform.transform.translation.x = pose.position.x;
    transform.transform.translation.y = pose.position.y;
    transform.transform.translation.z = pose.position.z;
    transform.transform.rotation = pose.orientation;
    tf_broadcaster_->sendTransform(transform);

    nav_msgs::msg::Odometry correction;
    correction.header = transform.header;
    correction.child_frame_id = odom_frame_;
    correction.pose.pose = pose;
    map_to_odom_publisher_->publish(correction);
  }

  void publishMetrics(double rmse, double overlap)
  {
    std_msgs::msg::Float64 rmse_message;
    rmse_message.data = rmse;
    rmse_publisher_->publish(rmse_message);
    std_msgs::msg::Float64 overlap_message;
    overlap_message.data = overlap;
    overlap_publisher_->publish(overlap_message);
  }

  void publishStatus(const std::string & status)
  {
    if (status == last_status_) {
      return;
    }
    last_status_ = status;
    std_msgs::msg::String message;
    message.data = status;
    status_publisher_->publish(message);
    RCLCPP_INFO(get_logger(), "Relocalization status: %s", status.c_str());
  }

  void relocalizeCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (!have_initial_pose_ && !localized_) {
      response->success = false;
      response->message = "No initial pose is available; publish /initialpose first.";
      return;
    }
    if (localized_ && latest_odom_) {
      initial_map_to_base_ = map_to_odom_ * poseToMatrix(latest_odom_->pose.pose);
      have_initial_pose_ = true;
    }
    localized_ = false;
    localization_requested_ = true;
    last_processed_scan_sequence_ = 0;
    publishStatus("RELOCALIZATION_REQUESTED");
    response->success = true;
    response->message = "Relocalization will run on the next fresh scan.";
  }

  void resetCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    localized_ = false;
    localization_requested_ = false;
    have_initial_pose_ = false;
    map_to_odom_ = Eigen::Matrix4d::Identity();
    publishStatus("WAITING_FOR_INITIAL_POSE: localization reset");
    response->success = true;
    response->message = "Global localization reset; publish /initialpose.";
  }

  std::string map_path_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string scan_topic_;
  std::string odom_topic_;
  std::string initial_pose_topic_;
  std::string map_topic_;
  std::string global_odom_topic_;
  double localization_frequency_{};
  double map_publish_frequency_{};
  double map_voxel_size_{};
  double scan_voxel_size_{};
  double coarse_voxel_size_{};
  double submap_radius_{};
  double coarse_max_correspondence_{};
  double fine_max_correspondence_{};
  int coarse_iterations_{};
  int fine_iterations_{};
  int minimum_scan_points_{};
  int minimum_submap_points_{};
  double minimum_overlap_{};
  double maximum_rmse_{};
  double maximum_update_translation_{};
  double maximum_update_rotation_{};
  double correction_alpha_{};
  bool auto_initialize_{};
  std::vector<double> initial_pose_xyzrpy_;

  Cloud::Ptr map_raw_;
  Cloud::Ptr map_fine_;
  Cloud::Ptr map_coarse_;
  Cloud::Ptr latest_scan_;
  builtin_interfaces::msg::Time latest_scan_stamp_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  Eigen::Matrix4d map_to_odom_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d initial_map_to_base_{Eigen::Matrix4d::Identity()};
  bool localized_{false};
  bool have_initial_pose_{false};
  bool localization_requested_{false};
  bool warned_scan_frame_{false};
  std::size_t scan_sequence_{0};
  std::size_t last_processed_scan_sequence_{0};
  std::string last_status_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_scan_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr global_odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr map_to_odom_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rmse_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr overlap_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr relocalize_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr localization_timer_;
  rclcpp::TimerBase::SharedPtr map_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MapLocalizer>());
  } catch (const std::exception & exception) {
    std::cerr << "map_localizer fatal error: " << exception.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
