// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include <Eigen/Core>
#include <memory>
#include <sophus/se3.hpp>
#include <utility>
#include <vector>

// KISS-ICP-ROS
#include "OdometryServer.hpp"
#include "Utils.hpp"

// KISS-ICP
#include "kiss_icp/pipeline/KissICP.hpp"

// ROS 2 headers
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

namespace {
Sophus::SE3d
LookupTransform(const std::string &target_frame,
                const std::string &source_frame,
                const std::unique_ptr<tf2_ros::Buffer> &tf2_buffer) {
  std::string err_msg;
  if (tf2_buffer->canTransform(target_frame, source_frame, tf2::TimePointZero,
                               &err_msg)) {
    try {
      auto tf = tf2_buffer->lookupTransform(target_frame, source_frame,
                                            tf2::TimePointZero);
      return tf2::transformToSophus(tf);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(rclcpp::get_logger("LookupTransform"), "%s", ex.what());
    }
  }
  RCLCPP_WARN(rclcpp::get_logger("LookupTransform"),
              "Failed to find tf. Reason=%s", err_msg.c_str());
  // default construction is the identity
  return Sophus::SE3d();
}
} // namespace

namespace kiss_icp_ros {

using utils::EigenToPointCloud2;
using utils::GetTimestamps;
using utils::PointCloud2ToEigen;

OdometryServer::OdometryServer(const rclcpp::NodeOptions &options)
    : rclcpp::Node("kiss_icp_node", options) {
  kiss_icp::pipeline::KISSConfig config;
  initializeParameters(config);

  // Construct the main KISS-ICP odometry node
  kiss_icp_ = std::make_unique<kiss_icp::pipeline::KissICP>(config);

  // Initialize subscribers based on mode
  if (multi_lidar_mode_) {
    // Multi-LiDAR mode: subscribe to all configured LiDAR topics
    const size_t num_lidars = lidar_topics_.size();
    RCLCPP_INFO(this->get_logger(), "Multi-LiDAR mode enabled (%zu LiDARs)",
                num_lidars);

    // Initialize cache vectors
    cached_clouds_.resize(num_lidars);
    cloud_timestamps_.resize(num_lidars);
    multi_cloud_subs_.resize(num_lidars);

    for (size_t i = 0; i < num_lidars; ++i) {
      RCLCPP_INFO(this->get_logger(), "  [%zu] Topic: %s (frame: %s)", i,
                  lidar_topics_[i].c_str(), lidar_frames_[i].c_str());

      multi_cloud_subs_[i] = create_subscription<sensor_msgs::msg::PointCloud2>(
          lidar_topics_[i], rclcpp::SensorDataQoS(),
          [this, i](const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
            this->MultiLidarCallback(msg, i);
          });
    }
  } else {
    // Single topic mode (original behavior)
    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "pointcloud_topic", rclcpp::SensorDataQoS(),
        std::bind(&OdometryServer::RegisterFrame, this, std::placeholders::_1));
  }

  // Initialize publishers
  rclcpp::QoS qos(
      (rclcpp::SystemDefaultsQoS().keep_last(1).durability_volatile()));
  odom_publisher_ =
      create_publisher<nav_msgs::msg::Odometry>("kiss/odometry", qos);
  if (publish_debug_clouds_) {
    frame_publisher_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("kiss/frame", qos);
    kpoints_publisher_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("kiss/keypoints", qos);
    map_publisher_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("kiss/local_map", qos);
  }

  // Initialize the transform broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf2_buffer_->setUsingDedicatedThread(true);
  tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);
  // Initialize service servers
  reset_service_ = create_service<std_srvs::srv::Empty>(
      "kiss/reset", std::bind(&OdometryServer::ResetService, this,
                              std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "KISS-ICP ROS 2 odometry node initialized");
}

void OdometryServer::initializeParameters(
    kiss_icp::pipeline::KISSConfig &config) {
  RCLCPP_INFO(this->get_logger(), "Initializing parameters");

  base_frame_ = declare_parameter<std::string>("base_frame", base_frame_);
  RCLCPP_INFO(this->get_logger(), "\tBase frame: %s", base_frame_.c_str());
  // child_frame defaults to base_frame if not specified
  child_frame_ = declare_parameter<std::string>("child_frame", base_frame_);
  if (child_frame_.empty())
    child_frame_ = base_frame_;
  RCLCPP_INFO(this->get_logger(), "\tChild frame (TF output): %s",
              child_frame_.c_str());
  lidar_odom_frame_ =
      declare_parameter<std::string>("lidar_odom_frame", lidar_odom_frame_);
  RCLCPP_INFO(this->get_logger(), "\tLiDAR odometry frame: %s",
              lidar_odom_frame_.c_str());
  publish_odom_tf_ =
      declare_parameter<bool>("publish_odom_tf", publish_odom_tf_);
  RCLCPP_INFO(this->get_logger(), "\tPublish odometry transform: %d",
              publish_odom_tf_);
  invert_odom_tf_ = declare_parameter<bool>("invert_odom_tf", invert_odom_tf_);
  RCLCPP_INFO(this->get_logger(), "\tInvert odometry transform: %d",
              invert_odom_tf_);
  publish_debug_clouds_ =
      declare_parameter<bool>("publish_debug_clouds", publish_debug_clouds_);
  RCLCPP_INFO(this->get_logger(), "\tPublish debug clouds: %d",
              publish_debug_clouds_);
  position_covariance_ = declare_parameter<double>("position_covariance", 0.1);
  RCLCPP_INFO(this->get_logger(), "\tPosition covariance: %.2f",
              position_covariance_);
  orientation_covariance_ =
      declare_parameter<double>("orientation_covariance", 0.1);
  RCLCPP_INFO(this->get_logger(), "\tOrientation covariance: %.2f",
              orientation_covariance_);

  // Multi-LiDAR mode parameters
  multi_lidar_mode_ = declare_parameter<bool>("multi_lidar.enabled", false);
  if (multi_lidar_mode_) {
    // Default topics for Triple LiDAR setup
    std::vector<std::string> default_topics = {
        "/sensing/lidar/left_lidar/pointcloud",
        "/sensing/lidar/right_lidar/pointcloud",
        "/sensing/lidar/rear_lidar/pointcloud"};
    std::vector<std::string> default_frames = {
        "left_lidar_link", "right_lidar_link", "rear_lidar_link"};

    lidar_topics_ = declare_parameter<std::vector<std::string>>(
        "multi_lidar.topics", default_topics);
    lidar_frames_ = declare_parameter<std::vector<std::string>>(
        "multi_lidar.frames", default_frames);
    sync_tolerance_sec_ =
        declare_parameter<double>("multi_lidar.sync_tolerance_sec", 0.05);

    // Validate topics and frames count match
    if (lidar_topics_.size() != lidar_frames_.size()) {
      RCLCPP_ERROR(
          this->get_logger(),
          "multi_lidar.topics and multi_lidar.frames must have same size!");
      throw std::runtime_error("Topic/frame count mismatch");
    }

    RCLCPP_INFO(this->get_logger(), "\tMulti-LiDAR mode: ENABLED (%zu sensors)",
                lidar_topics_.size());
    RCLCPP_INFO(this->get_logger(), "\t  Sync tolerance: %.3f sec",
                sync_tolerance_sec_);
  }

  config.max_range =
      declare_parameter<double>("data.max_range", config.max_range);
  RCLCPP_INFO(this->get_logger(), "\tMax range: %.2f", config.max_range);
  config.min_range =
      declare_parameter<double>("data.min_range", config.min_range);
  RCLCPP_INFO(this->get_logger(), "\tMin range: %.2f", config.min_range);
  config.deskew = declare_parameter<bool>("data.deskew", config.deskew);
  RCLCPP_INFO(this->get_logger(), "\tDeskew: %d", config.deskew);
  config.voxel_size =
      declare_parameter<double>("mapping.voxel_size", config.max_range / 100.0);
  RCLCPP_INFO(this->get_logger(), "\tVoxel size: %.2f", config.voxel_size);
  config.max_points_per_voxel = declare_parameter<int>(
      "mapping.max_points_per_voxel", config.max_points_per_voxel);
  RCLCPP_INFO(this->get_logger(), "\tMax points per voxel: %d",
              config.max_points_per_voxel);
  config.initial_threshold = declare_parameter<double>(
      "adaptive_threshold.initial_threshold", config.initial_threshold);
  RCLCPP_INFO(this->get_logger(), "\tInitial threshold: %.2f",
              config.initial_threshold);
  config.min_motion_th = declare_parameter<double>(
      "adaptive_threshold.min_motion_th", config.min_motion_th);
  RCLCPP_INFO(this->get_logger(), "\tMin motion threshold: %.2f",
              config.min_motion_th);
  config.max_num_iterations = declare_parameter<int>(
      "registration.max_num_iterations", config.max_num_iterations);
  RCLCPP_INFO(this->get_logger(), "\tMax number of iterations: %d",
              config.max_num_iterations);
  config.convergence_criterion = declare_parameter<double>(
      "registration.convergence_criterion", config.convergence_criterion);
  RCLCPP_INFO(this->get_logger(), "\tConvergence criterion: %.2f",
              config.convergence_criterion);
  config.max_num_threads = declare_parameter<int>(
      "registration.max_num_threads", config.max_num_threads);
  RCLCPP_INFO(this->get_logger(), "\tMax number of threads: %d",
              config.max_num_threads);
  if (config.max_range < config.min_range) {
    RCLCPP_WARN(get_logger(), "[WARNING] max_range is smaller than min_range, "
                              "setting min_range to 0.0");
    config.min_range = 0.0;
  }

  // ============== ESKF Initialization Waiting Parameters ==============
  wait_for_eskf_init_ = declare_parameter<bool>("eskf_init.enabled", false);
  if (wait_for_eskf_init_) {
    eskf_map_frame_ =
        declare_parameter<std::string>("eskf_init.map_frame", "map");
    eskf_base_frame_ =
        declare_parameter<std::string>("eskf_init.base_frame", "base_link");
    eskf_init_timeout_sec_ =
        declare_parameter<double>("eskf_init.timeout_sec", 30.0);
    eskf_wait_start_time_ = this->now();
    eskf_initialized_ = false;
    eskf_timeout_warned_ = false;

    RCLCPP_INFO(this->get_logger(),
                "\t[ESKF Init] Waiting for ESKF initialization: ENABLED");
    RCLCPP_INFO(this->get_logger(), "\t[ESKF Init]   Map frame: %s",
                eskf_map_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "\t[ESKF Init]   Base frame: %s",
                eskf_base_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "\t[ESKF Init]   Timeout: %.1f sec",
                eskf_init_timeout_sec_);
  } else {
    eskf_initialized_ = true; // Skip waiting if not enabled
    RCLCPP_INFO(this->get_logger(),
                "\t[ESKF Init] Waiting for ESKF initialization: DISABLED");
  }
}

void OdometryServer::RegisterFrame(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
  // Check ESKF initialization if enabled
  if (!eskf_initialized_ && !TryInitializeFromEskf()) {
    return; // Still waiting for ESKF
  }

  const auto cloud_frame_id = msg->header.frame_id;
  const auto points = PointCloud2ToEigen(msg);
  const auto timestamps = GetTimestamps(msg);

  // Register frame, main entry point to KISS-ICP pipeline
  const auto &[frame, keypoints] = kiss_icp_->RegisterFrame(points, timestamps);

  // Extract the last KISS-ICP pose, ego-centric to the LiDAR
  const Sophus::SE3d kiss_pose = kiss_icp_->pose();

  // Spit the current estimated pose to ROS msgs handling the desired target
  // frame
  PublishOdometry(kiss_pose, msg->header);
  // Publishing these clouds is a bit costly, so do it only if we are debugging
  if (publish_debug_clouds_) {
    PublishClouds(frame, keypoints, msg->header);
  }
}

void OdometryServer::PublishOdometry(const Sophus::SE3d &kiss_pose,
                                     const std_msgs::msg::Header &header) {
  // If necessary, transform the ego-centric pose to the specified
  // base_link/base_footprint frame
  const auto cloud_frame_id = header.frame_id;
  const auto egocentric_estimation =
      (base_frame_.empty() || base_frame_ == cloud_frame_id);
  const auto moving_frame =
      egocentric_estimation ? cloud_frame_id : base_frame_;
  const auto pose = [&]() -> Sophus::SE3d {
    if (egocentric_estimation)
      return kiss_pose;
    const Sophus::SE3d cloud2base =
        LookupTransform(base_frame_, cloud_frame_id, tf2_buffer_);
    return cloud2base * kiss_pose * cloud2base.inverse();
  }();

  // Determine the output child frame name
  // If child_frame_ is set, use it; otherwise fall back to moving_frame
  const auto output_child_frame =
      child_frame_.empty() ? moving_frame : child_frame_;

  // Broadcast the tf ---
  if (publish_odom_tf_) {
    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header.stamp = header.stamp;
    if (invert_odom_tf_) {
      transform_msg.header.frame_id = output_child_frame;
      transform_msg.child_frame_id = lidar_odom_frame_;
      transform_msg.transform = tf2::sophusToTransform(pose.inverse());
    } else {
      transform_msg.header.frame_id = lidar_odom_frame_;
      transform_msg.child_frame_id = output_child_frame;
      transform_msg.transform = tf2::sophusToTransform(pose);
    }
    tf_broadcaster_->sendTransform(transform_msg);
  }

  // publish odometry msg
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = header.stamp;
  odom_msg.header.frame_id = lidar_odom_frame_;
  odom_msg.child_frame_id = output_child_frame;
  odom_msg.pose.pose = tf2::sophusToPose(pose);
  odom_msg.pose.covariance.fill(0.0);
  odom_msg.pose.covariance[0] = position_covariance_;
  odom_msg.pose.covariance[7] = position_covariance_;
  odom_msg.pose.covariance[14] = position_covariance_;
  odom_msg.pose.covariance[21] = orientation_covariance_;
  odom_msg.pose.covariance[28] = orientation_covariance_;
  odom_msg.pose.covariance[35] = orientation_covariance_;
  odom_publisher_->publish(std::move(odom_msg));
}

void OdometryServer::PublishClouds(
    const std::vector<Eigen::Vector3d> &frame,
    const std::vector<Eigen::Vector3d> &keypoints,
    const std_msgs::msg::Header &header) {
  const auto kiss_map = kiss_icp_->LocalMap();

  frame_publisher_->publish(std::move(EigenToPointCloud2(frame, header)));
  kpoints_publisher_->publish(std::move(EigenToPointCloud2(keypoints, header)));
  auto local_map_header = header;
  local_map_header.frame_id = lidar_odom_frame_;
  map_publisher_->publish(
      std::move(EigenToPointCloud2(kiss_map, local_map_header)));
}
void OdometryServer::ResetService(
    [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request>
        request,
    [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Resetting KISS-ICP map and odometry");

  // Reset the KISS-ICP pipeline
  kiss_icp_->Reset();

  RCLCPP_INFO(this->get_logger(), "KISS-ICP reset completed successfully");
}

// ============== Multi-LiDAR Support Functions ==============

void OdometryServer::MultiLidarCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg,
    size_t lidar_index) {
  std::lock_guard<std::mutex> lock(cloud_mutex_);

  // Cache the cloud and its timestamp
  cached_clouds_[lidar_index] = msg;
  cloud_timestamps_[lidar_index] = rclcpp::Time(msg->header.stamp);

  // Check if we have all clouds synchronized
  bool all_received = true;
  for (size_t i = 0; i < cached_clouds_.size(); ++i) {
    if (!cached_clouds_[i]) {
      all_received = false;
      break;
    }
  }

  if (!all_received)
    return;

  // Check timestamps synchronization
  rclcpp::Time min_stamp = cloud_timestamps_[0];
  rclcpp::Time max_stamp = cloud_timestamps_[0];
  for (size_t i = 1; i < cloud_timestamps_.size(); ++i) {
    if (cloud_timestamps_[i] < min_stamp)
      min_stamp = cloud_timestamps_[i];
    if (cloud_timestamps_[i] > max_stamp)
      max_stamp = cloud_timestamps_[i];
  }

  const double time_diff = (max_stamp - min_stamp).seconds();
  if (time_diff <= sync_tolerance_sec_) {
    ProcessConcatenatedClouds();
  }
}

void OdometryServer::ProcessConcatenatedClouds() {
  // Check ESKF initialization if enabled
  if (!eskf_initialized_ && !TryInitializeFromEskf()) {
    // Clear cached clouds and return - we'll process fresh data after init
    for (auto &cloud : cached_clouds_) {
      cloud.reset();
    }
    return; // Still waiting for ESKF
  }

  // Verify all clouds are available
  for (const auto &cloud : cached_clouds_) {
    if (!cloud)
      return;
  }

  const std::string &target_frame =
      base_frame_.empty() ? "base_link" : base_frame_;

  // Collect all points transformed to base_frame
  std::vector<Eigen::Vector3d> all_points;
  size_t total_points = 0;

  // First pass: count total points for reservation
  for (const auto &cloud : cached_clouds_) {
    total_points += cloud->width * cloud->height;
  }
  all_points.reserve(total_points);

  // Second pass: convert and transform each cloud
  for (size_t i = 0; i < cached_clouds_.size(); ++i) {
    auto points = PointCloud2ToEigen(cached_clouds_[i]);
    auto transformed = TransformPoints(points, lidar_frames_[i], target_frame);

    // Move points into all_points (zero-copy)
    all_points.insert(all_points.end(),
                      std::make_move_iterator(transformed.begin()),
                      std::make_move_iterator(transformed.end()));
  }

  // Get timestamps from first cloud as reference
  auto timestamps = GetTimestamps(cached_clouds_[0]);

  // Register the concatenated frame
  const auto &[frame, keypoints] =
      kiss_icp_->RegisterFrame(all_points, timestamps);

  // Extract the last KISS-ICP pose
  const Sophus::SE3d kiss_pose = kiss_icp_->pose();

  // Create header for output (use base_frame as frame_id)
  std_msgs::msg::Header header;
  header.stamp = cached_clouds_[0]->header.stamp;
  header.frame_id = target_frame;

  // Publish odometry
  PublishOdometry(kiss_pose, header);

  // Publishing debug clouds
  if (publish_debug_clouds_) {
    PublishClouds(frame, keypoints, header);
  }

  // Clear all cached clouds to wait for next set
  for (auto &cloud : cached_clouds_) {
    cloud.reset();
  }
}

std::vector<Eigen::Vector3d>
OdometryServer::TransformPoints(const std::vector<Eigen::Vector3d> &points,
                                const std::string &source_frame,
                                const std::string &target_frame) {

  if (source_frame == target_frame || points.empty()) {
    return points; // No transformation needed
  }

  // Lookup transform
  Sophus::SE3d transform;
  try {
    auto tf = tf2_buffer_->lookupTransform(target_frame, source_frame,
                                           tf2::TimePointZero,
                                           tf2::durationFromSec(0.1));
    transform = LookupTransform(target_frame, source_frame, tf2_buffer_);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Could not transform %s to %s: %s",
                         source_frame.c_str(), target_frame.c_str(), ex.what());
    return points; // Return untransformed points as fallback
  }

  // Apply transformation to all points
  std::vector<Eigen::Vector3d> transformed_points;
  transformed_points.reserve(points.size());
  for (const auto &p : points) {
    transformed_points.push_back(transform * p);
  }

  return transformed_points;
}

// ============== ESKF Initialization Waiting ==============
bool OdometryServer::TryInitializeFromEskf() {
  if (eskf_initialized_) {
    return true; // Already initialized
  }

  // Initialize start time on first callback (for sim_time compatibility)
  const rclcpp::Time now = this->now();
  static bool first_call = true;
  if (first_call) {
    eskf_wait_start_time_ = now;
    first_call = false;
    RCLCPP_INFO(this->get_logger(),
                "[ESKF Init] Starting to wait for ESKF TF (%s -> %s)...",
                eskf_map_frame_.c_str(), eskf_base_frame_.c_str());
  }

  // Check for timeout
  const double elapsed = (now - eskf_wait_start_time_).seconds();
  if (elapsed > eskf_init_timeout_sec_) {
    if (!eskf_timeout_warned_) {
      RCLCPP_WARN(
          this->get_logger(),
          "[ESKF Init] Timeout (%.1f sec) waiting for ESKF TF (%s -> %s). "
          "Starting with identity pose.",
          eskf_init_timeout_sec_, eskf_map_frame_.c_str(),
          eskf_base_frame_.c_str());
      eskf_timeout_warned_ = true;
    }
    // Start with identity pose after timeout
    eskf_initialized_ = true;
    return true;
  }

  // Try to lookup ESKF TF (map -> base_link)
  std::string err_msg;
  if (!tf2_buffer_->canTransform(eskf_map_frame_, eskf_base_frame_,
                                 tf2::TimePointZero, &err_msg)) {
    // TF not yet available - log periodically
    static int log_counter = 0;
    if (++log_counter % 50 == 1) { // Log every ~50 callbacks (~1 sec at 50Hz)
      RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "[ESKF Init] Waiting for ESKF TF (%s -> %s)... (%.1f sec elapsed)",
          eskf_map_frame_.c_str(), eskf_base_frame_.c_str(), elapsed);
    }
    return false;
  }

  try {
    // Get the ESKF pose from TF
    auto tf = tf2_buffer_->lookupTransform(eskf_map_frame_, eskf_base_frame_,
                                           tf2::TimePointZero);

    // Convert TF to Sophus SE3
    const auto &t = tf.transform.translation;
    const auto &r = tf.transform.rotation;
    Eigen::Quaterniond q(r.w, r.x, r.y, r.z);
    Eigen::Vector3d pos(t.x, t.y, t.z);
    Sophus::SE3d eskf_pose(q, pos);

    // Set the initial pose for KISS-ICP
    kiss_icp_->pose() = eskf_pose;

    RCLCPP_INFO(this->get_logger(),
                "[ESKF Init] Initialized from ESKF TF (%s -> %s): "
                "pos=[%.2f, %.2f, %.2f], yaw=%.1f deg",
                eskf_map_frame_.c_str(), eskf_base_frame_.c_str(), pos.x(),
                pos.y(), pos.z(),
                std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                           1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z())) *
                    180.0 / M_PI);

    eskf_initialized_ = true;
    return true;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "[ESKF Init] TF lookup failed: %s", ex.what());
    return false;
  }
}

} // namespace kiss_icp_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(kiss_icp_ros::OdometryServer)
