#include "eskf_localization/debug/eskf_dead_reckoning_node.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Core>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>

namespace eskf_localization
{

namespace
{

double clamp_dt(double dt)
{
  if (!std::isfinite(dt)) {
    return 0.02;
  }
  dt = std::clamp(dt, 0.0001, 0.5);
  return dt;
}

} // namespace

EskfDeadReckoningNode::EskfDeadReckoningNode(const rclcpp::NodeOptions & options)
: Node("eskf_dead_reckoning", options),
  last_imu_stamp_(0, 0, RCL_ROS_TIME)
{
  map_frame_ = this->declare_parameter("map_frame", "map");
  base_frame_ = this->declare_parameter("base_frame", "base_link");
  eskf_dr_frame_ =
    this->declare_parameter("eskf_dr_frame", "eskf_base_link_eskf_dr");

  imu_topic_ = this->declare_parameter("imu_topic", "/sensing/imu/imu_data");
  velocity_topic_ = this->declare_parameter(
    "velocity_topic",
    "/vehicle/status/velocity_status");
  steering_topic_ = this->declare_parameter(
    "steering_topic",
    "/vehicle/status/steering_status");
  gt_odom_topic_ =
    this->declare_parameter("gt_odom_topic", "/localization/kinematic_state");

  enable_speed_update_ =
    this->declare_parameter("vehicle.enable_speed_update", true);
  speed_use_abs_ = this->declare_parameter("vehicle.speed_use_abs", false);
  speed_var_ = this->declare_parameter("vehicle.speed_var", 0.25);
  min_speed_mps_for_speed_update_ =
    this->declare_parameter("vehicle.min_speed_mps_for_speed_update", 1.0);
  speed_correction_scale_ =
    this->declare_parameter("vehicle.speed_correction_scale", 0.975);
  speed_correction_offset_mps_ =
    this->declare_parameter("vehicle.speed_correction_offset_mps", 0.0);
  if (!std::isfinite(speed_correction_scale_) || speed_correction_scale_ == 0.0) {
    speed_correction_scale_ = 1.0;
  }
  if (!std::isfinite(speed_correction_offset_mps_)) {
    speed_correction_offset_mps_ = 0.0;
  }

  enable_nhc_ = this->declare_parameter("vehicle.enable_nhc", true);
  nhc_var_ = this->declare_parameter("vehicle.nhc_var", 0.04);

  enable_zupt_ = this->declare_parameter("vehicle.enable_zupt", false);
  zupt_speed_threshold_mps_ =
    this->declare_parameter("vehicle.zupt_speed_threshold_mps", 0.2);
  zupt_var_ = this->declare_parameter("vehicle.zupt_var", 0.01);

  enable_yaw_rate_update_ =
    this->declare_parameter("vehicle.enable_yaw_rate_update", false);
  yaw_rate_min_speed_mps_ =
    this->declare_parameter("vehicle.yaw_rate_min_speed_mps", 1.0);
  yaw_rate_var_ = this->declare_parameter("vehicle.yaw_rate_var", 0.0004);
  wheelbase_ = this->declare_parameter("wheelbase", 7.0);
  path_stride_ = this->declare_parameter("path_stride", 5);
  path_max_poses_ = this->declare_parameter("path_max_poses", 5000);
  path_stride_ = std::max(1, path_stride_);
  path_max_poses_ = std::max(1, path_max_poses_);

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
    "/debug/eskf_dead_reckoning/odom", 10);
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "/debug/eskf_dead_reckoning/path", 10);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  gt_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    gt_odom_topic_, 10,
    std::bind(
      &EskfDeadReckoningNode::gt_odom_callback, this,
      std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, 50,
    std::bind(
      &EskfDeadReckoningNode::imu_callback, this,
      std::placeholders::_1));

  velocity_sub_ =
    this->create_subscription<autoware_vehicle_msgs::msg::VelocityReport>(
    velocity_topic_, 20,
    std::bind(
      &EskfDeadReckoningNode::velocity_callback, this,
      std::placeholders::_1));

  steering_sub_ =
    this->create_subscription<autoware_vehicle_msgs::msg::SteeringReport>(
    steering_topic_, 20,
    std::bind(
      &EskfDeadReckoningNode::steering_callback, this,
      std::placeholders::_1));
}

void EskfDeadReckoningNode::gt_odom_callback(
  const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  if (initialized_) {
    return;
  }

  const Eigen::Vector3d p(msg->pose.pose.position.x, msg->pose.pose.position.y,
    msg->pose.pose.position.z);
  const auto & q_msg = msg->pose.pose.orientation;
  Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
  if (!p.allFinite()) {
    return;
  }
  if (!std::isfinite(q.w()) || !std::isfinite(q.x()) || !std::isfinite(q.y()) ||
    !std::isfinite(q.z()))
  {
    return;
  }
  if (q.norm() < 1e-12) {
    q = Eigen::Quaterniond::Identity();
  }
  eskf_.initialize(p, q.normalized());
  initialized_ = true;
  last_imu_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  RCLCPP_INFO(
    this->get_logger(),
    "Initialized ESKF DR from GT odom: p=[%.3f, %.3f, %.3f]",
    p.x(), p.y(), p.z());
}

void EskfDeadReckoningNode::velocity_callback(
  const autoware_vehicle_msgs::msg::VelocityReport::ConstSharedPtr msg)
{
  const double v_raw = static_cast<double>(msg->longitudinal_velocity);
  double v_meas = speed_use_abs_ ? std::abs(v_raw) : v_raw;
  v_meas = v_meas * speed_correction_scale_ + speed_correction_offset_mps_;
  latest_speed_mps_ = v_meas;
  have_latest_speed_ = std::isfinite(v_meas);

  if (!initialized_) {
    return;
  }

  if (!std::isfinite(speed_var_) || !(speed_var_ > 0.0) ||
    !std::isfinite(nhc_var_) || !(nhc_var_ > 0.0) ||
    !std::isfinite(zupt_var_) || !(zupt_var_ > 0.0))
  {
    return;
  }

  bool stationary = false;
  if (enable_zupt_) {
    const double v_abs = std::abs(v_raw);
    if (std::isfinite(v_abs) && std::isfinite(zupt_speed_threshold_mps_) &&
      (v_abs < zupt_speed_threshold_mps_))
    {
      stationary = true;
    }
  }

  bool any_applied = false;
  if (stationary) {
    const auto dbg = eskf_.update_body_velocity_component(0, 0.0, zupt_var_);
    any_applied = any_applied || dbg.applied;
  } else if (enable_speed_update_ && std::isfinite(v_meas)) {
    const double v_gate = std::abs(v_meas);
    if (min_speed_mps_for_speed_update_ > 0.0 &&
      std::isfinite(v_gate) && (v_gate < min_speed_mps_for_speed_update_))
    {
    } else {
      const auto dbg = eskf_.update_body_velocity_component(0, v_meas, speed_var_);
      any_applied = any_applied || dbg.applied;
    }
  }

  if (enable_nhc_) {
    const auto dbg_y = eskf_.update_body_velocity_component(1, 0.0, nhc_var_);
    const auto dbg_z = eskf_.update_body_velocity_component(2, 0.0, nhc_var_);
    any_applied = any_applied || dbg_y.applied || dbg_z.applied;
  }

  if (any_applied) {
    publish_outputs(msg->header.stamp);
  }
}

void EskfDeadReckoningNode::steering_callback(
  const autoware_vehicle_msgs::msg::SteeringReport::ConstSharedPtr msg)
{
  if (!enable_yaw_rate_update_ || !initialized_ || !have_last_omega_ ||
    !have_latest_speed_)
  {
    return;
  }

  if (!std::isfinite(wheelbase_) || !(wheelbase_ > 0.0) ||
    !std::isfinite(yaw_rate_var_) || !(yaw_rate_var_ > 0.0))
  {
    return;
  }

  const double v = latest_speed_mps_;
  if (!std::isfinite(v) || (std::abs(v) < yaw_rate_min_speed_mps_)) {
    return;
  }

  const double delta = static_cast<double>(msg->steering_tire_angle);
  if (!std::isfinite(delta)) {
    return;
  }

  const double yaw_rate_ref = v * std::tan(delta) / wheelbase_;
  if (!std::isfinite(yaw_rate_ref)) {
    return;
  }

  const auto dbg = eskf_.update_yaw_rate_from_steer(
    last_omega_z_radps_, yaw_rate_ref, yaw_rate_var_);
  if (dbg.applied) {
    publish_outputs(msg->stamp);
  }
}

void EskfDeadReckoningNode::imu_callback(
  const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if (!initialized_) {
    return;
  }

  const rclcpp::Time stamp(msg->header.stamp);

  double dt = 0.02;
  if (last_imu_stamp_.seconds() > 0.0) {
    dt = (stamp - last_imu_stamp_).seconds();
  }
  dt = clamp_dt(dt);
  last_imu_stamp_ = stamp;

  const double wx = static_cast<double>(msg->angular_velocity.x);
  const double wy = static_cast<double>(msg->angular_velocity.y);
  const double wz = static_cast<double>(msg->angular_velocity.z);
  if (!std::isfinite(wx) || !std::isfinite(wy) || !std::isfinite(wz)) {
    return;
  }

  last_omega_z_radps_ = wz;
  have_last_omega_ = true;

  const Eigen::Vector3d omega(wx, wy, wz);
  const Eigen::Vector3d accel = eskf_.b_a(); // make (accel_meas - b_a) == 0
  eskf_.propagate(omega, accel, dt);

  publish_outputs(stamp);
}

void EskfDeadReckoningNode::publish_outputs(const rclcpp::Time & stamp)
{
  const auto & p = eskf_.p_map();
  const auto & q = eskf_.q_map_from_base();

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = map_frame_;
  tf.child_frame_id = eskf_dr_frame_;
  tf.transform.translation.x = p.x();
  tf.transform.translation.y = p.y();
  tf.transform.translation.z = p.z();
  tf.transform.rotation.w = q.w();
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf_broadcaster_->sendTransform(tf);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = map_frame_;
  odom.child_frame_id = eskf_dr_frame_;
  odom.pose.pose.position.x = p.x();
  odom.pose.pose.position.y = p.y();
  odom.pose.pose.position.z = p.z();
  odom.pose.pose.orientation = tf.transform.rotation;
  odom.twist.twist.linear.x = have_latest_speed_ ? latest_speed_mps_ : 0.0;
  odom_pub_->publish(odom);

  path_counter_++;
  if ((path_counter_ % path_stride_) == 0) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = odom.header;
    pose.pose = odom.pose.pose;
    path_poses_.push_back(pose);
    while (static_cast<int>(path_poses_.size()) > path_max_poses_) {
      path_poses_.pop_front();
    }

    nav_msgs::msg::Path path;
    path.header = odom.header;
    path.poses.assign(path_poses_.begin(), path_poses_.end());
    path_pub_->publish(path);
  }
}

} // namespace eskf_localization
