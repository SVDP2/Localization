#include "aruco_imu_eskf_localization_cpp/diagnostics_publisher.hpp"
#include "aruco_imu_eskf_localization_cpp/geometry.hpp"
#include "aruco_imu_eskf_localization_cpp/gyro_relative_eskf.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>
#include <optional>
#include <string>

namespace aruco_imu_eskf_localization_cpp
{
namespace
{

int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + static_cast<int64_t>(stamp.nanosec);
}

builtin_interfaces::msg::Time time_msg_from_ns(int64_t ns)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
  stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return stamp;
}

Eigen::Matrix<double, 6, 6> covariance_from_msg(
  const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  Eigen::Matrix<double, 6, 6> cov;
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) {
      cov(r, c) = msg.pose.covariance[r * 6 + c];
    }
  }
  return cov;
}

}  // namespace

struct ImuSample
{
  int64_t stamp_ns{0};
  Eigen::Vector3d angular_velocity_base{Eigen::Vector3d::Zero()};
};

struct ImuInterval
{
  int64_t start_ns{0};
  int64_t end_ns{0};
  Eigen::Vector3d angular_velocity_base{Eigen::Vector3d::Zero()};
};

class RelativeLocalizationNode : public rclcpp::Node
{
public:
  RelativeLocalizationNode()
  : Node("relative_localization_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameter("imu_topic", "imu");
    declare_parameter("board_pose_topic", "localization/aruco/board_pose");
    declare_parameter("odom_topic", "localization/relative/odom");
    declare_parameter("pose_topic", "localization/relative/pose");
    declare_parameter("leader_rear_odom_topic", "localization/leader_rear/odom");
    declare_parameter("leader_rear_pose_topic", "localization/leader_rear/pose");
    declare_parameter("diagnostics_topic", "diagnostics");
    declare_parameter("board_frame", "leader/board");
    declare_parameter("leader_rear_frame", "leader/leader_rear");
    declare_parameter("base_frame", "follower/base_link");
    declare_parameter("camera_frame", "follower/usb_cam");
    declare_parameter("publish_tf", true);
    declare_parameter("output_rate_hz", 100.0);
    declare_parameter("reset_timeout_sec", 1.0);
    declare_parameter("vision_delay_buffer_sec", 2.0);
    declare_parameter("aruco_position_gate_m", 1.0);
    declare_parameter("aruco_position_covariance_scale", 0.35);
    declare_parameter("position_smoothing_alpha", 0.55);
    declare_parameter("gyro_noise_std_radps", 0.05);
    declare_parameter("initial_position_std_m", 0.20);
    declare_parameter("initial_orientation_std_deg", 5.0);
    declare_parameter("gyro_z_bias_radps", 0.0);
    declare_parameter("enable_startup_gyro_bias_calibration", true);
    declare_parameter("startup_calibration_duration_sec", 2.0);
    declare_parameter("startup_calibration_min_samples", 100);
    declare_parameter("startup_calibration_max_abs_gyro_z_radps", 0.05);
    declare_parameter("startup_calibration_max_gyro_z_stddev_radps", 0.01);
    declare_parameter("require_stationary_for_startup_calibration", true);

    board_frame_ = get_parameter("board_frame").as_string();
    leader_rear_frame_ = get_parameter("leader_rear_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    camera_frame_ = get_parameter("camera_frame").as_string();
    publish_tf_ = get_parameter("publish_tf").as_bool();
    reset_timeout_sec_ = get_parameter("reset_timeout_sec").as_double();
    vision_delay_buffer_sec_ = get_parameter("vision_delay_buffer_sec").as_double();
    aruco_position_gate_m_ = get_parameter("aruco_position_gate_m").as_double();
    aruco_position_covariance_scale_ = std::clamp(
      get_parameter("aruco_position_covariance_scale").as_double(), 0.02, 100.0);
    fixed_gyro_z_bias_radps_ = get_parameter("gyro_z_bias_radps").as_double();
    enable_startup_gyro_bias_calibration_ =
      get_parameter("enable_startup_gyro_bias_calibration").as_bool();
    startup_calibration_duration_sec_ =
      std::max(get_parameter("startup_calibration_duration_sec").as_double(), 0.1);
    startup_calibration_min_samples_ = static_cast<int>(
      std::max<int64_t>(get_parameter("startup_calibration_min_samples").as_int(), 1));
    startup_calibration_max_abs_gyro_z_radps_ =
      std::max(get_parameter("startup_calibration_max_abs_gyro_z_radps").as_double(), 0.0);
    startup_calibration_max_gyro_z_stddev_radps_ =
      std::max(get_parameter("startup_calibration_max_gyro_z_stddev_radps").as_double(), 0.0);
    require_stationary_for_startup_calibration_ =
      get_parameter("require_stationary_for_startup_calibration").as_bool();
    gyro_bias_valid_ = !enable_startup_gyro_bias_calibration_;
    calibration_status_ = enable_startup_gyro_bias_calibration_ ? "collecting" : "disabled";

    GyroRelativeEskfOptions filter_options;
    filter_options.position_smoothing_alpha = get_parameter("position_smoothing_alpha").as_double();
    filter_options.gyro_noise_std_radps = get_parameter("gyro_noise_std_radps").as_double();
    filter_options.initial_position_std_m = get_parameter("initial_position_std_m").as_double();
    filter_options.initial_orientation_std_deg = get_parameter("initial_orientation_std_deg").as_double();
    filter_ = std::make_unique<GyroRelativeEskf>(filter_options);
    filter_options_ = filter_options;

    board_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      get_parameter("odom_topic").as_string(), 10);
    board_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      get_parameter("pose_topic").as_string(), 10);
    leader_rear_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      get_parameter("leader_rear_odom_topic").as_string(), 10);
    leader_rear_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      get_parameter("leader_rear_pose_topic").as_string(), 10);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      get_parameter("diagnostics_topic").as_string(), 10);

    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
      publish_static_board_to_leader_rear();
    }

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      get_parameter("imu_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {imu_callback(msg);});
    board_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      get_parameter("board_pose_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {board_pose_callback(msg);});

    const double output_rate = std::max(get_parameter("output_rate_hz").as_double(), 1.0);
    output_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / output_rate),
      [this]() {publish_timer_callback();});
    diagnostics_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {publish_diagnostics();});

    RCLCPP_INFO(get_logger(), "C++ ArUco+IMU ESKF running with translation-only ArUco updates");
    RCLCPP_INFO(
      get_logger(),
      "startup gyro-z calibration: enabled=%s manual_bias=%.6f duration=%.2fs min_samples=%d "
      "max_abs=%.4f max_stddev=%.4f require_stationary=%s",
      enable_startup_gyro_bias_calibration_ ? "true" : "false",
      fixed_gyro_z_bias_radps_,
      startup_calibration_duration_sec_,
      startup_calibration_min_samples_,
      startup_calibration_max_abs_gyro_z_radps_,
      startup_calibration_max_gyro_z_stddev_radps_,
      require_stationary_for_startup_calibration_ ? "true" : "false");
  }

private:
  std::optional<Eigen::Matrix3d> rotation_base_from_imu(const std::string & imu_frame)
  {
    const std::string frame = imu_frame.empty() ? base_frame_ : imu_frame;
    if (cached_imu_frame_ == frame && cached_rotation_base_from_imu_) {
      return cached_rotation_base_from_imu_;
    }
    if (frame == base_frame_) {
      cached_imu_frame_ = frame;
      cached_rotation_base_from_imu_ = Eigen::Matrix3d::Identity();
      return cached_rotation_base_from_imu_;
    }
    try {
      const auto tf = tf_buffer_.lookupTransform(base_frame_, frame, tf2::TimePointZero);
      cached_imu_frame_ = frame;
      cached_rotation_base_from_imu_ = transform_from_msg(tf).linear();
      return cached_rotation_base_from_imu_;
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "IMU TF %s <- %s unavailable: %s",
        base_frame_.c_str(), frame.c_str(), e.what());
      return Eigen::Matrix3d::Identity();
    }
  }

  std::optional<Eigen::Isometry3d> camera_from_base()
  {
    if (cached_camera_from_base_) {
      return cached_camera_from_base_;
    }
    try {
      const auto tf = tf_buffer_.lookupTransform(base_frame_, camera_frame_, tf2::TimePointZero);
      cached_camera_from_base_ = transform_from_msg(tf).inverse();
      return cached_camera_from_base_;
    } catch (const std::exception & e) {
      diag_.last_skip_reason = "camera_tf_unavailable";
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "camera TF %s <- %s unavailable: %s",
        base_frame_.c_str(), camera_frame_.c_str(), e.what());
      return std::nullopt;
    }
  }

  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    const int64_t ns = stamp_ns(msg->header.stamp);
    const auto rotation = rotation_base_from_imu(msg->header.frame_id);
    Eigen::Vector3d omega(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    omega = (*rotation) * omega;
    const double raw_gyro_z = omega.z();
    update_startup_gyro_z_calibration(ns, raw_gyro_z);
    omega.z() = raw_gyro_z - fixed_gyro_z_bias_radps_;
    diag_.raw_gyro_z_radps = raw_gyro_z;
    diag_.corrected_gyro_z_radps = omega.z();
    diag_.gyro_z_bias_radps = fixed_gyro_z_bias_radps_;
    diag_.gyro_bias_valid = gyro_bias_valid_;
    diag_.stationary_detected = stationary_detected_;
    diag_.calibration_status = calibration_status_;

    const auto & q = msg->orientation;
    if (std::abs(q.x) + std::abs(q.y) + std::abs(q.z) + std::abs(q.w) > 1.0e-9) {
      diag_.imu_yaw_reference_only_rad =
        yaw_from_quaternion(normalize_quaternion(q.x, q.y, q.z, q.w));
      diag_.imu_orientation_valid = true;
    }

    ImuSample sample{ns, omega};
    imu_samples_.push_back(sample);
    if (last_imu_sample_ && ns > last_imu_sample_->stamp_ns) {
      ImuInterval interval{last_imu_sample_->stamp_ns, ns, last_imu_sample_->angular_velocity_base};
      imu_intervals_.push_back(interval);
      if (filter_->initialized() && filter_->stamp_ns() && interval.end_ns > *filter_->stamp_ns()) {
        filter_->predict(interval.end_ns, interval.angular_velocity_base);
        record_snapshot();
      }
    }
    last_imu_sample_ = sample;
    prune_history(ns);
  }

  void update_startup_gyro_z_calibration(int64_t ns, double raw_gyro_z)
  {
    if (!enable_startup_gyro_bias_calibration_ || gyro_bias_valid_) {
      return;
    }
    if (!std::isfinite(raw_gyro_z)) {
      startup_gyro_z_samples_.clear();
      startup_calibration_start_ns_.reset();
      stationary_detected_ = false;
      calibration_status_ = "failed_motion_detected";
      return;
    }
    if (
      require_stationary_for_startup_calibration_ &&
      std::abs(raw_gyro_z) > startup_calibration_max_abs_gyro_z_radps_)
    {
      startup_gyro_z_samples_.clear();
      startup_calibration_start_ns_.reset();
      stationary_detected_ = false;
      calibration_status_ = "waiting_stationary";
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "startup gyro-z calibration waiting for stationary IMU: raw_gyro_z=%.6f max_abs=%.6f",
        raw_gyro_z, startup_calibration_max_abs_gyro_z_radps_);
      return;
    }

    if (!startup_calibration_start_ns_) {
      startup_calibration_start_ns_ = ns;
    }
    startup_gyro_z_samples_.push_back(raw_gyro_z);
    stationary_detected_ = true;
    calibration_status_ = "collecting";

    const double elapsed_sec =
      static_cast<double>(ns - *startup_calibration_start_ns_) * 1.0e-9;
    if (
      elapsed_sec < startup_calibration_duration_sec_ ||
      static_cast<int>(startup_gyro_z_samples_.size()) < startup_calibration_min_samples_)
    {
      return;
    }

    double sum = 0.0;
    double sum_sq = 0.0;
    for (const double sample : startup_gyro_z_samples_) {
      sum += sample;
      sum_sq += sample * sample;
    }
    const double count = static_cast<double>(startup_gyro_z_samples_.size());
    const double mean = sum / count;
    const double variance = std::max(0.0, (sum_sq / count) - (mean * mean));
    const double stddev = std::sqrt(variance);
    if (
      require_stationary_for_startup_calibration_ &&
      startup_calibration_max_gyro_z_stddev_radps_ > 0.0 &&
      stddev > startup_calibration_max_gyro_z_stddev_radps_)
    {
      startup_gyro_z_samples_.clear();
      startup_calibration_start_ns_.reset();
      stationary_detected_ = false;
      calibration_status_ = "failed_motion_detected";
      RCLCPP_WARN(
        get_logger(),
        "startup gyro-z calibration rejected window: stddev=%.6f max_stddev=%.6f samples=%zu",
        stddev, startup_calibration_max_gyro_z_stddev_radps_, startup_gyro_z_samples_.size());
      return;
    }

    fixed_gyro_z_bias_radps_ = mean;
    gyro_bias_valid_ = true;
    stationary_detected_ = true;
    calibration_status_ = "valid";
    RCLCPP_INFO(
      get_logger(),
      "startup gyro-z calibration valid: bias=%.6f rad/s stddev=%.6f samples=%zu duration=%.2fs",
      fixed_gyro_z_bias_radps_, stddev, startup_gyro_z_samples_.size(), elapsed_sec);
  }

  void board_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    const auto cam_from_base = camera_from_base();
    if (!cam_from_base) {
      return;
    }
    const int64_t ns = stamp_ns(msg->header.stamp);
    if (last_vision_stamp_ns_ && ns < *last_vision_stamp_ns_) {
      diag_.aruco_update_applied = false;
      diag_.aruco_update_reason = "out_of_order";
      diag_.last_skip_reason = "aruco_out_of_order";
      return;
    }

    Eigen::Isometry3d board_from_camera = Eigen::Isometry3d::Identity();
    const auto & p = msg->pose.pose.position;
    const auto & q = msg->pose.pose.orientation;
    board_from_camera.translation() = Eigen::Vector3d(p.x, p.y, p.z);
    board_from_camera.linear() = normalize_quaternion(q.x, q.y, q.z, q.w).toRotationMatrix();
    const Eigen::Isometry3d board_from_base = board_from_camera * (*cam_from_base);
    const Eigen::Isometry3d leader_from_base = transform_leader_rear_from_board() * board_from_base;
    const Eigen::Matrix<double, 6, 6> leader_cov =
      transform_pose_covariance(covariance_from_msg(*msg), transform_leader_rear_from_board());
    Eigen::Matrix3d pos_cov = leader_cov.block<3, 3>(0, 0);
    pos_cov *= aruco_position_covariance_scale_;

    if (
      !filter_->initialized() || !last_vision_stamp_ns_ ||
      (ns - *last_vision_stamp_ns_) > static_cast<int64_t>(reset_timeout_sec_ * 1.0e9))
    {
      const auto replay_target = filter_->stamp_ns();
      filter_->initialize_position_only(
        ns, leader_from_base.translation(), pos_cov);
      record_snapshot();
      if (replay_target && *replay_target > ns) {
        predict_filter_to_stamp(*replay_target);
        record_snapshot();
      }
      last_vision_stamp_ns_ = ns;
      diag_.aruco_update_applied = true;
      diag_.aruco_update_reason = "initialized_translation_only";
      diag_.aruco_position_innovation_m = 0.0;
      diag_.aruco_position_gate_m = aruco_position_gate_m_;
      diag_.aruco_position_covariance_scale = aruco_position_covariance_scale_;
      diag_.aruco_rotation_innovation_deg = 0.0;
      prune_history(std::max(ns, replay_target.value_or(ns)));
      return;
    }

    PositionUpdateResult update;
    const auto replay_target = filter_->stamp_ns();
    if (replay_target && ns < *replay_target) {
      const auto current = filter_->snapshot();
      if (!restore_snapshot_before(ns)) {
        diag_.aruco_update_applied = false;
        diag_.aruco_update_reason = "outside_history";
        diag_.last_skip_reason = "aruco_outside_history";
        return;
      }
      predict_filter_to_stamp(ns);
      update = filter_->update_position(leader_from_base.translation(), pos_cov, aruco_position_gate_m_);
      if (!update.accepted) {
        filter_->restore(current);
      } else {
        record_snapshot();
        predict_filter_to_stamp(*replay_target);
        record_snapshot();
      }
    } else {
      predict_filter_to_stamp(ns);
      update = filter_->update_position(leader_from_base.translation(), pos_cov, aruco_position_gate_m_);
      if (update.accepted) {
        record_snapshot();
      }
    }

    last_vision_stamp_ns_ = ns;
    diag_.aruco_update_applied = update.accepted;
    diag_.aruco_update_reason = update.reason;
    diag_.aruco_position_innovation_m = update.position_innovation_m;
    diag_.aruco_position_gate_m = aruco_position_gate_m_;
    diag_.aruco_position_covariance_scale = aruco_position_covariance_scale_;
    diag_.aruco_rotation_innovation_deg = 0.0;
    if (!update.accepted) {
      diag_.last_skip_reason = "aruco_" + update.reason;
    }
    prune_history(std::max(ns, filter_->stamp_ns().value_or(ns)));
  }

  void predict_filter_to_stamp(int64_t target_ns)
  {
    if (!filter_->initialized() || !filter_->stamp_ns() || target_ns <= *filter_->stamp_ns()) {
      return;
    }
    for (const auto & interval : imu_intervals_) {
      if (interval.end_ns <= *filter_->stamp_ns()) {
        continue;
      }
      if (interval.start_ns >= target_ns) {
        break;
      }
      filter_->predict(std::min(interval.end_ns, target_ns), interval.angular_velocity_base);
      if (filter_->stamp_ns() && *filter_->stamp_ns() >= target_ns) {
        return;
      }
    }
    if (last_imu_sample_ && filter_->stamp_ns() && *filter_->stamp_ns() < target_ns) {
      filter_->predict(target_ns, last_imu_sample_->angular_velocity_base);
    }
  }

  void record_snapshot()
  {
    if (!filter_->initialized() || !filter_->stamp_ns()) {
      return;
    }
    auto snapshot = filter_->snapshot();
    if (!state_history_.empty() && state_history_.back().stamp_ns == snapshot.stamp_ns) {
      state_history_.pop_back();
    }
    state_history_.push_back(snapshot);
  }

  bool restore_snapshot_before(int64_t target_ns)
  {
    for (auto it = state_history_.rbegin(); it != state_history_.rend(); ++it) {
      if (it->stamp_ns && *it->stamp_ns <= target_ns) {
        filter_->restore(*it);
        return true;
      }
    }
    return false;
  }

  void prune_history(int64_t reference_ns)
  {
    const int64_t keep_after = reference_ns - static_cast<int64_t>(std::max(vision_delay_buffer_sec_, 0.0) * 1.0e9);
    while (!imu_samples_.empty() && imu_samples_.front().stamp_ns < keep_after) {
      imu_samples_.pop_front();
    }
    while (!imu_intervals_.empty() && imu_intervals_.front().end_ns < keep_after) {
      imu_intervals_.pop_front();
    }
    while (!state_history_.empty() && state_history_.front().stamp_ns && *state_history_.front().stamp_ns < keep_after) {
      state_history_.pop_front();
    }
  }

  void publish_timer_callback()
  {
    if (!filter_->initialized()) {
      return;
    }
    GyroRelativeEskf extrapolated(filter_options_);
    extrapolated.restore(filter_->snapshot());
    const int64_t now_ns = now().nanoseconds();
    if (last_imu_sample_ && extrapolated.stamp_ns() && now_ns > *extrapolated.stamp_ns()) {
      extrapolated.predict(now_ns, last_imu_sample_->angular_velocity_base);
    }

    const Eigen::Isometry3d leader_from_base = extrapolated.pose_matrix();
    const Eigen::Matrix<double, 6, 6> leader_cov = extrapolated.pose_covariance();
    const Eigen::Isometry3d board_from_base = transform_board_from_leader_rear() * leader_from_base;
    const Eigen::Matrix<double, 6, 6> board_cov =
      transform_pose_covariance(leader_cov, transform_board_from_leader_rear());

    const auto stamp = time_msg_from_ns(now_ns);
    publish_outputs(
      stamp, board_from_base, board_cov, leader_from_base, leader_cov,
      extrapolated.linear_velocity_base_mps(), extrapolated.angular_velocity_base_radps());
    diag_.filter_yaw_rad = yaw_from_quaternion(Eigen::Quaterniond(leader_from_base.linear()));
  }

  nav_msgs::msg::Odometry make_odom(
    const builtin_interfaces::msg::Time & stamp,
    const std::string & frame,
    const std::string & child,
    const Eigen::Isometry3d & pose,
    const Eigen::Matrix<double, 6, 6> & cov,
    const Eigen::Vector3d & linear_velocity_base,
    const Eigen::Vector3d & angular_velocity_base)
  {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame;
    msg.child_frame_id = child;
    msg.pose.pose.position.x = pose.translation().x();
    msg.pose.pose.position.y = pose.translation().y();
    msg.pose.pose.position.z = pose.translation().z();
    msg.pose.pose.orientation = quaternion_msg_from_eigen(Eigen::Quaterniond(pose.linear()));
    for (int r = 0; r < 6; ++r) {
      for (int c = 0; c < 6; ++c) {
        msg.pose.covariance[r * 6 + c] = cov(r, c);
      }
    }
    msg.twist.twist.linear.x = linear_velocity_base.x();
    msg.twist.twist.linear.y = linear_velocity_base.y();
    msg.twist.twist.linear.z = linear_velocity_base.z();
    msg.twist.twist.angular.x = angular_velocity_base.x();
    msg.twist.twist.angular.y = angular_velocity_base.y();
    msg.twist.twist.angular.z = angular_velocity_base.z();
    return msg;
  }

  void publish_outputs(
    const builtin_interfaces::msg::Time & stamp,
    const Eigen::Isometry3d & board_from_base,
    const Eigen::Matrix<double, 6, 6> & board_cov,
    const Eigen::Isometry3d & leader_from_base,
    const Eigen::Matrix<double, 6, 6> & leader_cov,
    const Eigen::Vector3d & linear_velocity_base,
    const Eigen::Vector3d & angular_velocity_base)
  {
    const auto board_odom = make_odom(
      stamp, board_frame_, base_frame_, board_from_base, board_cov,
      linear_velocity_base, angular_velocity_base);
    board_odom_pub_->publish(board_odom);
    geometry_msgs::msg::PoseWithCovarianceStamped board_pose;
    board_pose.header = board_odom.header;
    board_pose.pose = board_odom.pose;
    board_pose_pub_->publish(board_pose);

    const auto leader_odom = make_odom(
      stamp, leader_rear_frame_, base_frame_, leader_from_base, leader_cov,
      linear_velocity_base, angular_velocity_base);
    leader_rear_odom_pub_->publish(leader_odom);
    geometry_msgs::msg::PoseWithCovarianceStamped leader_pose;
    leader_pose.header = leader_odom.header;
    leader_pose.pose = leader_odom.pose;
    leader_rear_pose_pub_->publish(leader_pose);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header = leader_odom.header;
      tf_msg.child_frame_id = base_frame_;
      tf_msg.transform.translation.x = leader_from_base.translation().x();
      tf_msg.transform.translation.y = leader_from_base.translation().y();
      tf_msg.transform.translation.z = leader_from_base.translation().z();
      tf_msg.transform.rotation = leader_odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf_msg);
    }
  }

  void publish_static_board_to_leader_rear()
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = now();
    tf_msg.header.frame_id = board_frame_;
    tf_msg.child_frame_id = leader_rear_frame_;
    const auto tf = transform_board_from_leader_rear();
    tf_msg.transform.rotation = quaternion_msg_from_eigen(Eigen::Quaterniond(tf.linear()));
    static_tf_broadcaster_->sendTransform(tf_msg);
  }

  void publish_diagnostics()
  {
    diag_.eskf_initialized = filter_->initialized();
    diag_.aruco_position_gate_m = aruco_position_gate_m_;
    diag_.aruco_position_covariance_scale = aruco_position_covariance_scale_;
    diag_.gyro_z_bias_radps = fixed_gyro_z_bias_radps_;
    diag_.gyro_bias_valid = gyro_bias_valid_;
    diag_.stationary_detected = stationary_detected_;
    diag_.calibration_status = calibration_status_;
    diagnostics_pub_->publish(diagnostics_builder_.build(now(), diag_));
  }

  GyroRelativeEskfOptions filter_options_;
  std::unique_ptr<GyroRelativeEskf> filter_;
  DiagnosticsPublisher diagnostics_builder_;
  ArucoImuEskfDiagnostics diag_;

  std::string board_frame_;
  std::string leader_rear_frame_;
  std::string base_frame_;
  std::string camera_frame_;
  bool publish_tf_{true};
  double reset_timeout_sec_{1.0};
  double vision_delay_buffer_sec_{2.0};
  double aruco_position_gate_m_{1.0};
  double aruco_position_covariance_scale_{0.35};
  double fixed_gyro_z_bias_radps_{0.0};
  bool enable_startup_gyro_bias_calibration_{true};
  double startup_calibration_duration_sec_{2.0};
  int startup_calibration_min_samples_{100};
  double startup_calibration_max_abs_gyro_z_radps_{0.05};
  double startup_calibration_max_gyro_z_stddev_radps_{0.01};
  bool require_stationary_for_startup_calibration_{true};
  bool gyro_bias_valid_{false};
  bool stationary_detected_{false};
  std::string calibration_status_{"collecting"};
  std::deque<double> startup_gyro_z_samples_;
  std::optional<int64_t> startup_calibration_start_ns_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr board_pose_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr board_odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr board_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr leader_rear_odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr leader_rear_pose_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr output_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  std::deque<ImuSample> imu_samples_;
  std::deque<ImuInterval> imu_intervals_;
  std::deque<GyroRelativeEskfSnapshot> state_history_;
  std::optional<ImuSample> last_imu_sample_;
  std::optional<int64_t> last_vision_stamp_ns_;
  std::optional<Eigen::Isometry3d> cached_camera_from_base_;
  std::string cached_imu_frame_;
  std::optional<Eigen::Matrix3d> cached_rotation_base_from_imu_;
};

}  // namespace aruco_imu_eskf_localization_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aruco_imu_eskf_localization_cpp::RelativeLocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
