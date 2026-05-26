#include "relative_localization_eskf/diagnostics_publisher.hpp"
#include "relative_localization_eskf/geometry.hpp"
#include "relative_localization_eskf/gyro_relative_eskf.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <kiss_icp/pipeline/KissICP.hpp>
#include <sophus/se3.hpp>

#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace relative_localization_eskf
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

double wrap_to_pi(double angle_rad)
{
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

double yaw_from_rotation(const Eigen::Matrix3d & rotation)
{
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

Eigen::Isometry3d odom_pose_matrix(const nav_msgs::msg::Odometry & msg)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  const auto & p = msg.pose.pose.position;
  const auto & q = msg.pose.pose.orientation;
  pose.translation() = Eigen::Vector3d(p.x, p.y, p.z);
  pose.linear() = normalize_quaternion(q.x, q.y, q.z, q.w).toRotationMatrix();
  return pose;
}

Eigen::Matrix<double, 6, 6> covariance_from_odom(const nav_msgs::msg::Odometry & msg)
{
  Eigen::Matrix<double, 6, 6> cov;
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) {
      cov(r, c) = msg.pose.covariance[r * 6 + c];
    }
  }
  return cov;
}

double covariance_value(const Eigen::Matrix<double, 6, 6> & cov, int row, int col, double fallback)
{
  const double value = cov(row, col);
  return std::isfinite(value) && value >= 0.0 ? value : fallback;
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
    node_start_ns_ = now().nanoseconds();
    declare_parameter("imu_topic", "imu");
    declare_parameter("board_pose_topic", "localization/aruco/board_pose");
    declare_parameter("odom_topic", "localization/relative/odom");
    declare_parameter("pose_topic", "localization/relative/pose");
    declare_parameter("leader_base_odom_topic", "localization/leader_base/odom");
    declare_parameter("leader_base_pose_topic", "localization/leader_base/pose");
    declare_parameter("diagnostics_topic", "diagnostics");
    declare_parameter("lidar_scan_topic", "/follower/scan");
    declare_parameter("board_frame", "leader/board");
    declare_parameter("leader_base_frame", "leader/base_link");
    declare_parameter("leader_rear_frame", "leader/leader_rear");
    declare_parameter("base_frame", "follower/base_link");
    declare_parameter("camera_frame", "follower/usb_cam");
    declare_parameter("publish_tf", true);
    declare_parameter("output_rate_hz", 100.0);
    declare_parameter("reset_timeout_sec", 1.0);
    declare_parameter("vision_delay_buffer_sec", 2.0);
    declare_parameter("aruco_position_gate_m", 0.75);
    declare_parameter("aruco_position_covariance_scale", 2.0);
    declare_parameter("enable_aruco_yaw_update", true);
    declare_parameter("aruco_yaw_gate_deg", 20.0);
    declare_parameter("aruco_yaw_covariance_scale", 2.0);
    declare_parameter("aruco_position_mahalanobis_gate", 3.0);
    declare_parameter("aruco_min_position_variance_m2", 0.0100);
    declare_parameter("aruco_min_yaw_variance_rad2", 0.0100);
    declare_parameter("aruco_max_position_variance_m2", 2.0);
    declare_parameter("aruco_max_yaw_variance_rad2", 0.50);
    declare_parameter("aruco_reinitialize_position_error_m", 0.30);
    declare_parameter("aruco_reinitialize_min_consecutive", 3);
    declare_parameter("position_smoothing_alpha", 0.55);
    declare_parameter("filter_min_position_variance_m2", 0.0025);
    declare_parameter("gyro_noise_std_radps", 0.05);
    declare_parameter("gyro_bias_noise_std_radps", 0.002);
    declare_parameter("initial_gyro_bias_std_radps", 0.02);
    declare_parameter("max_abs_gyro_bias_radps", 0.08);
    declare_parameter("initial_position_std_m", 0.20);
    declare_parameter("initial_orientation_std_deg", 5.0);
    declare_parameter("gyro_z_bias_radps", 0.0);
    declare_parameter("enable_startup_gyro_bias_calibration", true);
    declare_parameter("startup_calibration_duration_sec", 2.0);
    declare_parameter("startup_calibration_min_samples", 100);
    declare_parameter("startup_calibration_max_abs_gyro_z_radps", 0.05);
    declare_parameter("startup_calibration_max_gyro_z_stddev_radps", 0.01);
    declare_parameter("require_stationary_for_startup_calibration", true);
    declare_parameter("enable_lidar_icp_yaw", false);
    declare_parameter("lidar_icp_min_range_m", 0.15);
    declare_parameter("lidar_icp_max_range_m", 8.0);
    declare_parameter("lidar_icp_min_source_points", 45);
    declare_parameter("lidar_icp_yaw_var_rad2", 0.01);
    declare_parameter("lidar_icp_yaw_gate_deg", 8.0);
    declare_parameter("lidar_icp_max_abs_yaw_rate_radps", 1.5);
    declare_parameter("lidar_icp_voxel_size_m", 0.10);
    declare_parameter("lidar_icp_max_points_per_voxel", 20);
    declare_parameter("lidar_icp_max_num_iterations", 80);
    declare_parameter("lidar_icp_convergence_criterion", 0.0001);
    declare_parameter("lidar_icp_exclude_leader_box", true);
    declare_parameter("lidar_icp_exclude_x_min_m", 0.10);
    declare_parameter("lidar_icp_exclude_x_max_m", 1.20);
    declare_parameter("lidar_icp_exclude_abs_y_max_m", 0.45);
    declare_parameter("enable_lidar_icp_yaw_recovery", true);
    declare_parameter("lidar_icp_recovery_min_yaw_gate_rejects", 5);
    declare_parameter("lidar_icp_recovery_min_stable_samples", 5);
    declare_parameter("lidar_icp_recovery_yaw_gate_deg", 45.0);
    declare_parameter("lidar_icp_recovery_max_step_deg", 3.0);
    declare_parameter("lidar_icp_recovery_yaw_var_scale", 8.0);
    declare_parameter("lidar_icp_recovery_max_yaw_rate_delta_radps", 0.35);
    declare_parameter("camera_tf_startup_grace_sec", 2.0);
    declare_parameter("leader_base_to_rear_x_m", -0.275);
    declare_parameter("leader_base_to_rear_y_m", 0.0);
    declare_parameter("leader_base_to_rear_z_m", 0.0525);
    declare_parameter("enable_lidar_wheel_pose_update", true);
    declare_parameter("lidar_wheel_odom_topic", "/follower/localization/lidar_wheels/leader_base_detection");
    declare_parameter("lidar_wheel_pose_gate_m", 0.12);
    declare_parameter("lidar_wheel_yaw_gate_deg", 8.0);
    declare_parameter("enable_lidar_wheel_yaw_update", false);
    declare_parameter("allow_lidar_wheel_initialization", false);
    declare_parameter("lidar_wheel_position_mahalanobis_gate", 2.0);
    declare_parameter("lidar_wheel_position_covariance_scale", 6.0);
    declare_parameter("lidar_wheel_yaw_covariance_scale", 10.0);
    declare_parameter("lidar_wheel_min_position_variance_m2", 0.0100);
    declare_parameter("lidar_wheel_min_yaw_variance_rad2", 0.0300);
    declare_parameter("lidar_wheel_max_position_variance_m2", 0.0200);
    declare_parameter("lidar_wheel_max_yaw_variance_rad2", 0.0600);
    declare_parameter("enable_gps_pose_update", true);
    declare_parameter("gps_leader_odom_topic", "/v2v/leader/odom");
    declare_parameter("gps_follower_odom_topic", "/follower/localization/global/odom");
    declare_parameter("gps_pose_gate_m", 1.00);
    declare_parameter("gps_yaw_gate_deg", 25.0);
    declare_parameter("gps_odom_timeout_sec", 0.50);
    declare_parameter("gps_position_mahalanobis_gate", 4.0);
    declare_parameter("gps_position_covariance_scale", 1.0);
    declare_parameter("gps_yaw_covariance_scale", 1.0);
    declare_parameter("gps_min_position_variance_m2", 0.0025);
    declare_parameter("gps_min_yaw_variance_rad2", 0.0100);
    declare_parameter("gps_max_position_variance_m2", 1.0);
    declare_parameter("gps_max_yaw_variance_rad2", 0.50);

    board_frame_ = get_parameter("board_frame").as_string();
    leader_base_frame_ = get_parameter("leader_base_frame").as_string();
    leader_rear_frame_ = get_parameter("leader_rear_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    camera_frame_ = get_parameter("camera_frame").as_string();
    publish_tf_ = get_parameter("publish_tf").as_bool();
    reset_timeout_sec_ = get_parameter("reset_timeout_sec").as_double();
    vision_delay_buffer_sec_ = get_parameter("vision_delay_buffer_sec").as_double();
    aruco_position_gate_m_ = get_parameter("aruco_position_gate_m").as_double();
    aruco_position_covariance_scale_ = std::clamp(
      get_parameter("aruco_position_covariance_scale").as_double(), 0.02, 100.0);
    enable_aruco_yaw_update_ = get_parameter("enable_aruco_yaw_update").as_bool();
    aruco_yaw_gate_rad_ =
      std::max(get_parameter("aruco_yaw_gate_deg").as_double(), 0.0) * M_PI / 180.0;
    aruco_yaw_covariance_scale_ = std::clamp(
      get_parameter("aruco_yaw_covariance_scale").as_double(), 0.02, 100.0);
    aruco_position_mahalanobis_gate_ =
      std::max(get_parameter("aruco_position_mahalanobis_gate").as_double(), 0.0);
    aruco_min_position_variance_m2_ =
      std::max(get_parameter("aruco_min_position_variance_m2").as_double(), 1.0e-5);
    aruco_min_yaw_variance_rad2_ =
      std::max(get_parameter("aruco_min_yaw_variance_rad2").as_double(), 1.0e-9);
    aruco_max_position_variance_m2_ =
      std::max(get_parameter("aruco_max_position_variance_m2").as_double(), 0.0);
    aruco_max_yaw_variance_rad2_ =
      std::max(get_parameter("aruco_max_yaw_variance_rad2").as_double(), 0.0);
    aruco_reinitialize_position_error_m_ =
      std::max(get_parameter("aruco_reinitialize_position_error_m").as_double(), 0.0);
    aruco_reinitialize_min_consecutive_ = static_cast<int>(
      std::max<int64_t>(get_parameter("aruco_reinitialize_min_consecutive").as_int(), 1));
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
    enable_lidar_icp_yaw_ = get_parameter("enable_lidar_icp_yaw").as_bool();
    lidar_icp_min_range_m_ =
      std::max(get_parameter("lidar_icp_min_range_m").as_double(), 0.0);
    lidar_icp_max_range_m_ =
      std::max(get_parameter("lidar_icp_max_range_m").as_double(), lidar_icp_min_range_m_);
    lidar_icp_min_source_points_ =
      static_cast<int>(
        std::max<int64_t>(get_parameter("lidar_icp_min_source_points").as_int(), 1));
    lidar_icp_yaw_var_rad2_ =
      std::max(get_parameter("lidar_icp_yaw_var_rad2").as_double(), 1.0e-6);
    lidar_icp_yaw_gate_rad_ =
      std::max(get_parameter("lidar_icp_yaw_gate_deg").as_double(), 0.0) * M_PI / 180.0;
    lidar_icp_max_abs_yaw_rate_radps_ =
      std::max(get_parameter("lidar_icp_max_abs_yaw_rate_radps").as_double(), 0.0);
    lidar_icp_exclude_leader_box_ =
      get_parameter("lidar_icp_exclude_leader_box").as_bool();
    lidar_icp_exclude_x_min_m_ = get_parameter("lidar_icp_exclude_x_min_m").as_double();
    lidar_icp_exclude_x_max_m_ = get_parameter("lidar_icp_exclude_x_max_m").as_double();
    lidar_icp_exclude_abs_y_max_m_ =
      std::max(get_parameter("lidar_icp_exclude_abs_y_max_m").as_double(), 0.0);
    enable_lidar_icp_yaw_recovery_ =
      get_parameter("enable_lidar_icp_yaw_recovery").as_bool();
    lidar_icp_recovery_min_yaw_gate_rejects_ = static_cast<int>(
      std::max<int64_t>(get_parameter("lidar_icp_recovery_min_yaw_gate_rejects").as_int(), 1));
    lidar_icp_recovery_min_stable_samples_ = static_cast<int>(
      std::max<int64_t>(get_parameter("lidar_icp_recovery_min_stable_samples").as_int(), 1));
    lidar_icp_recovery_yaw_gate_rad_ =
      std::max(get_parameter("lidar_icp_recovery_yaw_gate_deg").as_double(), 0.0) *
      M_PI / 180.0;
    lidar_icp_recovery_max_step_rad_ =
      std::max(get_parameter("lidar_icp_recovery_max_step_deg").as_double(), 0.0) *
      M_PI / 180.0;
    lidar_icp_recovery_yaw_var_scale_ =
      std::max(get_parameter("lidar_icp_recovery_yaw_var_scale").as_double(), 1.0);
    lidar_icp_recovery_max_yaw_rate_delta_radps_ =
      std::max(get_parameter("lidar_icp_recovery_max_yaw_rate_delta_radps").as_double(), 0.0);
    camera_tf_startup_grace_sec_ =
      std::max(get_parameter("camera_tf_startup_grace_sec").as_double(), 0.0);
    leader_base_to_rear_x_m_ = get_parameter("leader_base_to_rear_x_m").as_double();
    leader_base_to_rear_y_m_ = get_parameter("leader_base_to_rear_y_m").as_double();
    leader_base_to_rear_z_m_ = get_parameter("leader_base_to_rear_z_m").as_double();
    enable_lidar_wheel_pose_update_ = get_parameter("enable_lidar_wheel_pose_update").as_bool();
    lidar_wheel_pose_gate_m_ = std::max(get_parameter("lidar_wheel_pose_gate_m").as_double(), 0.0);
    lidar_wheel_yaw_gate_rad_ =
      std::max(get_parameter("lidar_wheel_yaw_gate_deg").as_double(), 0.0) * M_PI / 180.0;
    enable_lidar_wheel_yaw_update_ = get_parameter("enable_lidar_wheel_yaw_update").as_bool();
    allow_lidar_wheel_initialization_ = get_parameter("allow_lidar_wheel_initialization").as_bool();
    lidar_wheel_position_mahalanobis_gate_ =
      std::max(get_parameter("lidar_wheel_position_mahalanobis_gate").as_double(), 0.0);
    lidar_wheel_position_covariance_scale_ =
      std::max(get_parameter("lidar_wheel_position_covariance_scale").as_double(), 1.0e-3);
    lidar_wheel_yaw_covariance_scale_ =
      std::max(get_parameter("lidar_wheel_yaw_covariance_scale").as_double(), 1.0e-3);
    lidar_wheel_min_position_variance_m2_ =
      std::max(get_parameter("lidar_wheel_min_position_variance_m2").as_double(), 1.0e-5);
    lidar_wheel_min_yaw_variance_rad2_ =
      std::max(get_parameter("lidar_wheel_min_yaw_variance_rad2").as_double(), 1.0e-9);
    lidar_wheel_max_position_variance_m2_ =
      std::max(get_parameter("lidar_wheel_max_position_variance_m2").as_double(), 0.0);
    lidar_wheel_max_yaw_variance_rad2_ =
      std::max(get_parameter("lidar_wheel_max_yaw_variance_rad2").as_double(), 0.0);
    enable_gps_pose_update_ = get_parameter("enable_gps_pose_update").as_bool();
    gps_pose_gate_m_ = std::max(get_parameter("gps_pose_gate_m").as_double(), 0.0);
    gps_yaw_gate_rad_ =
      std::max(get_parameter("gps_yaw_gate_deg").as_double(), 0.0) * M_PI / 180.0;
    gps_odom_timeout_sec_ = std::max(get_parameter("gps_odom_timeout_sec").as_double(), 0.0);
    gps_position_mahalanobis_gate_ =
      std::max(get_parameter("gps_position_mahalanobis_gate").as_double(), 0.0);
    gps_position_covariance_scale_ =
      std::max(get_parameter("gps_position_covariance_scale").as_double(), 1.0e-3);
    gps_yaw_covariance_scale_ =
      std::max(get_parameter("gps_yaw_covariance_scale").as_double(), 1.0e-3);
    gps_min_position_variance_m2_ =
      std::max(get_parameter("gps_min_position_variance_m2").as_double(), 1.0e-5);
    gps_min_yaw_variance_rad2_ =
      std::max(get_parameter("gps_min_yaw_variance_rad2").as_double(), 1.0e-9);
    gps_max_position_variance_m2_ =
      std::max(get_parameter("gps_max_position_variance_m2").as_double(), 0.0);
    gps_max_yaw_variance_rad2_ =
      std::max(get_parameter("gps_max_yaw_variance_rad2").as_double(), 0.0);

    GyroRelativeEskfOptions filter_options;
    filter_options.position_smoothing_alpha = get_parameter("position_smoothing_alpha").as_double();
    filter_options.min_position_variance =
      std::max(get_parameter("filter_min_position_variance_m2").as_double(), 1.0e-6);
    filter_options.gyro_noise_std_radps = get_parameter("gyro_noise_std_radps").as_double();
    filter_options.gyro_bias_noise_std_radps =
      get_parameter("gyro_bias_noise_std_radps").as_double();
    filter_options.initial_gyro_bias_std_radps =
      get_parameter("initial_gyro_bias_std_radps").as_double();
    filter_options.max_abs_gyro_bias_radps =
      get_parameter("max_abs_gyro_bias_radps").as_double();
    filter_options.initial_position_std_m = get_parameter("initial_position_std_m").as_double();
    filter_options.initial_orientation_std_deg = get_parameter("initial_orientation_std_deg").as_double();
    filter_ = std::make_unique<GyroRelativeEskf>(filter_options);
    filter_options_ = filter_options;

    if (enable_lidar_icp_yaw_) {
      kiss_icp::pipeline::KISSConfig config;
      config.deskew = false;
      config.min_range = lidar_icp_min_range_m_;
      config.max_range = lidar_icp_max_range_m_;
      config.voxel_size = std::max(get_parameter("lidar_icp_voxel_size_m").as_double(), 0.01);
      config.max_points_per_voxel = static_cast<int>(
        std::max<int64_t>(get_parameter("lidar_icp_max_points_per_voxel").as_int(), 1));
      config.max_num_iterations = static_cast<int>(
        std::max<int64_t>(get_parameter("lidar_icp_max_num_iterations").as_int(), 1));
      config.convergence_criterion =
        std::max(get_parameter("lidar_icp_convergence_criterion").as_double(), 1.0e-8);
      lidar_icp_ = std::make_unique<kiss_icp::pipeline::KissICP>(config);
    }

    board_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      get_parameter("odom_topic").as_string(), 10);
    board_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      get_parameter("pose_topic").as_string(), 10);
    leader_base_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      get_parameter("leader_base_odom_topic").as_string(), 10);
    leader_base_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      get_parameter("leader_base_pose_topic").as_string(), 10);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      get_parameter("diagnostics_topic").as_string(), 10);

    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
      publish_static_leader_rear_to_board();
    }

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      get_parameter("imu_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {imu_callback(msg);});
    board_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      get_parameter("board_pose_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {board_pose_callback(msg);});
    if (enable_lidar_icp_yaw_) {
      lidar_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        get_parameter("lidar_scan_topic").as_string(), rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {lidar_scan_callback(msg);});
    }
    if (enable_lidar_wheel_pose_update_) {
      lidar_wheel_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        get_parameter("lidar_wheel_odom_topic").as_string(), rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {lidar_wheel_odom_callback(msg);});
    }
    if (enable_gps_pose_update_) {
      gps_leader_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        get_parameter("gps_leader_odom_topic").as_string(), rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {gps_leader_odom_callback(msg);});
      gps_follower_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        get_parameter("gps_follower_odom_topic").as_string(), rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {gps_follower_odom_callback(msg);});
    }

    const double output_rate = std::max(get_parameter("output_rate_hz").as_double(), 1.0);
    output_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / output_rate),
      [this]() {publish_timer_callback();});
    diagnostics_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {publish_diagnostics();});

    RCLCPP_INFO(
      get_logger(),
      "C++ ArUco+IMU ESKF running with ArUco/LiDAR/GPS pose updates and LiDAR ICP yaw=%s",
      enable_lidar_icp_yaw_ ? "enabled" : "disabled");
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
      const double age_sec =
        static_cast<double>(now().nanoseconds() - node_start_ns_) * 1.0e-9;
      diag_.last_skip_reason =
        age_sec <= camera_tf_startup_grace_sec_ ? "camera_tf_waiting" : "camera_tf_unavailable";
      if (age_sec <= camera_tf_startup_grace_sec_) {
        return std::nullopt;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "camera TF %s <- %s unavailable: %s",
        base_frame_.c_str(), camera_frame_.c_str(), e.what());
      return std::nullopt;
    }
  }

  std::optional<Eigen::Isometry3d> base_from_lidar(const std::string & lidar_frame)
  {
    const std::string frame = lidar_frame.empty() ? base_frame_ : lidar_frame;
    if (cached_lidar_frame_ == frame && cached_base_from_lidar_) {
      return cached_base_from_lidar_;
    }
    if (frame == base_frame_) {
      cached_lidar_frame_ = frame;
      cached_base_from_lidar_ = Eigen::Isometry3d::Identity();
      return cached_base_from_lidar_;
    }
    try {
      const auto tf = tf_buffer_.lookupTransform(base_frame_, frame, tf2::TimePointZero);
      cached_lidar_frame_ = frame;
      cached_base_from_lidar_ = transform_from_msg(tf);
      return cached_base_from_lidar_;
    } catch (const std::exception & e) {
      set_lidar_icp_skip_reason("lidar_tf_unavailable");
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "LiDAR TF %s <- %s unavailable: %s",
        base_frame_.c_str(), frame.c_str(), e.what());
      return std::nullopt;
    }
  }


  Eigen::Isometry3d leader_base_to_rear_fallback() const
  {
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation() = Eigen::Vector3d(
      leader_base_to_rear_x_m_, leader_base_to_rear_y_m_, leader_base_to_rear_z_m_);
    return tf;
  }

  Eigen::Isometry3d leader_base_from_leader_rear()
  {
    if (cached_leader_base_from_rear_) {
      return *cached_leader_base_from_rear_;
    }
    try {
      const auto tf = tf_buffer_.lookupTransform(
        leader_base_frame_, leader_rear_frame_, tf2::TimePointZero);
      cached_leader_base_from_rear_ = transform_from_msg(tf);
    } catch (const std::exception & e) {
      cached_leader_base_from_rear_ = leader_base_to_rear_fallback();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "leader TF %s <- %s unavailable, using configured fallback: %s",
        leader_base_frame_.c_str(), leader_rear_frame_.c_str(), e.what());
    }
    return *cached_leader_base_from_rear_;
  }

  Eigen::Isometry3d leader_rear_from_leader_base()
  {
    return leader_base_from_leader_rear().inverse();
  }

  std::pair<bool, std::string> apply_pose_measurement(
    int64_t ns,
    const Eigen::Isometry3d & leader_from_base,
    const Eigen::Matrix<double, 6, 6> & covariance,
    double position_gate_m,
    double position_mahalanobis_gate,
    double yaw_gate_rad,
    bool use_yaw,
    bool allow_initialize,
    double position_covariance_scale,
    double yaw_covariance_scale,
    double min_position_variance,
    double min_yaw_variance,
    double max_position_variance,
    double max_yaw_variance,
    const std::string & source_name)
  {
    const double raw_x_var = covariance_value(covariance, 0, 0, min_position_variance);
    const double raw_y_var = covariance_value(covariance, 1, 1, min_position_variance);
    const double raw_yaw_var = covariance_value(covariance, 5, 5, min_yaw_variance);
    const double raw_max_position_var = std::max(raw_x_var, raw_y_var);
    if (max_position_variance > 0.0 && raw_max_position_var > max_position_variance) {
      return {false, source_name + "_position_covariance_gate"};
    }
    if (use_yaw && max_yaw_variance > 0.0 && raw_yaw_var > max_yaw_variance) {
      return {false, source_name + "_yaw_covariance_gate"};
    }

    Eigen::Matrix3d pos_cov = covariance.block<3, 3>(0, 0);
    pos_cov(0, 0) = std::max(raw_x_var, min_position_variance);
    pos_cov(1, 1) = std::max(raw_y_var, min_position_variance);
    pos_cov(2, 2) = std::max(covariance_value(covariance, 2, 2, 1.0), min_position_variance);
    if (!std::isfinite(pos_cov(0, 1)) || !std::isfinite(pos_cov(1, 0))) {
      pos_cov(0, 1) = 0.0;
      pos_cov(1, 0) = 0.0;
    }
    pos_cov *= std::max(position_covariance_scale, 1.0e-3);
    const double yaw = yaw_from_rotation(leader_from_base.linear());
    const double yaw_var = std::max(raw_yaw_var, min_yaw_variance) *
      std::max(yaw_covariance_scale, 1.0e-3);

    if (!filter_->initialized()) {
      if (!allow_initialize) {
        return {false, source_name + "_init_blocked"};
      }
      if (use_yaw) {
        filter_->initialize_pose(ns, leader_from_base.translation(), pos_cov, yaw, yaw_var);
      } else {
        filter_->initialize_position_only(ns, leader_from_base.translation(), pos_cov);
      }
      record_snapshot();
      prune_history(ns);
      return {true, source_name + "_initialized"};
    }

    const Eigen::Vector2d residual = leader_from_base.translation().head<2>() -
      filter_->pose_matrix().translation().head<2>();
    if (position_mahalanobis_gate > 0.0) {
      const Eigen::Matrix2d innovation_cov =
        filter_->pose_covariance().block<2, 2>(0, 0) + pos_cov.block<2, 2>(0, 0);
      if (!innovation_cov.allFinite() || std::abs(innovation_cov.determinant()) < 1.0e-18) {
        return {false, source_name + "_invalid_innovation_covariance"};
      }
      const double nis = residual.transpose() * innovation_cov.inverse() * residual;
      if (!std::isfinite(nis) || nis < 0.0) {
        return {false, source_name + "_invalid_position_mahalanobis"};
      }
      if (std::sqrt(nis) > position_mahalanobis_gate) {
        return {false, source_name + "_position_mahalanobis_gate"};
      }
    }

    PositionUpdateResult pos_update;
    YawUpdateResult yaw_update;
    const auto replay_target = filter_->stamp_ns();
    if (replay_target && ns < *replay_target) {
      const auto current = filter_->snapshot();
      if (!restore_snapshot_before(ns)) {
        return {false, source_name + "_outside_history"};
      }
      predict_filter_to_stamp(ns);
      pos_update = filter_->update_position(leader_from_base.translation(), pos_cov, position_gate_m);
      if (!pos_update.accepted) {
        filter_->restore(current);
        return {false, source_name + "_" + pos_update.reason};
      }
      if (use_yaw) {
        yaw_update = filter_->update_yaw(yaw, yaw_var, yaw_gate_rad);
      }
      record_snapshot();
      predict_filter_to_stamp(*replay_target);
      record_snapshot();
    } else {
      predict_filter_to_stamp(ns);
      pos_update = filter_->update_position(leader_from_base.translation(), pos_cov, position_gate_m);
      if (!pos_update.accepted) {
        return {false, source_name + "_" + pos_update.reason};
      }
      if (use_yaw) {
        yaw_update = filter_->update_yaw(yaw, yaw_var, yaw_gate_rad);
      }
      record_snapshot();
    }

    prune_history(std::max(ns, filter_->stamp_ns().value_or(ns)));
    if (use_yaw && !yaw_update.accepted) {
      return {true, source_name + "_position_ok_yaw_" + yaw_update.reason};
    }
    return {true, source_name + "_pose_update"};
  }

  bool point_in_lidar_icp_exclusion_box(const Eigen::Vector3d & point_base) const
  {
    if (!lidar_icp_exclude_leader_box_) {
      return false;
    }
    const double x_min = std::min(lidar_icp_exclude_x_min_m_, lidar_icp_exclude_x_max_m_);
    const double x_max = std::max(lidar_icp_exclude_x_min_m_, lidar_icp_exclude_x_max_m_);
    return point_base.x() >= x_min && point_base.x() <= x_max &&
           std::abs(point_base.y()) <= lidar_icp_exclude_abs_y_max_m_;
  }

  void set_lidar_icp_skip_reason(const std::string & reason)
  {
    diag_.lidar_icp_update_applied = false;
    diag_.lidar_icp_update_reason = reason;
    diag_.last_skip_reason = "lidar_icp_" + reason;
  }

  void reset_lidar_icp_recovery()
  {
    lidar_icp_yaw_gate_rejects_ = 0;
    lidar_icp_recovery_stable_samples_ = 0;
    lidar_icp_recovery_active_ = false;
    diag_.lidar_icp_recovery_active = false;
    diag_.lidar_icp_recovery_yaw_gate_rejects = 0;
    diag_.lidar_icp_recovery_stable_samples = 0;
  }

  void update_lidar_icp_recovery_diagnostics()
  {
    diag_.lidar_icp_recovery_active = lidar_icp_recovery_active_;
    diag_.lidar_icp_recovery_yaw_gate_rejects = lidar_icp_yaw_gate_rejects_;
    diag_.lidar_icp_recovery_stable_samples = lidar_icp_recovery_stable_samples_;
  }

  YawUpdateResult update_lidar_icp_yaw_with_recovery(
    double measured_yaw,
    bool stable_recovery_sample)
  {
    auto update = filter_->update_yaw(measured_yaw, lidar_icp_yaw_var_rad2_, lidar_icp_yaw_gate_rad_);
    if (update.accepted) {
      reset_lidar_icp_recovery();
      return update;
    }
    if (update.reason != "yaw_gate") {
      reset_lidar_icp_recovery();
      return update;
    }

    ++lidar_icp_yaw_gate_rejects_;
    if (stable_recovery_sample) {
      ++lidar_icp_recovery_stable_samples_;
    } else {
      lidar_icp_recovery_stable_samples_ = 0;
      lidar_icp_recovery_active_ = false;
    }

    const bool recovery_ready =
      enable_lidar_icp_yaw_recovery_ &&
      stable_recovery_sample &&
      lidar_icp_yaw_gate_rejects_ >= lidar_icp_recovery_min_yaw_gate_rejects_ &&
      lidar_icp_recovery_stable_samples_ >= lidar_icp_recovery_min_stable_samples_ &&
      std::abs(update.yaw_innovation_rad) <= lidar_icp_recovery_yaw_gate_rad_ &&
      lidar_icp_recovery_max_step_rad_ > 0.0;

    if (!recovery_ready) {
      update_lidar_icp_recovery_diagnostics();
      return update;
    }

    const double yaw_pred =
      yaw_from_quaternion(Eigen::Quaterniond(filter_->pose_matrix().linear()));
    const double limited_residual = std::clamp(
      update.yaw_innovation_rad,
      -lidar_icp_recovery_max_step_rad_,
      lidar_icp_recovery_max_step_rad_);
    const double limited_measured_yaw = wrap_to_pi(yaw_pred + limited_residual);
    auto recovery_update = filter_->update_yaw(
      limited_measured_yaw,
      lidar_icp_yaw_var_rad2_ * lidar_icp_recovery_yaw_var_scale_,
      lidar_icp_recovery_yaw_gate_rad_);
    if (recovery_update.accepted) {
      lidar_icp_recovery_active_ = true;
      recovery_update.reason = "recovery_yaw_update";
    } else {
      lidar_icp_recovery_active_ = false;
      recovery_update.reason = "recovery_" + recovery_update.reason;
    }
    update_lidar_icp_recovery_diagnostics();
    return recovery_update;
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
    diag_.residual_gyro_z_bias_radps =
      filter_ ? filter_->residual_gyro_z_bias_radps() : 0.0;
    diag_.final_gyro_z_radps = omega.z() - diag_.residual_gyro_z_bias_radps;
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

  void lidar_scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
  {
    diag_.lidar_icp_enabled = enable_lidar_icp_yaw_;
    diag_.lidar_icp_initialized = lidar_icp_initialized_;
    diag_.lidar_icp_update_applied = false;
    diag_.lidar_icp_raw_points = 0;
    diag_.lidar_icp_excluded_points = 0;
    diag_.lidar_icp_usable_points = 0;
    diag_.lidar_icp_source_points = 0;
    diag_.lidar_icp_dt_ms = 0.0;
    diag_.lidar_icp_yaw_delta_rad = 0.0;
    diag_.lidar_icp_yaw_rate_radps = 0.0;
    diag_.lidar_icp_yaw_innovation_rad = 0.0;
    diag_.lidar_icp_yaw_gate_rad = lidar_icp_yaw_gate_rad_;
    diag_.lidar_icp_yaw_variance_rad2 = lidar_icp_yaw_var_rad2_;
    update_lidar_icp_recovery_diagnostics();

    if (!enable_lidar_icp_yaw_ || !lidar_icp_) {
      reset_lidar_icp_recovery();
      set_lidar_icp_skip_reason("disabled");
      return;
    }
    if (!filter_->initialized()) {
      lidar_icp_initialized_ = false;
      have_lidar_icp_yaw_ref_ = false;
      last_lidar_icp_update_stamp_ns_.reset();
      last_lidar_icp_yaw_rate_radps_.reset();
      diag_.lidar_icp_initialized = false;
      reset_lidar_icp_recovery();
      set_lidar_icp_skip_reason("eskf_not_initialized");
      return;
    }

    const int64_t ns = stamp_ns(msg->header.stamp);
    if (last_lidar_scan_stamp_ns_ && ns <= *last_lidar_scan_stamp_ns_) {
      reset_lidar_icp_recovery();
      set_lidar_icp_skip_reason("out_of_order");
      return;
    }
    last_lidar_scan_stamp_ns_ = ns;

    const auto base_from_scan = base_from_lidar(msg->header.frame_id);
    if (!base_from_scan) {
      return;
    }

    std::vector<Eigen::Vector3d> points;
    points.reserve(msg->ranges.size());
    int raw_points = 0;
    int excluded_points = 0;
    double angle = msg->angle_min;
    for (const float range_f : msg->ranges) {
      const double range = static_cast<double>(range_f);
      if (std::isfinite(range) && range >= msg->range_min && range <= msg->range_max &&
        range >= lidar_icp_min_range_m_ && range <= lidar_icp_max_range_m_)
      {
        ++raw_points;
        Eigen::Vector3d point_lidar(
          range * std::cos(angle),
          range * std::sin(angle),
          0.0);
        Eigen::Vector3d point_base = (*base_from_scan) * point_lidar;
        point_base.z() = 0.0;
        if (!point_in_lidar_icp_exclusion_box(point_base)) {
          points.push_back(point_base);
        } else {
          ++excluded_points;
        }
      }
      angle += msg->angle_increment;
    }
    diag_.lidar_icp_raw_points = raw_points;
    diag_.lidar_icp_excluded_points = excluded_points;
    diag_.lidar_icp_usable_points = static_cast<int>(points.size());
    if (points.empty()) {
      reset_lidar_icp_recovery();
      set_lidar_icp_skip_reason("empty_points");
      return;
    }

    if (!lidar_icp_initialized_) {
      lidar_icp_->Reset();
      const Eigen::Isometry3d pose = filter_->pose_matrix();
      lidar_icp_->pose() = Sophus::SE3d(Eigen::Quaterniond(pose.linear()), pose.translation());
      lidar_icp_initialized_ = true;
      have_lidar_icp_yaw_ref_ = false;
      last_lidar_icp_update_stamp_ns_.reset();
      last_lidar_icp_yaw_rate_radps_.reset();
      reset_lidar_icp_recovery();
    }
    diag_.lidar_icp_initialized = lidar_icp_initialized_;

    const std::vector<double> point_timestamps;
    const auto & [registered_frame, source] = lidar_icp_->RegisterFrame(points, point_timestamps);
    (void)registered_frame;
    diag_.lidar_icp_source_points = static_cast<int>(source.size());
    if (static_cast<int>(source.size()) < lidar_icp_min_source_points_) {
      have_lidar_icp_yaw_ref_ = false;
      last_lidar_icp_yaw_rate_radps_.reset();
      reset_lidar_icp_recovery();
      set_lidar_icp_skip_reason("min_source_points");
      return;
    }

    const auto replay_target = filter_->stamp_ns();
    if (!replay_target) {
      reset_lidar_icp_recovery();
      set_lidar_icp_skip_reason("missing_filter_stamp");
      return;
    }

    if (!last_lidar_icp_update_stamp_ns_) {
      last_lidar_icp_update_stamp_ns_ = ns;
      last_lidar_icp_yaw_ref_rad_ =
        yaw_from_quaternion(Eigen::Quaterniond(filter_->pose_matrix().linear()));
      have_lidar_icp_yaw_ref_ = true;
      reset_lidar_icp_recovery();
      set_lidar_icp_skip_reason("bootstrap_dt");
      return;
    }

    const double dt = static_cast<double>(ns - *last_lidar_icp_update_stamp_ns_) * 1.0e-9;
    last_lidar_icp_update_stamp_ns_ = ns;
    diag_.lidar_icp_dt_ms = dt * 1.0e3;
    if (!std::isfinite(dt) || !(dt > 0.0)) {
      reset_lidar_icp_recovery();
      set_lidar_icp_skip_reason("invalid_dt");
      return;
    }

    const Sophus::SE3d delta = lidar_icp_->delta();
    const double yaw_delta = wrap_to_pi(yaw_from_rotation(delta.so3().matrix()));
    const double yaw_rate = yaw_delta / dt;
    diag_.lidar_icp_yaw_delta_rad = yaw_delta;
    diag_.lidar_icp_yaw_rate_radps = yaw_rate;
    if (lidar_icp_max_abs_yaw_rate_radps_ > 0.0 &&
      std::isfinite(yaw_rate) && std::abs(yaw_rate) > lidar_icp_max_abs_yaw_rate_radps_)
    {
      have_lidar_icp_yaw_ref_ = false;
      last_lidar_icp_yaw_rate_radps_.reset();
      reset_lidar_icp_recovery();
      set_lidar_icp_skip_reason("yaw_rate_gate");
      return;
    }
    if (!have_lidar_icp_yaw_ref_) {
      last_lidar_icp_yaw_ref_rad_ =
        yaw_from_quaternion(Eigen::Quaterniond(filter_->pose_matrix().linear()));
      have_lidar_icp_yaw_ref_ = true;
      reset_lidar_icp_recovery();
      set_lidar_icp_skip_reason("init_yaw_ref");
      return;
    }

    const double measured_yaw = wrap_to_pi(last_lidar_icp_yaw_ref_rad_ + yaw_delta);
    const bool stable_recovery_sample =
      !last_lidar_icp_yaw_rate_radps_ ||
      lidar_icp_recovery_max_yaw_rate_delta_radps_ <= 0.0 ||
      std::abs(yaw_rate - *last_lidar_icp_yaw_rate_radps_) <=
      lidar_icp_recovery_max_yaw_rate_delta_radps_;
    last_lidar_icp_yaw_rate_radps_ = yaw_rate;
    YawUpdateResult update;
    if (ns < *replay_target) {
      const auto current = filter_->snapshot();
      if (!restore_snapshot_before(ns)) {
        reset_lidar_icp_recovery();
        set_lidar_icp_skip_reason("outside_history");
        return;
      }
      predict_filter_to_stamp(ns);
      update = update_lidar_icp_yaw_with_recovery(measured_yaw, stable_recovery_sample);
      if (!update.accepted) {
        filter_->restore(current);
      } else {
        record_snapshot();
        predict_filter_to_stamp(*replay_target);
        record_snapshot();
      }
    } else {
      predict_filter_to_stamp(ns);
      update = update_lidar_icp_yaw_with_recovery(measured_yaw, stable_recovery_sample);
      if (update.accepted) {
        record_snapshot();
      }
    }

    diag_.lidar_icp_update_applied = update.accepted;
    diag_.lidar_icp_update_reason = update.reason;
    diag_.lidar_icp_yaw_innovation_rad = update.yaw_innovation_rad;
    diag_.lidar_icp_yaw_gate_rad = update.yaw_gate_rad;
    diag_.lidar_icp_yaw_variance_rad2 = update.yaw_variance_rad2;
    if (update.accepted) {
      last_lidar_icp_yaw_ref_rad_ = measured_yaw;
    } else {
      if (update.reason != "yaw_gate") {
        have_lidar_icp_yaw_ref_ = false;
      }
      diag_.last_skip_reason = "lidar_icp_" + update.reason;
    }
    prune_history(std::max(ns, filter_->stamp_ns().value_or(ns)));
  }

  void reset_filter_track()
  {
    filter_->reset();
    state_history_.clear();
    lidar_icp_initialized_ = false;
    have_lidar_icp_yaw_ref_ = false;
    last_lidar_icp_update_stamp_ns_.reset();
    last_lidar_icp_yaw_rate_radps_.reset();
    aruco_large_innovation_count_ = 0;
    reset_lidar_icp_recovery();
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
    const Eigen::Isometry3d leader_rear_from_base =
      transform_leader_rear_from_board() * board_from_base;
    const Eigen::Isometry3d leader_from_base =
      leader_base_from_leader_rear() * leader_rear_from_base;
    Eigen::Matrix<double, 6, 6> leader_cov = transform_pose_covariance(
      transform_pose_covariance(covariance_from_msg(*msg), transform_leader_rear_from_board()),
      leader_base_from_leader_rear());
    const bool stale = !last_vision_stamp_ns_ ||
      (ns - *last_vision_stamp_ns_) > static_cast<int64_t>(reset_timeout_sec_ * 1.0e9);
    if (stale && !enable_gps_pose_update_) {
      reset_filter_track();
    }

    diag_.aruco_position_innovation_m = filter_->initialized() ?
      (leader_from_base.translation().head<2>() -
      filter_->pose_matrix().translation().head<2>()).norm() : 0.0;
    if (filter_->initialized() && aruco_reinitialize_position_error_m_ > 0.0 &&
      diag_.aruco_position_innovation_m > aruco_reinitialize_position_error_m_)
    {
      ++aruco_large_innovation_count_;
      if (aruco_large_innovation_count_ >= aruco_reinitialize_min_consecutive_) {
        reset_filter_track();
        diag_.last_skip_reason = "aruco_reinitialized_large_innovation";
      }
    } else {
      aruco_large_innovation_count_ = 0;
    }

    const auto outcome = apply_pose_measurement(
      ns, leader_from_base, leader_cov, aruco_position_gate_m_,
      aruco_position_mahalanobis_gate_, aruco_yaw_gate_rad_, enable_aruco_yaw_update_, true,
      aruco_position_covariance_scale_, aruco_yaw_covariance_scale_,
      aruco_min_position_variance_m2_, aruco_min_yaw_variance_rad2_,
      aruco_max_position_variance_m2_, aruco_max_yaw_variance_rad2_, "aruco");

    last_vision_stamp_ns_ = ns;
    diag_.aruco_update_applied = outcome.first;
    diag_.aruco_update_reason = outcome.second;
    diag_.aruco_position_gate_m = aruco_position_gate_m_;
    diag_.aruco_position_covariance_scale = aruco_position_covariance_scale_;
    diag_.aruco_rotation_innovation_deg = 0.0;
    if (outcome.first) {
      aruco_large_innovation_count_ = 0;
    } else {
      diag_.last_skip_reason = outcome.second;
    }
  }

  void lidar_wheel_odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    const int64_t ns = stamp_ns(msg->header.stamp);
    const Eigen::Isometry3d follower_from_leader = odom_pose_matrix(*msg);
    const Eigen::Isometry3d leader_from_base = follower_from_leader.inverse();
    const Eigen::Matrix<double, 6, 6> leader_cov =
      transform_pose_covariance(covariance_from_odom(*msg), leader_from_base);
    const auto outcome = apply_pose_measurement(
      ns, leader_from_base, leader_cov, lidar_wheel_pose_gate_m_,
      lidar_wheel_position_mahalanobis_gate_, lidar_wheel_yaw_gate_rad_,
      enable_lidar_wheel_yaw_update_, allow_lidar_wheel_initialization_,
      lidar_wheel_position_covariance_scale_, lidar_wheel_yaw_covariance_scale_,
      lidar_wheel_min_position_variance_m2_, lidar_wheel_min_yaw_variance_rad2_,
      lidar_wheel_max_position_variance_m2_, lidar_wheel_max_yaw_variance_rad2_, "lidar_wheel");
    if (!outcome.first) {
      diag_.last_skip_reason = outcome.second;
    }
  }

  void gps_leader_odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    latest_gps_leader_odom_ = msg;
    try_gps_pose_update();
  }

  void gps_follower_odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    latest_gps_follower_odom_ = msg;
    try_gps_pose_update();
  }

  bool gps_odom_fresh(const nav_msgs::msg::Odometry::ConstSharedPtr & msg) const
  {
    if (!msg) {
      return false;
    }
    const double age_sec =
      static_cast<double>(now().nanoseconds() - stamp_ns(msg->header.stamp)) * 1.0e-9;
    return age_sec >= 0.0 && age_sec <= gps_odom_timeout_sec_;
  }

  void try_gps_pose_update()
  {
    if (!gps_odom_fresh(latest_gps_leader_odom_) || !gps_odom_fresh(latest_gps_follower_odom_)) {
      return;
    }
    if (latest_gps_leader_odom_->header.frame_id != latest_gps_follower_odom_->header.frame_id) {
      diag_.last_skip_reason = "gps_frame_mismatch";
      return;
    }

    const int64_t leader_ns = stamp_ns(latest_gps_leader_odom_->header.stamp);
    const int64_t follower_ns = stamp_ns(latest_gps_follower_odom_->header.stamp);
    const int64_t ns = std::min(leader_ns, follower_ns);
    if (last_gps_update_stamp_ns_ && ns <= *last_gps_update_stamp_ns_) {
      return;
    }
    last_gps_update_stamp_ns_ = ns;

    const Eigen::Isometry3d map_from_leader = odom_pose_matrix(*latest_gps_leader_odom_);
    const Eigen::Isometry3d map_from_follower = odom_pose_matrix(*latest_gps_follower_odom_);
    const Eigen::Isometry3d leader_from_base = map_from_leader.inverse() * map_from_follower;

    Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Zero();
    const auto leader_cov = covariance_from_odom(*latest_gps_leader_odom_);
    const auto follower_cov = covariance_from_odom(*latest_gps_follower_odom_);
    cov(0, 0) = covariance_value(leader_cov, 0, 0, 0.25) +
      covariance_value(follower_cov, 0, 0, 0.25);
    cov(1, 1) = covariance_value(leader_cov, 1, 1, 0.25) +
      covariance_value(follower_cov, 1, 1, 0.25);
    cov(2, 2) = covariance_value(leader_cov, 2, 2, 1.0) +
      covariance_value(follower_cov, 2, 2, 1.0);
    cov(5, 5) = covariance_value(leader_cov, 5, 5, 0.25) +
      covariance_value(follower_cov, 5, 5, 0.25);

    const auto outcome = apply_pose_measurement(
      ns, leader_from_base, cov, gps_pose_gate_m_, gps_position_mahalanobis_gate_,
      gps_yaw_gate_rad_, true, true, gps_position_covariance_scale_, gps_yaw_covariance_scale_,
      gps_min_position_variance_m2_, gps_min_yaw_variance_rad2_,
      gps_max_position_variance_m2_, gps_max_yaw_variance_rad2_, "gps");
    if (!outcome.first) {
      diag_.last_skip_reason = outcome.second;
    }
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

    const Eigen::Isometry3d leader_base_from_follower = extrapolated.pose_matrix();
    const Eigen::Matrix<double, 6, 6> leader_base_cov = extrapolated.pose_covariance();
    const Eigen::Isometry3d leader_rear_from_follower =
      leader_rear_from_leader_base() * leader_base_from_follower;
    const Eigen::Matrix<double, 6, 6> leader_rear_cov =
      transform_pose_covariance(leader_base_cov, leader_rear_from_leader_base());
    const Eigen::Isometry3d board_from_follower =
      transform_board_from_leader_rear() * leader_rear_from_follower;
    const Eigen::Matrix<double, 6, 6> board_cov =
      transform_pose_covariance(leader_rear_cov, transform_board_from_leader_rear());

    const auto stamp = time_msg_from_ns(now_ns);
    publish_outputs(
      stamp, board_from_follower, board_cov, leader_base_from_follower, leader_base_cov,
      extrapolated.linear_velocity_base_mps(), extrapolated.angular_velocity_base_radps());
    diag_.filter_yaw_rad = yaw_from_quaternion(Eigen::Quaterniond(leader_base_from_follower.linear()));
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
    const Eigen::Isometry3d & leader_base_from_base,
    const Eigen::Matrix<double, 6, 6> & leader_base_cov,
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

    const auto leader_base_odom = make_odom(
      stamp, leader_base_frame_, base_frame_, leader_base_from_base, leader_base_cov,
      linear_velocity_base, angular_velocity_base);
    leader_base_odom_pub_->publish(leader_base_odom);
    geometry_msgs::msg::PoseWithCovarianceStamped leader_base_pose;
    leader_base_pose.header = leader_base_odom.header;
    leader_base_pose.pose = leader_base_odom.pose;
    leader_base_pose_pub_->publish(leader_base_pose);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header = leader_base_odom.header;
      tf_msg.child_frame_id = base_frame_;
      tf_msg.transform.translation.x = leader_base_from_base.translation().x();
      tf_msg.transform.translation.y = leader_base_from_base.translation().y();
      tf_msg.transform.translation.z = leader_base_from_base.translation().z();
      tf_msg.transform.rotation = leader_base_odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf_msg);
    }
  }

  void publish_static_leader_rear_to_board()
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = now();
    tf_msg.header.frame_id = leader_rear_frame_;
    tf_msg.child_frame_id = board_frame_;
    const auto tf = transform_leader_rear_from_board();
    tf_msg.transform.rotation = quaternion_msg_from_eigen(Eigen::Quaterniond(tf.linear()));
    static_tf_broadcaster_->sendTransform(tf_msg);
  }

  void publish_diagnostics()
  {
    diag_.eskf_initialized = filter_->initialized();
    diag_.aruco_position_gate_m = aruco_position_gate_m_;
    diag_.aruco_position_covariance_scale = aruco_position_covariance_scale_;
    diag_.gyro_z_bias_radps = fixed_gyro_z_bias_radps_;
    diag_.residual_gyro_z_bias_radps =
      filter_ ? filter_->residual_gyro_z_bias_radps() : 0.0;
    diag_.final_gyro_z_radps = diag_.corrected_gyro_z_radps - diag_.residual_gyro_z_bias_radps;
    diag_.gyro_bias_valid = gyro_bias_valid_;
    diag_.stationary_detected = stationary_detected_;
    diag_.calibration_status = calibration_status_;
    diag_.lidar_icp_enabled = enable_lidar_icp_yaw_;
    diag_.lidar_icp_initialized = lidar_icp_initialized_;
    diag_.lidar_icp_yaw_gate_rad = lidar_icp_yaw_gate_rad_;
    diag_.lidar_icp_yaw_variance_rad2 = lidar_icp_yaw_var_rad2_;
    update_lidar_icp_recovery_diagnostics();
    diagnostics_pub_->publish(diagnostics_builder_.build(now(), diag_));
  }

  GyroRelativeEskfOptions filter_options_;
  std::unique_ptr<GyroRelativeEskf> filter_;
  DiagnosticsPublisher diagnostics_builder_;
  ArucoImuEskfDiagnostics diag_;

  int64_t node_start_ns_{0};
  std::string board_frame_;
  std::string leader_base_frame_;
  std::string leader_rear_frame_;
  std::string base_frame_;
  std::string camera_frame_;
  bool publish_tf_{true};
  double camera_tf_startup_grace_sec_{2.0};
  double leader_base_to_rear_x_m_{-0.275};
  double leader_base_to_rear_y_m_{0.0};
  double leader_base_to_rear_z_m_{0.0525};
  bool enable_lidar_wheel_pose_update_{true};
  double lidar_wheel_pose_gate_m_{0.12};
  double lidar_wheel_yaw_gate_rad_{8.0 * M_PI / 180.0};
  bool enable_lidar_wheel_yaw_update_{false};
  bool allow_lidar_wheel_initialization_{false};
  double lidar_wheel_position_mahalanobis_gate_{2.0};
  double lidar_wheel_position_covariance_scale_{6.0};
  double lidar_wheel_yaw_covariance_scale_{10.0};
  double lidar_wheel_min_position_variance_m2_{0.0100};
  double lidar_wheel_min_yaw_variance_rad2_{0.0300};
  double lidar_wheel_max_position_variance_m2_{0.0200};
  double lidar_wheel_max_yaw_variance_rad2_{0.0600};
  bool enable_gps_pose_update_{true};
  double gps_pose_gate_m_{1.00};
  double gps_yaw_gate_rad_{25.0 * M_PI / 180.0};
  double gps_odom_timeout_sec_{0.50};
  double gps_position_mahalanobis_gate_{4.0};
  double gps_position_covariance_scale_{1.0};
  double gps_yaw_covariance_scale_{1.0};
  double gps_min_position_variance_m2_{0.0025};
  double gps_min_yaw_variance_rad2_{0.0100};
  double gps_max_position_variance_m2_{1.0};
  double gps_max_yaw_variance_rad2_{0.50};
  double reset_timeout_sec_{1.0};
  double vision_delay_buffer_sec_{2.0};
  double aruco_position_gate_m_{0.75};
  double aruco_position_covariance_scale_{2.0};
  double aruco_position_mahalanobis_gate_{3.0};
  double aruco_min_position_variance_m2_{0.0100};
  double aruco_min_yaw_variance_rad2_{0.0100};
  double aruco_max_position_variance_m2_{2.0};
  double aruco_max_yaw_variance_rad2_{0.50};
  double aruco_reinitialize_position_error_m_{0.30};
  int aruco_reinitialize_min_consecutive_{3};
  int aruco_large_innovation_count_{0};
  bool enable_aruco_yaw_update_{true};
  double aruco_yaw_gate_rad_{20.0 * M_PI / 180.0};
  double aruco_yaw_covariance_scale_{2.0};
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
  bool enable_lidar_icp_yaw_{false};
  double lidar_icp_min_range_m_{0.15};
  double lidar_icp_max_range_m_{8.0};
  int lidar_icp_min_source_points_{80};
  double lidar_icp_yaw_var_rad2_{0.05};
  double lidar_icp_yaw_gate_rad_{8.0 * M_PI / 180.0};
  double lidar_icp_max_abs_yaw_rate_radps_{1.5};
  bool lidar_icp_exclude_leader_box_{true};
  double lidar_icp_exclude_x_min_m_{0.10};
  double lidar_icp_exclude_x_max_m_{4.00};
  double lidar_icp_exclude_abs_y_max_m_{1.00};
  bool enable_lidar_icp_yaw_recovery_{true};
  int lidar_icp_recovery_min_yaw_gate_rejects_{5};
  int lidar_icp_recovery_min_stable_samples_{5};
  double lidar_icp_recovery_yaw_gate_rad_{45.0 * M_PI / 180.0};
  double lidar_icp_recovery_max_step_rad_{3.0 * M_PI / 180.0};
  double lidar_icp_recovery_yaw_var_scale_{8.0};
  double lidar_icp_recovery_max_yaw_rate_delta_radps_{0.35};
  int lidar_icp_yaw_gate_rejects_{0};
  int lidar_icp_recovery_stable_samples_{0};
  bool lidar_icp_recovery_active_{false};
  bool lidar_icp_initialized_{false};
  bool have_lidar_icp_yaw_ref_{false};
  double last_lidar_icp_yaw_ref_rad_{0.0};
  std::unique_ptr<kiss_icp::pipeline::KissICP> lidar_icp_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr board_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_wheel_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_leader_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_follower_odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr board_odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr board_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr leader_base_odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr leader_base_pose_pub_;
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
  std::optional<int64_t> last_lidar_scan_stamp_ns_;
  std::optional<int64_t> last_lidar_icp_update_stamp_ns_;
  std::optional<double> last_lidar_icp_yaw_rate_radps_;
  std::optional<Eigen::Isometry3d> cached_camera_from_base_;
  std::optional<Eigen::Isometry3d> cached_base_from_lidar_;
  std::optional<Eigen::Isometry3d> cached_leader_base_from_rear_;
  std::string cached_lidar_frame_;
  std::string cached_imu_frame_;
  std::optional<Eigen::Matrix3d> cached_rotation_base_from_imu_;
  nav_msgs::msg::Odometry::ConstSharedPtr latest_gps_leader_odom_;
  nav_msgs::msg::Odometry::ConstSharedPtr latest_gps_follower_odom_;
  std::optional<int64_t> last_gps_update_stamp_ns_;
};

}  // namespace relative_localization_eskf

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<relative_localization_eskf::RelativeLocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
