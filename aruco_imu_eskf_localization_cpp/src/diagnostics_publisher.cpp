#include "aruco_imu_eskf_localization_cpp/diagnostics_publisher.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <string>

namespace aruco_imu_eskf_localization_cpp
{

void DiagnosticsPublisher::add_key_value(
  diagnostic_msgs::msg::DiagnosticStatus & status,
  const std::string & key,
  const std::string & value)
{
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = key;
  kv.value = value;
  status.values.push_back(kv);
}

diagnostic_msgs::msg::DiagnosticArray DiagnosticsPublisher::build(
  const rclcpp::Time & stamp,
  const ArucoImuEskfDiagnostics & input) const
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = stamp;

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "aruco_imu_eskf_localization";
  status.level = input.eskf_initialized ?
    diagnostic_msgs::msg::DiagnosticStatus::OK :
    diagnostic_msgs::msg::DiagnosticStatus::WARN;
  status.message = input.eskf_initialized ? "running" : "waiting_for_aruco_init";

  add_key_value(status, "eskf_initialized", input.eskf_initialized ? "true" : "false");
  add_key_value(status, "aruco_update_applied", input.aruco_update_applied ? "true" : "false");
  add_key_value(status, "aruco_update_reason", input.aruco_update_reason);
  add_key_value(
    status, "aruco_position_innovation_m",
    std::to_string(input.aruco_position_innovation_m));
  add_key_value(status, "aruco_position_gate_m", std::to_string(input.aruco_position_gate_m));
  add_key_value(
    status, "aruco_position_covariance_scale",
    std::to_string(input.aruco_position_covariance_scale));
  add_key_value(
    status, "aruco_rotation_innovation_deg",
    std::to_string(input.aruco_rotation_innovation_deg));
  add_key_value(status, "filter_yaw_rad", std::to_string(input.filter_yaw_rad));
  add_key_value(
    status, "imu_yaw_reference_only_rad",
    std::to_string(input.imu_yaw_reference_only_rad));
  add_key_value(status, "imu_orientation_valid", input.imu_orientation_valid ? "true" : "false");
  add_key_value(status, "raw_gyro_z_radps", std::to_string(input.raw_gyro_z_radps));
  add_key_value(status, "corrected_gyro_z_radps", std::to_string(input.corrected_gyro_z_radps));
  add_key_value(status, "final_gyro_z_radps", std::to_string(input.final_gyro_z_radps));
  add_key_value(status, "gyro_z_bias_radps", std::to_string(input.gyro_z_bias_radps));
  add_key_value(
    status, "residual_gyro_z_bias_radps",
    std::to_string(input.residual_gyro_z_bias_radps));
  add_key_value(status, "gyro_bias_valid", input.gyro_bias_valid ? "true" : "false");
  add_key_value(status, "stationary_detected", input.stationary_detected ? "true" : "false");
  add_key_value(status, "calibration_status", input.calibration_status);
  add_key_value(status, "lidar_icp_enabled", input.lidar_icp_enabled ? "true" : "false");
  add_key_value(
    status, "lidar_icp_initialized", input.lidar_icp_initialized ? "true" : "false");
  add_key_value(
    status, "lidar_icp_update_applied",
    input.lidar_icp_update_applied ? "true" : "false");
  add_key_value(status, "lidar_icp_update_reason", input.lidar_icp_update_reason);
  add_key_value(status, "lidar_icp_raw_points", std::to_string(input.lidar_icp_raw_points));
  add_key_value(
    status, "lidar_icp_excluded_points",
    std::to_string(input.lidar_icp_excluded_points));
  add_key_value(
    status, "lidar_icp_usable_points", std::to_string(input.lidar_icp_usable_points));
  add_key_value(status, "lidar_icp_source_points", std::to_string(input.lidar_icp_source_points));
  add_key_value(status, "lidar_icp_dt_ms", std::to_string(input.lidar_icp_dt_ms));
  add_key_value(
    status, "lidar_icp_yaw_delta_rad", std::to_string(input.lidar_icp_yaw_delta_rad));
  add_key_value(
    status, "lidar_icp_yaw_rate_radps", std::to_string(input.lidar_icp_yaw_rate_radps));
  add_key_value(
    status, "lidar_icp_yaw_innovation_rad",
    std::to_string(input.lidar_icp_yaw_innovation_rad));
  add_key_value(
    status, "lidar_icp_yaw_gate_rad", std::to_string(input.lidar_icp_yaw_gate_rad));
  add_key_value(
    status, "lidar_icp_yaw_variance_rad2",
    std::to_string(input.lidar_icp_yaw_variance_rad2));
  add_key_value(
    status, "lidar_icp_recovery_active",
    input.lidar_icp_recovery_active ? "true" : "false");
  add_key_value(
    status, "lidar_icp_recovery_yaw_gate_rejects",
    std::to_string(input.lidar_icp_recovery_yaw_gate_rejects));
  add_key_value(
    status, "lidar_icp_recovery_stable_samples",
    std::to_string(input.lidar_icp_recovery_stable_samples));
  add_key_value(status, "last_skip_reason", input.last_skip_reason);

  array.status.push_back(status);
  return array;
}

}  // namespace aruco_imu_eskf_localization_cpp
