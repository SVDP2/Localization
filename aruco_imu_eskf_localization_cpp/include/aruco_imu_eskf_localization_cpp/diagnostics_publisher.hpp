#ifndef ARUCO_IMU_ESKF_LOCALIZATION_CPP__DIAGNOSTICS_PUBLISHER_HPP_
#define ARUCO_IMU_ESKF_LOCALIZATION_CPP__DIAGNOSTICS_PUBLISHER_HPP_

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/time.hpp>

#include <string>

namespace aruco_imu_eskf_localization_cpp
{

struct ArucoImuEskfDiagnostics
{
  bool eskf_initialized{false};
  bool aruco_update_applied{false};
  std::string aruco_update_reason{"not_received"};
  double aruco_position_innovation_m{0.0};
  double aruco_position_gate_m{0.0};
  double aruco_position_covariance_scale{1.0};
  double aruco_rotation_innovation_deg{0.0};
  double filter_yaw_rad{0.0};
  double imu_yaw_reference_only_rad{0.0};
  bool imu_orientation_valid{false};
  double raw_gyro_z_radps{0.0};
  double corrected_gyro_z_radps{0.0};
  double gyro_z_bias_radps{0.0};
  bool gyro_bias_valid{false};
  bool stationary_detected{false};
  std::string calibration_status{"disabled"};
  bool lidar_icp_enabled{false};
  bool lidar_icp_initialized{false};
  bool lidar_icp_update_applied{false};
  std::string lidar_icp_update_reason{"disabled"};
  int lidar_icp_source_points{0};
  double lidar_icp_dt_ms{0.0};
  double lidar_icp_yaw_delta_rad{0.0};
  double lidar_icp_yaw_rate_radps{0.0};
  double lidar_icp_yaw_innovation_rad{0.0};
  double lidar_icp_yaw_gate_rad{0.0};
  double lidar_icp_yaw_variance_rad2{0.0};
  bool lidar_icp_recovery_active{false};
  int lidar_icp_recovery_yaw_gate_rejects{0};
  int lidar_icp_recovery_stable_samples{0};
  std::string last_skip_reason{"none"};
};

class DiagnosticsPublisher
{
public:
  diagnostic_msgs::msg::DiagnosticArray build(
    const rclcpp::Time & stamp,
    const ArucoImuEskfDiagnostics & input) const;

private:
  static void add_key_value(
    diagnostic_msgs::msg::DiagnosticStatus & status,
    const std::string & key,
    const std::string & value);
};

}  // namespace aruco_imu_eskf_localization_cpp

#endif  // ARUCO_IMU_ESKF_LOCALIZATION_CPP__DIAGNOSTICS_PUBLISHER_HPP_
