#ifndef ARUCO_IMU_ESKF_LOCALIZATION_CPP__GYRO_RELATIVE_ESKF_HPP_
#define ARUCO_IMU_ESKF_LOCALIZATION_CPP__GYRO_RELATIVE_ESKF_HPP_

#include <Eigen/Dense>

#include <cstdint>
#include <optional>
#include <string>

namespace aruco_imu_eskf_localization_cpp
{

struct GyroRelativeEskfOptions
{
  double gyro_noise_std_radps{0.05};
  double gyro_bias_noise_std_radps{0.002};
  double initial_gyro_bias_std_radps{0.02};
  double max_abs_gyro_bias_radps{0.08};
  double initial_position_std_m{0.20};
  double initial_orientation_std_deg{5.0};
  double position_smoothing_alpha{0.55};
  double min_position_variance{1.0e-5};
};

struct GyroRelativeEskfSnapshot
{
  std::optional<int64_t> stamp_ns;
  Eigen::Vector3d position_m{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
  Eigen::Matrix4d covariance{Eigen::Matrix4d::Zero()};
  Eigen::Vector3d last_angular_velocity_base{Eigen::Vector3d::Zero()};
  double residual_gyro_z_bias_radps{0.0};
  bool initialized{false};
};

struct PositionUpdateResult
{
  bool accepted{false};
  bool initialized{false};
  double position_innovation_m{0.0};
  std::string reason{"not_initialized"};
};

struct YawUpdateResult
{
  bool accepted{false};
  bool initialized{false};
  double yaw_innovation_rad{0.0};
  double yaw_gate_rad{0.0};
  double yaw_variance_rad2{0.0};
  std::string reason{"not_initialized"};
};

class GyroRelativeEskf
{
public:
  explicit GyroRelativeEskf(const GyroRelativeEskfOptions & options = {});

  void reset();
  bool initialized() const {return initialized_;}
  std::optional<int64_t> stamp_ns() const {return stamp_ns_;}
  GyroRelativeEskfSnapshot snapshot() const;
  void restore(const GyroRelativeEskfSnapshot & snapshot);

  void initialize_position_only(
    int64_t stamp_ns,
    const Eigen::Vector3d & position_m,
    const Eigen::Matrix3d & position_covariance);
  void predict(int64_t target_stamp_ns, const Eigen::Vector3d & angular_velocity_base);
  PositionUpdateResult update_position(
    const Eigen::Vector3d & measured_position_m,
    const Eigen::Matrix3d & position_covariance,
    double gate_m);
  YawUpdateResult update_yaw(
    double measured_yaw_rad,
    double yaw_variance_rad2,
    double gate_rad);

  Eigen::Isometry3d pose_matrix() const;
  Eigen::Matrix<double, 6, 6> pose_covariance() const;
  Eigen::Vector3d linear_velocity_base_mps() const;
  Eigen::Vector3d angular_velocity_base_radps() const;
  double residual_gyro_z_bias_radps() const {return residual_gyro_z_bias_radps_;}

private:
  GyroRelativeEskfOptions options_;
  std::optional<int64_t> stamp_ns_;
  Eigen::Vector3d position_m_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond rotation_{Eigen::Quaterniond::Identity()};
  Eigen::Matrix4d covariance_{Eigen::Matrix4d::Zero()};
  Eigen::Vector3d last_angular_velocity_base_{Eigen::Vector3d::Zero()};
  double residual_gyro_z_bias_radps_{0.0};
  bool initialized_{false};

  Eigen::Matrix4d initial_covariance() const;
  double clamped_residual_bias(double bias_radps) const;
};

Eigen::Matrix<double, 6, 6> transform_pose_covariance(
  const Eigen::Matrix<double, 6, 6> & covariance,
  const Eigen::Isometry3d & left_transform);

}  // namespace aruco_imu_eskf_localization_cpp

#endif  // ARUCO_IMU_ESKF_LOCALIZATION_CPP__GYRO_RELATIVE_ESKF_HPP_
