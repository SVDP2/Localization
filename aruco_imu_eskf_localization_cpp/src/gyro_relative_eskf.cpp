#include "aruco_imu_eskf_localization_cpp/gyro_relative_eskf.hpp"

#include "aruco_imu_eskf_localization_cpp/geometry.hpp"

#include <algorithm>
#include <cmath>

namespace aruco_imu_eskf_localization_cpp
{

GyroRelativeEskf::GyroRelativeEskf(const GyroRelativeEskfOptions & options)
: options_(options)
{
  reset();
}

void GyroRelativeEskf::reset()
{
  stamp_ns_.reset();
  position_m_.setZero();
  rotation_ = Eigen::Quaterniond::Identity();
  covariance_ = initial_covariance();
  last_angular_velocity_base_.setZero();
  initialized_ = false;
}

Eigen::Matrix3d GyroRelativeEskf::initial_covariance() const
{
  Eigen::Matrix3d p = Eigen::Matrix3d::Zero();
  const double ori_std_rad = std::max(options_.initial_orientation_std_deg, 1.0e-3) * M_PI / 180.0;
  p(0, 0) = std::pow(std::max(options_.initial_position_std_m, 1.0e-3), 2);
  p(1, 1) = std::pow(std::max(options_.initial_position_std_m, 1.0e-3), 2);
  p(2, 2) = std::pow(ori_std_rad, 2);
  return p;
}

GyroRelativeEskfSnapshot GyroRelativeEskf::snapshot() const
{
  GyroRelativeEskfSnapshot s;
  s.stamp_ns = stamp_ns_;
  s.position_m = position_m_;
  s.rotation = rotation_;
  s.covariance = covariance_;
  s.last_angular_velocity_base = last_angular_velocity_base_;
  s.initialized = initialized_;
  return s;
}

void GyroRelativeEskf::restore(const GyroRelativeEskfSnapshot & snapshot)
{
  stamp_ns_ = snapshot.stamp_ns;
  position_m_ = snapshot.position_m;
  position_m_.z() = 0.0;
  rotation_ = Eigen::Quaterniond(
    Eigen::AngleAxisd(yaw_from_quaternion(snapshot.rotation), Eigen::Vector3d::UnitZ()));
  covariance_ = snapshot.covariance;
  last_angular_velocity_base_ = snapshot.last_angular_velocity_base;
  initialized_ = snapshot.initialized;
}

void GyroRelativeEskf::initialize_position_only(
  int64_t stamp_ns,
  const Eigen::Vector3d & position_m,
  const Eigen::Matrix3d & position_covariance)
{
  stamp_ns_ = stamp_ns;
  position_m_ = position_m;
  position_m_.z() = 0.0;
  rotation_ = Eigen::Quaterniond::Identity();
  covariance_ = initial_covariance();
  covariance_(0, 0) = std::max(position_covariance(0, 0), options_.min_position_variance);
  covariance_(1, 1) = std::max(position_covariance(1, 1), options_.min_position_variance);
  initialized_ = true;
}

void GyroRelativeEskf::predict(int64_t target_stamp_ns, const Eigen::Vector3d & angular_velocity_base)
{
  if (!initialized_) {
    return;
  }
  if (!stamp_ns_) {
    stamp_ns_ = target_stamp_ns;
    last_angular_velocity_base_ = angular_velocity_base;
    return;
  }

  const double dt = std::max(0.0, static_cast<double>(target_stamp_ns - *stamp_ns_) * 1.0e-9);
  if (dt <= 0.0) {
    last_angular_velocity_base_ = angular_velocity_base;
    return;
  }

  const double yaw_rate = angular_velocity_base.z();
  const double yaw = yaw_from_quaternion(rotation_) + yaw_rate * dt;
  rotation_ = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

  covariance_(2, 2) += std::pow(std::max(options_.gyro_noise_std_radps, 1.0e-5), 2) * dt;
  stamp_ns_ = target_stamp_ns;
  last_angular_velocity_base_ = angular_velocity_base;
}

PositionUpdateResult GyroRelativeEskf::update_position(
  const Eigen::Vector3d & measured_position_m,
  const Eigen::Matrix3d & position_covariance,
  double gate_m)
{
  PositionUpdateResult result;
  if (!initialized_) {
    result.reason = "not_initialized";
    return result;
  }

  const Eigen::Vector2d measured = measured_position_m.head<2>();
  const Eigen::Vector2d residual = measured - position_m_.head<2>();
  result.position_innovation_m = residual.norm();
  if (gate_m > 0.0 && result.position_innovation_m > gate_m) {
    result.reason = "position_gate";
    return result;
  }

  const double alpha = std::clamp(options_.position_smoothing_alpha, 0.0, 1.0);
  position_m_.head<2>() += alpha * residual;
  position_m_.z() = 0.0;

  covariance_(0, 0) = std::max(position_covariance(0, 0), options_.min_position_variance);
  covariance_(1, 1) = std::max(position_covariance(1, 1), options_.min_position_variance);

  result.accepted = true;
  result.initialized = true;
  result.reason = "aruco_position_smoothing";
  return result;
}

Eigen::Isometry3d GyroRelativeEskf::pose_matrix() const
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = position_m_;
  pose.linear() = rotation_.normalized().toRotationMatrix();
  return pose;
}

Eigen::Matrix<double, 6, 6> GyroRelativeEskf::pose_covariance() const
{
  Eigen::Matrix<double, 6, 6> out = Eigen::Matrix<double, 6, 6>::Zero();
  out(0, 0) = covariance_(0, 0);
  out(1, 1) = covariance_(1, 1);
  out(2, 2) = 1.0e-6;
  out(3, 3) = 1.0e-6;
  out(4, 4) = 1.0e-6;
  out(5, 5) = covariance_(2, 2);
  return out;
}

Eigen::Vector3d GyroRelativeEskf::linear_velocity_base_mps() const
{
  return Eigen::Vector3d::Zero();
}

Eigen::Vector3d GyroRelativeEskf::angular_velocity_base_radps() const
{
  return Eigen::Vector3d(0.0, 0.0, last_angular_velocity_base_.z());
}

Eigen::Matrix<double, 6, 6> transform_pose_covariance(
  const Eigen::Matrix<double, 6, 6> & covariance,
  const Eigen::Isometry3d & left_transform)
{
  Eigen::Matrix<double, 6, 6> adj = Eigen::Matrix<double, 6, 6>::Zero();
  adj.block<3, 3>(0, 0) = left_transform.linear();
  adj.block<3, 3>(0, 3) = skew(left_transform.translation()) * left_transform.linear();
  adj.block<3, 3>(3, 3) = left_transform.linear();
  return adj * covariance * adj.transpose();
}

}  // namespace aruco_imu_eskf_localization_cpp
