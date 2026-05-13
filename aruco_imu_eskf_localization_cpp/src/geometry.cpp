#include "aruco_imu_eskf_localization_cpp/geometry.hpp"

#include <cmath>

namespace aruco_imu_eskf_localization_cpp
{

Eigen::Matrix3d skew(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d out;
  out << 0.0, -v.z(), v.y(),
    v.z(), 0.0, -v.x(),
    -v.y(), v.x(), 0.0;
  return out;
}

Eigen::Matrix3d rotation_leader_rear_from_board()
{
  Eigen::Matrix3d r;
  r << 0.0, 0.0, -1.0,
    -1.0, 0.0, 0.0,
    0.0, 1.0, 0.0;
  return r;
}

Eigen::Matrix3d rotation_board_from_leader_rear()
{
  return rotation_leader_rear_from_board().transpose();
}

Eigen::Isometry3d transform_leader_rear_from_board()
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.linear() = rotation_leader_rear_from_board();
  return tf;
}

Eigen::Isometry3d transform_board_from_leader_rear()
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.linear() = rotation_board_from_leader_rear();
  return tf;
}

Eigen::Quaterniond normalize_quaternion(double x, double y, double z, double w)
{
  Eigen::Quaterniond q(w, x, y, z);
  if (!std::isfinite(q.norm()) || q.norm() < 1.0e-12) {
    return Eigen::Quaterniond::Identity();
  }
  q.normalize();
  return q;
}

double yaw_from_quaternion(const Eigen::Quaterniond & q_in)
{
  Eigen::Quaterniond q = q_in.normalized();
  const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
  const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  return std::atan2(siny_cosp, cosy_cosp);
}

double wrap_angle(double angle_rad)
{
  while (angle_rad > M_PI) {
    angle_rad -= 2.0 * M_PI;
  }
  while (angle_rad < -M_PI) {
    angle_rad += 2.0 * M_PI;
  }
  return angle_rad;
}

Eigen::Isometry3d transform_from_msg(const geometry_msgs::msg::TransformStamped & msg)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  const auto & t = msg.transform.translation;
  const auto & q = msg.transform.rotation;
  tf.translation() = Eigen::Vector3d(t.x, t.y, t.z);
  tf.linear() = normalize_quaternion(q.x, q.y, q.z, q.w).toRotationMatrix();
  return tf;
}

geometry_msgs::msg::Quaternion quaternion_msg_from_eigen(const Eigen::Quaterniond & q_in)
{
  Eigen::Quaterniond q = q_in.normalized();
  geometry_msgs::msg::Quaternion msg;
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
  return msg;
}

}  // namespace aruco_imu_eskf_localization_cpp
