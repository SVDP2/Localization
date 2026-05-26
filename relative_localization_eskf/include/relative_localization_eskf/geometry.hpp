#ifndef ARUCO_IMU_ESKF_LOCALIZATION_CPP__GEOMETRY_HPP_
#define ARUCO_IMU_ESKF_LOCALIZATION_CPP__GEOMETRY_HPP_

#include <Eigen/Dense>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace relative_localization_eskf
{

Eigen::Matrix3d skew(const Eigen::Vector3d & v);
Eigen::Matrix3d rotation_leader_rear_from_board();
Eigen::Matrix3d rotation_board_from_leader_rear();
Eigen::Isometry3d transform_leader_rear_from_board();
Eigen::Isometry3d transform_board_from_leader_rear();
Eigen::Quaterniond normalize_quaternion(double x, double y, double z, double w);
double yaw_from_quaternion(const Eigen::Quaterniond & q);
double wrap_angle(double angle_rad);
Eigen::Isometry3d transform_from_msg(const geometry_msgs::msg::TransformStamped & msg);
geometry_msgs::msg::Quaternion quaternion_msg_from_eigen(const Eigen::Quaterniond & q);

}  // namespace relative_localization_eskf

#endif  // ARUCO_IMU_ESKF_LOCALIZATION_CPP__GEOMETRY_HPP_
