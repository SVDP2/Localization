#include "eskf_localization/eskf/odom_builder.hpp"

#include <cstddef>

namespace eskf_localization
{

nav_msgs::msg::Odometry OdomBuilder::build_uninitialized(
  const rclcpp::Time & stamp) const
{
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = stamp;
  odom_msg.header.frame_id = config_.map_frame;
  odom_msg.child_frame_id = config_.base_frame;

  odom_msg.pose.pose.position.x = 0.0;
  odom_msg.pose.pose.position.y = 0.0;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation.w = 1.0;
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = 0.0;

  for (size_t i = 0; i < 36; ++i) {
    odom_msg.pose.covariance[i] = 0.0;
    odom_msg.twist.covariance[i] = 0.0;
  }
  odom_msg.pose.covariance[0] = 1e6;
  odom_msg.pose.covariance[7] = 1e6;
  odom_msg.pose.covariance[14] = 1e6;
  odom_msg.pose.covariance[21] = 1e3;
  odom_msg.pose.covariance[28] = 1e3;
  odom_msg.pose.covariance[35] = 1e3;

  odom_msg.twist.covariance[0] = 1e3;
  odom_msg.twist.covariance[7] = 1e3;
  odom_msg.twist.covariance[14] = 1e3;
  odom_msg.twist.covariance[21] = 1e3;
  odom_msg.twist.covariance[28] = 1e3;
  odom_msg.twist.covariance[35] = 1e3;

  return odom_msg;
}

nav_msgs::msg::Odometry OdomBuilder::build_initialized(
  const rclcpp::Time & stamp,
  const Eigen::Vector3d & p_map,
  const Eigen::Vector3d & v_map,
  const Eigen::Quaterniond & q_map_from_base,
  const EskfCore::P15 & P,
  const geometry_msgs::msg::Vector3 & omega_base,
  bool have_omega) const
{
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = stamp;
  odom_msg.header.frame_id = config_.map_frame;
  odom_msg.child_frame_id = config_.base_frame;

  odom_msg.pose.pose.position.x = p_map.x();
  odom_msg.pose.pose.position.y = p_map.y();
  odom_msg.pose.pose.position.z = p_map.z();
  odom_msg.pose.pose.orientation.w = q_map_from_base.w();
  odom_msg.pose.pose.orientation.x = q_map_from_base.x();
  odom_msg.pose.pose.orientation.y = q_map_from_base.y();
  odom_msg.pose.pose.orientation.z = q_map_from_base.z();

  const Eigen::Matrix3d R = q_map_from_base.toRotationMatrix();
  const Eigen::Vector3d v_base = R.transpose() * v_map;
  odom_msg.twist.twist.linear.x = v_base.x();
  odom_msg.twist.twist.linear.y = v_base.y();
  odom_msg.twist.twist.linear.z = v_base.z();

  if (have_omega) {
    odom_msg.twist.twist.angular = omega_base;
  }

  Eigen::Matrix<double, 6, 6> cov_pose = Eigen::Matrix<double, 6, 6>::Zero();
  cov_pose.block<3, 3>(0, 0) = P.block<3, 3>(0, 0);
  cov_pose.block<3, 3>(0, 3) = P.block<3, 3>(0, 6);
  cov_pose.block<3, 3>(3, 0) = P.block<3, 3>(6, 0);
  cov_pose.block<3, 3>(3, 3) = P.block<3, 3>(6, 6);

  for (size_t r = 0; r < 6; ++r) {
    for (size_t c = 0; c < 6; ++c) {
      odom_msg.pose.covariance[r * 6 + c] = cov_pose(r, c);
    }
  }

  Eigen::Matrix<double, 6, 6> cov_twist = Eigen::Matrix<double, 6, 6>::Zero();
  const Eigen::Matrix3d cov_v_base =
    R.transpose() * P.block<3, 3>(3, 3) * R;
  cov_twist.block<3, 3>(0, 0) = cov_v_base;
  const double w_var = config_.gyro_noise_std * config_.gyro_noise_std;
  cov_twist(3, 3) = w_var;
  cov_twist(4, 4) = w_var;
  cov_twist(5, 5) = w_var;

  for (size_t r = 0; r < 6; ++r) {
    for (size_t c = 0; c < 6; ++c) {
      odom_msg.twist.covariance[r * 6 + c] = cov_twist(r, c);
    }
  }

  return odom_msg;
}

void OdomBuilder::build_tf(
  const nav_msgs::msg::Odometry & odom_msg,
  geometry_msgs::msg::TransformStamped & tf_msg) const
{
  tf_msg.header.stamp = odom_msg.header.stamp;
  tf_msg.header.frame_id = odom_msg.header.frame_id;
  tf_msg.child_frame_id = odom_msg.child_frame_id;
  tf_msg.transform.translation.x = odom_msg.pose.pose.position.x;
  tf_msg.transform.translation.y = odom_msg.pose.pose.position.y;
  tf_msg.transform.translation.z = odom_msg.pose.pose.position.z;
  tf_msg.transform.rotation = odom_msg.pose.pose.orientation;
}

} // namespace eskf_localization
