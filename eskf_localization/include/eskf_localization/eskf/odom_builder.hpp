#ifndef ESKF_LOCALIZATION__ESKF__ODOM_BUILDER_HPP_
#define ESKF_LOCALIZATION__ESKF__ODOM_BUILDER_HPP_

#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "eskf_localization/eskf/eskf_core.hpp"

namespace eskf_localization
{

struct OdomBuilderConfig
{
  std::string map_frame{"map"};
  std::string base_frame{"base_link"};
  double gyro_noise_std{0.02};
};

class OdomBuilder
{
public:
  explicit OdomBuilder(const OdomBuilderConfig & config = OdomBuilderConfig{})
  : config_(config) {}

  void set_config(const OdomBuilderConfig & config) {config_ = config;}
  const OdomBuilderConfig & config() const {return config_;}

  nav_msgs::msg::Odometry build_uninitialized(const rclcpp::Time & stamp) const;

  nav_msgs::msg::Odometry build_initialized(
    const rclcpp::Time & stamp,
    const Eigen::Vector3d & p_map,
    const Eigen::Vector3d & v_map,
    const Eigen::Quaterniond & q_map_from_base,
    const EskfCore::P15 & P,
    const geometry_msgs::msg::Vector3 & omega_base,
    bool have_omega) const;

  void build_tf(
    const nav_msgs::msg::Odometry & odom_msg,
    geometry_msgs::msg::TransformStamped & tf_msg) const;

private:
  OdomBuilderConfig config_{};
};

} // namespace eskf_localization

#endif // ESKF_LOCALIZATION__ESKF__ODOM_BUILDER_HPP_
