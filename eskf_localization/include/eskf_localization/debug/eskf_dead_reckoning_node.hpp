#ifndef ESKF_LOCALIZATION__ESKF_DEAD_RECKONING_NODE_HPP_
#define ESKF_LOCALIZATION__ESKF_DEAD_RECKONING_NODE_HPP_

#include <memory>
#include <deque>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include "eskf_localization/eskf/eskf_core.hpp"

namespace eskf_localization
{

class EskfDeadReckoningNode : public rclcpp::Node
{
public:
  explicit EskfDeadReckoningNode(const rclcpp::NodeOptions & options);

private:
  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void velocity_callback(
    const autoware_vehicle_msgs::msg::VelocityReport::ConstSharedPtr msg);
  void steering_callback(
    const autoware_vehicle_msgs::msg::SteeringReport::ConstSharedPtr msg);
  void gt_odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  void publish_outputs(const rclcpp::Time & stamp);

  // Parameters
  std::string map_frame_;
  std::string base_frame_;
  std::string eskf_dr_frame_;
  std::string imu_topic_;
  std::string velocity_topic_;
  std::string steering_topic_;
  std::string gt_odom_topic_;

  // Vehicle constraint params (mirrors main node behavior, without GNSS gates)
  bool enable_speed_update_{true};
  bool speed_use_abs_{false};
  double speed_var_{0.25};
  double min_speed_mps_for_speed_update_{1.0};
  double speed_correction_scale_{1.0};
  double speed_correction_offset_mps_{0.0};

  bool enable_nhc_{true};
  double nhc_var_{0.04};

  bool enable_zupt_{false};
  double zupt_speed_threshold_mps_{0.2};
  double zupt_var_{0.01};

  bool enable_yaw_rate_update_{false};
  double yaw_rate_min_speed_mps_{1.0};
  double yaw_rate_var_{0.0004};
  double wheelbase_{7.0};

  // Subscribers / publishers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr
    velocity_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr
    steering_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  // TF
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // State
  EskfCore eskf_;
  bool initialized_{false};
  rclcpp::Time last_imu_stamp_;

  bool have_last_omega_{false};
  double last_omega_z_radps_{0.0};

  bool have_latest_speed_{false};
  double latest_speed_mps_{0.0};

  int path_stride_{5};
  int path_max_poses_{5000};
  int path_counter_{0};
  std::deque<geometry_msgs::msg::PoseStamped> path_poses_;
};

} // namespace eskf_localization

#endif // ESKF_LOCALIZATION__ESKF_DEAD_RECKONING_NODE_HPP_
