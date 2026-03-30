#ifndef ESKF_LOCALIZATION__OBD_DEAD_RECKONING_NODE_HPP_
#define ESKF_LOCALIZATION__OBD_DEAD_RECKONING_NODE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

namespace eskf_localization
{

class ObdDeadReckoningNode : public rclcpp::Node
{
public:
  explicit ObdDeadReckoningNode(const rclcpp::NodeOptions & t_options);

private:
  // Callback functions
  void velocity_callback(
    const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr t_msg);
  void steering_callback(
    const autoware_vehicle_msgs::msg::SteeringReport::SharedPtr t_msg);

  // OBD DR helper functions
  void initialize_obd_dr(
    const geometry_msgs::msg::Point & t_position,
    double t_yaw);
  void update_obd_dr(double t_velocity, double t_steering_angle, double t_dt);

  // Subscribers
  rclcpp::Subscription<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr
    m_velocity_sub;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr
    m_steering_sub;

  // TF broadcaster and listener
  std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
  std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;

  // Parameters
  std::string m_map_frame;
  std::string m_obd_dr_frame;
  std::string m_base_frame;

  // Vehicle constants
  static constexpr double WHEELBASE = 10.0; // L_eff, default 6.67

  // OBD Dead Reckoning state
  bool m_obd_dr_initialized{false};
  geometry_msgs::msg::Point m_p_obd_dr; // Position (map frame)
  double m_yaw_obd_dr{0.0};             // Yaw (map frame, radians)

  // Latest steering for yaw rate computation
  autoware_vehicle_msgs::msg::SteeringReport::SharedPtr m_latest_steering;

  // Last velocity stamp for dt computation
  rclcpp::Time m_last_velocity_stamp;
};

} // namespace eskf_localization

#endif // ESKF_LOCALIZATION__OBD_DEAD_RECKONING_NODE_HPP_
