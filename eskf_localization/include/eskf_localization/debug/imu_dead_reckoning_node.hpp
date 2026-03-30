#ifndef ESKF_LOCALIZATION__IMU_DEAD_RECKONING_NODE_HPP_
#define ESKF_LOCALIZATION__IMU_DEAD_RECKONING_NODE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace eskf_localization
{

class ImuDeadReckoningNode : public rclcpp::Node
{
public:
  explicit ImuDeadReckoningNode(const rclcpp::NodeOptions & t_options);

private:
  // Callback functions
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr t_msg);

  // IMU DR helper functions
  void initialize_imu_dr(
    const geometry_msgs::msg::Point & t_position,
    const geometry_msgs::msg::Quaternion & t_orientation);
  void update_imu_dr(
    const geometry_msgs::msg::Vector3 & t_angular_velocity,
    const geometry_msgs::msg::Vector3 & t_linear_acceleration,
    double t_dt);

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;

  // TF broadcaster and listener
  std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
  std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;

  // Parameters
  std::string m_map_frame;
  std::string m_imu_dr_frame;
  std::string m_base_frame;

  // IMU Dead Reckoning state
  bool m_imu_dr_initialized{false};
  geometry_msgs::msg::Point m_p_imu_dr;      // Position (map frame)
  geometry_msgs::msg::Vector3 m_v_imu_dr;    // Velocity (map frame)
  geometry_msgs::msg::Quaternion m_q_imu_dr; // Orientation (map frame)

  // Last IMU stamp for dt computation
  rclcpp::Time m_last_imu_stamp;
};

} // namespace eskf_localization

#endif // ESKF_LOCALIZATION__IMU_DEAD_RECKONING_NODE_HPP_
