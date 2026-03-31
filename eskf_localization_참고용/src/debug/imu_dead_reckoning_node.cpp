#include "eskf_localization/debug/imu_dead_reckoning_node.hpp"

#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>

namespace eskf_localization
{

ImuDeadReckoningNode::ImuDeadReckoningNode(const rclcpp::NodeOptions & t_options)
: Node("imu_dead_reckoning", t_options),
  m_last_imu_stamp(0, 0, RCL_ROS_TIME)
{
  RCLCPP_INFO(this->get_logger(), "Initializing IMU Dead Reckoning Node");

  // Load parameters
  m_map_frame = this->declare_parameter("map_frame", "map");
  m_imu_dr_frame =
    this->declare_parameter("imu_dr_frame", "eskf_base_link_imu_dr");
  m_base_frame = this->declare_parameter("base_frame", "base_link");

  // Create TF broadcaster and listener
  m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  m_tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

  // Create subscribers
  m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
    "/sensing/imu/imu_data", 10,
    std::bind(
      &ImuDeadReckoningNode::imu_callback, this,
      std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "IMU Dead Reckoning Node initialized");
  RCLCPP_INFO(this->get_logger(), " Map frame: %s", m_map_frame.c_str());
  RCLCPP_INFO(this->get_logger(), " IMU DR frame: %s", m_imu_dr_frame.c_str());
  RCLCPP_INFO(this->get_logger(), " Base frame: %s", m_base_frame.c_str());
  RCLCPP_INFO(this->get_logger(), "Waiting for %s TF...", m_base_frame.c_str());
}

void ImuDeadReckoningNode::imu_callback(
  const sensor_msgs::msg::Imu::SharedPtr t_msg)
{
  const rclcpp::Time current_stamp(t_msg->header.stamp);

  // Initialize from base_link TF if not yet initialized
  if (!m_imu_dr_initialized) {
    try {
      // Lookup base_link TF
      const auto transform = m_tf_buffer->lookupTransform(
        m_map_frame, m_base_frame, tf2::TimePointZero);

      // Initialize IMU DR at eskf_base_link position/orientation
      geometry_msgs::msg::Point position;
      position.x = transform.transform.translation.x;
      position.y = transform.transform.translation.y;
      position.z = transform.transform.translation.z;

      initialize_imu_dr(position, transform.transform.rotation);
    } catch (const tf2::TransformException & ex) {
      // TF not available yet, wait
      RCLCPP_DEBUG_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 5000, "Waiting for base_link TF: %s", ex.what());
      return;
    }
  }

  // Update IMU DR if initialized
  if (m_imu_dr_initialized) {
    // Compute dt
    double dt = 0.02; // Default 50Hz
    if (m_last_imu_stamp.seconds() > 0.0) {
      dt = (current_stamp - m_last_imu_stamp).seconds();
      // Clamp dt to reasonable range
      if (dt < 0.0001) {
        dt = 0.0001;
      }
      if (dt > 0.5) {
        dt = 0.5;
      }
    }

    // Update IMU DR state
    update_imu_dr(t_msg->angular_velocity, t_msg->linear_acceleration, dt);

    // Publish TF: map -> imu_dr_frame
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = current_stamp;
    transform.header.frame_id = m_map_frame;
    transform.child_frame_id = m_imu_dr_frame;
    transform.transform.translation.x = m_p_imu_dr.x;
    transform.transform.translation.y = m_p_imu_dr.y;
    transform.transform.translation.z = m_p_imu_dr.z;
    transform.transform.rotation = m_q_imu_dr;

    m_tf_broadcaster->sendTransform(transform);
  }

  m_last_imu_stamp = current_stamp;
}

void ImuDeadReckoningNode::initialize_imu_dr(
  const geometry_msgs::msg::Point & t_position,
  const geometry_msgs::msg::Quaternion & t_orientation)
{
  m_p_imu_dr = t_position;
  m_q_imu_dr = t_orientation;
  m_v_imu_dr.x = 0.0;
  m_v_imu_dr.y = 0.0;
  m_v_imu_dr.z = 0.0;
  m_imu_dr_initialized = true;

  RCLCPP_INFO(
    this->get_logger(), "IMU dead reckoning initialized at position [%.3f, %.3f, %.3f]", t_position.x, t_position.y,
    t_position.z);
}

void ImuDeadReckoningNode::update_imu_dr(
  const geometry_msgs::msg::Vector3 & t_angular_velocity,
  const geometry_msgs::msg::Vector3 & t_linear_acceleration, double t_dt)
{
  if (!m_imu_dr_initialized || t_dt <= 0.0) {
    return;
  }

  // 1. Orientation update: integrate angular velocity
  // quaternion derivative: q_dot = 0.5 * q * omega (quaternion multiplication)
  const double wx = t_angular_velocity.x;
  const double wy = t_angular_velocity.y;
  const double wz = t_angular_velocity.z;

  // Quaternion representation of angular velocity (pure quaternion)
  // omega_quat = [0, wx, wy, wz]
  const double qw = m_q_imu_dr.w;
  const double qx = m_q_imu_dr.x;
  const double qy = m_q_imu_dr.y;
  const double qz = m_q_imu_dr.z;

  // q_dot = 0.5 * q * omega_quat
  const double dqw = 0.5 * (-qx * wx - qy * wy - qz * wz);
  const double dqx = 0.5 * (qw * wx + qy * wz - qz * wy);
  const double dqy = 0.5 * (qw * wy - qx * wz + qz * wx);
  const double dqz = 0.5 * (qw * wz + qx * wy - qy * wx);

  // Integrate: q = q + q_dot * dt
  m_q_imu_dr.w += dqw * t_dt;
  m_q_imu_dr.x += dqx * t_dt;
  m_q_imu_dr.y += dqy * t_dt;
  m_q_imu_dr.z += dqz * t_dt;

  // Normalize quaternion
  const double norm =
    std::sqrt(
    m_q_imu_dr.w * m_q_imu_dr.w + m_q_imu_dr.x * m_q_imu_dr.x +
    m_q_imu_dr.y * m_q_imu_dr.y + m_q_imu_dr.z * m_q_imu_dr.z);
  if (norm > 0.0) {
    m_q_imu_dr.w /= norm;
    m_q_imu_dr.x /= norm;
    m_q_imu_dr.y /= norm;
    m_q_imu_dr.z /= norm;
  }

  // 2. Transform acceleration from body frame to world frame
  // a_world = R(q) * a_body
  // Using quaternion rotation: v' = q * v * q^-1
  const double ax = t_linear_acceleration.x;
  const double ay = t_linear_acceleration.y;
  const double az = t_linear_acceleration.z;

  // Rotate acceleration vector by quaternion
  // a_world = R(q) * a_body
  const double ax_world = (1.0 - 2.0 * (qy * qy + qz * qz)) * ax +
    2.0 * (qx * qy - qw * qz) * ay +
    2.0 * (qx * qz + qw * qy) * az;
  const double ay_world = 2.0 * (qx * qy + qw * qz) * ax +
    (1.0 - 2.0 * (qx * qx + qz * qz)) * ay +
    2.0 * (qy * qz - qw * qx) * az;
  const double az_world = 2.0 * (qx * qz - qw * qy) * ax +
    2.0 * (qy * qz + qw * qx) * ay +
    (1.0 - 2.0 * (qx * qx + qy * qy)) * az;

  // 3. Velocity update: v = v + a_world * dt
  m_v_imu_dr.x += ax_world * t_dt;
  m_v_imu_dr.y += ay_world * t_dt;
  m_v_imu_dr.z += az_world * t_dt;

  // 4. Position update: p = p + v * dt
  m_p_imu_dr.x += m_v_imu_dr.x * t_dt;
  m_p_imu_dr.y += m_v_imu_dr.y * t_dt;
  m_p_imu_dr.z += m_v_imu_dr.z * t_dt;
}

} // namespace eskf_localization
