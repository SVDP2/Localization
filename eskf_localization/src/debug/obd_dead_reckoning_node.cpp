#include "eskf_localization/debug/obd_dead_reckoning_node.hpp"

#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>

namespace eskf_localization
{

ObdDeadReckoningNode::ObdDeadReckoningNode(const rclcpp::NodeOptions & t_options)
: Node("obd_dead_reckoning", t_options),
  m_last_velocity_stamp(0, 0, RCL_ROS_TIME)
{
  RCLCPP_INFO(this->get_logger(), "Initializing OBD Dead Reckoning Node");

  // Load parameters
  m_map_frame = this->declare_parameter("map_frame", "map");
  m_obd_dr_frame =
    this->declare_parameter("obd_dr_frame", "eskf_base_link_obd_dr");
  m_base_frame = this->declare_parameter("base_frame", "base_link");

  // Create TF broadcaster and listener
  m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  m_tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

  // Create subscribers
  m_velocity_sub =
    this->create_subscription<autoware_vehicle_msgs::msg::VelocityReport>(
    "/vehicle/status/velocity_status", 10,
    std::bind(
      &ObdDeadReckoningNode::velocity_callback, this,
      std::placeholders::_1));

  m_steering_sub =
    this->create_subscription<autoware_vehicle_msgs::msg::SteeringReport>(
    "/vehicle/status/steering_status", 10,
    std::bind(
      &ObdDeadReckoningNode::steering_callback, this,
      std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "OBD Dead Reckoning Node initialized");
  RCLCPP_INFO(this->get_logger(), " Map frame: %s", m_map_frame.c_str());
  RCLCPP_INFO(this->get_logger(), " OBD DR frame: %s", m_obd_dr_frame.c_str());
  RCLCPP_INFO(this->get_logger(), " Base frame: %s", m_base_frame.c_str());
  RCLCPP_INFO(this->get_logger(), " Wheelbase (L_eff): %.2f m", WHEELBASE);
  RCLCPP_INFO(this->get_logger(), "Waiting for %s TF...", m_base_frame.c_str());
}

void ObdDeadReckoningNode::velocity_callback(
  const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr t_msg)
{
  const rclcpp::Time current_stamp(t_msg->header.stamp);

  // Initialize from base_link TF if not yet initialized
  if (!m_obd_dr_initialized) {
    try {
      // Lookup base_link TF
      const auto transform = m_tf_buffer->lookupTransform(
        m_map_frame, m_base_frame, tf2::TimePointZero);

      // Extract yaw from quaternion
      const auto & q = transform.transform.rotation;
      const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
      const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
      const double yaw = std::atan2(siny_cosp, cosy_cosp);

      // Initialize OBD DR at base_link position/yaw
      geometry_msgs::msg::Point position;
      position.x = transform.transform.translation.x;
      position.y = transform.transform.translation.y;
      position.z = transform.transform.translation.z;

      initialize_obd_dr(position, yaw);
    } catch (const tf2::TransformException & ex) {
      // TF not available yet, wait
      RCLCPP_DEBUG_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 5000, "Waiting for base_link TF: %s", ex.what());
      return;
    }
  }

  // Update OBD DR if initialized
  if (m_obd_dr_initialized) {
    // Compute dt
    double dt = 0.033; // Default 30Hz
    if (m_last_velocity_stamp.seconds() > 0.0) {
      dt = (current_stamp - m_last_velocity_stamp).seconds();
      // Clamp dt to reasonable range
      if (dt < 0.0001) {
        dt = 0.0001;
      }
      if (dt > 0.5) {
        dt = 0.5;
      }
    }

    // Get velocity and steering
    const double velocity = static_cast<double>(t_msg->longitudinal_velocity);
    const double steering_angle =
      m_latest_steering ?
      static_cast<double>(m_latest_steering->steering_tire_angle) :
      0.0;

    // Update OBD DR state
    update_obd_dr(velocity, steering_angle, dt);

    // Publish TF: map -> obd_dr_frame
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = current_stamp;
    transform.header.frame_id = m_map_frame;
    transform.child_frame_id = m_obd_dr_frame;
    transform.transform.translation.x = m_p_obd_dr.x;
    transform.transform.translation.y = m_p_obd_dr.y;
    transform.transform.translation.z = m_p_obd_dr.z;

    // Convert yaw to quaternion
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, m_yaw_obd_dr);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    m_tf_broadcaster->sendTransform(transform);
  }

  m_last_velocity_stamp = current_stamp;
}

void ObdDeadReckoningNode::steering_callback(
  const autoware_vehicle_msgs::msg::SteeringReport::SharedPtr t_msg)
{
  m_latest_steering = t_msg;
}

void ObdDeadReckoningNode::initialize_obd_dr(
  const geometry_msgs::msg::Point & t_position, double t_yaw)
{
  m_p_obd_dr = t_position;
  m_yaw_obd_dr = t_yaw;
  m_obd_dr_initialized = true;

  RCLCPP_INFO(
    this->get_logger(), "OBD dead reckoning initialized at position [%.3f, %.3f, %.3f], yaw=%.3f", t_position.x, t_position.y, t_position.z,
    t_yaw);
}

void ObdDeadReckoningNode::update_obd_dr(
  double t_velocity,
  double t_steering_angle, double t_dt)
{
  if (!m_obd_dr_initialized || t_dt <= 0.0) {
    return;
  }

  // Compute yaw rate: yaw_rate = v * tan(delta) / L
  double yaw_rate = 0.0;
  if (std::isfinite(t_velocity) && std::isfinite(t_steering_angle) &&
    WHEELBASE > 0.0)
  {
    yaw_rate = t_velocity * std::tan(t_steering_angle) / WHEELBASE;
    if (!std::isfinite(yaw_rate)) {
      yaw_rate = 0.0;
    }
  }

  // 1. Yaw update
  m_yaw_obd_dr += yaw_rate * t_dt;

  // Normalize yaw to [-pi, pi]
  while (m_yaw_obd_dr > M_PI) {
    m_yaw_obd_dr -= 2.0 * M_PI;
  }
  while (m_yaw_obd_dr < -M_PI) {
    m_yaw_obd_dr += 2.0 * M_PI;
  }

  // 2. Position update (2D planar motion)
  m_p_obd_dr.x += t_velocity * std::cos(m_yaw_obd_dr) * t_dt;
  m_p_obd_dr.y += t_velocity * std::sin(m_yaw_obd_dr) * t_dt;
  // z stays constant (planar motion)
}

} // namespace eskf_localization
