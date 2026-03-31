#include "eskf_localization/eskf_localization_node.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>

namespace eskf_localization
{

namespace
{

inline bool is_valid_quat(const geometry_msgs::msg::Quaternion & q)
{
  const double n2 =
    q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
  return std::isfinite(n2) && (n2 > 1e-12);
}

} // namespace

void ESKFLocalizationNode::initialpose_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr t_msg)
{
  if (!t_msg) {
    return;
  }

  // In AUTO mode, external initialpose is still useful for dev (RViz reset).
  // In EXTERNAL mode, it is the primary initialization path.
  const bool reset_always = m_node_params.init.reset_on_external_pose;

  {
    std::scoped_lock<std::mutex> lock(m_state_mutex);
    if (!reset_always && m_eskf.initialized()) {
      return;
    }
  }

  const std::string target_frame = m_node_params.io.map_frame;
  geometry_msgs::msg::PoseWithCovarianceStamped msg = *t_msg;

  // Transform to map frame if required and TF is available.
  const std::string src_frame = msg.header.frame_id;
  if (!src_frame.empty() && (src_frame != target_frame) && m_tf_buffer) {
    try {
      // RViz sometimes publishes with a zero stamp. Use the latest available TF in that case.
      const bool zero_stamp = (msg.header.stamp.sec == 0) && (msg.header.stamp.nanosec == 0);
      const auto tf =
        zero_stamp ? m_tf_buffer->lookupTransform(target_frame, src_frame, tf2::TimePointZero) :
        m_tf_buffer->lookupTransform(
        target_frame, src_frame, msg.header.stamp,
        rclcpp::Duration::from_seconds(0.2));
      tf2::doTransform(msg, msg, tf);
      msg.header.frame_id = target_frame;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(),
        "initialpose: TF transform failed (target=%s, src=%s): %s",
        target_frame.c_str(), src_frame.c_str(), ex.what());
      return;
    }
  } else if (!src_frame.empty() && (src_frame != target_frame)) {
    RCLCPP_WARN(
      this->get_logger(),
      "initialpose: frame_id mismatch (target=%s, src=%s) and TF buffer unavailable",
      target_frame.c_str(), src_frame.c_str());
    return;
  }

  if (!is_valid_quat(msg.pose.pose.orientation)) {
    RCLCPP_WARN(this->get_logger(), "initialpose: invalid quaternion, ignoring");
    return;
  }

  const auto & p = msg.pose.pose.position;
  const auto & q = msg.pose.pose.orientation;
  const Eigen::Vector3d p_map(p.x, p.y, p.z);
  const Eigen::Quaterniond q_map_from_base(q.w, q.x, q.y, q.z);

  {
    std::scoped_lock<std::mutex> lock(m_state_mutex);
    m_eskf.initialize(p_map, q_map_from_base);
    m_state_stamp = rclcpp::Time(msg.header.stamp);
    m_eskf_init_stamp_ =
      (m_state_stamp.seconds() > 0.0) ? m_state_stamp : this->now();

    // Clear any internal "pending init" state so auto-init won't immediately override.
    m_pending_init_position = false;
    m_pending_init_p_map.setZero();
    m_pending_init_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);

    // Reset last update debug snapshots on hard re-init.
    m_last_gnss_pos_update_dbg = EskfGnssPosUpdateDebug{};
    m_last_gnss_vel_update_dbg = EskfGnssVelUpdateDebug{};
    m_last_heading_yaw_update_dbg = EskfYawUpdateDebug{};
  }

  // If KISS-ICP tight coupling is enabled, force it to re-initialize from the new ESKF pose.
  if (m_kiss_icp_) {
    m_kiss_initialized_ = false;
    m_have_kiss_yaw_ref_ = false;
    std::scoped_lock<std::mutex> diag_lock(m_kiss_diag_mutex_);
    m_kiss_diag_.initialized = false;
    m_kiss_diag_.skip_reason = "external_reinit";
  }

  RCLCPP_INFO(
    this->get_logger(),
    "ESKF initialized from external initialpose (frame=%s, p=[%.3f, %.3f, %.3f])",
    msg.header.frame_id.c_str(), p_map.x(), p_map.y(), p_map.z());
}

void ESKFLocalizationNode::service_trigger_node(
  const std_srvs::srv::SetBool::Request::SharedPtr req,
  std_srvs::srv::SetBool::Response::SharedPtr res)
{
  if (!req || !res) {
    return;
  }

  if (req->data) {
    reset_imu_dt_stats();
    m_is_activated_ = true;
    res->success = true;
    res->message = "activated";
  } else {
    m_is_activated_ = false;
    res->success = true;
    res->message = "deactivated";
  }
}

} // namespace eskf_localization
