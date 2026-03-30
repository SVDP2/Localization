#include "eskf_localization/eskf_localization_node.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace eskf_localization
{

namespace
{

inline double wrap_to_pi(const double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

inline double yaw_from_rot(const Eigen::Matrix3d & R)
{
  return std::atan2(R(1, 0), R(0, 0));
}

inline double clamp01(const double x)
{
  if (!std::isfinite(x)) {
    return 0.0;
  }
  return std::clamp(x, 0.0, 1.0);
}

} // namespace

void ESKFLocalizationNode::pointcloud_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr t_msg)
{
  const bool activated = (!m_node_params.init.require_trigger) || m_is_activated_;

  const auto reset_kiss_diag_snapshot = [this]() {
      std::scoped_lock<std::mutex> diag_lock(m_kiss_diag_mutex_);
      m_kiss_diag_.enabled = m_node_params.kiss_icp.enable;
      m_kiss_diag_.initialized = m_kiss_initialized_;
      m_kiss_diag_.yaw_enabled = m_node_params.kiss_icp.enable_yaw_update;
      m_kiss_diag_.vy_enabled = m_node_params.kiss_icp.enable_vy_update;
      m_kiss_diag_.source_points = 0;
      m_kiss_diag_.dt_ms = 0.0;
      m_kiss_diag_.trust = m_kiss_trust_;
      m_kiss_diag_.target_trust = m_kiss_trust_;
      m_kiss_diag_.yaw_rate_radps = 0.0;
      m_kiss_diag_.vy_mps = 0.0;
      m_kiss_diag_.yaw_var_eff = 0.0;
      m_kiss_diag_.vy_var_eff = 0.0;
      m_kiss_diag_.yaw_applied = false;
      m_kiss_diag_.yaw_reason.clear();
      m_kiss_diag_.yaw_nis = 0.0;
      m_kiss_diag_.yaw_R = 0.0;
      m_kiss_diag_.vy_applied = false;
      m_kiss_diag_.vy_reason.clear();
      m_kiss_diag_.vy_nis = 0.0;
      m_kiss_diag_.vy_R = 0.0;
      m_kiss_diag_.skip_reason.clear();
      m_kiss_diag_.time_alignment_error_ms = 0.0;
      m_kiss_diag_.reset_candidate = m_kiss_reset_candidate_;
      m_kiss_diag_.reset_count = m_kiss_reset_count_;
    };
  const auto set_kiss_skip_reason = [this](const std::string & reason) {
      std::scoped_lock<std::mutex> diag_lock(m_kiss_diag_mutex_);
      m_kiss_diag_.skip_reason = reason;
      m_kiss_diag_.initialized = m_kiss_initialized_;
      m_kiss_diag_.reset_candidate = m_kiss_reset_candidate_;
      m_kiss_diag_.reset_count = m_kiss_reset_count_;
    };

  reset_kiss_diag_snapshot();

  if (!m_node_params.kiss_icp.enable || !m_kiss_icp_) {
    set_kiss_skip_reason("disabled");
    return;
  }

  if (!activated) {
    set_kiss_skip_reason("not_activated");
    return;
  }

  const rclcpp::Time current_stamp(t_msg->header.stamp);
  const rclcpp::Time now = this->now();
  if (!validate_stamp_and_order(
      current_stamp, now, m_last_pointcloud_stamp,
      "KISS-ICP"))
  {
    set_kiss_skip_reason("invalid_stamp_or_order");
    return;
  }
  m_last_pointcloud_stamp = current_stamp;

  // Only do heavy ICP work after ESKF is initialized (tight-coupled mode).
  Sophus::SE3d T_eskf_init;
  bool eskf_initialized = false;
  bool in_init_grace = false;
  {
    std::scoped_lock<std::mutex> lock(m_state_mutex);
    eskf_initialized = m_eskf.initialized();
    if (!eskf_initialized) {
      m_have_kiss_yaw_ref_ = false;
    } else {
      T_eskf_init = Sophus::SE3d(m_eskf.q_map_from_base(), m_eskf.p_map());
      const double since_init_sec = (current_stamp - m_eskf_init_stamp_).seconds();
      in_init_grace =
        std::isfinite(since_init_sec) &&
        (since_init_sec < k_heading_rate_gate_init_grace_sec_);
    }
  }
  if (!eskf_initialized) {
    set_kiss_skip_reason("eskf_not_initialized");
    return;
  }
  // Match heading path behavior: skip KISS fusion briefly after ESKF init.
  if (in_init_grace) {
    set_kiss_skip_reason("init_grace");
    return;
  }

  // Initialize KISS-ICP pose in map frame once ESKF is ready.
  if (!m_kiss_initialized_) {
    m_kiss_icp_->Reset();
    m_kiss_icp_->pose() = T_eskf_init;
    m_kiss_initialized_ = true;
    m_last_kiss_update_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    m_have_kiss_yaw_ref_ = false;
    m_kiss_trust_ = 0.0;
    {
      std::scoped_lock<std::mutex> diag_lock(m_kiss_diag_mutex_);
      m_kiss_diag_.initialized = true;
    }
  }

  const size_t n_points =
    static_cast<size_t>(t_msg->width) * static_cast<size_t>(t_msg->height);
  if (n_points == 0) {
    set_kiss_skip_reason("empty_cloud");
    return;
  }

  std::vector<Eigen::Vector3d> points;
  points.reserve(n_points);
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*t_msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*t_msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*t_msg, "z");
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    const float x = *iter_x;
    const float y = *iter_y;
    const float z = *iter_z;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    points.emplace_back(
      static_cast<double>(x), static_cast<double>(y),
      static_cast<double>(z));
  }
  if (points.empty()) {
    set_kiss_skip_reason("empty_points");
    return;
  }

  // For initial integration, pass empty per-point timestamps.
  // If deskew is enabled in KISS-ICP, it will be skipped when timestamps are
  // empty.
  const std::vector<double> timestamps;

  const auto &[frame, source] = m_kiss_icp_->RegisterFrame(points, timestamps);
  (void)frame;

  {
    std::scoped_lock<std::mutex> diag_lock(m_kiss_diag_mutex_);
    m_kiss_diag_.source_points = static_cast<int>(source.size());
    m_kiss_diag_.initialized = m_kiss_initialized_;
  }

  if (static_cast<int>(source.size()) < m_node_params.kiss_icp.min_source_points) {
    set_kiss_skip_reason("min_source_points");
    return;
  }

  if (m_node_params.kiss_icp.publish_debug_tf && m_tf_broadcaster) {
    const Sophus::SE3d T_kiss = m_kiss_icp_->pose();
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = t_msg->header.stamp;
    tf_msg.header.frame_id = m_node_params.io.map_frame;
    tf_msg.child_frame_id = m_node_params.kiss_icp.debug_child_frame;
    tf_msg.transform.translation.x = T_kiss.translation().x();
    tf_msg.transform.translation.y = T_kiss.translation().y();
    tf_msg.transform.translation.z = T_kiss.translation().z();
    const Eigen::Quaterniond q = T_kiss.unit_quaternion();
    tf_msg.transform.rotation.w = q.w();
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    m_tf_broadcaster->sendTransform(tf_msg);
  }

  if (m_last_kiss_update_stamp.seconds() <= 0.0) {
    m_last_kiss_update_stamp = current_stamp;
    set_kiss_skip_reason("bootstrap_dt");
    return;
  }
  const double dt = (current_stamp - m_last_kiss_update_stamp).seconds();
  m_last_kiss_update_stamp = current_stamp;
  {
    std::scoped_lock<std::mutex> diag_lock(m_kiss_diag_mutex_);
    m_kiss_diag_.dt_ms = dt * 1e3;
  }
  if (!std::isfinite(dt) || !(dt > 0.0)) {
    set_kiss_skip_reason("invalid_dt");
    return;
  }

  const Sophus::SE3d delta = m_kiss_icp_->delta();
  const Eigen::Vector3d dp = delta.translation();
  const Eigen::Matrix3d dR = delta.so3().matrix();
  const double yaw_delta = wrap_to_pi(yaw_from_rot(dR));
  const double yaw_rate = yaw_delta / dt;
  const double vy_mps = dp.y() / dt;
  {
    std::scoped_lock<std::mutex> diag_lock(m_kiss_diag_mutex_);
    m_kiss_diag_.yaw_rate_radps = yaw_rate;
    m_kiss_diag_.vy_mps = vy_mps;
  }

  if (std::isfinite(m_node_params.kiss_icp.max_abs_yaw_rate_radps) &&
    m_node_params.kiss_icp.max_abs_yaw_rate_radps > 0.0 &&
    std::isfinite(yaw_rate) &&
    std::abs(yaw_rate) > m_node_params.kiss_icp.max_abs_yaw_rate_radps)
  {
    set_kiss_skip_reason("yaw_rate_gate");
    return;
  }
  if (std::isfinite(m_node_params.kiss_icp.max_abs_vy_mps) &&
    m_node_params.kiss_icp.max_abs_vy_mps > 0.0 && std::isfinite(vy_mps) &&
    std::abs(vy_mps) > m_node_params.kiss_icp.max_abs_vy_mps)
  {
    set_kiss_skip_reason("vy_gate");
    return;
  }

  // Trust ramping by GNSS status (low-pass filtered).
  const auto latest_gnss = m_latest_gnss;
  const int gnss_status = latest_gnss ? static_cast<int>(latest_gnss->status.status) : -999;
  const double target_trust = [&]() -> double {
      if (gnss_status < 0) {
        return m_node_params.kiss_icp.trust_status_neg;
      }
      if (gnss_status == 0) {
        return m_node_params.kiss_icp.trust_status_0;
      }
      if (gnss_status == 1) {
        return m_node_params.kiss_icp.trust_status_1;
      }
      return m_node_params.kiss_icp.trust_status_2;
    }();
  {
    std::scoped_lock<std::mutex> diag_lock(m_kiss_diag_mutex_);
    m_kiss_diag_.target_trust = target_trust;
  }

  const double tau = std::max(0.0, m_node_params.kiss_icp.trust_tau_sec);
  const double alpha = (tau > 1e-6) ? clamp01(dt / tau) : 1.0;
  m_kiss_trust_ = (1.0 - alpha) * m_kiss_trust_ + alpha * clamp01(target_trust);
  const double trust = std::clamp(m_kiss_trust_, 0.05, 1.0);
  {
    std::scoped_lock<std::mutex> diag_lock(m_kiss_diag_mutex_);
    m_kiss_diag_.trust = m_kiss_trust_;
  }

  double yaw_var = m_node_params.kiss_icp.yaw_var;
  double vy_var = m_node_params.kiss_icp.vy_var;
  if (!std::isfinite(yaw_var) || !(yaw_var > 0.0)) {
    yaw_var = 0.05;
  }
  if (!std::isfinite(vy_var) || !(vy_var > 0.0)) {
    vy_var = 0.25;
  }
  // Lower trust -> larger variance (weaker update)
  yaw_var /= trust;
  vy_var /= trust;
  {
    std::scoped_lock<std::mutex> diag_lock(m_kiss_diag_mutex_);
    m_kiss_diag_.yaw_var_eff = yaw_var;
    m_kiss_diag_.vy_var_eff = vy_var;
  }

  std::scoped_lock<std::mutex> lock(m_state_mutex);
  // Time alignment check vs current state stamp
  const auto tol = rclcpp::Duration::from_seconds(
    std::max(0.0, m_node_params.kiss_icp.time_alignment_tolerance_sec));
  if (m_state_stamp.seconds() > 0.0 && (current_stamp + tol) < m_state_stamp) {
    const double alignment_error_ms =
      std::max(0.0, (m_state_stamp - current_stamp).seconds() * 1e3);
    {
      std::scoped_lock<std::mutex> diag_lock(m_kiss_diag_mutex_);
      m_kiss_diag_.time_alignment_error_ms = alignment_error_ms;
    }
    set_kiss_skip_reason("time_alignment");
    return;
  }

  // Initialize yaw reference on first usable frame
  const double yaw_pred = yaw_from_rot(m_eskf.q_map_from_base().toRotationMatrix());
  if (!m_have_kiss_yaw_ref_) {
    m_last_kiss_yaw_ref_rad_ = yaw_pred;
    m_have_kiss_yaw_ref_ = true;
    set_kiss_skip_reason("init_yaw_ref");
    return;
  }

  bool any_applied = false;

  if (m_node_params.kiss_icp.enable_yaw_update) {
    // Convert relative yaw measurement into an absolute yaw measurement using
    // the last yaw reference. This yields residual:
    //   r = wrap(z_yaw - yaw_pred) = wrap(yaw_delta - (yaw_pred - yaw_ref))
    const double z_yaw = wrap_to_pi(m_last_kiss_yaw_ref_rad_ + yaw_delta);
    const auto dbg = m_eskf.update_heading_yaw(z_yaw, yaw_var);
    any_applied = any_applied || dbg.applied;
    {
      std::scoped_lock<std::mutex> diag_lock(m_kiss_diag_mutex_);
      m_kiss_diag_.yaw_applied = dbg.applied;
      m_kiss_diag_.yaw_nis = dbg.nis;
      m_kiss_diag_.yaw_R = dbg.R;
      if (!dbg.reason.empty()) {
        m_kiss_diag_.yaw_reason = dbg.reason;
      } else if (dbg.applied) {
        m_kiss_diag_.yaw_reason = "kiss_yaw";
      }
    }
    if (dbg.applied) {
      const double yaw_after =
        yaw_from_rot(m_eskf.q_map_from_base().toRotationMatrix());
      m_last_kiss_yaw_ref_rad_ = yaw_after;
    }
  } else {
    std::scoped_lock<std::mutex> diag_lock(m_kiss_diag_mutex_);
    m_kiss_diag_.yaw_reason = "disabled";
  }

  if (m_node_params.kiss_icp.enable_vy_update) {
    const auto dbg = m_eskf.update_body_velocity_component(1, vy_mps, vy_var);
    any_applied = any_applied || dbg.applied;
    {
      std::scoped_lock<std::mutex> diag_lock(m_kiss_diag_mutex_);
      m_kiss_diag_.vy_applied = dbg.applied;
      m_kiss_diag_.vy_nis = dbg.nis;
      m_kiss_diag_.vy_R = dbg.R;
      if (!dbg.reason.empty()) {
        m_kiss_diag_.vy_reason = dbg.reason;
      } else if (dbg.applied) {
        m_kiss_diag_.vy_reason = "kiss_vy";
      }
    }
  } else {
    std::scoped_lock<std::mutex> diag_lock(m_kiss_diag_mutex_);
    m_kiss_diag_.vy_reason = "disabled";
  }

  if (any_applied && current_stamp > m_state_stamp) {
    m_state_stamp = current_stamp;
  }

  // Optional hard reset when ESKF is trusted and KISS drifts far away.
  if (m_node_params.kiss_icp.enable_auto_reset && gnss_status >= 2) {
    const Sophus::SE3d T_kiss = m_kiss_icp_->pose();
    const Sophus::SE3d T_eskf(m_eskf.q_map_from_base(), m_eskf.p_map());
    const Sophus::SE3d T_err = T_eskf * T_kiss.inverse();
    const double err_p = T_err.translation().norm();
    const double err_yaw = wrap_to_pi(yaw_from_rot(T_err.so3().matrix()));
    const bool over =
      (std::isfinite(err_p) && err_p > m_node_params.kiss_icp.auto_reset_err_pos_m) ||
      (std::isfinite(err_yaw) && std::abs(err_yaw) > m_node_params.kiss_icp.auto_reset_err_yaw_rad);

    if (over) {
      if (!m_kiss_reset_candidate_) {
        m_kiss_reset_candidate_ = true;
        m_kiss_reset_candidate_since = current_stamp;
      } else {
        const double held =
          (current_stamp - m_kiss_reset_candidate_since).seconds();
        if (std::isfinite(held) &&
          held > std::max(0.0, m_node_params.kiss_icp.auto_reset_hold_sec))
        {
          m_kiss_icp_->Reset();
          m_kiss_icp_->pose() = T_eskf;
          m_have_kiss_yaw_ref_ = false;
          m_kiss_reset_candidate_ = false;
          ++m_kiss_reset_count_;
        }
      }
    } else {
      m_kiss_reset_candidate_ = false;
    }
  } else {
    m_kiss_reset_candidate_ = false;
  }

  {
    std::scoped_lock<std::mutex> diag_lock(m_kiss_diag_mutex_);
    m_kiss_diag_.initialized = m_kiss_initialized_;
    m_kiss_diag_.reset_candidate = m_kiss_reset_candidate_;
    m_kiss_diag_.reset_count = m_kiss_reset_count_;
    if (!any_applied && m_kiss_diag_.skip_reason.empty()) {
      m_kiss_diag_.skip_reason = "no_update_applied";
    }
  }
}

} // namespace eskf_localization
