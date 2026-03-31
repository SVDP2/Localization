#include "eskf_localization/eskf_localization_node.hpp"

#include <cmath>

namespace eskf_localization
{

// NOTE: Vehicle 콜백은 속도/조향 기반의 제약 업데이트를 담당한다.
void ESKFLocalizationNode::velocity_callback(
  const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr t_msg)
{
  const bool activated = (!m_node_params.init.require_trigger) || m_is_activated_;

  const rclcpp::Time current_stamp(t_msg->header.stamp);
  const rclcpp::Time now = this->now();
  if (!validate_stamp_and_order(
      current_stamp, now, m_last_velocity_stamp,
      "Velocity"))
  {
    return;
  }

  m_latest_velocity = t_msg;
  m_last_velocity_stamp = current_stamp;
  m_velocity_count++;

  if (!activated) {
    return;
  }

  const auto & constraints = m_node_params.vehicle_constraints;
  if (!(constraints.enable_speed_update || constraints.enable_nhc ||
    constraints.enable_zupt))
  {
    return;
  }

  if (!std::isfinite(constraints.speed_var) || !(constraints.speed_var > 0.0) ||
    !std::isfinite(constraints.nhc_var) || !(constraints.nhc_var > 0.0) ||
    !std::isfinite(constraints.zupt_var) || !(constraints.zupt_var > 0.0))
  {
    return;
  }

  const double v_raw = static_cast<double>(t_msg->longitudinal_velocity);
  const double v_abs = std::abs(v_raw);
  const double v_meas =
    constraints.speed_use_abs ? v_abs : v_raw;

  // 정지 판단
  // - 1차: OBD 속도 기반 정지 후보
  // - 2차: GNSS 속도가 1km/h 이상이면 저속 크립
  // ZUPT는 "OBD 정지 후보" + "GNSS 속도도 충분히 저속"일 때만 적용
  static constexpr double kPvtZuptSpeedGateMps = 1.0 / 3.6;
  bool is_stationary =
    constraints.enable_zupt && std::isfinite(v_abs) &&
    (v_abs < constraints.zupt_speed_threshold_mps);
  if (is_stationary && m_latest_gnss_vel && (m_last_gnss_vel_stamp.seconds() > 0.0)) {
    const double dt_pvt_sec =
      std::abs((current_stamp - m_last_gnss_vel_stamp).seconds());
    if (std::isfinite(dt_pvt_sec) && (dt_pvt_sec < 0.5) &&
      std::isfinite(m_latest_gnss_vel->twist.linear.x) &&
      std::isfinite(m_latest_gnss_vel->twist.linear.y))
    {
      const double ve = static_cast<double>(m_latest_gnss_vel->twist.linear.x);
      const double vn = static_cast<double>(m_latest_gnss_vel->twist.linear.y);
      const double pvt_speed_mps = std::sqrt(
        vn * vn + ve * ve);
      if (std::isfinite(pvt_speed_mps) && (pvt_speed_mps >= kPvtZuptSpeedGateMps)) {
        is_stationary = false;
      }
    }
  }

  std::scoped_lock<std::mutex> lock(m_state_mutex);
  if (!m_eskf.initialized()) {
    return;
  }
  const auto tol = rclcpp::Duration::from_seconds(
    std::max(0.0, m_node_params.time_alignment_tolerance_sec));
  if (m_state_stamp.seconds() > 0.0 && (current_stamp + tol) < m_state_stamp) {
    return;
  }

  bool any_applied = false;

  // 속도 업데이트 또는 ZUPT
  if (is_stationary) {
    const auto dbg =
      m_eskf.update_body_velocity_component(0, 0.0, constraints.zupt_var);
    any_applied = any_applied || dbg.applied;
  } else if (constraints.enable_speed_update && std::isfinite(v_meas)) {
    const double v_gate = std::abs(v_meas);
    if (constraints.min_speed_mps_for_speed_update > 0.0 &&
      (!std::isfinite(v_gate) ||
      v_gate < constraints.min_speed_mps_for_speed_update))
    {
    } else {
      const auto dbg = m_eskf.update_body_velocity_component(
        0, v_meas, constraints.speed_var);
      any_applied = any_applied || dbg.applied;
    }
  }

  // NHC (측면/수직 속도 ≈ 0)
  if (constraints.enable_nhc) {
    const auto dbg_y =
      m_eskf.update_body_velocity_component(1, 0.0, constraints.nhc_var);
    const auto dbg_z =
      m_eskf.update_body_velocity_component(2, 0.0, constraints.nhc_var);
    any_applied = any_applied || dbg_y.applied || dbg_z.applied;
  }

  if (any_applied) {
    if (current_stamp > m_state_stamp) {
      m_state_stamp = current_stamp;
    }
  }
}

void ESKFLocalizationNode::steering_callback(
  const autoware_vehicle_msgs::msg::SteeringReport::SharedPtr t_msg)
{
  const bool activated = (!m_node_params.init.require_trigger) || m_is_activated_;

  const rclcpp::Time current_stamp(t_msg->stamp);
  const rclcpp::Time now = this->now();
  if (!validate_stamp_and_order(
      current_stamp, now, m_last_steering_stamp,
      "Steering"))
  {
    return;
  }

  m_latest_steering = t_msg;
  m_last_steering_stamp = current_stamp;
  m_steering_count++;

  if (!activated) {
    return;
  }

  const auto & constraints = m_node_params.vehicle_constraints;
  if (!constraints.enable_yaw_rate_update) {
    return;
  }

  // yaw rate 업데이트는 속도/각속도 정보가 필요
  if (!m_latest_velocity) {
    return;
  }
  if (!m_have_last_omega_base) {
    return;
  }
  if (!std::isfinite(m_node_params.vehicle.wheelbase) ||
    !(m_node_params.vehicle.wheelbase > 0.0) ||
    !std::isfinite(constraints.yaw_rate_var) ||
    !(constraints.yaw_rate_var > 0.0))
  {
    return;
  }

  const double v =
    static_cast<double>(m_latest_velocity->longitudinal_velocity);
  if (!std::isfinite(v) || (std::abs(v) < constraints.yaw_rate_min_speed_mps)) {
    return;
  }

  const double delta = static_cast<double>(t_msg->steering_tire_angle);
  if (!std::isfinite(delta)) {
    return;
  }

  // 기준 yaw rate 계산 (v*tan(delta)/L)
  const double yaw_rate_ref =
    v * std::tan(delta) / m_node_params.vehicle.wheelbase;
  if (!std::isfinite(yaw_rate_ref)) {
    return;
  }

  const double omega_meas_z = static_cast<double>(m_last_omega_base.z);
  if (!std::isfinite(omega_meas_z)) {
    return;
  }

  std::scoped_lock<std::mutex> lock(m_state_mutex);
  if (!m_eskf.initialized()) {
    return;
  }
  const auto tol = rclcpp::Duration::from_seconds(
    std::max(0.0, m_node_params.time_alignment_tolerance_sec));
  if (m_state_stamp.seconds() > 0.0 && (current_stamp + tol) < m_state_stamp) {
    return;
  }

  const auto dbg = m_eskf.update_yaw_rate_from_steer(
    omega_meas_z, yaw_rate_ref, constraints.yaw_rate_var);
  if (dbg.applied) {
    if (current_stamp > m_state_stamp) {
      m_state_stamp = current_stamp;
    }
  }
}

} // namespace eskf_localization
