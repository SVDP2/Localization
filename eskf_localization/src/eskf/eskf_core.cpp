#include "eskf_localization/eskf/eskf_core.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Cholesky>

namespace eskf_localization
{

namespace
{

inline double wrap_to_pi(const double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

} // namespace

// NOTE: ESKF 코어는 상태/예측/업데이트를 한 파일에서 유지하되, 수치 안정화 헬퍼를 공용화한다.
EskfCore::EskfCore(const EskfCoreParams & params)
: m_params(params)
{
  m_P.setZero();
}

bool EskfCore::finite() const
{
  const Eigen::Vector4d q(m_q_map_from_base.w(), m_q_map_from_base.x(),
    m_q_map_from_base.y(), m_q_map_from_base.z());
  return m_p_map.allFinite() && m_v_map.allFinite() && q.allFinite() &&
         m_b_g.allFinite() && m_b_a.allFinite() && m_P.allFinite();
}

void EskfCore::reset()
{
  m_initialized = false;
  m_p_map.setZero();
  m_v_map.setZero();
  m_q_map_from_base = Eigen::Quaterniond::Identity();
  m_b_g.setZero();
  m_b_a.setZero();
  m_P.setZero();
  m_r_eff_inflate_gnss_pos_ = 1.0;
  m_r_eff_inflate_gnss_vel_ = 1.0;
  m_r_eff_inflate_heading_yaw_ = 1.0;
}

void EskfCore::initialize(
  const Eigen::Vector3d & p_map,
  const Eigen::Quaterniond & q_map_from_base)
{
  m_initialized = true;

  m_p_map = p_map;
  m_v_map.setZero();
  m_q_map_from_base = q_map_from_base.normalized();
  m_b_g.setZero();
  m_b_a.setZero();

  m_P.setZero();
  m_P.block<3, 3>(0, 0).diagonal().setConstant(m_params.init_pos_var);
  m_P.block<3, 3>(3, 3).diagonal().setConstant(m_params.init_vel_var);
  m_P.block<3, 3>(6, 6).diagonal().setConstant(m_params.init_att_var);
  m_P.block<3, 3>(9, 9).diagonal().setConstant(m_params.init_bg_var);
  m_P.block<3, 3>(12, 12).diagonal().setConstant(m_params.init_ba_var);
  m_r_eff_inflate_gnss_pos_ = 1.0;
  m_r_eff_inflate_gnss_vel_ = 1.0;
  m_r_eff_inflate_heading_yaw_ = 1.0;
}

Eigen::Matrix3d EskfCore::skew(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d S;
  S << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return S;
}

Eigen::Quaterniond EskfCore::delta_quat_from_dtheta(
  const Eigen::Vector3d & dtheta)
{
  const double angle = dtheta.norm();
  if (angle < 1e-12) {
    return Eigen::Quaterniond(
      1.0, 0.5 * dtheta.x(), 0.5 * dtheta.y(),
      0.5 * dtheta.z())
           .normalized();
  }

  const Eigen::Vector3d axis = dtheta / angle;
  const double half = 0.5 * angle;
  const double s = std::sin(half);
  return Eigen::Quaterniond(
    std::cos(half), s * axis.x(), s * axis.y(),
    s * axis.z());
}

void EskfCore::propagate(
  const Eigen::Vector3d & omega_base_radps,
  const Eigen::Vector3d & accel_base_mps2,
  const double dt_sec)
{
  // 예측 단계: 상태/공분산 전파
  if (!m_initialized || !(dt_sec > 0.0) || !std::isfinite(dt_sec)) {
    return;
  }
  if (!omega_base_radps.allFinite() || !accel_base_mps2.allFinite()) {
    return;
  }
  if (!finite()) {
    reset();
    return;
  }

  const Eigen::Vector3d omega = omega_base_radps - m_b_g;
  const Eigen::Vector3d accel = accel_base_mps2 - m_b_a;

  // Nominal propagation
  const Eigen::Quaterniond dq_body = delta_quat_from_dtheta(omega * dt_sec);
  m_q_map_from_base = (m_q_map_from_base * dq_body).normalized();

  const Eigen::Matrix3d R = m_q_map_from_base.toRotationMatrix();
  const Eigen::Vector3d a_map = R * accel;

  m_p_map += m_v_map * dt_sec + 0.5 * a_map * dt_sec * dt_sec;
  m_v_map += a_map * dt_sec;

  // Error-state covariance propagation (discrete Euler)
  Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Zero();
  F.block<3, 3>(0, 3).setIdentity();
  F.block<3, 3>(3, 6) = -R * skew(accel);
  F.block<3, 3>(3, 12) = -R;
  F.block<3, 3>(6, 6) = -skew(omega);
  F.block<3, 3>(6, 9) = -Eigen::Matrix3d::Identity();

  const Eigen::Matrix<double, 15, 15> Fd =
    Eigen::Matrix<double, 15, 15>::Identity() + F * dt_sec;

  // Noise: [gyro, accel, gyro_bias_rw, accel_bias_rw]
  Eigen::Matrix<double, 15, 12> G = Eigen::Matrix<double, 15, 12>::Zero();
  G.block<3, 3>(6, 0).setIdentity();  // gyro noise -> dtheta
  G.block<3, 3>(3, 3) = R;            // accel noise -> dv (map)
  G.block<3, 3>(9, 6).setIdentity();  // gyro bias rw
  G.block<3, 3>(12, 9).setIdentity(); // accel bias rw

  Eigen::Matrix<double, 12, 12> Qc = Eigen::Matrix<double, 12, 12>::Zero();
  Qc.block<3, 3>(0, 0).diagonal().setConstant(
    m_params.gyro_noise_std *
    m_params.gyro_noise_std);
  Qc.block<3, 3>(3, 3).diagonal().setConstant(
    m_params.accel_noise_std *
    m_params.accel_noise_std);
  Qc.block<3, 3>(6, 6).diagonal().setConstant(
    m_params.gyro_bias_noise_std * m_params.gyro_bias_noise_std);
  Qc.block<3, 3>(9, 9).diagonal().setConstant(
    m_params.accel_bias_noise_std * m_params.accel_bias_noise_std);

  const Eigen::Matrix<double, 15, 15> Qd = G * Qc * G.transpose() * dt_sec;

  m_P = Fd * m_P * Fd.transpose() + Qd;
  symmetrize_P();
  if (!finite()) {
    reset();
  }
}

const char * EskfCore::check_large_correction(const V15 & dx) const
{
  // 과도한 보정량 방지 (수치 안정성)
  const double dp_norm = dx.segment<3>(0).norm();
  const double dv_norm = dx.segment<3>(3).norm();
  const double dtheta_norm = dx.segment<3>(6).norm();
  if (std::isfinite(m_params.max_correction_pos_m) &&
    m_params.max_correction_pos_m > 0.0 &&
    std::isfinite(dp_norm) && dp_norm > m_params.max_correction_pos_m)
  {
    return "large_correction_pos_skip";
  }
  if (std::isfinite(m_params.max_correction_vel_mps) &&
    m_params.max_correction_vel_mps > 0.0 &&
    std::isfinite(dv_norm) && dv_norm > m_params.max_correction_vel_mps)
  {
    return "large_correction_vel_skip";
  }
  if (std::isfinite(m_params.max_correction_att_rad) &&
    m_params.max_correction_att_rad > 0.0 &&
    std::isfinite(dtheta_norm) &&
    dtheta_norm > m_params.max_correction_att_rad)
  {
    return "large_correction_att_skip";
  }
  return nullptr;
}

double EskfCore::smooth_r_eff_inflate(
  const RInflateChannel channel,
  const double target_factor)
{
  double * state = nullptr;
  switch (channel) {
    case RInflateChannel::kGnssPos:
      state = &m_r_eff_inflate_gnss_pos_;
      break;
    case RInflateChannel::kGnssVel:
      state = &m_r_eff_inflate_gnss_vel_;
      break;
    case RInflateChannel::kHeadingYaw:
      state = &m_r_eff_inflate_heading_yaw_;
      break;
    case RInflateChannel::kNone:
    default:
      return std::max(1.0, target_factor);
  }

  const double target = std::max(1.0, target_factor);
  if (!std::isfinite(*state) || !(*state >= 1.0)) {
    *state = 1.0;
  }

  if (target > *state) {
    *state = target;
    return *state;
  }

  const double tau_updates = m_params.r_eff_decay_tau_updates;
  if (!(std::isfinite(tau_updates) && tau_updates > 0.0)) {
    *state = target;
    return *state;
  }
  const double decay = std::exp(-1.0 / tau_updates);
  *state = target + (*state - target) * decay;
  if (!std::isfinite(*state) || !(*state >= 1.0)) {
    *state = 1.0;
  }
  return *state;
}

template<int Dim, typename DebugType>
DebugType EskfCore::update_vector_measurement(
  const Eigen::Matrix<double, Dim, 15> & H,
  const Eigen::Matrix<double, Dim, 1> & r,
  const Eigen::Matrix<double, Dim, Dim> & R, const double gate,
  const RInflateChannel channel)
{
  // 공통 벡터 측정 업데이트 (GNSS 위치/속도)
  DebugType dbg;
  if (!m_initialized) {
    dbg.reason = "not_initialized";
    return dbg;
  }
  if (!finite()) {
    reset();
    dbg.reason = "non_finite_reset";
    return dbg;
  }
  if (!r.allFinite() || !R.allFinite()) {
    dbg.reason = "non_finite_input";
    return dbg;
  }

  dbg.residual.setZero();
  dbg.residual.template head<Dim>() = r;

  const Eigen::Matrix<double, Dim, Dim> R_sym = 0.5 * (R + R.transpose());
  const Eigen::Matrix<double, Dim, Dim> HPHT = H * m_P * H.transpose();

  Eigen::Matrix<double, Dim, Dim> S_base =
    0.5 * (HPHT + R_sym + (HPHT + R_sym).transpose());
  Eigen::LDLT<Eigen::Matrix<double, Dim, Dim>> ldlt_base(S_base);
  if (ldlt_base.info() != Eigen::Success) {
    dbg.reason = "S_ldlt_fail";
    return dbg;
  }
  Eigen::Matrix<double, Dim, Dim> Sinv_base =
    ldlt_base.solve(Eigen::Matrix<double, Dim, Dim>::Identity());
  if (!Sinv_base.allFinite()) {
    dbg.reason = "S_inv_nonfinite";
    return dbg;
  }

  const double nis = r.transpose() * Sinv_base * r;
  dbg.nis = nis;
  double target_factor = 1.0;
  if (std::isfinite(gate) && gate > 0.0 && std::isfinite(nis) && nis > gate) {
    if (!m_params.nis_gate_inflate) {
      dbg.reason = "nis_gate_skip";
      return dbg;
    }
    target_factor = std::clamp(
      nis / gate, 1.0, std::max(1.0, m_params.nis_gate_inflate_max));
  }

  const double factor_eff = smooth_r_eff_inflate(channel, target_factor);
  const Eigen::Matrix<double, Dim, Dim> R_eff = R_sym * factor_eff;
  Eigen::Matrix<double, Dim, Dim> S =
    0.5 * (HPHT + R_eff + (HPHT + R_eff).transpose());
  Eigen::LDLT<Eigen::Matrix<double, Dim, Dim>> ldlt(S);
  if (ldlt.info() != Eigen::Success) {
    dbg.reason = "S_ldlt_fail_after_nis_inflate";
    return dbg;
  }
  Eigen::Matrix<double, Dim, Dim> Sinv =
    ldlt.solve(Eigen::Matrix<double, Dim, Dim>::Identity());
  if (!Sinv.allFinite()) {
    dbg.reason = "S_inv_nonfinite_after_nis_inflate";
    return dbg;
  }
  if (target_factor > 1.0) {
    dbg.reason = "nis_inflated";
  } else if (factor_eff > 1.0 + 1e-9) {
    dbg.reason = "nis_inflate_decay";
  }

  const Eigen::Matrix<double, 15, Dim> K = m_P * H.transpose() * Sinv;
  if (!K.allFinite()) {
    dbg.reason = "K_nonfinite";
    return dbg;
  }
  const V15 dx = K * r;
  if (!dx.allFinite()) {
    dbg.reason = "dx_nonfinite";
    return dbg;
  }

  if (const char * reason = check_large_correction(dx)) {
    dbg.reason = reason;
    return dbg;
  }

  const Eigen::Matrix<double, 15, 15> I =
    Eigen::Matrix<double, 15, 15>::Identity();
  const Eigen::Matrix<double, 15, 15> I_KH = I - K * H;
  m_P = I_KH * m_P * I_KH.transpose() + K * R_eff * K.transpose();

  inject_and_reset(dx);
  symmetrize_P();
  if (!finite()) {
    reset();
    dbg.applied = false;
    dbg.reason = "non_finite_after_update_reset";
    return dbg;
  }

  dbg.applied = true;
  dbg.R_diag = R_eff.diagonal();
  return dbg;
}

EskfScalarUpdateDebug EskfCore::update_scalar_measurement(
  const Eigen::Matrix<double, 1, 15> & H, const double r, const double var,
  const double gate,
  const RInflateChannel channel)
{
  // 공통 스칼라 측정 업데이트 (heading/차량 제약)
  EskfScalarUpdateDebug dbg;
  if (!m_initialized) {
    dbg.reason = "not_initialized";
    return dbg;
  }
  if (!finite()) {
    reset();
    dbg.reason = "non_finite_reset";
    return dbg;
  }
  if (!std::isfinite(r) || !std::isfinite(var) || !(var > 0.0)) {
    dbg.reason = "invalid_input";
    return dbg;
  }

  dbg.residual_rad = r;

  const double S_base = (H * m_P * H.transpose())(0, 0) + var;
  if (!std::isfinite(S_base) || !(S_base > 0.0)) {
    dbg.reason = "S_invalid";
    return dbg;
  }

  const double Sinv_base = 1.0 / S_base;
  if (!std::isfinite(Sinv_base)) {
    dbg.reason = "S_inv_nonfinite";
    return dbg;
  }

  const double nis = r * r * Sinv_base;
  dbg.nis = nis;

  double target_factor = 1.0;
  if (std::isfinite(gate) && gate > 0.0 && std::isfinite(nis) && nis > gate) {
    if (!m_params.nis_gate_inflate) {
      dbg.reason = "nis_gate_skip";
      return dbg;
    }
    target_factor = std::clamp(
      nis / gate, 1.0, std::max(1.0, m_params.nis_gate_inflate_max));
  }

  const double factor_eff = smooth_r_eff_inflate(channel, target_factor);
  const double R_eff = var * factor_eff;
  const double S = (H * m_P * H.transpose())(0, 0) + R_eff;
  if (!std::isfinite(S) || !(S > 0.0)) {
    dbg.reason = "S_invalid_after_nis_inflate";
    return dbg;
  }
  const double Sinv = 1.0 / S;
  if (!std::isfinite(Sinv)) {
    dbg.reason = "S_inv_nonfinite_after_nis_inflate";
    return dbg;
  }
  if (target_factor > 1.0) {
    dbg.reason = "nis_inflated";
  } else if (factor_eff > 1.0 + 1e-9) {
    dbg.reason = "nis_inflate_decay";
  }

  const Eigen::Matrix<double, 15, 1> K = m_P * H.transpose() * Sinv;
  if (!K.allFinite()) {
    dbg.reason = "K_nonfinite";
    return dbg;
  }
  const V15 dx = K * r;
  if (!dx.allFinite()) {
    dbg.reason = "dx_nonfinite";
    return dbg;
  }

  if (const char * reason = check_large_correction(dx)) {
    dbg.reason = reason;
    return dbg;
  }

  const Eigen::Matrix<double, 15, 15> I =
    Eigen::Matrix<double, 15, 15>::Identity();
  const Eigen::Matrix<double, 15, 15> I_KH = I - K * H;
  m_P = I_KH * m_P * I_KH.transpose() + K * R_eff * K.transpose();

  inject_and_reset(dx);
  symmetrize_P();
  if (!finite()) {
    reset();
    dbg.applied = false;
    dbg.reason = "non_finite_after_update_reset";
    return dbg;
  }

  dbg.applied = true;
  dbg.R = R_eff;
  return dbg;
}

EskfGnssPosUpdateDebug
EskfCore::update_gnss_position_3d(
  const Eigen::Vector3d & z_p_map,
  const Eigen::Matrix3d & R)
{
  Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
  H.block<3, 3>(0, 0).setIdentity();
  const Eigen::Vector3d r = z_p_map - m_p_map;
  return update_vector_measurement<3, EskfGnssPosUpdateDebug>(
    H, r, R, m_params.nis_gate_gnss_pos_3d, RInflateChannel::kGnssPos);
}

EskfGnssVelUpdateDebug
EskfCore::update_gnss_velocity_3d(
  const Eigen::Vector3d & z_v_map,
  const Eigen::Matrix3d & R)
{
  Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
  H.block<3, 3>(0, 3).setIdentity();
  const Eigen::Vector3d r = z_v_map - m_v_map;
  return update_vector_measurement<3, EskfGnssVelUpdateDebug>(
    H, r, R, m_params.nis_gate_gnss_vel_3d, RInflateChannel::kGnssVel);
}

EskfYawUpdateDebug EskfCore::update_heading_yaw(
  const double z_yaw_rad,
  const double yaw_var_rad2)
{
  const Eigen::Matrix3d R = m_q_map_from_base.toRotationMatrix();
  const double yaw_pred = std::atan2(R(1, 0), R(0, 0));
  const double r = wrap_to_pi(z_yaw_rad - yaw_pred);

  Eigen::Matrix<double, 1, 15> H = Eigen::Matrix<double, 1, 15>::Zero();
  // For right-multiplicative error-state, dtheta is in body frame.
  // Yaw error about map z corresponds to component along R^T * e_z (body frame).
  H.block<1, 3>(0, 6) = R.row(2); // (e_z^T * R)

  return update_scalar_measurement(
    H, r, yaw_var_rad2,
    m_params.nis_gate_heading_yaw, RInflateChannel::kHeadingYaw);
}

EskfScalarUpdateDebug EskfCore::update_body_velocity_component(
  const int axis, const double z_v_base_mps, const double var_mps2)
{
  if (axis < 0 || axis > 2) {
    EskfScalarUpdateDebug dbg;
    dbg.reason = "invalid_axis";
    return dbg;
  }
  if (!std::isfinite(z_v_base_mps) || !std::isfinite(var_mps2) ||
    !(var_mps2 > 0.0))
  {
    EskfScalarUpdateDebug dbg;
    dbg.reason = "invalid_input";
    return dbg;
  }

  const Eigen::Matrix3d R = m_q_map_from_base.toRotationMatrix();
  const Eigen::Vector3d v_base = R.transpose() * m_v_map;
  const double h = v_base(axis);
  const double r = z_v_base_mps - h;

  Eigen::Matrix<double, 1, 15> H = Eigen::Matrix<double, 1, 15>::Zero();
  // dv_map -> dv_base = R^T dv_map, so component(axis) is row(axis) of R^T
  H.block<1, 3>(0, 3) = R.col(axis).transpose();

  return update_scalar_measurement(
    H, r, var_mps2,
    m_params.nis_gate_heading_yaw);
}

EskfScalarUpdateDebug EskfCore::update_yaw_rate_from_steer(
  const double omega_meas_z_radps, const double yaw_rate_ref_radps,
  const double var_radps2)
{
  if (!std::isfinite(omega_meas_z_radps) || !std::isfinite(yaw_rate_ref_radps) ||
    !std::isfinite(var_radps2) || !(var_radps2 > 0.0))
  {
    EskfScalarUpdateDebug dbg;
    dbg.reason = "invalid_input";
    return dbg;
  }

  const double h = omega_meas_z_radps - m_b_g.z();
  const double r = yaw_rate_ref_radps - h;

  Eigen::Matrix<double, 1, 15> H = Eigen::Matrix<double, 1, 15>::Zero();
  // h = omega_meas_z - (b_g_nom + dbg_z) => dh/ddbg_z = -1
  H(0, 9 + 2) = -1.0;

  return update_scalar_measurement(
    H, r, var_radps2,
    m_params.nis_gate_heading_yaw);
}

void EskfCore::inject_and_reset(const V15 & dx)
{
  if (!dx.allFinite()) {
    reset();
    return;
  }

  m_p_map += dx.segment<3>(0);
  m_v_map += dx.segment<3>(3);

  const Eigen::Vector3d dtheta = dx.segment<3>(6);
  if (!dtheta.allFinite()) {
    reset();
    return;
  }
  const Eigen::Quaterniond dq = delta_quat_from_dtheta(dtheta);
  m_q_map_from_base = (m_q_map_from_base * dq).normalized();

  m_b_g += dx.segment<3>(9);
  m_b_a += dx.segment<3>(12);

  // Reset: transform covariance for the new error-state definition
  // (right-multiplicative, small-angle approximation).
  const Eigen::Matrix3d hat = skew(dtheta);
  Eigen::Matrix3d G_theta = Eigen::Matrix3d::Identity() - 0.5 * hat;
  if (m_params.use_so3_jacobian_reset) {
    const double angle = dtheta.norm();
    if (angle < 1e-8) {
      // Series: J_r^{-1}(phi) ≈ I - 0.5*hat(phi) + 1/12*hat(phi)^2
      G_theta = Eigen::Matrix3d::Identity() - 0.5 * hat + (1.0 / 12.0) * (hat * hat);
    } else {
      const double half = 0.5 * angle;
      const double sin_half = std::sin(half);
      if (std::abs(sin_half) > 1e-12) {
        const double cot_half = std::cos(half) / sin_half;
        const double coeff = 1.0 / (angle * angle) - cot_half / (2.0 * angle);
        G_theta = Eigen::Matrix3d::Identity() - 0.5 * hat + coeff * (hat * hat);
      } else {
        // Near singularity (angle ~ pi): fall back to first-order
        G_theta = Eigen::Matrix3d::Identity() - 0.5 * hat;
      }
    }
  }
  Eigen::Matrix<double, 15, 15> G = Eigen::Matrix<double, 15, 15>::Identity();
  G.block<3, 3>(6, 6) = G_theta;
  m_P = G * m_P * G.transpose();
}

void EskfCore::symmetrize_P()
{
  m_P = 0.5 * (m_P + m_P.transpose());
}

} // namespace eskf_localization
