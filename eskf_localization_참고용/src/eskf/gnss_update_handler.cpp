#include "eskf_localization/eskf/gnss_update_handler.hpp"

#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <algorithm>
#include <cmath>

#include <Eigen/Cholesky>

namespace eskf_localization
{
// NOTE: GNSS 업데이트에서 측정 공분산 구성과 2D/3D 선택 로직을 담당한다.

namespace
{

inline double clamp_positive(const double x, const double lo, const double hi)
{
  if (!std::isfinite(x)) {
    return lo;
  }
  const double y = std::max(x, lo);
  return std::min(y, hi);
}

inline bool is_spd_3x3(const Eigen::Matrix3d & A)
{
  // LLT requires SPD (strict). Treat near-zero / negative as failure.
  Eigen::LLT<Eigen::Matrix3d> llt(A);
  return llt.info() == Eigen::Success;
}

} // namespace

Eigen::Matrix3d GnssUpdateHandler::build_position_covariance(
  const sensor_msgs::msg::NavSatFix & msg,
  const GnssPosUpdateConfig & config) const
{
  // GNSS covariance 가공(스케일/클램프/SPD) 핵심 구간
  Eigen::Matrix3d R3 = Eigen::Matrix3d::Zero();
  bool R_valid = false;

  if (config.use_navsatfix_covariance &&
    msg.position_covariance_type !=
    sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN)
  {
    const auto & c = msg.position_covariance; // row-major 3x3
    R3(0, 0) = c[0];
    R3(0, 1) = c[1];
    R3(0, 2) = c[2];
    R3(1, 0) = c[3];
    R3(1, 1) = c[4];
    R3(1, 2) = c[5];
    R3(2, 0) = c[6];
    R3(2, 1) = c[7];
    R3(2, 2) = c[8];

    R3 = 0.5 * (R3 + R3.transpose());
    R_valid = R3.allFinite() && (R3(0, 0) > 0.0) && (R3(1, 1) > 0.0) &&
      (R3(2, 2) > 0.0);
  }

  if (!R_valid) {
    R3.setZero();
    R3.diagonal().setConstant(config.pos_var_fallback);
  }

  // Apply global scaling
  R3 *= config.covariance_scale;

  // Clamp diagonal variances to prevent overconfidence / extreme values
  for (int i = 0; i < 3; ++i) {
    R3(i, i) = clamp_positive(R3(i, i), config.pos_var_min, config.pos_var_max);
  }

  // Optionally drop off-diagonal terms (most robust)
  Eigen::Matrix3d R_sym = 0.5 * (R3 + R3.transpose());
  if (config.cov_diag_only) {
    return R_sym.diagonal().asDiagonal();
  }

  // Ensure SPD for stable inversion / Joseph update
  if (is_spd_3x3(R_sym)) {
    return R_sym;
  }

  // Fallback: keep only diagonal terms (already clamped)
  return R_sym.diagonal().asDiagonal();
}

Eigen::Matrix3d GnssUpdateHandler::build_velocity_covariance(
  const GnssVelUpdateConfig & config) const
{
  Eigen::Matrix3d R3 = Eigen::Matrix3d::Identity();
  R3.diagonal().setConstant(config.vel_var_fallback);
  R3 *= config.covariance_scale;
  for (int i = 0; i < 3; ++i) {
    R3(i, i) = clamp_positive(R3(i, i), config.vel_var_min, config.vel_var_max);
  }
  return R3;
}

Eigen::Matrix3d GnssUpdateHandler::compute_position_covariance(
  const sensor_msgs::msg::NavSatFix & msg,
  const GnssPosUpdateConfig & config) const
{
  // 위치 공분산: SPD 보장 후 반환
  return build_position_covariance(msg, config);
}

Eigen::Matrix3d GnssUpdateHandler::compute_velocity_covariance(
  const GnssVelUpdateConfig & config) const
{
  // 속도 공분산: 기본값/스케일 적용
  return build_velocity_covariance(config);
}

EskfGnssPosUpdateDebug GnssUpdateHandler::apply_position_update(
  EskfCore & eskf,
  const Eigen::Vector3d & z_p_map,
  const sensor_msgs::msg::NavSatFix & msg,
  const GnssPosUpdateConfig & config) const
{
  // GNSS 위치 업데이트 (3D)
  const Eigen::Matrix3d R3 = build_position_covariance(msg, config);
  return eskf.update_gnss_position_3d(z_p_map, R3);
}

EskfGnssVelUpdateDebug GnssUpdateHandler::apply_velocity_update(
  EskfCore & eskf,
  const Eigen::Vector3d & z_v_map,
  const GnssVelUpdateConfig & config) const
{
  // GNSS 속도 업데이트 (3D)
  const Eigen::Matrix3d R3 = build_velocity_covariance(config);
  return eskf.update_gnss_velocity_3d(z_v_map, R3);
}

} // namespace eskf_localization
