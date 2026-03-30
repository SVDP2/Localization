#ifndef ESKF_LOCALIZATION__ESKF__GNSS_UPDATE_HANDLER_HPP_
#define ESKF_LOCALIZATION__ESKF__GNSS_UPDATE_HANDLER_HPP_

#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <Eigen/Core>

#include "eskf_localization/eskf/eskf_core.hpp"

namespace eskf_localization
{

struct GnssPosUpdateConfig
{
  bool use_navsatfix_covariance{true};
  double covariance_scale{1.0};
  double pos_var_fallback{4.0};

  // Robustness: covariance clamping / SPD enforcement (internal: falls back to
  // diagonal-only if SPD check fails)
  double pos_var_min{1.0e-4};  // (m^2) lower bound (default: (1cm)^2)
  double pos_var_max{1.0e6};   // (m^2) upper bound to avoid extreme values
  bool cov_diag_only{false};   // if true, ignore off-diagonal terms
};

struct GnssVelUpdateConfig
{
  double covariance_scale{1.0};
  double vel_var_fallback{0.25};

  // Robustness: variance clamping
  double vel_var_min{0.01};   // (m/s)^2
  double vel_var_max{1.0e4};  // (m/s)^2
};

class GnssUpdateHandler
{
public:
  EskfGnssPosUpdateDebug apply_position_update(
    EskfCore & eskf,
    const Eigen::Vector3d & z_p_map,
    const sensor_msgs::msg::NavSatFix & msg,
    const GnssPosUpdateConfig & config) const;

  EskfGnssVelUpdateDebug apply_velocity_update(
    EskfCore & eskf,
    const Eigen::Vector3d & z_v_map,
    const GnssVelUpdateConfig & config) const;

  // Diagnostics helper: compute measurement covariance actually used (after
  // scale/clamp/SPD robustification). This does NOT include NIS-based inflation
  // inside EskfCore; that factor can be computed from dbg.nis/gate.
  Eigen::Matrix3d compute_position_covariance(
    const sensor_msgs::msg::NavSatFix & msg,
    const GnssPosUpdateConfig & config) const;

  Eigen::Matrix3d compute_velocity_covariance(
    const GnssVelUpdateConfig & config) const;

private:
  Eigen::Matrix3d build_position_covariance(
    const sensor_msgs::msg::NavSatFix & msg,
    const GnssPosUpdateConfig & config) const;

  Eigen::Matrix3d build_velocity_covariance(
    const GnssVelUpdateConfig & config) const;
};

} // namespace eskf_localization

#endif // ESKF_LOCALIZATION__ESKF__GNSS_UPDATE_HANDLER_HPP_
