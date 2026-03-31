#include "eskf_localization/preprocess/gnss_heading_arbitrator.hpp"

#include <algorithm>

namespace eskf_localization
{

void GnssHeadingArbitrator::reset()
{
  m_latest_gnss_status = std::numeric_limits<int>::min();
  m_latest_gnss_status_stamp_sec = std::numeric_limits<double>::quiet_NaN();
  m_gphdt_holdoff_until_sec = 0.0;

  m_have_gphdt = false;
  m_gphdt_yaw_rad = std::numeric_limits<double>::quiet_NaN();
  m_gphdt_yaw_var_rad2 = std::numeric_limits<double>::quiet_NaN();
  m_gphdt_stamp_sec = std::numeric_limits<double>::quiet_NaN();
}

void GnssHeadingArbitrator::update_gnss_status(
  const int t_status,
  const double t_stamp_sec)
{
  const int prev = m_latest_gnss_status;
  m_latest_gnss_status = t_status;
  m_latest_gnss_status_stamp_sec = t_stamp_sec;

  // Hard safety: status < 0 is treated as outage/garbage. Clear candidates.
  if (t_status < 0) {
    m_have_gphdt = false;
    return;
  }

  // Recovery hold-off: when transitioning from degraded (<1) to good (>=1),
  // delay usage of GPHDT for a short period.
  if (m_params.gphdt_recover_holdoff_sec > 0.0 && is_finite(t_stamp_sec)) {
    const int min_status =
      std::max(0, std::min(2, m_params.min_status_for_gphdt));
    const bool prev_good = (prev >= min_status);
    const bool now_good = (t_status >= min_status);
    if (!prev_good && now_good) {
      m_gphdt_holdoff_until_sec =
        t_stamp_sec + std::max(0.0, m_params.gphdt_recover_holdoff_sec);
    }
  }
}

void GnssHeadingArbitrator::update_gphdt(
  const double t_yaw_rad,
  const double t_yaw_var_rad2,
  const double t_stamp_sec)
{
  if (!is_finite(t_yaw_rad) || !is_finite(t_yaw_var_rad2) ||
    !(t_yaw_var_rad2 > 0.0) || !is_finite(t_stamp_sec))
  {
    return;
  }
  m_have_gphdt = true;
  m_gphdt_yaw_rad = t_yaw_rad;
  m_gphdt_yaw_var_rad2 = t_yaw_var_rad2;
  m_gphdt_stamp_sec = t_stamp_sec;
}

bool GnssHeadingArbitrator::gphdt_sample_usable(
  const double t_yaw_rad,
  const double t_yaw_var_rad2,
  const double t_stamp_sec,
  const double t_now_sec,
  const char ** t_reason) const
{
  if (t_reason) {
    *t_reason = nullptr;
  }
  if (m_latest_gnss_status < 0) {
    if (t_reason) {
      *t_reason = "gnss_status_neg1_skip";
    }
    return false;
  }
  const int min_status = std::max(0, std::min(2, m_params.min_status_for_gphdt));
  if (m_latest_gnss_status < min_status) {
    if (t_reason) {
      *t_reason = "gnss_status_skip";
    }
    return false;
  }
  if (!is_finite(t_yaw_rad) || !is_finite(t_yaw_var_rad2) ||
    !(t_yaw_var_rad2 > 0.0) ||
    !is_finite(t_stamp_sec) || !is_finite(t_now_sec))
  {
    if (t_reason) {
      *t_reason = "heading_non_finite";
    }
    return false;
  }
  if (!is_fresh(t_now_sec, t_stamp_sec)) {
    if (t_reason) {
      *t_reason = "heading_timeout";
    }
    return false;
  }
  if (m_params.gphdt_recover_holdoff_sec > 0.0) {
    if (t_now_sec < m_gphdt_holdoff_until_sec ||
      t_stamp_sec < m_gphdt_holdoff_until_sec)
    {
      if (t_reason) {
        *t_reason = "heading_recover_holdoff";
      }
      return false;
    }
  }
  return true;
}

bool GnssHeadingArbitrator::is_fresh(
  const double t_now_sec,
  const double t_stamp_sec) const
{
  if (!is_finite(t_now_sec) || !is_finite(t_stamp_sec)) {
    return false;
  }
  const double max_dt = std::max(0.0, m_params.max_time_diff_sec);
  if (!(max_dt > 0.0)) {
    return true;
  }
  const double dt = std::abs(t_now_sec - t_stamp_sec);
  return std::isfinite(dt) && (dt <= max_dt);
}

bool GnssHeadingArbitrator::gphdt_usable(
  const double t_now_sec,
  const char ** t_reason) const
{
  if (t_reason) {
    *t_reason = nullptr;
  }
  if (m_latest_gnss_status < 0) {
    if (t_reason) {
      *t_reason = "gnss_status_neg1_skip";
    }
    return false;
  }
  const int min_status = std::max(0, std::min(2, m_params.min_status_for_gphdt));
  if (m_latest_gnss_status < min_status) {
    if (t_reason) {
      *t_reason = "gnss_status_skip";
    }
    return false;
  }
  if (!m_have_gphdt) {
    if (t_reason) {
      *t_reason = "heading_not_received";
    }
    return false;
  }
  if (!is_finite(m_gphdt_yaw_rad) || !is_finite(m_gphdt_yaw_var_rad2) ||
    !(m_gphdt_yaw_var_rad2 > 0.0))
  {
    if (t_reason) {
      *t_reason = "heading_non_finite";
    }
    return false;
  }
  if (!is_fresh(t_now_sec, m_gphdt_stamp_sec)) {
    if (t_reason) {
      *t_reason = "heading_timeout";
    }
    return false;
  }

  if (m_params.gphdt_recover_holdoff_sec > 0.0) {
    if (t_now_sec < m_gphdt_holdoff_until_sec ||
      m_gphdt_stamp_sec < m_gphdt_holdoff_until_sec)
    {
      if (t_reason) {
        *t_reason = "heading_recover_holdoff";
      }
      return false;
    }
  }

  return true;
}

GnssYawMeasurement GnssHeadingArbitrator::select(const double t_now_sec) const
{
  GnssYawMeasurement out;

  const char * reason = nullptr;
  if (gphdt_usable(t_now_sec, &reason)) {
    out.source = GnssHeadingSource::kGphdt;
    out.yaw_rad = m_gphdt_yaw_rad;
    out.yaw_var_rad2 = m_gphdt_yaw_var_rad2;
    out.stamp_sec = m_gphdt_stamp_sec;
    return out;
  }

  out.source = GnssHeadingSource::kNone;
  out.reason = reason;
  return out;
}

} // namespace eskf_localization
