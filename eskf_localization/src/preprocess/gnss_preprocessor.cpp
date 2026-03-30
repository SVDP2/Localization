#include "eskf_localization/preprocess/gnss_preprocessor.hpp"

#include <sensor_msgs/msg/nav_sat_status.hpp>

#include <cmath>

namespace eskf_localization
{

GnssPreprocessStatus GnssPreprocessor::preprocess(
  const sensor_msgs::msg::NavSatFix & msg, const MapProjector & projector,
  const ExtrinsicCache & extrinsics, double heading_yaw_rad,
  GnssPreprocessResult & out) const
{
  out = GnssPreprocessResult{};

  if (!projector.valid()) {
    return GnssPreprocessStatus::kNoMapProjector;
  }

  const int8_t status = msg.status.status;
  if (status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
    return GnssPreprocessStatus::kNoFix;
  }

  if (!std::isfinite(msg.latitude) || !std::isfinite(msg.longitude) ||
    std::abs(msg.latitude) > 90.0 || std::abs(msg.longitude) > 180.0)
  {
    return GnssPreprocessStatus::kInvalidLatLon;
  }

  const double lat = msg.latitude;
  const double lon = msg.longitude;
  double alt = msg.altitude;

  // Some GNSS drivers output sentinel altitudes (e.g., -2e10) during outage.
  if (!std::isfinite(alt) || std::abs(alt) > 1.0e6) {
    out.altitude_sanitized = true;
    alt = projector.info().map_origin.altitude;
  }

  geometry_msgs::msg::Point antenna{};
  std::string error;
  const auto proj_status =
    projector.project_forward(lat, lon, alt, antenna, &error);
  // kFallbackUsed is also acceptable (local ENU approximation was used)
  if (proj_status != ProjectionStatus::kOk &&
    proj_status != ProjectionStatus::kFallbackUsed)
  {
    out.projection_error = error;
    return GnssPreprocessStatus::kProjectionFailed;
  }

  out.antenna_position_map = antenna;
  out.base_position_map = antenna;

  if (!std::isfinite(antenna.x) || !std::isfinite(antenna.y) ||
    !std::isfinite(antenna.z))
  {
    return GnssPreprocessStatus::kNonFiniteProjection;
  }

  if (extrinsics.gnss_valid) {
    // Lever-arm offset in base_link frame
    const double dx = extrinsics.base_to_gnss.transform.translation.x;
    const double dy = extrinsics.base_to_gnss.transform.translation.y;
    const double dz = extrinsics.base_to_gnss.transform.translation.z;

    // 회전 적용: antenna->base offset을 heading으로 회전
    // p_base = p_antenna + R(heading) * t_base_to_antenna
    // 여기서 t_base_to_gnss가 주어졌으므로, t_antenna_to_base = -R^T *
    // t_base_to_gnss 단, antenna와 base 사이에 회전이 없다고 가정하면 (static
    // TF가 translation만): p_base = p_antenna - R(heading) * t_base_to_antenna
    const double cos_yaw = std::cos(heading_yaw_rad);
    const double sin_yaw = std::sin(heading_yaw_rad);

    // 2D rotation matrix applied to offset vector
    // [cos -sin] [dx]   [dx_rotated]
    // [sin  cos] [dy] = [dy_rotated]
    const double dx_rotated = cos_yaw * dx - sin_yaw * dy;
    const double dy_rotated = sin_yaw * dx + cos_yaw * dy;

    out.base_position_map.x = antenna.x - dx_rotated;
    out.base_position_map.y = antenna.y - dy_rotated;
    out.base_position_map.z = antenna.z - dz;
  }

  return GnssPreprocessStatus::kOk;
}

} // namespace eskf_localization
