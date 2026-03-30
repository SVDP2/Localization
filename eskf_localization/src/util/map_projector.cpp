#include "eskf_localization/util/map_projector.hpp"

#include <geography_utils/projection.hpp>

#include <geographic_msgs/msg/geo_point.hpp>

#include <cmath>
#include <exception>

namespace eskf_localization
{

void MapProjector::set_info(const tier4_map_msgs::msg::MapProjectorInfo & info)
{
  constexpr double DEG_TO_RAD = M_PI / 180.0;

  info_ = info;
  origin_.lat0_rad = info.map_origin.latitude * DEG_TO_RAD;
  origin_.lon0_rad = info.map_origin.longitude * DEG_TO_RAD;
  origin_.alt0_m = info.map_origin.altitude;
  valid_ = true;
}

ProjectionStatus MapProjector::project_forward(
  const double latitude_deg,
  const double longitude_deg,
  const double altitude_m,
  geometry_msgs::msg::Point & out,
  std::string * error) const
{
  if (!valid_) {
    if (error) {
      *error = "map_projector_info not received";
    }
    return ProjectionStatus::kNotReady;
  }

  // Prefer Autoware's projector implementation to match other nodes
  // (gnss_poser).
  bool use_fallback = false;
  if (info_.projector_type != tier4_map_msgs::msg::MapProjectorInfo::LOCAL) {
    geographic_msgs::msg::GeoPoint gps_point;
    gps_point.latitude = latitude_deg;
    gps_point.longitude = longitude_deg;
    gps_point.altitude = altitude_m;

    try {
      out = geography_utils::project_forward(gps_point, info_);
      return ProjectionStatus::kOk;
    } catch (const std::exception & e) {
      if (error) {
        *error = e.what();
      }
      use_fallback = true; // Mark that we're falling back due to exception
    }
  } else {
    use_fallback = true; // LOCAL type always uses fallback
  }

  // Fallback: ENU flat-earth approximation around map origin.
  constexpr double DEG_TO_RAD = M_PI / 180.0;
  const double lat_rad = latitude_deg * DEG_TO_RAD;
  const double lon_rad = longitude_deg * DEG_TO_RAD;

  // WGS84 Earth radius (meters)
  constexpr double EARTH_RADIUS = 6378137.0;

  const double dlat = lat_rad - origin_.lat0_rad;
  const double dlon = lon_rad - origin_.lon0_rad;
  const double dalt = altitude_m - origin_.alt0_m;

  out.x = dlon * EARTH_RADIUS * std::cos(origin_.lat0_rad);
  out.y = dlat * EARTH_RADIUS;
  out.z = dalt;
  return use_fallback ? ProjectionStatus::kFallbackUsed : ProjectionStatus::kOk;
}

} // namespace eskf_localization
