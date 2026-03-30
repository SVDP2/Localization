#ifndef ESKF_LOCALIZATION__MAP_PROJECTOR_HPP_
#define ESKF_LOCALIZATION__MAP_PROJECTOR_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <tier4_map_msgs/msg/map_projector_info.hpp>

#include <string>

namespace eskf_localization
{

struct MapOriginRad
{
  double lat0_rad{0.0};
  double lon0_rad{0.0};
  double alt0_m{0.0};
};

enum class ProjectionStatus
{
  kOk = 0,
  kNotReady,
  kFailed,
  kFallbackUsed, // Local ENU approximation was used instead of projector
};

class MapProjector
{
public:
  void set_info(const tier4_map_msgs::msg::MapProjectorInfo & info);

  bool valid() const {return valid_;}
  const tier4_map_msgs::msg::MapProjectorInfo & info() const {return info_;}
  const MapOriginRad & origin() const {return origin_;}

  ProjectionStatus project_forward(
    double latitude_deg, double longitude_deg,
    double altitude_m,
    geometry_msgs::msg::Point & out,
    std::string * error = nullptr) const;

private:
  bool valid_{false};
  tier4_map_msgs::msg::MapProjectorInfo info_{};
  MapOriginRad origin_{};
};

} // namespace eskf_localization

#endif // ESKF_LOCALIZATION__MAP_PROJECTOR_HPP_
