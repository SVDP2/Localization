#ifndef ESKF_LOCALIZATION__TYPES_HPP_
#define ESKF_LOCALIZATION__TYPES_HPP_

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <string>

namespace eskf_localization
{

// Phase 1: Node skeleton + I/O contract parameters
struct ESKFParameters
{
  // Input topic names (remappable/configurable)
  std::string imu_topic;
  std::string gnss_topic;
  std::string gnss_vel_topic;
  std::string heading_topic;
  std::string map_projector_info_topic;
  std::string velocity_topic;
  std::string steering_topic;

  // Output topic names
  std::string output_odom_topic;

  // Frame IDs
  std::string map_frame;
  std::string base_frame;

  // TF publishing (default OFF for add-on phase)
  bool publish_tf;

  // Output publish rate (Hz)
  double publish_rate;

  // Default constructor with sensible defaults
  ESKFParameters()
  : imu_topic("/sensing/imu/imu_raw"),
    gnss_topic("/sensing/gnss/navsatfix"),
    gnss_vel_topic("/sensing/gnss/vel"),
    heading_topic("/sensing/gnss/heading"),
    map_projector_info_topic("/map/map_projector_info"),
    velocity_topic("/vehicle/status/velocity_status"),
    steering_topic("/vehicle/status/steering_status"),
    output_odom_topic("/localization/kinematic_state"),
    map_frame("map"), base_frame("base_link"), publish_tf(true),
    publish_rate(200.0) {}
};

// Phase 2: Time processing parameters
struct TimeProcessingParams
{
  double max_dt;           // dt clamp upper bound (seconds)
  double min_dt;           // dt clamp lower bound (seconds)
  double max_delay;        // max acceptable delay (seconds)
  double future_tolerance; // tolerance for future stamps (seconds)

  TimeProcessingParams()
  : max_dt(0.5), min_dt(0.0001), max_delay(1.0), future_tolerance(0.1) {}
};

// Phase 3: TF extrinsic cache
struct ExtrinsicCache
{
  bool imu_valid;
  bool gnss_valid;
  geometry_msgs::msg::TransformStamped base_to_imu;
  geometry_msgs::msg::TransformStamped base_to_gnss;

  ExtrinsicCache()
  : imu_valid(false), gnss_valid(false) {}
};

// Phase 3: Vehicle geometry parameters
struct VehicleParams
{
  double wheelbase; // L_eff: distance from steering axle to rear axle (m)

  VehicleParams()
  : wheelbase(7.0)     // Default 6.76m, will be overridden by parameter
  {}
};

// NOTE: Map projection uses tier4_map_msgs::msg::MapProjectorInfo directly
// and MapOriginRad (defined in map_projector.hpp) for internal state.

} // namespace eskf_localization

#endif // ESKF_LOCALIZATION__TYPES_HPP_
