#ifndef EBIMU_DRIVER__EBIMU_SAMPLE_HPP_
#define EBIMU_DRIVER__EBIMU_SAMPLE_HPP_

#include <array>
#include <cstdint>
#include <string>

namespace ebimu_driver
{

struct DriverConfig
{
  std::string output_mode{"ascii"};
  std::string orientation_source{"quaternion"};
  bool enable_gyro{true};
  bool enable_accel{true};
  bool enable_magnetometer{false};
  bool enable_temperature{false};
  bool enable_timestamp{true};
};

struct EbimuSample
{
  bool has_quaternion{false};
  std::array<double, 4> quaternion_xyzw{0.0, 0.0, 0.0, 1.0};

  bool has_euler{false};
  std::array<double, 3> euler_deg{0.0, 0.0, 0.0};

  bool has_gyro{false};
  std::array<double, 3> gyro_dps{0.0, 0.0, 0.0};

  bool has_accel{false};
  std::array<double, 3> accel_g{0.0, 0.0, 0.0};

  bool has_magnetometer{false};
  std::array<double, 3> magnetometer_ut{0.0, 0.0, 0.0};

  bool has_temperature{false};
  double temperature_c{0.0};

  bool has_timestamp{false};
  uint16_t timestamp_ms{0};
};

struct ParserStats
{
  uint64_t valid_frames{0};
  uint64_t parse_errors{0};
  uint64_t checksum_errors{0};
};

}  // namespace ebimu_driver

#endif  // EBIMU_DRIVER__EBIMU_SAMPLE_HPP_
