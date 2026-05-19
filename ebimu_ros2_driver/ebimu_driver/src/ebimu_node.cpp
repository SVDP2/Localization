#include "ebimu_driver/ebimu_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <sstream>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace ebimu_driver
{
namespace
{
constexpr double kGravity = 9.80665;
constexpr double kDegToRad = M_PI / 180.0;

std::array<double, 9> toArray9(const std::vector<double> & values)
{
  std::array<double, 9> out{};
  for (size_t i = 0; i < std::min(values.size(), out.size()); ++i) {
    out[i] = values[i];
  }
  return out;
}

std::array<int64_t, 3> toAxisMap(const std::vector<int64_t> & values)
{
  std::array<int64_t, 3> out{0, 1, 2};
  for (size_t i = 0; i < std::min(values.size(), out.size()); ++i) {
    out[i] = std::clamp<int64_t>(values[i], 0, 2);
  }
  return out;
}

std::array<double, 3> toAxisSign(const std::vector<double> & values)
{
  std::array<double, 3> out{1.0, 1.0, 1.0};
  for (size_t i = 0; i < std::min(values.size(), out.size()); ++i) {
    out[i] = values[i] < 0.0 ? -1.0 : 1.0;
  }
  return out;
}
}  // namespace

EbimuNode::EbimuNode()
: Node("ebimu_driver"),
  parser_(driver_config_),
  diagnostics_(this)
{
  declareAndLoadParameters();
  parser_.setConfig(driver_config_);

  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::SensorDataQoS());
  imu_raw_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::SensorDataQoS());
  mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", rclcpp::SensorDataQoS());
  temperature_pub_ = create_publisher<std_msgs::msg::Float64>("imu/temperature", rclcpp::QoS(10));
  status_pub_ = create_publisher<ebimu_interfaces::msg::EbimuStatus>(status_topic_, rclcpp::QoS(10));

  diagnostics_.setHardwareID("ebimu-9dofv6");
  diagnostics_.add("EBIMU Driver", this, &EbimuNode::updateDiagnostics);

  last_status_time_ = std::chrono::steady_clock::now();
  status_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      publishStatus();
      diagnostics_.force_update();
    });

  startIoThread();
}

EbimuNode::~EbimuNode()
{
  running_ = false;
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
  serial_.close();
}

void EbimuNode::declareAndLoadParameters()
{
  declare_parameter("port", "/dev/imu");
  declare_parameter("baudrate", 115200);
  declare_parameter("output_mode", "ascii");
  declare_parameter("output_interval_ms", 10);
  declare_parameter("orientation_source", "quaternion");
  declare_parameter("enable_gyro", true);
  declare_parameter("enable_accel", true);
  declare_parameter("enable_magnetometer", false);
  declare_parameter("enable_temperature", false);
  declare_parameter("enable_timestamp", true);
  declare_parameter("frame_id", "imu_link");
  declare_parameter("publish_data_raw", true);
  declare_parameter("status_topic", "ebimu/status");
  declare_parameter("axis_map", std::vector<int64_t>{0, 1, 2});
  declare_parameter("axis_sign", std::vector<double>{1.0, 1.0, 1.0});
  declare_parameter("orientation_covariance", std::vector<double>(9, 0.0));
  declare_parameter("angular_velocity_covariance", std::vector<double>(9, 0.0));
  declare_parameter("linear_acceleration_covariance", std::vector<double>(9, 0.0));

  port_ = get_parameter("port").as_string();
  baudrate_ = static_cast<uint32_t>(get_parameter("baudrate").as_int());
  driver_config_.output_mode = get_parameter("output_mode").as_string();
  output_interval_ms_ = static_cast<uint16_t>(get_parameter("output_interval_ms").as_int());
  driver_config_.orientation_source = get_parameter("orientation_source").as_string();
  driver_config_.enable_gyro = get_parameter("enable_gyro").as_bool();
  driver_config_.enable_accel = get_parameter("enable_accel").as_bool();
  driver_config_.enable_magnetometer = get_parameter("enable_magnetometer").as_bool();
  driver_config_.enable_temperature = get_parameter("enable_temperature").as_bool();
  driver_config_.enable_timestamp = get_parameter("enable_timestamp").as_bool();
  frame_id_ = get_parameter("frame_id").as_string();
  publish_data_raw_ = get_parameter("publish_data_raw").as_bool();
  status_topic_ = get_parameter("status_topic").as_string();
  axis_map_ = toAxisMap(get_parameter("axis_map").as_integer_array());
  axis_sign_ = toAxisSign(get_parameter("axis_sign").as_double_array());
  orientation_covariance_ = toArray9(get_parameter("orientation_covariance").as_double_array());
  angular_velocity_covariance_ = toArray9(get_parameter("angular_velocity_covariance").as_double_array());
  linear_acceleration_covariance_ = toArray9(get_parameter("linear_acceleration_covariance").as_double_array());
}

void EbimuNode::startIoThread()
{
  running_ = true;
  io_thread_ = std::thread(&EbimuNode::ioLoop, this);
}

void EbimuNode::ioLoop()
{
  while (running_) {
    if (!serial_.isOpen()) {
      std::string error;
      if (serial_.open(port_, baudrate_, error)) {
        connected_ = true;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          last_error_.clear();
        }
        RCLCPP_INFO(get_logger(), "Opened EBIMU serial port %s at %u", port_.c_str(), baudrate_);
      } else {
        connected_ = false;
        ++reconnects_;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          last_error_ = error;
        }
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "Failed to open %s: %s", port_.c_str(), error.c_str());
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }
    }

    std::string error;
    const auto bytes = serial_.readAvailable(1024, 50, error);
    if (!error.empty()) {
      connected_ = false;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        last_error_ = error;
      }
      serial_.close();
      ++reconnects_;
      continue;
    }
    if (bytes.empty()) {
      ++serial_timeouts_;
      continue;
    }

    std::vector<EbimuSample> samples;
    {
      std::lock_guard<std::mutex> lock(parser_mutex_);
      samples = parser_.parse(bytes);
    }
    for (const auto & sample : samples) {
      publishSample(sample);
    }
  }
}

void EbimuNode::publishSample(const EbimuSample & sample)
{
  sensor_msgs::msg::Imu imu;
  imu.header.stamp = now();
  imu.header.frame_id = frame_id_;
  setCovariance(imu);

  if (sample.has_quaternion) {
    tf2::Quaternion q(
      sample.quaternion_xyzw[0],
      sample.quaternion_xyzw[1],
      sample.quaternion_xyzw[2],
      sample.quaternion_xyzw[3]);
    if (q.length2() > 1e-12) {
      q.normalize();
    }
    imu.orientation.x = q.x();
    imu.orientation.y = q.y();
    imu.orientation.z = q.z();
    imu.orientation.w = q.w();
  } else if (sample.has_euler) {
    tf2::Quaternion q;
    q.setRPY(
      sample.euler_deg[0] * kDegToRad,
      sample.euler_deg[1] * kDegToRad,
      sample.euler_deg[2] * kDegToRad);
    q.normalize();
    imu.orientation.x = q.x();
    imu.orientation.y = q.y();
    imu.orientation.z = q.z();
    imu.orientation.w = q.w();
  } else {
    imu.orientation_covariance[0] = -1.0;
  }

  if (sample.has_gyro) {
    const auto gyro = mapVector(sample.gyro_dps);
    imu.angular_velocity.x = gyro[0] * kDegToRad;
    imu.angular_velocity.y = gyro[1] * kDegToRad;
    imu.angular_velocity.z = gyro[2] * kDegToRad;
  } else {
    imu.angular_velocity_covariance[0] = -1.0;
  }

  if (sample.has_accel) {
    const auto accel = mapVector(sample.accel_g);
    imu.linear_acceleration.x = accel[0] * kGravity;
    imu.linear_acceleration.y = accel[1] * kGravity;
    imu.linear_acceleration.z = accel[2] * kGravity;
  } else {
    imu.linear_acceleration_covariance[0] = -1.0;
  }

  imu_pub_->publish(imu);
  if (publish_data_raw_) {
    imu_raw_pub_->publish(imu);
  }

  if (sample.has_magnetometer) {
    sensor_msgs::msg::MagneticField mag;
    mag.header = imu.header;
    const auto mapped = mapVector(sample.magnetometer_ut);
    mag.magnetic_field.x = mapped[0] / 1000000.0;
    mag.magnetic_field.y = mapped[1] / 1000000.0;
    mag.magnetic_field.z = mapped[2] / 1000000.0;
    mag_pub_->publish(mag);
  }

  if (sample.has_temperature) {
    std_msgs::msg::Float64 temp;
    temp.data = sample.temperature_c;
    temperature_pub_->publish(temp);
  }

  const auto steady_now = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_sample_time_ = steady_now;
    has_last_sample_time_ = true;
    if (sample.has_timestamp) {
      if (has_last_device_timestamp_) {
        const uint16_t prev = last_device_timestamp_;
        const uint16_t curr = sample.timestamp_ms;
        const uint32_t dt = curr >= prev ? curr - prev : static_cast<uint32_t>(curr) + 65536U - prev;
        const double dt_ms = static_cast<double>(dt);
        if (device_dt_count_ == 0) {
          device_dt_min_ms_ = dt_ms;
          device_dt_max_ms_ = dt_ms;
          device_dt_avg_ms_ = dt_ms;
        } else {
          device_dt_min_ms_ = std::min(device_dt_min_ms_, dt_ms);
          device_dt_max_ms_ = std::max(device_dt_max_ms_, dt_ms);
          device_dt_avg_ms_ =
            (device_dt_avg_ms_ * static_cast<double>(device_dt_count_) + dt_ms) /
            static_cast<double>(device_dt_count_ + 1);
        }
        ++device_dt_count_;
      }
      last_device_timestamp_ = sample.timestamp_ms;
      has_last_device_timestamp_ = true;
    }
  }
}

void EbimuNode::publishStatus()
{
  ParserStats stats;
  {
    std::lock_guard<std::mutex> lock(parser_mutex_);
    stats = parser_.stats();
  }

  const auto steady_now = std::chrono::steady_clock::now();
  const double elapsed =
    std::chrono::duration<double>(steady_now - last_status_time_).count();
  if (elapsed > 0.0) {
    actual_rx_rate_hz_ =
      static_cast<double>(stats.valid_frames - last_status_valid_count_) / elapsed;
  }
  last_status_valid_count_ = stats.valid_frames;
  last_status_time_ = steady_now;

  ebimu_interfaces::msg::EbimuStatus msg;
  msg.connected = connected_;
  msg.port = port_;
  msg.baudrate = baudrate_;
  msg.output_mode = driver_config_.output_mode;
  msg.output_interval_ms = output_interval_ms_;
  msg.expected_rate_hz = output_interval_ms_ > 0 ? 1000.0 / output_interval_ms_ : 0.0;
  msg.actual_rx_rate_hz = actual_rx_rate_hz_;
  msg.valid_frame_count = stats.valid_frames;
  msg.parse_error_count = stats.parse_errors;
  msg.checksum_error_count = stats.checksum_errors;
  msg.serial_timeout_count = serial_timeouts_;
  msg.reconnect_count = reconnects_;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    msg.last_error = last_error_;
    msg.device_timestamp_dt_avg_ms = device_dt_avg_ms_;
    msg.device_timestamp_dt_min_ms = device_dt_min_ms_;
    msg.device_timestamp_dt_max_ms = device_dt_max_ms_;
    if (has_last_sample_time_) {
      msg.last_sample_age_ms =
        std::chrono::duration<double, std::milli>(steady_now - last_sample_time_).count();
    } else {
      msg.last_sample_age_ms = -1.0;
    }
  }

  status_pub_->publish(msg);
}

void EbimuNode::updateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  ParserStats stats;
  {
    std::lock_guard<std::mutex> lock(parser_mutex_);
    stats = parser_.stats();
  }

  if (!connected_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "serial disconnected");
  } else if (stats.checksum_errors > 0 || stats.parse_errors > 0) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "receiving with parser errors");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "receiving");
  }

  stat.add("port", port_);
  stat.add("baudrate", baudrate_);
  stat.add("output_mode", driver_config_.output_mode);
  stat.add("output_interval_ms", output_interval_ms_);
  stat.add("actual_rx_rate_hz", actual_rx_rate_hz_);
  stat.add("valid_frame_count", stats.valid_frames);
  stat.add("parse_error_count", stats.parse_errors);
  stat.add("checksum_error_count", stats.checksum_errors);
  stat.add("serial_timeout_count", serial_timeouts_.load());
  stat.add("reconnect_count", reconnects_.load());
}

void EbimuNode::setCovariance(sensor_msgs::msg::Imu & msg)
{
  msg.orientation_covariance = orientation_covariance_;
  msg.angular_velocity_covariance = angular_velocity_covariance_;
  msg.linear_acceleration_covariance = linear_acceleration_covariance_;
}

std::array<double, 3> EbimuNode::mapVector(const std::array<double, 3> & value) const
{
  std::array<double, 3> out{};
  for (size_t i = 0; i < 3; ++i) {
    out[i] = axis_sign_[i] * value[static_cast<size_t>(axis_map_[i])];
  }
  return out;
}

}  // namespace ebimu_driver
