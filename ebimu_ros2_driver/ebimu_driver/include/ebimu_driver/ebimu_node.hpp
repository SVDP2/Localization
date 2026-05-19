#ifndef EBIMU_DRIVER__EBIMU_NODE_HPP_
#define EBIMU_DRIVER__EBIMU_NODE_HPP_

#include <atomic>
#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <ebimu_interfaces/msg/ebimu_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/float64.hpp>

#include "ebimu_driver/ebimu_parser.hpp"
#include "ebimu_driver/serial_port.hpp"

namespace ebimu_driver
{

class EbimuNode : public rclcpp::Node
{
public:
  EbimuNode();
  ~EbimuNode() override;

private:
  void declareAndLoadParameters();
  void startIoThread();
  void ioLoop();
  void publishSample(const EbimuSample & sample);
  void publishStatus();
  void updateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void setCovariance(sensor_msgs::msg::Imu & msg);
  std::array<double, 3> mapVector(const std::array<double, 3> & value) const;

  std::string port_;
  uint32_t baudrate_{115200};
  uint16_t output_interval_ms_{10};
  bool publish_data_raw_{true};
  std::string frame_id_{"imu_link"};
  std::string status_topic_{"ebimu/status"};
  DriverConfig driver_config_;
  std::array<int64_t, 3> axis_map_{0, 1, 2};
  std::array<double, 3> axis_sign_{1.0, 1.0, 1.0};
  std::array<double, 9> orientation_covariance_{};
  std::array<double, 9> angular_velocity_covariance_{};
  std::array<double, 9> linear_acceleration_covariance_{};

  SerialPort serial_;
  EbimuParser parser_;
  std::thread io_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> connected_{false};
  std::atomic<uint64_t> serial_timeouts_{0};
  std::atomic<uint64_t> reconnects_{0};
  std::string last_error_;
  std::mutex state_mutex_;
  std::mutex parser_mutex_;

  uint64_t last_status_valid_count_{0};
  std::chrono::steady_clock::time_point last_status_time_;
  std::chrono::steady_clock::time_point last_sample_time_;
  bool has_last_sample_time_{false};
  bool has_last_device_timestamp_{false};
  uint16_t last_device_timestamp_{0};
  double device_dt_avg_ms_{0.0};
  double device_dt_min_ms_{0.0};
  double device_dt_max_ms_{0.0};
  uint64_t device_dt_count_{0};
  double actual_rx_rate_hz_{0.0};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr temperature_pub_;
  rclcpp::Publisher<ebimu_interfaces::msg::EbimuStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  diagnostic_updater::Updater diagnostics_;
};

}  // namespace ebimu_driver

#endif  // EBIMU_DRIVER__EBIMU_NODE_HPP_
