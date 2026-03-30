#ifndef ESKF_LOCALIZATION__PREPROCESS__IMU_CALIBRATION_MANAGER_HPP_
#define ESKF_LOCALIZATION__PREPROCESS__IMU_CALIBRATION_MANAGER_HPP_

#include <string>

#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>

#include <sensor_msgs/msg/imu.hpp>

#include "eskf_localization/preprocess/imu_preprocessor.hpp"

namespace eskf_localization
{

class ImuCalibrationManager
{
public:
  enum class State
  {
    kNotStarted = 0,
    kInProgress,
    kCompleted,
    kFailed,
  };

  void start(const rclcpp::Time & start_time, double duration_sec);

  State state() const {return state_;}
  bool in_progress() const
  {
    return state_ == State::kInProgress || state_ == State::kFailed;
  }

  double remaining_seconds(const rclcpp::Time & now) const;

  void process_sample(
    ImuPreprocessor & preprocessor,
    const sensor_msgs::msg::Imu & msg,
    const rclcpp::Time & stamp,
    const std::string & calibration_file_path,
    const rclcpp::Logger & logger);

private:
  State state_{State::kNotStarted};
  double duration_sec_{0.0};
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_log_time_{0, 0, RCL_ROS_TIME};
};

} // namespace eskf_localization

#endif // ESKF_LOCALIZATION__PREPROCESS__IMU_CALIBRATION_MANAGER_HPP_
