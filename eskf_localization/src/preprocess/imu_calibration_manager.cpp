#include "eskf_localization/preprocess/imu_calibration_manager.hpp"

#include <rclcpp/rclcpp.hpp>

namespace eskf_localization
{

void ImuCalibrationManager::start(
  const rclcpp::Time & start_time,
  double duration_sec)
{
  state_ = State::kInProgress;
  duration_sec_ = duration_sec;
  start_time_ = start_time;
  last_log_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

double ImuCalibrationManager::remaining_seconds(
  const rclcpp::Time & now) const
{
  if (state_ == State::kNotStarted) {
    return duration_sec_;
  }
  const double elapsed = (now - start_time_).seconds();
  return duration_sec_ - elapsed;
}

void ImuCalibrationManager::process_sample(
  ImuPreprocessor & preprocessor,
  const sensor_msgs::msg::Imu & msg,
  const rclcpp::Time & stamp,
  const std::string & calibration_file_path,
  const rclcpp::Logger & logger)
{
  if (!in_progress()) {
    return;
  }

  preprocessor.add_calibration_sample(msg);

  const double elapsed = (stamp - start_time_).seconds();
  const double remaining = duration_sec_ - elapsed;

  if ((stamp - last_log_time_).seconds() >= 5.0 || remaining <= 0.0) {
    if (remaining > 0.0) {
      RCLCPP_INFO(
        logger, "IMU calibration: %.1f seconds remaining (samples: %zu)", remaining,
        preprocessor.sample_count());
    }
    last_log_time_ = stamp;
  }

  if (elapsed >= duration_sec_) {
    if (preprocessor.finalize_calibration(calibration_file_path)) {
      RCLCPP_INFO(
        logger, "IMU calibration completed (%zu samples)",
        preprocessor.calibration().sample_count);
      RCLCPP_INFO(
        logger, " Gyro bias: [%.6f, %.6f, %.6f] rad/s",
        preprocessor.calibration().gyro_bias.x,
        preprocessor.calibration().gyro_bias.y, preprocessor.calibration().gyro_bias.z);
      RCLCPP_INFO(logger, " Saved to: %s", calibration_file_path.c_str());
      state_ = State::kCompleted;
    } else {
      RCLCPP_ERROR(logger, "Failed to finalize IMU calibration");
      state_ = State::kFailed;
    }
  }
}

} // namespace eskf_localization
