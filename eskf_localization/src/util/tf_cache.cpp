#include "eskf_localization/util/tf_cache.hpp"

#include <tf2/exceptions.h>
#include <tf2/time.h>

#include <tf2_ros/buffer.h>

#include <rclcpp/logging.hpp>

#include <vector>

namespace eskf_localization
{

TfCache::TfCache(
  const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
  std::string base_frame)
: tf_buffer_(tf_buffer), base_frame_(std::move(base_frame)) {}

void TfCache::set_tf_buffer(const std::shared_ptr<tf2_ros::Buffer> & tf_buffer)
{
  tf_buffer_ = tf_buffer;
}

void TfCache::set_base_frame(std::string base_frame)
{
  base_frame_ = std::move(base_frame);
}

bool TfCache::lookup_transform(
  const std::string & from_frame,
  const std::string & to_frame,
  geometry_msgs::msg::TransformStamped & transform,
  const rclcpp::Logger & logger) const
{
  if (!tf_buffer_) {
    RCLCPP_WARN(
      logger, "TF buffer not initialized (lookup %s -> %s)",
      from_frame.c_str(), to_frame.c_str());
    return false;
  }

  try {
    transform = tf_buffer_->lookupTransform(
      from_frame, to_frame,
      tf2::TimePointZero,
      tf2::durationFromSec(0.5));
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      logger, "TF lookup failed (%s -> %s): %s", from_frame.c_str(),
      to_frame.c_str(), ex.what());
    return false;
  }
}

void TfCache::cache_common_extrinsics(const rclcpp::Logger & logger)
{
  if (!extrinsics_.imu_valid) {
    const std::vector<std::string> imu_frame_candidates = {"imu_link",
      "imu_frame"};
    for (const auto & frame : imu_frame_candidates) {
      if (lookup_transform(
          base_frame_, frame, extrinsics_.base_to_imu,
          logger))
      {
        extrinsics_.imu_valid = true;
        RCLCPP_INFO(logger, "IMU extrinsic cached: %s -> %s", base_frame_.c_str(), frame.c_str());
        break;
      }
    }
  }

  if (!extrinsics_.gnss_valid) {
    const std::vector<std::string> gnss_frame_candidates = {
      // Common GNSS link names used across our sensor kits / drivers.
      // Keep this list permissive so a frame rename (e.g., right->left antenna)
      // doesn't silently break extrinsic caching.
      "gnss_link",
      "gnss_antenna",
      "gps_link",
      "gnss_left_antenna_link",
      "gnss_right_antenna_link",
    };
    for (const auto & frame : gnss_frame_candidates) {
      if (lookup_transform(
          base_frame_, frame, extrinsics_.base_to_gnss,
          logger))
      {
        extrinsics_.gnss_valid = true;
        RCLCPP_INFO(logger, "GNSS extrinsic cached: %s -> %s", base_frame_.c_str(), frame.c_str());
        break;
      }
    }
  }
}

bool TfCache::cache_imu_from_frame_id(
  const std::string & imu_frame_id,
  const rclcpp::Logger & logger)
{
  if (extrinsics_.imu_valid || imu_frame_id.empty()) {
    return extrinsics_.imu_valid;
  }

  if (lookup_transform(
      base_frame_, imu_frame_id, extrinsics_.base_to_imu,
      logger))
  {
    extrinsics_.imu_valid = true;
    RCLCPP_INFO(
      logger, "IMU extrinsic cached from message: %s -> %s",
      base_frame_.c_str(), imu_frame_id.c_str());
  }
  return extrinsics_.imu_valid;
}

bool TfCache::cache_gnss_from_frame_id(
  const std::string & gnss_frame_id,
  const rclcpp::Logger & logger)
{
  if (extrinsics_.gnss_valid || gnss_frame_id.empty()) {
    return extrinsics_.gnss_valid;
  }

  if (lookup_transform(
      base_frame_, gnss_frame_id, extrinsics_.base_to_gnss,
      logger))
  {
    extrinsics_.gnss_valid = true;
    RCLCPP_INFO(
      logger, "GNSS extrinsic cached from message: %s -> %s",
      base_frame_.c_str(), gnss_frame_id.c_str());
  }
  return extrinsics_.gnss_valid;
}

} // namespace eskf_localization
