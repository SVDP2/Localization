#ifndef ESKF_LOCALIZATION__TIME_PROCESSING_HPP_
#define ESKF_LOCALIZATION__TIME_PROCESSING_HPP_

#include <rclcpp/time.hpp>

#include <algorithm>
#include <limits>

#include "eskf_localization/types.hpp"

namespace eskf_localization
{

enum class StampValidationResult
{
  kOk = 0,
  kInvalidStamp,
  kTooOld,
  kFutureStamp,
};

enum class OrderValidationResult
{
  kOk = 0,
  kOutOfOrderOrDuplicate,
};

struct ImuDtStats
{
  double dt_min{std::numeric_limits<double>::max()};
  double dt_max{0.0};
  double dt_sum{0.0};
  size_t dt_count{0};

  void reset()
  {
    dt_min = std::numeric_limits<double>::max();
    dt_max = 0.0;
    dt_sum = 0.0;
    dt_count = 0;
  }

  void update(const double dt)
  {
    dt_min = std::min(dt_min, dt);
    dt_max = std::max(dt_max, dt);
    dt_sum += dt;
    dt_count++;
  }

  bool has_samples() const {return dt_count > 0;}
  double mean() const {return dt_sum / static_cast<double>(dt_count);}
};

class TimeProcessor
{
public:
  explicit TimeProcessor(const TimeProcessingParams & params)
  : params_(params) {}

  const TimeProcessingParams & params() const {return params_;}

  StampValidationResult validate_stamp(
    const rclcpp::Time & stamp,
    const rclcpp::Time & now) const
  {
    if (stamp.seconds() <= 0.0) {
      return StampValidationResult::kInvalidStamp;
    }

    const double delay = (now - stamp).seconds();
    if (delay > params_.max_delay) {
      return StampValidationResult::kTooOld;
    }

    if (delay < -params_.future_tolerance) {
      return StampValidationResult::kFutureStamp;
    }

    return StampValidationResult::kOk;
  }

  OrderValidationResult validate_order(
    const rclcpp::Time & last_stamp,
    const rclcpp::Time & current_stamp) const
  {
    if (last_stamp.seconds() <= 0.0) {
      return OrderValidationResult::kOk;
    }

    const double dt = (current_stamp - last_stamp).seconds();
    if (dt <= 0.0) {
      return OrderValidationResult::kOutOfOrderOrDuplicate;
    }

    return OrderValidationResult::kOk;
  }

  double compute_clamped_dt(
    const rclcpp::Time & last_stamp,
    const rclcpp::Time & current_stamp) const
  {
    const double dt = (current_stamp - last_stamp).seconds();
    return std::clamp(dt, params_.min_dt, params_.max_dt);
  }

private:
  TimeProcessingParams params_;
};

} // namespace eskf_localization

#endif // ESKF_LOCALIZATION__TIME_PROCESSING_HPP_
