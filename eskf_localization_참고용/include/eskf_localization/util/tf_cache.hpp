#ifndef ESKF_LOCALIZATION__TF_CACHE_HPP_
#define ESKF_LOCALIZATION__TF_CACHE_HPP_

#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/logger.hpp>
#include <tf2_ros/buffer.h>

#include "eskf_localization/types.hpp"

namespace eskf_localization
{

class TfCache
{
public:
  TfCache() = default;
  TfCache(
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
    std::string base_frame);

  void set_tf_buffer(const std::shared_ptr<tf2_ros::Buffer> & tf_buffer);
  void set_base_frame(std::string base_frame);

  const std::string & base_frame() const {return base_frame_;}
  const ExtrinsicCache & extrinsics() const {return extrinsics_;}

  bool cache_imu_from_frame_id(
    const std::string & imu_frame_id,
    const rclcpp::Logger & logger);
  bool cache_gnss_from_frame_id(
    const std::string & gnss_frame_id,
    const rclcpp::Logger & logger);

  void cache_common_extrinsics(const rclcpp::Logger & logger);

private:
  bool lookup_transform(
    const std::string & from_frame,
    const std::string & to_frame,
    geometry_msgs::msg::TransformStamped & transform,
    const rclcpp::Logger & logger) const;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string base_frame_{"base_link"};
  ExtrinsicCache extrinsics_;
};

} // namespace eskf_localization

#endif // ESKF_LOCALIZATION__TF_CACHE_HPP_
