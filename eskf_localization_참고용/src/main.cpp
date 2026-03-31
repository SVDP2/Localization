#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "eskf_localization/eskf_localization_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // use_sim_time is automatically handled by ROS2 from launch file
  auto node = std::make_shared<eskf_localization::ESKFLocalizationNode>(rclcpp::NodeOptions());

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
