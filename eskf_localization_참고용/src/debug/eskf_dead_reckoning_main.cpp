#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "eskf_localization/debug/eskf_dead_reckoning_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<eskf_localization::EskfDeadReckoningNode>(
      rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
