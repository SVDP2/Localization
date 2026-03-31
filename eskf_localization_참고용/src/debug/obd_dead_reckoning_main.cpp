#include "eskf_localization/debug/obd_dead_reckoning_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<eskf_localization::ObdDeadReckoningNode>(
      rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
