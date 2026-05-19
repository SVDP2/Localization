#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ebimu_driver/ebimu_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ebimu_driver::EbimuNode>());
  rclcpp::shutdown();
  return 0;
}
