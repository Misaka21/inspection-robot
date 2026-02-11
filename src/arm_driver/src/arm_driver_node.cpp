#include <exception>
#include <memory>

#include "arm_driver/arm_driver_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = rclcpp::Node::make_shared("arm_driver_node");
    auto arm_driver = std::make_shared<arm_driver::ArmDriverNode>(node);
    (void)arm_driver;
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("arm_driver_node"), "Fatal error: %s", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
