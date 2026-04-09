#include <exception>
#include <memory>

#include "arm_driver/arm_driver_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = rclcpp::Node::make_shared("arm_driver_node");

    // elfin_ethercat_driver 会在接收到的节点上创建 service（如 clear_fault/SetBool），
    // 与 arm_driver 的同名 service（clear_fault/Trigger）类型不同导致 DDS 拒绝。
    // 给厂商驱动一个独立节点，service 落在不同的 fully-qualified name 下，避免冲突。
    // YAML 使用 /**:，同进程内所有节点都能继承参数；elfin 节点开启
    // automatically_declare_parameters_from_overrides 让厂商驱动的 declare_parameter 正常工作。
    // 全局参数含 __ns:=/inspection/arm，会覆盖构造函数里指定的命名空间。
    // 用局部 arguments 追加一条 __ns 覆盖全局值，同时保留全局 --params-file。
    const std::string elfin_ns = std::string(node->get_namespace()) + "/elfin_internal";
    rclcpp::NodeOptions elfin_options;
    elfin_options.arguments({"--ros-args", "-r", "__ns:=" + elfin_ns});
    auto elfin_node = rclcpp::Node::make_shared("elfin_internal", elfin_options);

    auto arm_driver = std::make_shared<arm_driver::ArmDriverNode>(node, elfin_node);
    (void)arm_driver;

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    exec.add_node(elfin_node);
    exec.spin();
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("arm_driver_node"), "Fatal error: %s", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
