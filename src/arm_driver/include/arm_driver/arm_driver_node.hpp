#ifndef ARM_DRIVER__ARM_DRIVER_NODE_HPP_
#define ARM_DRIVER__ARM_DRIVER_NODE_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "elfin_ethercat_driver/elfin_ethercat_driver.h"
#include "inspection_interface/msg/arm_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace arm_driver
{

class ArmDriverNode
{
public:
  explicit ArmDriverNode(const rclcpp::Node::SharedPtr & node);

private:
  struct AxisState
  {
    std::string name;
    double reduction_ratio{0.0};
    double axis_position_factor{0.0};
    double axis_torque_factor{0.0};
    double count_rad_factor{0.0};
    double count_rad_per_s_factor{0.0};
    double count_nm_factor{0.0};
    double count_zero{0.0};

    double position{0.0};
    double velocity{0.0};
    double effort{0.0};
    double command_position{0.0};
  };

  struct ModuleState
  {
    elfin_ethercat_driver::ElfinEtherCATClient * client{nullptr};
    AxisState axis1;
    AxisState axis2;
  };

  using DriverServiceFn = bool (elfin_ethercat_driver::ElfinEtherCATDriver::*)(
    const std::shared_ptr<std_srvs::srv::SetBool::Request>,
    const std::shared_ptr<std_srvs::srv::SetBool::Response>);

  static constexpr double kVelocityScale = 750.3;
  static constexpr size_t kAxesPerModule = 2;

  static std::vector<std::string> default_command_joint_names();

  void initialize_core();
  void setup_ros_interfaces();
  void initialize_axis_layout();
  void init_axis_from_core(AxisState & axis, size_t joint_index);
  void build_command_order_mapping();

  AxisState & axis_from_internal_index(size_t internal_index);
  const AxisState & axis_from_internal_index(size_t internal_index) const;

  static double counts_to_position(int32_t pos_count, const AxisState & axis);
  static double counts_to_velocity(int16_t vel_count, const AxisState & axis);
  static double counts_to_effort(int16_t trq_count, const AxisState & axis);
  static int32_t position_to_counts(double position, const AxisState & axis);

  void align_count_zeros_locked();
  void align_axis_count_zero_locked(AxisState & axis, int32_t pos_count);
  bool read_hardware_state_locked();
  void synchronize_commands_to_current_locked();
  bool write_joint_commands_locked();

  bool fill_command_targets_locked(
    const sensor_msgs::msg::JointState & msg,
    std::vector<double> & targets,
    std::vector<bool> & has_target);

  bool invoke_core_service_locked(DriverServiceFn fn, std::string & message);

  void on_joint_command(const sensor_msgs::msg::JointState::SharedPtr msg);
  void on_state_timer();
  void on_status_timer();

  void on_enable(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void on_disable(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void on_clear_fault(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void on_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  rclcpp::Node::SharedPtr node_;

  std::string ethernet_name_;
  double state_publish_rate_hz_{50.0};
  double status_publish_rate_hz_{10.0};
  double motion_threshold_{5e-5};
  bool stop_disable_motors_{false};

  std::vector<std::string> internal_joint_names_;
  std::unordered_map<std::string, size_t> internal_index_by_name_;
  std::vector<std::string> command_joint_names_;
  std::vector<size_t> command_to_internal_indices_;
  std::vector<double> last_reported_positions_;

  std::unique_ptr<elfin_ethercat_driver::EtherCatManager> manager_;
  std::unique_ptr<elfin_ethercat_driver::ElfinEtherCATDriver> core_driver_;
  std::vector<ModuleState> modules_;

  bool connected_{false};
  std::string last_error_;
  std::mutex data_mutex_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<inspection_interface::msg::ArmStatus>::SharedPtr status_pub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_fault_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;

  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace arm_driver

#endif  // ARM_DRIVER__ARM_DRIVER_NODE_HPP_
