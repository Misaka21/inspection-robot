#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <numeric>
#include <stdexcept>
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
namespace
{

constexpr double kTwoPi = 2.0 * M_PI;
constexpr double kVelocityScale = 750.3;
constexpr size_t kAxesPerModule = 2;

std::vector<std::string> default_command_joint_names()
{
  return {
    "elfin_joint1",
    "elfin_joint2",
    "elfin_joint3",
    "elfin_joint4",
    "elfin_joint5",
    "elfin_joint6"};
}

}  // namespace

class ArmDriverNode
{
public:
  explicit ArmDriverNode(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    ethernet_name_ = node_->declare_parameter<std::string>("elfin_ethernet_name", "eth0");
    state_publish_rate_hz_ = node_->declare_parameter<double>("state_publish_rate_hz", 50.0);
    status_publish_rate_hz_ = node_->declare_parameter<double>("status_publish_rate_hz", 10.0);
    motion_threshold_ = node_->declare_parameter<double>("motion_threshold", 5e-5);
    stop_disable_motors_ = node_->declare_parameter<bool>("stop_disable_motors", false);
    command_joint_names_ = node_->declare_parameter<std::vector<std::string>>(
      "command_joint_names", default_command_joint_names());

    initialize_core();
    setup_ros_interfaces();

    RCLCPP_INFO(
      node_->get_logger(),
      "arm_driver_node started (ethernet=%s, joints=%zu)",
      ethernet_name_.c_str(),
      internal_joint_names_.size());
  }

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

  void initialize_core()
  {
    manager_ = std::make_unique<elfin_ethercat_driver::EtherCatManager>(ethernet_name_);
    core_driver_ = std::make_unique<elfin_ethercat_driver::ElfinEtherCATDriver>(
      manager_.get(), "elfin", node_);

    initialize_axis_layout();

    std::lock_guard<std::mutex> lock(data_mutex_);
    align_count_zeros_locked();
    if (!read_hardware_state_locked()) {
      throw std::runtime_error("Failed to read initial hardware state: " + last_error_);
    }
    synchronize_commands_to_current_locked();
  }

  void setup_ros_interfaces()
  {
    joint_cmd_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "~/joint_cmd",
      10,
      std::bind(&ArmDriverNode::on_joint_command, this, std::placeholders::_1));

    joint_states_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    status_pub_ = node_->create_publisher<inspection_interface::msg::ArmStatus>("~/status", 10);

    enable_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/enable",
      std::bind(&ArmDriverNode::on_enable, this, std::placeholders::_1, std::placeholders::_2));
    disable_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/disable",
      std::bind(&ArmDriverNode::on_disable, this, std::placeholders::_1, std::placeholders::_2));
    clear_fault_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/clear_fault",
      std::bind(&ArmDriverNode::on_clear_fault, this, std::placeholders::_1, std::placeholders::_2));
    stop_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/stop",
      std::bind(&ArmDriverNode::on_stop, this, std::placeholders::_1, std::placeholders::_2));

    const auto safe_hz = [](const double hz) {
        return std::max(hz, 1.0);
      };
    const auto state_period_ns = static_cast<int64_t>(1e9 / safe_hz(state_publish_rate_hz_));
    const auto status_period_ns = static_cast<int64_t>(1e9 / safe_hz(status_publish_rate_hz_));

    state_timer_ = node_->create_wall_timer(
      std::chrono::nanoseconds(state_period_ns),
      std::bind(&ArmDriverNode::on_state_timer, this));
    status_timer_ = node_->create_wall_timer(
      std::chrono::nanoseconds(status_period_ns),
      std::bind(&ArmDriverNode::on_status_timer, this));
  }

  void initialize_axis_layout()
  {
    const size_t module_count = core_driver_->getEtherCATClientNumber();
    if (module_count == 0) {
      throw std::runtime_error("No EtherCAT module detected for elfin arm");
    }

    modules_.clear();
    modules_.reserve(module_count);
    internal_joint_names_.clear();
    internal_joint_names_.reserve(module_count * kAxesPerModule);
    internal_index_by_name_.clear();

    for (size_t module_idx = 0; module_idx < module_count; ++module_idx) {
      ModuleState module;
      module.client = core_driver_->getEtherCATClientPtr(module_idx);
      init_axis_from_core(module.axis1, 2 * module_idx);
      init_axis_from_core(module.axis2, 2 * module_idx + 1);

      modules_.push_back(module);
      internal_joint_names_.push_back(module.axis1.name);
      internal_index_by_name_[module.axis1.name] = internal_joint_names_.size() - 1;
      internal_joint_names_.push_back(module.axis2.name);
      internal_index_by_name_[module.axis2.name] = internal_joint_names_.size() - 1;
    }

    build_command_order_mapping();
  }

  void init_axis_from_core(AxisState & axis, const size_t joint_index)
  {
    axis.name = core_driver_->getJointName(joint_index);
    axis.reduction_ratio = core_driver_->getReductionRatio(joint_index);
    axis.axis_position_factor = core_driver_->getAxisPositionFactor(joint_index);
    axis.axis_torque_factor = core_driver_->getAxisTorqueFactor(joint_index);
    axis.count_zero = static_cast<double>(core_driver_->getCountZero(joint_index));

    axis.count_rad_factor = axis.reduction_ratio * axis.axis_position_factor / kTwoPi;
    axis.count_rad_per_s_factor = axis.count_rad_factor / kVelocityScale;
    axis.count_nm_factor = axis.axis_torque_factor / axis.reduction_ratio;
  }

  void build_command_order_mapping()
  {
    command_to_internal_indices_.clear();
    command_to_internal_indices_.reserve(command_joint_names_.size());

    bool map_valid = (command_joint_names_.size() == internal_joint_names_.size());
    if (map_valid) {
      for (const auto & joint_name : command_joint_names_) {
        const auto it = internal_index_by_name_.find(joint_name);
        if (it == internal_index_by_name_.end()) {
          map_valid = false;
          break;
        }
        command_to_internal_indices_.push_back(it->second);
      }
    }

    if (!map_valid) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Parameter command_joint_names does not match hardware joint names, fallback to internal order");
      command_joint_names_ = internal_joint_names_;
      command_to_internal_indices_.resize(command_joint_names_.size());
      std::iota(command_to_internal_indices_.begin(), command_to_internal_indices_.end(), 0U);
    }

    last_reported_positions_.assign(command_joint_names_.size(), 0.0);
  }

  AxisState & axis_from_internal_index(const size_t internal_index)
  {
    auto & module = modules_.at(internal_index / kAxesPerModule);
    return (internal_index % kAxesPerModule == 0) ? module.axis1 : module.axis2;
  }

  const AxisState & axis_from_internal_index(const size_t internal_index) const
  {
    const auto & module = modules_.at(internal_index / kAxesPerModule);
    return (internal_index % kAxesPerModule == 0) ? module.axis1 : module.axis2;
  }

  static double counts_to_position(const int32_t pos_count, const AxisState & axis)
  {
    return -1.0 * (static_cast<double>(pos_count) - axis.count_zero) / axis.count_rad_factor;
  }

  static double counts_to_velocity(const int16_t vel_count, const AxisState & axis)
  {
    return -1.0 * static_cast<double>(vel_count) / axis.count_rad_per_s_factor;
  }

  static double counts_to_effort(const int16_t trq_count, const AxisState & axis)
  {
    return -1.0 * static_cast<double>(trq_count) / axis.count_nm_factor;
  }

  static int32_t position_to_counts(const double position, const AxisState & axis)
  {
    const double cmd_count = -1.0 * position * axis.count_rad_factor + axis.count_zero;
    return static_cast<int32_t>(cmd_count);
  }

  void align_count_zeros_locked()
  {
    for (auto & module : modules_) {
      const int32_t pos_count1 = module.client->getAxis1PosCnt();
      align_axis_count_zero_locked(module.axis1, pos_count1);

      const int32_t pos_count2 = module.client->getAxis2PosCnt();
      align_axis_count_zero_locked(module.axis2, pos_count2);
    }
  }

  void align_axis_count_zero_locked(AxisState & axis, const int32_t pos_count)
  {
    const double position_tmp =
      (static_cast<double>(pos_count) - axis.count_zero) / axis.count_rad_factor;
    if (position_tmp >= M_PI) {
      axis.count_zero += axis.count_rad_factor * kTwoPi;
    } else if (position_tmp < -M_PI) {
      axis.count_zero -= axis.count_rad_factor * kTwoPi;
    }
  }

  bool read_hardware_state_locked()
  {
    try {
      for (auto & module : modules_) {
        const int32_t pos_count1 = module.client->getAxis1PosCnt();
        const int16_t vel_count1 = module.client->getAxis1VelCnt();
        const int16_t trq_count1 = module.client->getAxis1TrqCnt();
        module.axis1.position = counts_to_position(pos_count1, module.axis1);
        module.axis1.velocity = counts_to_velocity(vel_count1, module.axis1);
        module.axis1.effort = counts_to_effort(trq_count1, module.axis1);

        const int32_t pos_count2 = module.client->getAxis2PosCnt();
        const int16_t vel_count2 = module.client->getAxis2VelCnt();
        const int16_t trq_count2 = module.client->getAxis2TrqCnt();
        module.axis2.position = counts_to_position(pos_count2, module.axis2);
        module.axis2.velocity = counts_to_velocity(vel_count2, module.axis2);
        module.axis2.effort = counts_to_effort(trq_count2, module.axis2);
      }
    } catch (const std::exception & ex) {
      connected_ = false;
      last_error_ = ex.what();
      return false;
    }

    connected_ = true;
    last_error_.clear();
    return true;
  }

  void synchronize_commands_to_current_locked()
  {
    for (auto & module : modules_) {
      module.axis1.command_position = module.axis1.position;
      module.axis2.command_position = module.axis2.position;
    }
  }

  bool write_joint_commands_locked()
  {
    try {
      for (auto & module : modules_) {
        if (!module.client->inPosBasedMode()) {
          module.client->setPosMode();
        }

        module.client->setAxis1PosCnt(position_to_counts(module.axis1.command_position, module.axis1));
        module.client->setAxis2PosCnt(position_to_counts(module.axis2.command_position, module.axis2));
        module.client->setAxis1VelFFCnt(0);
        module.client->setAxis2VelFFCnt(0);
      }
    } catch (const std::exception & ex) {
      connected_ = false;
      last_error_ = ex.what();
      return false;
    }

    return true;
  }

  bool fill_command_targets_locked(
    const sensor_msgs::msg::JointState & msg,
    std::vector<double> & targets,
    std::vector<bool> & has_target)
  {
    const size_t joint_count = internal_joint_names_.size();
    targets.assign(joint_count, 0.0);
    has_target.assign(joint_count, false);

    if (msg.position.empty()) {
      return false;
    }

    if (msg.name.size() == msg.position.size() && !msg.name.empty()) {
      bool matched = false;
      for (size_t i = 0; i < msg.name.size(); ++i) {
        const auto it = internal_index_by_name_.find(msg.name[i]);
        if (it == internal_index_by_name_.end()) {
          continue;
        }
        targets[it->second] = msg.position[i];
        has_target[it->second] = true;
        matched = true;
      }
      return matched;
    }

    if (msg.position.size() == command_to_internal_indices_.size()) {
      for (size_t i = 0; i < msg.position.size(); ++i) {
        const size_t internal_idx = command_to_internal_indices_[i];
        targets[internal_idx] = msg.position[i];
        has_target[internal_idx] = true;
      }
      return true;
    }

    if (msg.position.size() == joint_count) {
      for (size_t i = 0; i < msg.position.size(); ++i) {
        targets[i] = msg.position[i];
        has_target[i] = true;
      }
      return true;
    }

    return false;
  }

  bool invoke_core_service_locked(DriverServiceFn fn, std::string & message)
  {
    if (!core_driver_) {
      message = "core driver is not initialized";
      return false;
    }

    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    auto resp = std::make_shared<std_srvs::srv::SetBool::Response>();
    req->data = true;

    (core_driver_.get()->*fn)(req, resp);
    message = resp->message;
    return resp->success;
  }

  void on_joint_command(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (!connected_) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        2000,
        "drop joint_cmd because arm is not connected");
      return;
    }
    if (!core_driver_->getEnableState()) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        2000,
        "drop joint_cmd because arm is disabled");
      return;
    }

    std::vector<double> targets;
    std::vector<bool> has_target;
    if (!fill_command_targets_locked(*msg, targets, has_target)) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        2000,
        "invalid joint_cmd: provide joint names or %zu positions in configured order",
        command_joint_names_.size());
      return;
    }

    for (size_t internal_idx = 0; internal_idx < has_target.size(); ++internal_idx) {
      if (!has_target[internal_idx]) {
        continue;
      }
      axis_from_internal_index(internal_idx).command_position = targets[internal_idx];
    }

    if (!write_joint_commands_locked()) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        2000,
        "write joint command failed: %s",
        last_error_.c_str());
    }
  }

  void on_state_timer()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (!read_hardware_state_locked()) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        2000,
        "read hardware state failed: %s",
        last_error_.c_str());
      return;
    }

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = node_->now();
    msg.name = command_joint_names_;
    msg.position.reserve(command_to_internal_indices_.size());
    msg.velocity.reserve(command_to_internal_indices_.size());
    msg.effort.reserve(command_to_internal_indices_.size());

    for (const size_t internal_idx : command_to_internal_indices_) {
      const auto & axis = axis_from_internal_index(internal_idx);
      msg.position.push_back(axis.position);
      msg.velocity.push_back(axis.velocity);
      msg.effort.push_back(axis.effort);
    }

    joint_states_pub_->publish(msg);
  }

  void on_status_timer()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    inspection_interface::msg::ArmStatus status;
    status.connected = connected_;
    status.current_joints.resize(command_to_internal_indices_.size(), 0.0);
    status.manipulability = 0.0F;

    for (size_t i = 0; i < command_to_internal_indices_.size(); ++i) {
      status.current_joints[i] = axis_from_internal_index(command_to_internal_indices_[i]).position;
    }

    bool moving_by_delta = false;
    if (last_reported_positions_.size() == status.current_joints.size()) {
      for (size_t i = 0; i < status.current_joints.size(); ++i) {
        if (std::fabs(status.current_joints[i] - last_reported_positions_[i]) > motion_threshold_) {
          moving_by_delta = true;
          break;
        }
      }
    }
    last_reported_positions_ = status.current_joints;

    bool moving_by_core = false;
    bool pos_aligned = false;
    bool enabled = false;
    bool fault = false;
    if (core_driver_ != nullptr && connected_) {
      moving_by_core = core_driver_->getMotionState();
      pos_aligned = core_driver_->getPosAlignState();
      enabled = core_driver_->getEnableState();
      fault = core_driver_->getFaultState();
    }

    status.moving = moving_by_delta || moving_by_core;
    status.arrived = (!status.moving) && pos_aligned;

    if (!connected_ && !last_error_.empty()) {
      status.error_code = last_error_;
    } else if (fault) {
      status.error_code = "FAULT";
    } else if (!enabled) {
      status.error_code = "DISABLED";
    } else {
      status.error_code.clear();
    }

    status_pub_->publish(status);
  }

  void on_enable(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    std::string message;
    const bool ok = invoke_core_service_locked(&elfin_ethercat_driver::ElfinEtherCATDriver::enableRobot_cb, message);
    if (ok) {
      if (!read_hardware_state_locked()) {
        response->success = false;
        response->message = "enable succeeded but state read failed: " + last_error_;
        return;
      }
      synchronize_commands_to_current_locked();
      write_joint_commands_locked();
    }

    response->success = ok;
    response->message = message;
  }

  void on_disable(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    std::string message;
    const bool ok =
      invoke_core_service_locked(&elfin_ethercat_driver::ElfinEtherCATDriver::disableRobot_cb, message);
    response->success = ok;
    response->message = message;
  }

  void on_clear_fault(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    std::string message;
    const bool ok =
      invoke_core_service_locked(&elfin_ethercat_driver::ElfinEtherCATDriver::clearFault_cb, message);
    response->success = ok;
    response->message = message;
  }

  void on_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (!read_hardware_state_locked()) {
      response->success = false;
      response->message = "read hardware state failed: " + last_error_;
      return;
    }
    synchronize_commands_to_current_locked();
    if (!write_joint_commands_locked()) {
      response->success = false;
      response->message = "stop command write failed: " + last_error_;
      return;
    }

    std::string message = "stop command applied";
    bool ok = true;
    if (stop_disable_motors_) {
      std::string disable_msg;
      ok = invoke_core_service_locked(
        &elfin_ethercat_driver::ElfinEtherCATDriver::disableRobot_cb, disable_msg);
      message += "; disable: " + disable_msg;
    }

    response->success = ok;
    response->message = message;
  }

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
