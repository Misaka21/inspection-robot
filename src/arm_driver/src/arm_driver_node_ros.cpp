#include "arm_driver/arm_driver_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>

namespace arm_driver
{

void ArmDriverNode::setup_ros_interfaces()
{
  joint_cmd_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "joint_cmd",
    10,
    std::bind(&ArmDriverNode::on_joint_command, this, std::placeholders::_1));

  joint_states_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  status_pub_ = node_->create_publisher<inspection_interface::msg::ArmStatus>("status", 10);

  enable_srv_ = node_->create_service<std_srvs::srv::Trigger>(
    "enable",
    std::bind(&ArmDriverNode::on_enable, this, std::placeholders::_1, std::placeholders::_2));
  disable_srv_ = node_->create_service<std_srvs::srv::Trigger>(
    "disable",
    std::bind(&ArmDriverNode::on_disable, this, std::placeholders::_1, std::placeholders::_2));
  clear_fault_srv_ = node_->create_service<std_srvs::srv::Trigger>(
    "clear_fault",
    std::bind(&ArmDriverNode::on_clear_fault, this, std::placeholders::_1, std::placeholders::_2));
  stop_srv_ = node_->create_service<std_srvs::srv::Trigger>(
    "stop",
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

void ArmDriverNode::on_joint_command(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // data_mutex_ 同时保护 on_joint_command（ROS 回调线程）和 on_state_timer（定时器线程）
  // 两者都访问 modules_ 中的 command_position 与 position，必须互斥
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

void ArmDriverNode::on_state_timer()
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

void ArmDriverNode::on_status_timer()
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
      // 相邻两次状态帧之间的关节角变化量超过阈值，说明机械臂仍在运动
      // 补充底层驱动 getMotionState 可能延迟的情况（驱动未报告运动但实际还在走）
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

  // moving_by_core：驱动层自报的运动标志（来自厂商 API getMotionState）
  // moving_by_delta：从连续帧位置差推算的运动标志
  // 两者取 OR 是为了应对驱动标志延迟置位/提前复位的边界情况
  status.moving = moving_by_delta || moving_by_core;
  // pos_aligned = 驱动报告目标位置已对齐（getPosAlignState）
  // arrived 语义：位置已对齐 且 没有任何运动迹象
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

void ArmDriverNode::on_enable(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  std::string message;
  const bool ok =
    invoke_core_service_locked(&elfin_ethercat_driver::ElfinEtherCATDriver::enableRobot_cb, message);
  if (ok) {
    if (!read_hardware_state_locked()) {
      response->success = false;
      response->message = "enable succeeded but state read failed: " + last_error_;
      return;
    }
    // enable 后必须先读硬件当前位置，再把 command_position 同步到当前位置
    // 再立即 write，目的是让驱动以"当前实际位置"作为第一条指令
    // 如果跳过这步，驱动会以上次的 command_position（可能已过期）为起点，导致电机突跳
    synchronize_commands_to_current_locked();
    write_joint_commands_locked();
  }

  response->success = ok;
  response->message = message;
}

void ArmDriverNode::on_disable(
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

void ArmDriverNode::on_clear_fault(
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

void ArmDriverNode::on_stop(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  // stop 的实现是"读→同步→写"三步：
  // 1. read：获取当前实际位置
  // 2. synchronize：把 command_position 覆盖为当前实际位置
  // 3. write：下发"原地不动"指令
  // 这样驱动收到的目标位置等于现在所在位置，电机会立刻锁住，不会跳到之前缓存的指令位置
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

}  // namespace arm_driver
