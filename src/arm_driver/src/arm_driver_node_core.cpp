#include "arm_driver/arm_driver_node.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace arm_driver
{
namespace
{

constexpr double kTwoPi = 2.0 * M_PI;

}  // namespace

std::vector<std::string> ArmDriverNode::default_command_joint_names()
{
  return {
    "elfin_joint1",
    "elfin_joint2",
    "elfin_joint3",
    "elfin_joint4",
    "elfin_joint5",
    "elfin_joint6"};
}

ArmDriverNode::ArmDriverNode(const rclcpp::Node::SharedPtr & node)
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

void ArmDriverNode::initialize_core()
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

void ArmDriverNode::initialize_axis_layout()
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

void ArmDriverNode::init_axis_from_core(AxisState & axis, const size_t joint_index)
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

void ArmDriverNode::build_command_order_mapping()
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

ArmDriverNode::AxisState & ArmDriverNode::axis_from_internal_index(const size_t internal_index)
{
  auto & module = modules_.at(internal_index / kAxesPerModule);
  return (internal_index % kAxesPerModule == 0) ? module.axis1 : module.axis2;
}

const ArmDriverNode::AxisState & ArmDriverNode::axis_from_internal_index(const size_t internal_index) const
{
  const auto & module = modules_.at(internal_index / kAxesPerModule);
  return (internal_index % kAxesPerModule == 0) ? module.axis1 : module.axis2;
}

double ArmDriverNode::counts_to_position(const int32_t pos_count, const AxisState & axis)
{
  return -1.0 * (static_cast<double>(pos_count) - axis.count_zero) / axis.count_rad_factor;
}

double ArmDriverNode::counts_to_velocity(const int16_t vel_count, const AxisState & axis)
{
  return -1.0 * static_cast<double>(vel_count) / axis.count_rad_per_s_factor;
}

double ArmDriverNode::counts_to_effort(const int16_t trq_count, const AxisState & axis)
{
  return -1.0 * static_cast<double>(trq_count) / axis.count_nm_factor;
}

int32_t ArmDriverNode::position_to_counts(const double position, const AxisState & axis)
{
  const double cmd_count = -1.0 * position * axis.count_rad_factor + axis.count_zero;
  return static_cast<int32_t>(cmd_count);
}

void ArmDriverNode::align_count_zeros_locked()
{
  for (auto & module : modules_) {
    const int32_t pos_count1 = module.client->getAxis1PosCnt();
    align_axis_count_zero_locked(module.axis1, pos_count1);

    const int32_t pos_count2 = module.client->getAxis2PosCnt();
    align_axis_count_zero_locked(module.axis2, pos_count2);
  }
}

void ArmDriverNode::align_axis_count_zero_locked(AxisState & axis, const int32_t pos_count)
{
  const double position_tmp =
    (static_cast<double>(pos_count) - axis.count_zero) / axis.count_rad_factor;
  if (position_tmp >= M_PI) {
    axis.count_zero += axis.count_rad_factor * kTwoPi;
  } else if (position_tmp < -M_PI) {
    axis.count_zero -= axis.count_rad_factor * kTwoPi;
  }
}

bool ArmDriverNode::read_hardware_state_locked()
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

void ArmDriverNode::synchronize_commands_to_current_locked()
{
  for (auto & module : modules_) {
    module.axis1.command_position = module.axis1.position;
    module.axis2.command_position = module.axis2.position;
  }
}

bool ArmDriverNode::write_joint_commands_locked()
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

bool ArmDriverNode::fill_command_targets_locked(
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

bool ArmDriverNode::invoke_core_service_locked(DriverServiceFn fn, std::string & message)
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

}  // namespace arm_driver
