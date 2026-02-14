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

// 核心初始化：建立 EtherCAT 管理器和驱动，读取初始硬件状态。
// 先对齐 count_zero（防止跨 π 边界的计数溢出），再同步命令位置为当前实际位置（防止上电跳变）。
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

// 扫描 EtherCAT 总线上的模块，建立"模块列表"和"关节名->内部索引"映射。
// 每个模块对应 2 个轴（axis1/axis2），顺序由厂商 elfin_ethercat_driver 确定。
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

// 从厂商驱动读取单个轴的参数并计算换算系数：
//   count_rad_factor = reduction_ratio * axis_position_factor / (2π)
//     表示 1 弧度对应多少个编码器计数
//   count_rad_per_s_factor = count_rad_factor / kVelocityScale
//   count_nm_factor = axis_torque_factor / reduction_ratio
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

// 建立"外部命令关节名顺序"到"内部轴索引"的映射。
// 意义：MoveIt2 发出的 JointState 关节名顺序可能与硬件轴顺序不一致，
// 这里将二者对齐，避免张冠李戴发错关节指令。
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

// 对齐所有轴的 count_zero：若当前计数对应的角度在 [-π, π] 之外，
// 则将 count_zero 偏移一整圈（2π 对应的计数），使得关节角始终在正常范围内。
// 这是为了处理上电时绝对编码器位置可能超出表示范围的问题。
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

// 从 EtherCAT 读取所有关节的位置/速度/力矩计数，并换算为 SI 单位（弧度/rad·s⁻¹/N·m）。
// 必须在 data_mutex_ 保护下调用（由定时器/线程持锁后执行）。
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

// 将命令位置同步为当前实际位置，用于初始化或紧急停止后防止位置跳变。
// 确保机械臂下一次写入命令时从当前位置平滑开始运动。
void ArmDriverNode::synchronize_commands_to_current_locked()
{
  for (auto & module : modules_) {
    module.axis1.command_position = module.axis1.position;
    module.axis2.command_position = module.axis2.position;
  }
}

// 将 command_position 写入到 EtherCAT PDO（位置控制模式），速度前馈设为 0。
// 若驱动未处于位置模式则先切换，保证控制模式一致。
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

// 从 JointState 消息中提取目标位置到内部 targets 数组。
// 支持三种格式：
//   1. 带 name 字段（按关节名查找，可部分指定）
//   2. 不带 name、长度 = command_joint_names_ 数量（按 command 顺序）
//   3. 不带 name、长度 = 所有内部关节数量（按内部顺序）
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
