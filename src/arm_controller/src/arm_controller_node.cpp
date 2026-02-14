#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "inspection_interface/srv/move_to_pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace arm_controller
{

namespace
{

double clamp01(const double v)
{
  // 先过滤 NaN/Inf，防止 std::clamp 在无效输入下行为未定义（某些 STL 实现会 UB 或 assert）
  // NaN 比较永远为 false，不过滤会直接传给 MoveIt 导致规划崩溃
  if (std::isnan(v) || std::isinf(v)) {
    return 0.0;
  }
  return std::clamp(v, 0.0, 1.0);
}

int64_t duration_to_ns(const builtin_interfaces::msg::Duration & d)
{
  return static_cast<int64_t>(d.sec) * 1000000000LL + static_cast<int64_t>(d.nanosec);
}

}  // namespace

class ArmControllerNode
{
public:
  explicit ArmControllerNode(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    planning_group_ = node_->declare_parameter<std::string>("planning_group", "elfin_arm");
    planning_frame_ = node_->declare_parameter<std::string>("planning_frame", "world");
    planning_time_sec_ = node_->declare_parameter<double>("planning_time", 5.0);
    velocity_scaling_ = clamp01(node_->declare_parameter<double>("velocity_scaling", 0.5));
    acceleration_scaling_ = clamp01(node_->declare_parameter<double>("acceleration_scaling", 0.5));

    stream_trajectory_ = node_->declare_parameter<bool>("stream_trajectory", true);
    arm_driver_joint_cmd_topic_ = node_->declare_parameter<std::string>(
      "arm_driver_joint_cmd_topic", "/inspection/arm/joint_cmd");

    auto_enable_driver_ = node_->declare_parameter<bool>("auto_enable_driver", false);
    arm_driver_enable_service_ = node_->declare_parameter<std::string>(
      "arm_driver_enable_service", "/inspection/arm/enable");

    joint_cmd_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(arm_driver_joint_cmd_topic_, 10);

    motion_status_pub_ = node_->create_publisher<std_msgs::msg::String>("motion_status", 10);
    trajectory_progress_pub_ = node_->create_publisher<std_msgs::msg::Float64>("trajectory_progress", 10);

    velocity_scaling_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
      "velocity_scaling",
      10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        velocity_scaling_ = clamp01(msg->data);
      });

    cart_goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "cart_goal",
      10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // Fire-and-forget：规划 + 执行 全程在 ROS 回调线程里阻塞完成
        // 优点：实现简单；缺点：此回调线程在执行期间无法处理其他消息（包括新的 cart_goal）
        // 后续如需并发/取消能力，需改为独立 worker 线程 + 取消令牌
        // Fire-and-forget behavior: plan + execute in callback thread.
        (void)execute_pose_goal(*msg, velocity_scaling_);
      });

    joint_goal_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "joint_goal",
      10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        // joint_goal 是直接透传：跳过 MoveIt 规划，直接把关节角目标发给 arm_driver
        // 适用于调试或外部已经规划好轨迹点的场景，控制层只做转发，不做碰撞检查
        // Direct joint command passthrough (controller layer responsibility ends here).
        std::lock_guard<std::mutex> lock(mutex_);
        sensor_msgs::msg::JointState cmd = *msg;
        cmd.header.stamp = node_->now();
        joint_cmd_pub_->publish(cmd);
      });

    move_to_pose_srv_ = node_->create_service<inspection_interface::srv::MoveToPose>(
      "move_to_pose",
      [this](
        const std::shared_ptr<inspection_interface::srv::MoveToPose::Request> req,
        std::shared_ptr<inspection_interface::srv::MoveToPose::Response> resp) {
        geometry_msgs::msg::PoseStamped goal;
        goal.header.stamp = node_->now();
        goal.header.frame_id = planning_frame_;
        goal.pose = req->target_pose;

        const double speed = clamp01(static_cast<double>(req->speed));
        const bool ok = execute_pose_goal(goal, speed);
        resp->success = ok;
        resp->message = ok ? "ok" : last_error_;
      });

    enable_client_ = node_->create_client<std_srvs::srv::Trigger>(arm_driver_enable_service_);

    setup_moveit();
  }

private:
  void publish_status(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    motion_status_pub_->publish(msg);
  }

  void publish_progress(const double value)
  {
    std_msgs::msg::Float64 msg;
    msg.data = std::clamp(value, 0.0, 1.0);
    trajectory_progress_pub_->publish(msg);
  }

  void setup_moveit()
  {
    try {
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, planning_group_);
      move_group_->setPlanningTime(std::max(0.1, planning_time_sec_));
      move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
      move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);
      move_group_->startStateMonitor(2.0);
    } catch (const std::exception & ex) {
      last_error_ = std::string("MoveIt init failed: ") + ex.what();
      RCLCPP_ERROR(node_->get_logger(), "%s", last_error_.c_str());
    }
  }

  bool maybe_enable_driver()
  {
    if (!auto_enable_driver_) {
      return true;
    }
    if (!enable_client_->service_is_ready()) {
      last_error_ = "arm_driver enable service not ready: " + arm_driver_enable_service_;
      return false;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = enable_client_->async_send_request(req);
    const auto rc = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5));
    if (rc != rclcpp::FutureReturnCode::SUCCESS) {
      last_error_ = "arm_driver enable service call timeout";
      return false;
    }
    const auto resp = future.get();
    if (!resp->success) {
      last_error_ = "arm_driver enable failed: " + resp->message;
      return false;
    }
    return true;
  }

  bool execute_pose_goal(const geometry_msgs::msg::PoseStamped & goal, const double speed_scaling)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_error_.clear();

    if (move_group_ == nullptr) {
      last_error_ = "MoveIt not initialized";
      return false;
    }
    if (!maybe_enable_driver()) {
      publish_status("enable_driver_failed");
      return false;
    }

    publish_status("planning");
    publish_progress(0.0);

    const double speed = clamp01(speed_scaling);
    move_group_->setMaxVelocityScalingFactor(speed);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);
    move_group_->setPoseTarget(goal);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto code = move_group_->plan(plan);
    if (code != moveit::core::MoveItErrorCode::SUCCESS) {
      last_error_ = "planning failed";
      publish_status("plan_failed");
      return false;
    }

    const auto & traj = plan.trajectory_.joint_trajectory;
    if (traj.joint_names.empty() || traj.points.empty()) {
      last_error_ = "empty trajectory";
      publish_status("plan_empty");
      return false;
    }

    publish_status("executing");
    // stream_trajectory_ = true：逐点流式下发，每个轨迹点按时间间隔依次发送
    //   优点：机械臂能平滑跟踪整条轨迹，过程中可监控进度；缺点：耗时等于轨迹总时长
    // stream_trajectory_ = false：只发最后一个点，让驱动自行插值到终点
    //   优点：响应快；缺点：中间过程无法干预，路径不受 MoveIt 规划约束
    const bool ok = stream_trajectory_ ? execute_trajectory(traj) : execute_final_point(traj);
    publish_status(ok ? "done" : "execute_failed");
    publish_progress(ok ? 1.0 : 0.0);
    return ok;
  }

  bool execute_final_point(const trajectory_msgs::msg::JointTrajectory & traj)
  {
    if (traj.points.empty()) {
      last_error_ = "trajectory has no points";
      return false;
    }
    const auto & last = traj.points.back();
    if (last.positions.size() != traj.joint_names.size()) {
      last_error_ = "trajectory point size mismatch";
      return false;
    }
    sensor_msgs::msg::JointState cmd;
    cmd.header.stamp = node_->now();
    cmd.name = traj.joint_names;
    cmd.position = last.positions;
    joint_cmd_pub_->publish(cmd);
    return true;
  }

  bool execute_trajectory(const trajectory_msgs::msg::JointTrajectory & traj)
  {
    int64_t prev_ns = 0;
    const auto points_n = traj.points.size();
    for (size_t i = 0; i < points_n && rclcpp::ok(); ++i) {
      const auto & pt = traj.points[i];
      if (pt.positions.size() != traj.joint_names.size()) {
        last_error_ = "trajectory point size mismatch";
        return false;
      }

      sensor_msgs::msg::JointState cmd;
      cmd.header.stamp = node_->now();
      cmd.name = traj.joint_names;
      cmd.position = pt.positions;
      joint_cmd_pub_->publish(cmd);

      // points_n > 1 时：progress = i / (points_n - 1)，使进度在 [0, 1] 均匀分布
      // points_n == 1 时直接发布 1.0，避免除零
      publish_progress(points_n > 1 ? static_cast<double>(i) / static_cast<double>(points_n - 1) : 1.0);

      // now_ns = 当前点相对于轨迹起点的绝对时间戳（纳秒）
      // sleep_ns = now_ns - prev_ns = 相邻两轨迹点之间的时间间隔
      // 用 max(0, ...) 防止时间戳乱序时 sleep 负值
      const int64_t now_ns = duration_to_ns(pt.time_from_start);
      const int64_t sleep_ns = std::max<int64_t>(0, now_ns - prev_ns);
      prev_ns = now_ns;
      if (sleep_ns > 0) {
        std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_ns));
      }
    }
    return rclcpp::ok();
  }

  rclcpp::Node::SharedPtr node_;
  std::mutex mutex_;

  std::string planning_group_;
  std::string planning_frame_;
  double planning_time_sec_{5.0};
  double velocity_scaling_{0.5};
  double acceleration_scaling_{0.5};

  bool stream_trajectory_{true};
  std::string arm_driver_joint_cmd_topic_;

  bool auto_enable_driver_{false};
  std::string arm_driver_enable_service_;

  std::string last_error_;

  moveit::planning_interface::MoveGroupInterfacePtr move_group_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motion_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr trajectory_progress_pub_;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_scaling_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cart_goal_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_goal_sub_;

  rclcpp::Service<inspection_interface::srv::MoveToPose>::SharedPtr move_to_pose_srv_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr enable_client_;
};

}  // namespace arm_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("arm_controller_node", options);

  try {
    auto controller = std::make_shared<arm_controller::ArmControllerNode>(node);
    (void)controller;
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(node->get_logger(), "Fatal error: %s", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
