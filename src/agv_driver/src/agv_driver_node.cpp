#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include "agv_driver/agv_client.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "inspection_interface/msg/agv_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace agv_driver
{
namespace
{

// AGV 导航任务状态：task_status=4 表示已到达目标点（vendor 协议约定值）
constexpr int TASK_STATUS_ARRIVED = 4;
// 地图加载状态：0=失败, 1=成功, 2=加载中
constexpr int LOADMAP_STATUS_FAILED = 0;
constexpr int LOADMAP_STATUS_SUCCESS = 1;
constexpr int LOADMAP_STATUS_LOADING = 2;

// 重定位状态：0=初始化, 1=成功, 2=定位中, 3=完成(旧版本兼容)
constexpr int RELOC_STATUS_INIT = 0;
constexpr int RELOC_STATUS_SUCCESS = 1;
constexpr int RELOC_STATUS_RELOCING = 2;
constexpr int RELOC_STATUS_COMPLETED_LEGACY = 3;

bool nearly_zero(const double value, const double epsilon = 1e-4)
{
  return std::abs(value) <= epsilon;
}

// 将偏航角（弧度）转为四元数（只绕 Z 轴旋转，2D 平面移动）
geometry_msgs::msg::Quaternion quaternion_from_yaw(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  const double half = yaw * 0.5;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(half);
  q.w = std::cos(half);
  return q;
}

// 从四元数提取偏航角（atan2 公式，仅适用于平面旋转）
double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace

class AgvDriverNode : public rclcpp::Node
{
public:
  explicit AgvDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("agv_driver_node", options)
  {
    _agv_ip = declare_parameter<std::string>("agv_ip", "127.0.0.1");
    _protocol_version = static_cast<uint8_t>(declare_parameter<int>("protocol_version", 1));
    _request_timeout_ms = declare_parameter<int>("request_timeout_ms", 1000);
    _poll_interval_ms = declare_parameter<int>("poll_interval_ms", 200);
    _cmd_vel_duration_ms = declare_parameter<int>("cmd_vel_duration_ms", 200);
    _publish_tf = declare_parameter<bool>("publish_tf", true);
    _stop_on_zero_cmd_vel = declare_parameter<bool>("stop_on_zero_cmd_vel", true);
    _map_frame_id = declare_parameter<std::string>("map_frame_id", "map");
    _base_frame_id = declare_parameter<std::string>("base_frame_id", "base_link");

    _enable_bootstrap = declare_parameter<bool>("enable_bootstrap", true);
    _enable_control_lock = declare_parameter<bool>("enable_control_lock", false);
    _control_nick_name = declare_parameter<std::string>("control_nick_name", "inspection_agv_driver");
    _initial_map_name = declare_parameter<std::string>("initial_map_name", "");
    _auto_relocate = declare_parameter<bool>("auto_relocate", true);
    _skip_reloc_if_localized = declare_parameter<bool>("skip_reloc_if_localized", true);
    _require_confirm_loc = declare_parameter<bool>("require_confirm_loc", false);
    _bootstrap_timeout_ms = declare_parameter<int>("bootstrap_timeout_ms", 60000);
    _bootstrap_poll_interval_ms = declare_parameter<int>("bootstrap_poll_interval_ms", 500);
    _bootstrap_retry_interval_ms = declare_parameter<int>("bootstrap_retry_interval_ms", 5000);

    const bool log_io = declare_parameter<bool>("log_io", false);
    const int log_io_max_chars = declare_parameter<int>("log_io_max_chars", 2048);

    _agv_client = std::make_unique<AgvClient>(_agv_ip, _protocol_version, _request_timeout_ms);
    _agv_client->set_log_io(log_io, static_cast<size_t>(std::max(0, log_io_max_chars)));

    // Public ROS API: avoid node-private (~/) names so other nodes don't depend on node name.
    _goal_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        on_goal_pose(msg);
      });

    _cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        on_cmd_vel(msg);
      });

    _current_pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", 10);
    _odom_pub = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    _status_pub = create_publisher<inspection_interface::msg::AgvStatus>("status", 10);

    if (_publish_tf) {
      _tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    _poll_timer = create_wall_timer(
      std::chrono::milliseconds(_poll_interval_ms),
      [this]() {
        on_poll_timer();
      });

    RCLCPP_INFO(
      get_logger(),
      "agv_driver_node started (ip=%s, timeout=%dms, poll=%dms)",
      _agv_ip.c_str(),
      _request_timeout_ms,
      _poll_interval_ms);

    try_bootstrap();
  }

  ~AgvDriverNode() override
  {
    if (_enable_control_lock && _agv_client != nullptr) {
      std::string error;
      if (!_agv_client->unlock_control(&error)) {
        RCLCPP_WARN(get_logger(), "unlock control failed on shutdown: %s", error.c_str());
      }
    }
  }

private:
  void on_goal_pose(const geometry_msgs::msg::PoseStamped::SharedPtr & msg)
  {
    // Bootstrap 未完成时丢弃目标（AGV 尚未定位成功，贸然导航会出错）
    if (_enable_bootstrap && !_bootstrap_ready) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "drop goal because bootstrap is not ready");
      return;
    }

    if (!msg->header.frame_id.empty() && msg->header.frame_id != _map_frame_id) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "drop goal because frame_id mismatch (got=%s expected=%s)",
        msg->header.frame_id.c_str(),
        _map_frame_id.c_str());
      return;
    }

    const double yaw = yaw_from_quaternion(msg->pose.orientation);

    std::string error;
    if (!_agv_client->send_goal(msg->pose.position.x, msg->pose.position.y, yaw, &error)) {
      _last_error = error;
      RCLCPP_WARN(
        get_logger(),
        "send goal failed: %s (x=%.3f y=%.3f yaw=%.3f)",
        error.c_str(),
        msg->pose.position.x,
        msg->pose.position.y,
        yaw);
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "goal sent: x=%.3f y=%.3f yaw=%.3f",
      msg->pose.position.x,
      msg->pose.position.y,
      yaw);
  }

  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr & msg)
  {
    // 判断是否是零速指令（线速度和角速度都接近零）
    const bool zero_cmd =
      nearly_zero(msg->linear.x) &&
      nearly_zero(msg->linear.y) &&
      nearly_zero(msg->angular.z);

    std::string error;
    bool ok = false;

    // 零速 + stop_on_zero_cmd_vel=true：发送停止指令（更安全，避免 AGV 继续惯性运动）
    if (zero_cmd && _stop_on_zero_cmd_vel) {
      ok = _agv_client->stop_open_loop_motion(&error);
    } else {
      ok = _agv_client->send_open_loop_motion(
        msg->linear.x,
        msg->linear.y,
        msg->angular.z,
        _cmd_vel_duration_ms,
        &error);
    }

    if (!ok) {
      _last_error = error;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "cmd_vel request failed: %s",
        error.c_str());
    }
  }

  // 定时轮询（默认 200ms）：读取 AGV 最新状态并发布到 ROS。
  // 若 bootstrap 未完成则先推进 bootstrap；轮询失败不清空上一帧状态（避免短暂断连导致数据抖动）。
  void on_poll_timer()
  {
    if (_enable_bootstrap && !_bootstrap_ready) {
      try_bootstrap();
    }

    AgvPollState current_state;
    std::string error;

    const bool ok = _agv_client->poll_state(&current_state, &error);
    if (!ok) {
      _last_error = error;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "poll state failed: %s",
        error.c_str());
    }

    if (ok || !_last_state.connected) {
      _last_state = current_state;
    } else {
      _last_state.connected = false;
    }

    publish_outputs(_last_state);
  }

  // Bootstrap 推进入口：带重试间隔（_bootstrap_retry_interval_ms）防止连续重试打爆 TCP
  void try_bootstrap()
  {
    if (!_enable_bootstrap || _bootstrap_ready) {
      _bootstrap_ready = true;
      return;
    }

    const auto now_time = now();
    if (_has_bootstrap_try) {
      const auto elapsed_ns = (now_time - _last_bootstrap_try_time).nanoseconds();
      const auto retry_ns = static_cast<int64_t>(_bootstrap_retry_interval_ms) * 1000000LL;
      if (elapsed_ns < retry_ns) {
        return;
      }
    }

    _has_bootstrap_try = true;
    _last_bootstrap_try_time = now_time;

    std::string error;
    if (bootstrap_once(&error)) {
      _bootstrap_ready = true;
      _bootstrap_error.clear();
      RCLCPP_INFO(get_logger(), "bootstrap finished");
      return;
    }

    _bootstrap_ready = false;
    _bootstrap_error = error;
    RCLCPP_WARN(get_logger(), "bootstrap failed: %s", error.c_str());
  }

  // 执行一次完整 Bootstrap 流程：控制锁 -> 地图加载 -> 重定位。
  // 返回 false 表示任一步骤失败，下次 try_bootstrap 会重试。
  bool bootstrap_once(std::string * error)
  {
    if (_agv_client == nullptr) {
      if (error != nullptr) {
        *error = "agv client is null";
      }
      return false;
    }

    if (_enable_control_lock) {
      bool locked = false;
      std::string owner;
      std::string lock_error;
      if (!_agv_client->query_current_lock(&locked, &owner, &lock_error)) {
        if (error != nullptr) {
          *error = "query lock failed: " + lock_error;
        }
        return false;
      }

      if (!locked || owner != _control_nick_name) {
        if (!_agv_client->lock_control(_control_nick_name, &lock_error)) {
          if (error != nullptr) {
            *error = "lock control failed: " + lock_error;
          }
          return false;
        }
      }
    }

    if (!_initial_map_name.empty()) {
      std::string map_error;
      if (!_agv_client->load_map(_initial_map_name, &map_error)) {
        if (error != nullptr) {
          *error = "load map failed: " + map_error;
        }
        return false;
      }
    }

    if (!wait_loadmap_ready(error)) {
      return false;
    }

    if (!_auto_relocate) {
      return true;
    }

    int reloc_status = RELOC_STATUS_INIT;
    if (!_agv_client->query_reloc_status(&reloc_status, error)) {
      return false;
    }

    if (_skip_reloc_if_localized && reloc_status == RELOC_STATUS_SUCCESS) {
      return true;
    }

    std::string reloc_error;
    if (!_agv_client->start_reloc_auto(&reloc_error)) {
      if (error != nullptr) {
        *error = "start reloc failed: " + reloc_error;
      }
      return false;
    }

    if (!wait_reloc_ready(error)) {
      return false;
    }

    return true;
  }

  // 轮询等待地图加载完成，有超时保护（_bootstrap_timeout_ms）
  bool wait_loadmap_ready(std::string * error)
  {
    const auto deadline = now() + rclcpp::Duration::from_seconds(_bootstrap_timeout_ms / 1000.0);

    while (rclcpp::ok()) {
      int status = LOADMAP_STATUS_LOADING;
      std::string status_error;
      if (!_agv_client->query_loadmap_status(&status, &status_error)) {
        if (error != nullptr) {
          *error = "query loadmap status failed: " + status_error;
        }
        return false;
      }

      if (status == LOADMAP_STATUS_SUCCESS) {
        return true;
      }

      if (status == LOADMAP_STATUS_FAILED) {
        if (error != nullptr) {
          *error = "loadmap status failed";
        }
        return false;
      }

      if (now() > deadline) {
        if (error != nullptr) {
          *error = "wait loadmap status timeout";
        }
        return false;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(_bootstrap_poll_interval_ms));
    }

    if (error != nullptr) {
      *error = "wait loadmap interrupted";
    }
    return false;
  }

  // 轮询等待重定位完成，支持旧版本的 COMPLETED_LEGACY 状态处理。
  // _require_confirm_loc=true 时，需要额外调用 confirm_loc 确认定位结果。
  bool wait_reloc_ready(std::string * error)
  {
    const auto deadline = now() + rclcpp::Duration::from_seconds(_bootstrap_timeout_ms / 1000.0);

    while (rclcpp::ok()) {
      int status = RELOC_STATUS_INIT;
      std::string status_error;
      if (!_agv_client->query_reloc_status(&status, &status_error)) {
        if (error != nullptr) {
          *error = "query reloc status failed: " + status_error;
        }
        return false;
      }

      if (status == RELOC_STATUS_SUCCESS) {
        return true;
      }

      if (status == RELOC_STATUS_COMPLETED_LEGACY) {
        if (_require_confirm_loc) {
          std::string confirm_error;
          if (!_agv_client->confirm_loc(&confirm_error)) {
            if (error != nullptr) {
              *error = "confirm reloc failed: " + confirm_error;
            }
            return false;
          }
        } else {
          return true;
        }
      }

      if (status != RELOC_STATUS_INIT && status != RELOC_STATUS_RELOCING &&
        status != RELOC_STATUS_COMPLETED_LEGACY)
      {
        if (error != nullptr) {
          *error = "unexpected reloc status=" + std::to_string(status);
        }
        return false;
      }

      if (now() > deadline) {
        if (error != nullptr) {
          *error = "wait reloc status timeout";
        }
        return false;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(_bootstrap_poll_interval_ms));
    }

    if (error != nullptr) {
      *error = "wait reloc interrupted";
    }
    return false;
  }

  // 将 AgvPollState 转换并发布到 ROS：
  //   - current_pose (PoseStamped)：仅 has_pose=true 时发
  //   - odom (Odometry)：包含速度信息
  //   - TF：map -> base_link（可关闭，让其他节点接管 TF）
  //   - status (AgvStatus)：始终发布，connected=false 时其他字段可能无效
  void publish_outputs(const AgvPollState & state)
  {
    const auto stamp = now();

    geometry_msgs::msg::Pose pose;
    pose.position.x = state.x;
    pose.position.y = state.y;
    pose.position.z = 0.0;
    pose.orientation = quaternion_from_yaw(state.yaw);

    if (state.has_pose) {
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.stamp = stamp;
      pose_msg.header.frame_id = _map_frame_id;
      pose_msg.pose = pose;
      _current_pose_pub->publish(pose_msg);

      nav_msgs::msg::Odometry odom_msg;
      odom_msg.header = pose_msg.header;
      odom_msg.child_frame_id = _base_frame_id;
      odom_msg.pose.pose = pose_msg.pose;
      odom_msg.twist.twist.linear.x = state.vx;
      odom_msg.twist.twist.linear.y = state.vy;
      odom_msg.twist.twist.angular.z = state.w;
      _odom_pub->publish(odom_msg);

      if (_publish_tf && _tf_broadcaster != nullptr) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header = pose_msg.header;
        transform.child_frame_id = _base_frame_id;
        transform.transform.translation.x = state.x;
        transform.transform.translation.y = state.y;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation = pose_msg.pose.orientation;
        _tf_broadcaster->sendTransform(transform);
      }
    }

    inspection_interface::msg::AgvStatus status;
    status.connected = state.connected;
    status.arrived = is_arrived(state);
    status.stopped = is_stopped(state);
    status.moving = state.connected && !status.stopped;
    status.current_pose = pose;
    if (state.has_speed) {
      status.linear_speed = static_cast<float>(std::hypot(state.vx, state.vy));
      status.angular_speed = static_cast<float>(state.w);
    } else {
      status.linear_speed = 0.0F;
      status.angular_speed = 0.0F;
    }
    status.frame_id = _map_frame_id;
    status.battery_percent = to_battery_percent(state.battery_level);
    status.error_code = map_error_code(state);

    _status_pub->publish(status);
  }

  // AGV 到位判断：导航任务状态必须是 ARRIVED（task_status=4）
  bool is_arrived(const AgvPollState & state) const
  {
    return state.has_nav && state.task_status == TASK_STATUS_ARRIVED;
  }

  bool is_stopped(const AgvPollState & state) const
  {
    return !state.has_speed || state.is_stop;
  }

  // 电量归一化：厂商可能返回 0~1（比例）或 0~100（百分比），统一转成百分比
  // 返回 -1 表示数据无效（查询失败）
  float to_battery_percent(const double battery_level) const
  {
    if (battery_level < 0.0) {
      return -1.0F;
    }
    if (battery_level <= 1.0) {
      return static_cast<float>(battery_level * 100.0);
    }
    return static_cast<float>(std::min(100.0, battery_level));
  }

  // 错误码优先级：BOOTSTRAP_PENDING > DISCONNECTED > EMERGENCY > BLOCKED > ALARM > 最近错误 > OK
  std::string map_error_code(const AgvPollState & state) const
  {
    if (_enable_bootstrap && !_bootstrap_ready) {
      if (_bootstrap_error.empty()) {
        return "BOOTSTRAP_PENDING";
      }
      return "BOOTSTRAP_FAILED";
    }
    if (!state.connected) {
      return "DISCONNECTED";
    }
    if (state.emergency) {
      return "EMERGENCY";
    }
    if (state.blocked) {
      return "BLOCKED";
    }
    if (!state.alarm_level.empty()) {
      return state.alarm_level;
    }
    if (!_last_error.empty()) {
      return _last_error;
    }
    return "OK";
  }

  std::string _agv_ip;
  uint8_t _protocol_version = 1U;
  int _request_timeout_ms = 1000;
  int _poll_interval_ms = 200;
  int _cmd_vel_duration_ms = 200;
  bool _publish_tf = true;
  bool _stop_on_zero_cmd_vel = true;
  std::string _map_frame_id = "map";
  std::string _base_frame_id = "base_link";

  bool _enable_bootstrap = true;
  bool _enable_control_lock = false;
  std::string _control_nick_name = "inspection_agv_driver";
  std::string _initial_map_name;
  bool _auto_relocate = true;
  bool _skip_reloc_if_localized = true;
  bool _require_confirm_loc = false;
  int _bootstrap_timeout_ms = 60000;
  int _bootstrap_poll_interval_ms = 500;
  int _bootstrap_retry_interval_ms = 5000;

  bool _bootstrap_ready = false;
  bool _has_bootstrap_try = false;
  rclcpp::Time _last_bootstrap_try_time;
  std::string _bootstrap_error;

  std::unique_ptr<AgvClient> _agv_client;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _goal_sub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_sub;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _current_pose_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odom_pub;
  rclcpp::Publisher<inspection_interface::msg::AgvStatus>::SharedPtr _status_pub;

  std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
  rclcpp::TimerBase::SharedPtr _poll_timer;

  AgvPollState _last_state;
  std::string _last_error;
};

}  // namespace agv_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<agv_driver::AgvDriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
