#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

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

constexpr int TASK_STATUS_ARRIVED = 4;

bool nearly_zero(const double value, const double epsilon = 1e-4)
{
  return std::abs(value) <= epsilon;
}

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

    _agv_client = std::make_unique<AgvClient>(_agv_ip, _protocol_version, _request_timeout_ms);

    _goal_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/goal_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        on_goal_pose(msg);
      });

    _cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>(
      "~/cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        on_cmd_vel(msg);
      });

    _current_pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>("~/current_pose", 10);
    _odom_pub = create_publisher<nav_msgs::msg::Odometry>("~/odom", 10);
    _status_pub = create_publisher<inspection_interface::msg::AgvStatus>("~/status", 10);

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
  }

private:
  void on_goal_pose(const geometry_msgs::msg::PoseStamped::SharedPtr & msg)
  {
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
    const bool zero_cmd =
      nearly_zero(msg->linear.x) &&
      nearly_zero(msg->linear.y) &&
      nearly_zero(msg->angular.z);

    std::string error;
    bool ok = false;

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

  void on_poll_timer()
  {
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
    status.battery_percent = to_battery_percent(state.battery_level);
    status.error_code = map_error_code(state);

    _status_pub->publish(status);
  }

  bool is_arrived(const AgvPollState & state) const
  {
    return state.has_nav && state.task_status == TASK_STATUS_ARRIVED;
  }

  bool is_stopped(const AgvPollState & state) const
  {
    return !state.has_speed || state.is_stop;
  }

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

  std::string map_error_code(const AgvPollState & state) const
  {
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
