#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "agv_driver/agv_transport.hpp"

namespace agv_driver
{

struct AgvPollState
{
  bool connected = false;

  bool has_pose = false;    // 是否有有效位姿数据（AGV 尚未定位时为 false）
  double x = 0.0;           // 当前位置 x（地图坐标系，米）
  double y = 0.0;           // 当前位置 y（地图坐标系，米）
  double yaw = 0.0;         // 当前朝向角（弧度，相对地图坐标系）

  bool has_speed = false;
  double vx = 0.0;          // 底盘前向速度（m/s）
  double vy = 0.0;          // 底盘侧向速度（m/s，全向底盘）
  double w = 0.0;           // 底盘旋转角速度（rad/s）
  bool is_stop = true;      // AGV 是否处于静止状态（速度接近零）

  bool has_nav = false;
  int task_status = -1;     // 导航任务状态码（-1=未知，各厂商定义不同，详见协议文档）

  bool has_battery = false;
  double battery_level = -1.0; // 电池电量百分比（0~100），-1 表示未获取到

  bool blocked = false;     // 路径是否被障碍物阻塞（无法继续导航）
  bool emergency = false;   // 是否触发急停（安全门/紧急停止按钮）
  std::string alarm_level;  // 告警等级字符串（如 "warning"/"error"/"none"）
};

class AgvClient
{
public:
  AgvClient(std::string agv_ip, uint8_t protocol_version, int timeout_ms);

  void set_log_io(bool enabled, size_t max_chars);

  // 控制权锁：AGV 控制器支持多客户端连接，但同时只有一个客户端可以发送运动指令
  // lock_control 需要传入当前客户端的昵称（用于 AGV 界面显示当前控制方）
  bool lock_control(const std::string & nick_name, std::string * error);
  bool unlock_control(std::string * error);
  bool query_current_lock(bool * locked, std::string * owner_nick_name, std::string * error);

  // 地图加载：AGV 需要先加载与当前环境匹配的地图才能定位和导航
  bool load_map(const std::string & map_name, std::string * error);
  bool query_loadmap_status(int * loadmap_status, std::string * error);

  // 自动重定位：让 AGV 在已加载地图中搜索自身位置（常用于上电后或迷路后恢复）
  bool start_reloc_auto(std::string * error);
  bool cancel_reloc(std::string * error);
  bool confirm_loc(std::string * error);
  bool query_reloc_status(int * reloc_status, std::string * error);

  // 导航目标：发送地图坐标系下的位姿目标，AGV 自动规划路径行驶
  bool send_goal(double x, double y, double yaw, std::string * error);
  // 开环速度控制：直接发送底盘速度（调试或无地图场景），duration_ms 为持续时间
  bool send_open_loop_motion(double vx, double vy, double w, int duration_ms, std::string * error);
  bool stop_open_loop_motion(std::string * error);

  // poll_state：聚合查询所有状态（位置/速度/导航/电量/告警），返回结构化状态
  // 设计为单次轮询而非订阅，由 Node 层定时调用，便于控制轮询频率与超时处理
  bool poll_state(AgvPollState * state, std::string * error);

private:
  bool query_position(AgvPollState * state, std::string * error);
  bool query_speed(AgvPollState * state, std::string * error);
  bool query_nav_status(AgvPollState * state, std::string * error);
  bool query_battery(AgvPollState * state, std::string * error);
  bool query_blocked(AgvPollState * state, std::string * error);
  bool query_emergency(AgvPollState * state, std::string * error);
  bool query_alarm(AgvPollState * state, std::string * error);

  bool request(
    uint16_t cmd_type,
    const std::string & json_payload,
    std::string * response,
    std::string * error);

  AgvTransport _transport;
};

}  // namespace agv_driver
