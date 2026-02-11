#pragma once

#include <cstdint>
#include <string>

#include "agv_driver/agv_transport.hpp"

namespace agv_driver
{

struct AgvPollState
{
  bool connected = false;

  bool has_pose = false;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;

  bool has_speed = false;
  double vx = 0.0;
  double vy = 0.0;
  double w = 0.0;
  bool is_stop = true;

  bool has_nav = false;
  int task_status = -1;

  bool has_battery = false;
  double battery_level = -1.0;

  bool blocked = false;
  bool emergency = false;
  std::string alarm_level;
};

class AgvClient
{
public:
  AgvClient(std::string agv_ip, uint8_t protocol_version, int timeout_ms);

  bool send_goal(double x, double y, double yaw, std::string * error);
  bool send_open_loop_motion(double vx, double vy, double w, int duration_ms, std::string * error);
  bool stop_open_loop_motion(std::string * error);

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
