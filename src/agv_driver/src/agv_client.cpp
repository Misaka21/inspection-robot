#include "agv_driver/agv_client.hpp"

#include <cmath>
#include <optional>
#include <regex>
#include <string>

namespace agv_driver
{
namespace
{

bool nearly_zero(const double value, const double epsilon = 1e-4)
{
  return std::abs(value) <= epsilon;
}

std::optional<int> json_get_int(const std::string & json, const std::string & key)
{
  const std::regex re("\\\"" + key + "\\\"\\s*:\\s*(-?[0-9]+)");
  std::smatch match;
  if (!std::regex_search(json, match, re) || match.size() < 2U) {
    return std::nullopt;
  }

  try {
    return std::stoi(match[1].str());
  } catch (...) {
    return std::nullopt;
  }
}

std::optional<double> json_get_double(const std::string & json, const std::string & key)
{
  const std::regex re("\\\"" + key + "\\\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?)");
  std::smatch match;
  if (!std::regex_search(json, match, re) || match.size() < 2U) {
    return std::nullopt;
  }

  try {
    return std::stod(match[1].str());
  } catch (...) {
    return std::nullopt;
  }
}

std::optional<bool> json_get_bool(const std::string & json, const std::string & key)
{
  const std::regex re("\\\"" + key + "\\\"\\s*:\\s*(true|false)");
  std::smatch match;
  if (!std::regex_search(json, match, re) || match.size() < 2U) {
    return std::nullopt;
  }
  return match[1].str() == "true";
}

std::string json_escape_string(const std::string & raw)
{
  std::string escaped;
  escaped.reserve(raw.size());
  for (const char ch : raw) {
    if (ch == '\\' || ch == '\"') {
      escaped.push_back('\\');
    }
    escaped.push_back(ch);
  }
  return escaped;
}

}  // namespace

AgvClient::AgvClient(std::string agv_ip, const uint8_t protocol_version, const int timeout_ms)
: _transport(std::move(agv_ip), protocol_version, timeout_ms)
{
}

bool AgvClient::lock_control(const std::string & nick_name, std::string * error)
{
  std::string response;
  const std::string payload =
    "{\"nick_name\":\"" + json_escape_string(nick_name) + "\"}";
  return request(4005U, payload, &response, error);
}

bool AgvClient::unlock_control(std::string * error)
{
  std::string response;
  return request(4006U, "", &response, error);
}

bool AgvClient::query_current_lock(
  bool * locked,
  std::string * owner_nick_name,
  std::string * error)
{
  std::string response;
  if (!request(1060U, "", &response, error)) {
    return false;
  }

  if (locked != nullptr) {
    *locked = json_get_bool(response, "locked").value_or(false);
  }

  if (owner_nick_name != nullptr) {
    const std::regex re("\\\"nick_name\\\"\\s*:\\s*\\\"([^\\\"]*)\\\"");
    std::smatch match;
    if (std::regex_search(response, match, re) && match.size() >= 2U) {
      *owner_nick_name = match[1].str();
    } else {
      owner_nick_name->clear();
    }
  }

  return true;
}

bool AgvClient::load_map(const std::string & map_name, std::string * error)
{
  std::string response;
  const std::string payload =
    "{\"map_name\":\"" + json_escape_string(map_name) + "\"}";
  return request(2022U, payload, &response, error);
}

bool AgvClient::query_loadmap_status(int * loadmap_status, std::string * error)
{
  std::string response;
  if (!request(1022U, "", &response, error)) {
    return false;
  }

  const auto status = json_get_int(response, "loadmap_status");
  if (!status.has_value()) {
    if (error != nullptr) {
      *error = "1022 response missing loadmap_status";
    }
    return false;
  }

  if (loadmap_status != nullptr) {
    *loadmap_status = status.value();
  }

  return true;
}

bool AgvClient::start_reloc_auto(std::string * error)
{
  std::string response;
  return request(2002U, "{\"isAuto\":true}", &response, error);
}

bool AgvClient::cancel_reloc(std::string * error)
{
  std::string response;
  return request(2004U, "", &response, error);
}

bool AgvClient::confirm_loc(std::string * error)
{
  std::string response;
  return request(2003U, "", &response, error);
}

bool AgvClient::query_reloc_status(int * reloc_status, std::string * error)
{
  std::string response;
  if (!request(1021U, "", &response, error)) {
    return false;
  }

  const auto status = json_get_int(response, "reloc_status");
  if (!status.has_value()) {
    if (error != nullptr) {
      *error = "1021 response missing reloc_status";
    }
    return false;
  }

  if (reloc_status != nullptr) {
    *reloc_status = status.value();
  }

  return true;
}

bool AgvClient::send_goal(const double x, const double y, const double yaw, std::string * error)
{
  std::string response;
  const std::string payload =
    "{\"freeGo\":{\"x\":" + std::to_string(x) +
    ",\"y\":" + std::to_string(y) +
    ",\"theta\":" + std::to_string(yaw) +
    "},\"id\":\"SELF_POSITION\"}";

  return request(3051U, payload, &response, error);
}

bool AgvClient::send_open_loop_motion(
  const double vx,
  const double vy,
  const double w,
  const int duration_ms,
  std::string * error)
{
  std::string response;
  const std::string payload =
    "{\"vx\":" + std::to_string(vx) +
    ",\"vy\":" + std::to_string(vy) +
    ",\"w\":" + std::to_string(w) +
    ",\"duration\":" + std::to_string(duration_ms) + "}";

  return request(2010U, payload, &response, error);
}

bool AgvClient::stop_open_loop_motion(std::string * error)
{
  std::string response;
  return request(2000U, "", &response, error);
}

bool AgvClient::poll_state(AgvPollState * state, std::string * error)
{
  if (state == nullptr) {
    if (error != nullptr) {
      *error = "state is null";
    }
    return false;
  }

  AgvPollState next;
  bool ok = true;
  std::string first_error;
  std::string request_error;

  if (!query_position(&next, &request_error)) {
    ok = false;
    first_error = request_error;
  }
  if (!query_speed(&next, &request_error)) {
    ok = false;
    if (first_error.empty()) {
      first_error = request_error;
    }
  }
  if (!query_nav_status(&next, &request_error)) {
    ok = false;
    if (first_error.empty()) {
      first_error = request_error;
    }
  }
  if (!query_battery(&next, &request_error)) {
    ok = false;
    if (first_error.empty()) {
      first_error = request_error;
    }
  }
  if (!query_blocked(&next, &request_error)) {
    ok = false;
    if (first_error.empty()) {
      first_error = request_error;
    }
  }
  if (!query_emergency(&next, &request_error)) {
    ok = false;
    if (first_error.empty()) {
      first_error = request_error;
    }
  }
  if (!query_alarm(&next, &request_error)) {
    ok = false;
    if (first_error.empty()) {
      first_error = request_error;
    }
  }

  next.connected = ok;
  *state = next;

  if (!ok && error != nullptr) {
    *error = first_error.empty() ? "poll failed" : first_error;
  }
  return ok;
}

bool AgvClient::query_position(AgvPollState * state, std::string * error)
{
  std::string response;
  if (!request(1004U, "", &response, error)) {
    return false;
  }

  const auto x = json_get_double(response, "x");
  const auto y = json_get_double(response, "y");
  const auto yaw = json_get_double(response, "angle");

  if (!x.has_value() || !y.has_value() || !yaw.has_value()) {
    if (error != nullptr) {
      *error = "1004 response missing x/y/angle";
    }
    return false;
  }

  state->has_pose = true;
  state->x = x.value();
  state->y = y.value();
  state->yaw = yaw.value();
  return true;
}

bool AgvClient::query_speed(AgvPollState * state, std::string * error)
{
  std::string response;
  if (!request(1005U, "", &response, error)) {
    return false;
  }

  state->has_speed = true;
  state->vx = json_get_double(response, "vx").value_or(0.0);
  state->vy = json_get_double(response, "vy").value_or(0.0);
  state->w = json_get_double(response, "w").value_or(0.0);

  const auto is_stop = json_get_bool(response, "is_stop");
  if (is_stop.has_value()) {
    state->is_stop = is_stop.value();
  } else {
    state->is_stop =
      nearly_zero(state->vx, 1e-3) &&
      nearly_zero(state->vy, 1e-3) &&
      nearly_zero(state->w, 1e-3);
  }

  return true;
}

bool AgvClient::query_nav_status(AgvPollState * state, std::string * error)
{
  std::string response;
  if (!request(1020U, "{\"simple\":true}", &response, error)) {
    return false;
  }

  state->has_nav = true;
  state->task_status = json_get_int(response, "task_status").value_or(-1);
  return true;
}

bool AgvClient::query_battery(AgvPollState * state, std::string * error)
{
  std::string response;
  if (!request(1007U, "{\"simple\":true}", &response, error)) {
    return false;
  }

  state->has_battery = true;
  state->battery_level = json_get_double(response, "battery_level").value_or(-1.0);
  return true;
}

bool AgvClient::query_blocked(AgvPollState * state, std::string * error)
{
  std::string response;
  if (!request(1006U, "", &response, error)) {
    return false;
  }

  state->blocked = json_get_bool(response, "blocked").value_or(false);
  return true;
}

bool AgvClient::query_emergency(AgvPollState * state, std::string * error)
{
  std::string response;
  if (!request(1012U, "", &response, error)) {
    return false;
  }

  const bool emergency = json_get_bool(response, "emergency").value_or(false);
  const bool soft_emc = json_get_bool(response, "soft_emc").value_or(false);
  state->emergency = emergency || soft_emc;
  return true;
}

bool AgvClient::query_alarm(AgvPollState * state, std::string * error)
{
  std::string response;
  if (!request(1050U, "", &response, error)) {
    return false;
  }

  const bool has_fatal = response.find("\"fatals\":[{") != std::string::npos;
  const bool has_error = response.find("\"errors\":[{") != std::string::npos;
  const bool has_warning = response.find("\"warnings\":[{") != std::string::npos;

  if (has_fatal) {
    state->alarm_level = "FATAL";
  } else if (has_error) {
    state->alarm_level = "ERROR";
  } else if (has_warning) {
    state->alarm_level = "WARNING";
  } else {
    state->alarm_level.clear();
  }

  return true;
}

bool AgvClient::request(
  const uint16_t cmd_type,
  const std::string & json_payload,
  std::string * response,
  std::string * error)
{
  return _transport.request(cmd_type, json_payload, response, error);
}

}  // namespace agv_driver
