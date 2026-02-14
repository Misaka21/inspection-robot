#include "agv_driver/agv_transport.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cerrno>
#include <cstring>
#include <optional>
#include <regex>
#include <string>

#include <rclcpp/logging.hpp>

namespace agv_driver
{
namespace
{

// 协议同步字节，每帧的第一个字节必须是 0x5A，否则视为非法帧
constexpr uint8_t PROTOCOL_SYNC = 0x5A;
// 固定 16 字节协议头大小
constexpr size_t PROTOCOL_HEADER_SIZE = 16U;

// 日志截断辅助：防止超长 JSON 刷爆日志，只保留前 max_chars 个字符
std::string trim_for_log(const std::string & input, const size_t max_chars)
{
  if (max_chars == 0U || input.size() <= max_chars) {
    return input;
  }
  return input.substr(0, max_chars) + "...(truncated, total=" + std::to_string(input.size()) + ")";
}

// 用正则从 JSON 字符串里提取整型字段（避免引入重量级 JSON 库）
// 示例：json_get_int(`{"ret_code":0}`, "ret_code") -> 0
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

}  // namespace

AgvTransport::AgvTransport(std::string agv_ip, uint8_t protocol_version, int timeout_ms)
: _agv_ip(std::move(agv_ip)), _protocol_version(protocol_version), _timeout_ms(timeout_ms)
{
}

AgvTransport::~AgvTransport()
{
  close_all();
}

void AgvTransport::set_log_io(const bool enabled, const size_t max_chars)
{
  std::lock_guard<std::mutex> lock(_socket_mutex);
  _log_io = enabled;
  _log_io_max_chars = max_chars;
}

// 核心入口：发送一条 AGV 请求并等待响应。
// 流程：cmd_type -> 查端口 -> 获取/建立 TCP 连接 -> 编包发送 -> 接收16字节头 -> 接收载荷 -> 检查 ret_code
// 线程安全：内部用 _socket_mutex 保护 socket 状态
bool AgvTransport::request(
  const uint16_t cmd_type,
  const std::string & json_payload,
  std::string * response,
  std::string * error)
{
  const auto start_time = std::chrono::steady_clock::now();
  // 根据 cmd_type 查出对应的 TCP 端口（不同功能区分不同端口）
  const auto port = resolve_port(cmd_type);
  if (!port.has_value()) {
    if (error != nullptr) {
      *error = "unsupported command type=" + std::to_string(cmd_type);
    }
    return false;
  }

  const auto logger = rclcpp::get_logger("agv_transport");
  std::lock_guard<std::mutex> lock(_socket_mutex);

  int socket_fd = -1;
  // 如果对应端口已有 TCP 连接则复用，否则新建
  if (!get_or_connect_socket_locked(port.value(), &socket_fd, error)) {
    return false;
  }

  // 自增序列号，用于调试追踪请求/响应对
  const uint16_t seq = _seq++;
  const auto packet = encode_request_packet(_protocol_version, seq, cmd_type, json_payload);

  if (_log_io) {
    RCLCPP_INFO(
      logger,
      "AGV >> cmd=%u port=%u seq=%u payload=%s",
      static_cast<unsigned>(cmd_type),
      static_cast<unsigned>(port.value()),
      static_cast<unsigned>(seq),
      trim_for_log(json_payload, _log_io_max_chars).c_str());
  }

  if (!send_all(socket_fd, packet.data(), packet.size(), error)) {
    // 发送失败时关闭 socket，下次请求会重新建连
    close_socket_locked(port.value());
    return false;
  }

  // 先接收固定16字节协议头，获取载荷长度
  uint8_t raw_header[PROTOCOL_HEADER_SIZE] = {0};
  if (!recv_all(socket_fd, raw_header, sizeof(raw_header), error)) {
    close_socket_locked(port.value());
    return false;
  }

  ProtocolHeader header;
  if (!decode_protocol_header(raw_header, sizeof(raw_header), &header)) {
    if (error != nullptr) {
      *error = "invalid protocol header";
    }
    close_socket_locked(port.value());
    return false;
  }

  // 再按头里的 length 字段接收 JSON 载荷
  std::string body;
  body.resize(header.length);
  if (header.length > 0U) {
    if (!recv_all(socket_fd, reinterpret_cast<uint8_t *>(body.data()), body.size(), error)) {
      close_socket_locked(port.value());
      return false;
    }
  }

  if (response != nullptr) {
    *response = body;
  }

  if (_log_io) {
    const auto elapsed_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count();
    const int ret_code_value = json_get_int(body, "ret_code").value_or(-1);
    RCLCPP_INFO(
      logger,
      "AGV << cmd=%u port=%u seq=%u ret_code=%d cost_ms=%ld body=%s",
      static_cast<unsigned>(cmd_type),
      static_cast<unsigned>(port.value()),
      static_cast<unsigned>(header.seq),
      ret_code_value,
      static_cast<long>(elapsed_ms),
      trim_for_log(body, _log_io_max_chars).c_str());
  }

  const auto ret_code = json_get_int(body, "ret_code");
  if (ret_code.has_value() && ret_code.value() != 0) {
    if (error != nullptr) {
      *error = "ret_code=" + std::to_string(ret_code.value()) +
        ", cmd_type=" + std::to_string(cmd_type);
    }
    return false;
  }

  return true;
}

void AgvTransport::close_all()
{
  std::lock_guard<std::mutex> lock(_socket_mutex);
  for (const auto & [port, socket_fd] : _socket_by_port) {
    (void)port;
    if (socket_fd >= 0) {
      (void)::close(socket_fd);
    }
  }
  _socket_by_port.clear();
}

  // cmd_type -> TCP 端口路由表（厂商协议约定，不同功能区使用不同端口）:
  //   9300 / 19301  -> 19301 (特殊心跳/控制锁命令)
  //   1000-1999     -> 19204 (查询类命令: 位置/速度/导航状态/电量/告警等)
  //   2000-2999     -> 19205 (控制类命令: 重定位/开环运动/载图等)
  //   3000-3999     -> 19206 (任务导航命令: 发送目标位姿)
  //   4000-4999     -> 19207 (控制锁命令: lock/unlock)
  //   6000-6999     -> 19210 (地图相关命令)
std::optional<uint16_t> AgvTransport::resolve_port(const uint16_t cmd_type)
{
  if (cmd_type == 9300U || cmd_type == 19301U) {
    return 19301U;
  }
  if (cmd_type >= 1000U && cmd_type <= 1999U) {
    return 19204U;
  }
  if (cmd_type >= 2000U && cmd_type <= 2999U) {
    return 19205U;
  }
  if (cmd_type >= 3000U && cmd_type <= 3999U) {
    return 19206U;
  }
  if (cmd_type >= 4000U && cmd_type <= 4999U) {
    return 19207U;
  }
  if (cmd_type >= 6000U && cmd_type <= 6999U) {
    return 19210U;
  }
  return std::nullopt;
}

// 将请求打包成协议帧。所有多字节字段转换为大端序（网络字节序）。
// 返回：头(16字节) + JSON载荷 的连续字节数组
std::vector<uint8_t> AgvTransport::encode_request_packet(
  const uint8_t version,
  const uint16_t seq,
  const uint16_t cmd_type,
  const std::string & json_payload)
{
  const uint16_t seq_be = htons(seq);
  const uint32_t len_be = htonl(static_cast<uint32_t>(json_payload.size()));
  const uint16_t type_be = htons(cmd_type);

  std::vector<uint8_t> packet(PROTOCOL_HEADER_SIZE + json_payload.size(), 0U);
  packet[0] = PROTOCOL_SYNC;
  packet[1] = version;
  std::memcpy(packet.data() + 2, &seq_be, sizeof(seq_be));
  std::memcpy(packet.data() + 4, &len_be, sizeof(len_be));
  std::memcpy(packet.data() + 8, &type_be, sizeof(type_be));

  if (!json_payload.empty()) {
    std::memcpy(packet.data() + PROTOCOL_HEADER_SIZE, json_payload.data(), json_payload.size());
  }

  return packet;
}

// 解析收到的16字节协议头，填充 ProtocolHeader 结构体。
// 若同步字节不为 0x5A，则视为帧错位/连接异常，返回 false。
bool AgvTransport::decode_protocol_header(const uint8_t * raw, const size_t size, ProtocolHeader * header)
{
  if (raw == nullptr || header == nullptr || size < PROTOCOL_HEADER_SIZE) {
    return false;
  }

  header->sync = raw[0];
  header->version = raw[1];

  if (header->sync != PROTOCOL_SYNC) {
    return false;
  }

  uint16_t seq_be = 0U;
  uint32_t len_be = 0U;
  uint16_t type_be = 0U;

  std::memcpy(&seq_be, raw + 2, sizeof(seq_be));
  std::memcpy(&len_be, raw + 4, sizeof(len_be));
  std::memcpy(&type_be, raw + 8, sizeof(type_be));

  header->seq = ntohs(seq_be);
  header->length = ntohl(len_be);
  header->type = ntohs(type_be);
  std::memcpy(header->reserved, raw + 10, sizeof(header->reserved));

  return true;
}

// 按端口号查找已有 socket；若不存在则新建 TCP 连接。
// 实现"TCP 长连接复用"：每个端口只维护一条持久连接，避免频繁握手开销。
bool AgvTransport::get_or_connect_socket_locked(
  const uint16_t port,
  int * socket_fd,
  std::string * error)
{
  auto it = _socket_by_port.find(port);
  if (it != _socket_by_port.end()) {
    *socket_fd = it->second;
    return true;
  }

  int connected_fd = -1;
  if (!connect_socket(port, &connected_fd, error)) {
    return false;
  }

  _socket_by_port[port] = connected_fd;
  *socket_fd = connected_fd;
  return true;
}

// 创建 TCP socket 并连接到 AGV 指定端口。
// 设置发送/接收超时（SO_RCVTIMEO / SO_SNDTIMEO），防止阻塞无限等待。
bool AgvTransport::connect_socket(const uint16_t port, int * socket_fd, std::string * error) const
{
  int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    if (error != nullptr) {
      *error = "socket create failed";
    }
    return false;
  }

  timeval tv {};
  tv.tv_sec = _timeout_ms / 1000;
  tv.tv_usec = (_timeout_ms % 1000) * 1000;

  (void)::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  (void)::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

  sockaddr_in address {};
  address.sin_family = AF_INET;
  address.sin_port = htons(port);

  if (::inet_pton(AF_INET, _agv_ip.c_str(), &address.sin_addr) != 1) {
    if (error != nullptr) {
      *error = "invalid agv_ip=" + _agv_ip;
    }
    ::close(fd);
    return false;
  }

  if (::connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0) {
    if (error != nullptr) {
      *error = "connect failed to " + _agv_ip + ":" + std::to_string(port) +
        ", errno=" + std::to_string(errno);
    }
    ::close(fd);
    return false;
  }

  *socket_fd = fd;
  return true;
}

// 确保将 size 字节全部发送完毕（TCP 可能分多次发送）
bool AgvTransport::send_all(
  const int socket_fd,
  const uint8_t * data,
  const size_t size,
  std::string * error) const
{
  size_t offset = 0U;
  while (offset < size) {
    const ssize_t sent = ::send(socket_fd, data + offset, size - offset, 0);
    if (sent <= 0) {
      if (error != nullptr) {
        *error = "socket send failed, errno=" + std::to_string(errno);
      }
      return false;
    }
    offset += static_cast<size_t>(sent);
  }
  return true;
}

// 确保将 size 字节全部接收完毕（TCP 可能分多次接收）
bool AgvTransport::recv_all(
  const int socket_fd,
  uint8_t * data,
  const size_t size,
  std::string * error) const
{
  size_t offset = 0U;
  while (offset < size) {
    const ssize_t recved = ::recv(socket_fd, data + offset, size - offset, 0);
    if (recved <= 0) {
      if (error != nullptr) {
        *error = "socket recv failed, errno=" + std::to_string(errno);
      }
      return false;
    }
    offset += static_cast<size_t>(recved);
  }
  return true;
}

void AgvTransport::close_socket_locked(const uint16_t port)
{
  auto it = _socket_by_port.find(port);
  if (it == _socket_by_port.end()) {
    return;
  }

  if (it->second >= 0) {
    (void)::close(it->second);
  }
  _socket_by_port.erase(it);
}

}  // namespace agv_driver
