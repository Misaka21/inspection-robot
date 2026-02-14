#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace agv_driver
{

class AgvTransport
{
public:
  AgvTransport(std::string agv_ip, uint8_t protocol_version, int timeout_ms);
  ~AgvTransport();

  void set_log_io(bool enabled, size_t max_chars);

  bool request(
    uint16_t cmd_type,
    const std::string & json_payload,
    std::string * response,
    std::string * error);

  void close_all();

private:
  struct ProtocolHeader
  {
    uint8_t sync = 0x5A;    // 同步字节 0x5A，用于接收端定位报文头部起始位置
    uint8_t version = 0x01; // 协议版本号，当前固定为 0x01
    uint16_t seq = 0U;      // 请求序列号，单调递增，用于匹配请求与响应（防止错序）
    uint32_t length = 0U;   // JSON payload 的字节长度（不含协议头本身）
    uint16_t type = 0U;     // 命令类型号（cmd_type），决定路由到哪个 TCP 端口
    uint8_t reserved[6] = {0, 0, 0, 0, 0, 0}; // 保留字段，填零对齐至 16 字节头
  };

  static std::optional<uint16_t> resolve_port(uint16_t cmd_type);
  static std::vector<uint8_t> encode_request_packet(
    uint8_t version,
    uint16_t seq,
    uint16_t cmd_type,
    const std::string & json_payload);
  static bool decode_protocol_header(const uint8_t * raw, size_t size, ProtocolHeader * header);

  bool get_or_connect_socket_locked(uint16_t port, int * socket_fd, std::string * error);
  bool connect_socket(uint16_t port, int * socket_fd, std::string * error) const;
  bool send_all(int socket_fd, const uint8_t * data, size_t size, std::string * error) const;
  bool recv_all(int socket_fd, uint8_t * data, size_t size, std::string * error) const;

  void close_socket_locked(uint16_t port);

  std::string _agv_ip;
  uint8_t _protocol_version = 1U;
  int _timeout_ms = 1000;

  // _socket_mutex：保护 _socket_by_port 和 _seq 的并发访问
  // AgvClient 可能从 ROS 定时器（poll）和 ROS 服务回调同时调用 request()，必须互斥
  std::mutex _socket_mutex;
  // _socket_by_port：port -> 已建立的 TCP socket fd 映射，实现 TCP 长连接复用
  // 不同 cmd_type 路由到不同 port，每个 port 只建立一次连接，失败时重连
  std::unordered_map<uint16_t, int> _socket_by_port;
  // _seq：请求序列号，每次 request() 调用自增，用于 AGV 控制器匹配响应包
  uint16_t _seq = 1U;

  bool _log_io = false;
  size_t _log_io_max_chars = 0U;
};

}  // namespace agv_driver
