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
    uint8_t sync = 0x5A;
    uint8_t version = 0x01;
    uint16_t seq = 0U;
    uint32_t length = 0U;
    uint16_t type = 0U;
    uint8_t reserved[6] = {0, 0, 0, 0, 0, 0};
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

  std::mutex _socket_mutex;
  std::unordered_map<uint16_t, int> _socket_by_port;
  uint16_t _seq = 1U;

  bool _log_io = false;
  size_t _log_io_max_chars = 0U;
};

}  // namespace agv_driver
