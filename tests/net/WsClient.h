#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace tactics::testing {

// Minimal blocking-handshake, polled RFC 6455 text client for 127.0.0.1
// (test-only: the shipped native app never connects). Sends masked frames,
// answers pings, and surfaces complete text messages.
class WsClient {
 public:
  WsClient() = default;
  ~WsClient() { Close(); }
  WsClient(const WsClient&) = delete;
  WsClient& operator=(const WsClient&) = delete;

  bool Connect(uint16_t port, std::string* error);
  bool Send(const std::string& text);
  // Waits up to timeoutMs for traffic and appends complete text messages to
  // *out. Returns false once the connection is closed.
  bool Poll(int timeoutMs, std::vector<std::string>* out);
  void Close();
  bool open() const { return fd_ >= 0 && !closed_; }

 private:
  bool SendRaw(const std::string& bytes);
  bool SendFrame(uint8_t opcode, const std::string& payload);
  void Parse(std::vector<std::string>* out);

  int fd_ = -1;
  bool closed_ = false;
  std::string inbuf_;
};

}  // namespace tactics::testing
