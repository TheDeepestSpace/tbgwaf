#pragma once

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <string>

#include "server/Lobby.h"
#include "server/WebSocket.h"

namespace tactics::server {

struct ServerConfig {
  uint16_t port = 8080;       // 0 = pick a free port (see Server::port()).
  std::string bindAddress = "0.0.0.0";
  size_t maxConnections = 128;
  int handshakeTimeoutMs = 5000;
  int idleTimeoutMs = 60000;   // Dropped if silent this long (clients are pinged at half).
  int maxMessagesPerSecond = 60;
};

// Single-threaded poll() WebSocket server feeding a Lobby. Also answers
// `GET /health` (200 "ok") so platforms can health-check it.
class Server {
 public:
  explicit Server(ServerConfig config);
  ~Server();

  // Binds and listens. Returns false (with *error set) on failure.
  bool Listen(std::string* error);
  uint16_t port() const { return port_; }

  // Runs one poll iteration (waits up to timeoutMs). Call in a loop.
  void RunOnce(int timeoutMs);
  // Runs until Stop() (or a signal handler flipping the flag) says so.
  void Run();
  void Stop() { stop_ = true; }

 private:
  struct Conn {
    int fd = -1;
    bool open = false;      // Handshake complete.
    bool closing = false;   // Flush outbuf then drop.
    std::string inbuf, outbuf;
    std::unique_ptr<ws::FrameParser> parser;
    int64_t connectedMs = 0, lastActivityMs = 0, lastPingMs = 0;
    int64_t windowStartMs = 0;
    int windowCount = 0;
  };

  void Accept();
  void OnReadable(int id, Conn& c, int64_t now);
  void ProcessHandshake(int id, Conn& c);
  void ProcessFrames(int id, Conn& c, int64_t now);
  void Flush(Conn& c);
  void Drop(int id);
  void Queue(Conn& c, const std::string& bytes) { c.outbuf += bytes; }
  static int64_t NowMs();

  ServerConfig config_;
  int listenFd_ = -1;
  uint16_t port_ = 0;
  std::atomic<bool> stop_{false};
  int nextId_ = 1;
  std::map<int, Conn> conns_;
  Lobby lobby_;
};

}  // namespace tactics::server
