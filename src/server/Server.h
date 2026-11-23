#pragma once

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include "server/GameStore.h"
#include "server/Lobby.h"
#include "server/SceneStore.h"
#include "server/WebSocket.h"

namespace tactics::server {

struct ServerConfig {
  uint16_t port = 8080;       // 0 = pick a free port (see Server::port()).
  std::string bindAddress = "0.0.0.0";
  size_t maxConnections = 128;
  int handshakeTimeoutMs = 5000;
  int closeTimeoutMs = 10000;  // A closing connection that cannot flush is dropped after this.
  size_t maxOutbufBytes = 4u << 20;  // A peer that lets this much pile up unread is dropped.
  int heartbeatMs = 30000;     // Period of the "stats" log line.
  int stallWarnMs = 3000;      // The watchdog logs when one loop pass takes longer than this.
  int idleTimeoutMs = 60000;   // Dropped if silent this long (clients are pinged at half).
  int maxMessagesPerSecond = 60;
  // Test-only control tap: a second loopback listener (same WebSocket
  // framing) that lets a test runner load a scenario scene into a room, dump
  // the server's unfiltered state and shut the server down. -1 = disabled
  // (the shipped server never enables it), 0 = pick a free port.
  int controlPort = -1;
  // Server-side Jev: base URL of the Jev adapter (server/jev-adapter), plain
  // HTTP, e.g. "http://127.0.0.1:8787" (the sidecar in the shipped image).
  // Empty disables the Jev vs Jev / Blue vs Jev modes.
  std::string jevAdapterUrl;
  int maxBotRooms = 4;  // Concurrent Jev matches; each one spends the Jev budget.
  // Directory for stored Builder scenes (one JSON file each); empty keeps
  // them in memory only, lost on restart. See SceneStore.
  std::string scenesDir;
  // SQLite file holding persisted two-player matches (see GameStore); empty
  // keeps matches in memory only (lost on restart). On a Fly machine this
  // lives on the mounted volume.
  std::string dbPath;
  // Human matches end as a draw after this many rounds (0 = unlimited). Only
  // the test-control build sets it (--round-limit).
  int roundLimit = 0;
};

// Single-threaded poll() WebSocket server feeding a Lobby. Also answers
// `GET /health` (200 "ok") so platforms can health-check it.
class Server {
 public:
  explicit Server(ServerConfig config);
  ~Server();

  // Starts the stall watchdog (a thread that logs where the loop is stuck if a
  // single pass runs longer than stallWarnMs). Call once, before Run().
  void StartWatchdog();

  // Binds and listens. Returns false (with *error set) on failure.
  bool Listen(std::string* error);
  uint16_t port() const { return port_; }
  uint16_t control_port() const { return controlPortBound_; }

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
    bool control = false;   // Accepted on the control tap, not the game port.
    std::string inbuf, outbuf;
    std::unique_ptr<ws::FrameParser> parser;
    int64_t connectedMs = 0, lastActivityMs = 0, lastPingMs = 0;
    int64_t windowStartMs = 0;
    int windowCount = 0;
    int64_t closingSinceMs = 0;
    const char* closeReason = "";
    size_t bytesIn = 0, bytesOut = 0;
    uint64_t messages = 0;
  };

  void Accept(int listenFd, bool control);
  void HandleControl(Conn& c, const std::string& text);
  bool BindListener(const std::string& address, int port, int* fd, uint16_t* bound, std::string* error);
  void OnReadable(int id, Conn& c, int64_t now);
  void ProcessHandshake(int id, Conn& c);
  void ProcessFrames(int id, Conn& c, int64_t now);
  void Flush(Conn& c);
  void Drop(int id);
  // Marks `c` for closing (once; the first reason wins) and optionally discards unsent output.
  void BeginClose(Conn& c, const char* reason, bool discardOutput = false);
  void Heartbeat(int64_t now);
  void Queue(Conn& c, const std::string& bytes) { c.outbuf += bytes; }
  static int64_t NowMs();

  ServerConfig config_;
  int listenFd_ = -1;
  uint16_t port_ = 0;
  int controlFd_ = -1;
  uint16_t controlPortBound_ = 0;
  std::atomic<bool> stop_{false};
  int nextId_ = 1;
  std::map<int, Conn> conns_;
  // Diagnostics. `phase_` is read by the watchdog thread, so it is atomic.
  std::atomic<const char*> phase_{"init"};
  std::atomic<int64_t> passStartMs_{0};  // 0 while idle in poll().
  std::atomic<bool> watchdogStop_{false};
  std::thread watchdog_;
  int64_t startMs_ = 0, lastHeartbeatMs_ = 0, maxPassMs_ = 0;
  uint64_t accepted_ = 0, dropped_ = 0, passes_ = 0;
  JevBotRunner bots_;  // Declared before lobby_, which holds a pointer to it.
  SceneStore scenes_;  // Declared before lobby_, which holds a pointer to it.
  std::unique_ptr<GameStore> games_;  // Likewise; null without ServerConfig::dbPath.
  Lobby lobby_;
};

}  // namespace tactics::server
