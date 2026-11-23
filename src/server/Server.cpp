#include <algorithm>
#include "server/Server.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <exception>
#include <random>
#include <vector>

#include "net/Protocol.h"
#include "server/Log.h"
#include <dirent.h>
#include <fstream>
#include <sstream>

namespace tactics::server {

namespace {
constexpr size_t kMaxHandshakeBytes = 8192;
constexpr size_t kMaxControlBytes = 1 << 20;

void SetNonBlocking(int fd) { fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK); }

// Resident memory in KiB and open fd count, for the heartbeat (0 if unreadable).
long RssKb() {
  std::ifstream f("/proc/self/statm");
  long pages = 0, rss = 0;
  if (f >> pages >> rss) return rss * (sysconf(_SC_PAGESIZE) / 1024);
  return 0;
}
int OpenFds() {
  DIR* d = opendir("/proc/self/fd");
  if (!d) return 0;
  int n = 0;
  while (readdir(d)) ++n;
  closedir(d);
  return n - 3;  // ".", ".." and the DIR's own fd.
}

uint32_t RandomSeed() {
  static std::mt19937 rng{std::random_device{}()};
  return rng() % 1000000u + 1u;
}
}  // namespace

int64_t Server::NowMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

Server::Server(ServerConfig config)
    : config_(std::move(config)),
      bots_(JevBotConfig{config_.jevAdapterUrl}),
      scenes_(config_.scenesDir),
      lobby_(
          [this](int conn, const std::string& text) {
            auto it = conns_.find(conn);
            if (it == conns_.end() || !it->second.open) return;
            Queue(it->second, ws::EncodeFrame(ws::Opcode::Text, text));
          },
          RandomSeed) {
  lobby_.SetBots(&bots_, config_.maxBotRooms);
  lobby_.SetSceneStore(&scenes_);
}

Server::~Server() {
  watchdogStop_ = true;
  if (watchdog_.joinable()) watchdog_.join();
  for (auto& [id, c] : conns_) close(c.fd);
  if (listenFd_ >= 0) close(listenFd_);
  if (controlFd_ >= 0) close(controlFd_);
}

bool Server::BindListener(const std::string& address, int port, int* fd, uint16_t* bound,
                          std::string* error) {
  *fd = socket(AF_INET, SOCK_STREAM, 0);
  if (*fd < 0) {
    *error = std::string("socket: ") + std::strerror(errno);
    return false;
  }
  int one = 1;
  setsockopt(*fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  if (inet_pton(AF_INET, address.c_str(), &addr.sin_addr) != 1) {
    *error = "bad bind address: " + address;
    return false;
  }
  if (bind(*fd, reinterpret_cast<sockaddr*>(&addr), sizeof addr) != 0 || listen(*fd, 64) != 0) {
    *error = std::string("bind/listen: ") + std::strerror(errno);
    return false;
  }
  socklen_t len = sizeof addr;
  getsockname(*fd, reinterpret_cast<sockaddr*>(&addr), &len);
  *bound = ntohs(addr.sin_port);
  SetNonBlocking(*fd);
  return true;
}

bool Server::Listen(std::string* error) {
  if (!BindListener(config_.bindAddress, config_.port, &listenFd_, &port_, error)) return false;
  if (config_.controlPort >= 0 &&
      !BindListener("127.0.0.1", config_.controlPort, &controlFd_, &controlPortBound_, error)) {
    return false;
  }
  return true;
}

void Server::StartWatchdog() {
  if (watchdog_.joinable()) return;
  watchdog_ = std::thread([this] {
    int64_t reportedPass = 0;
    int64_t lastReportMs = 0;
    while (!watchdogStop_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      const int64_t started = passStartMs_.load();
      if (started == 0) continue;  // Idle in poll().
      const int64_t now = NowMs();
      const int64_t busy = now - started;
      if (busy < config_.stallWarnMs) continue;
      // One line when the stall is first noticed, then one every 10 s.
      if (started != reportedPass || now - lastReportMs >= 10000) {
        log::Error("watchdog: loop pass stuck for %lld ms in phase '%s'", static_cast<long long>(busy),
                   phase_.load());
        reportedPass = started;
        lastReportMs = now;
      }
    }
  });
}

void Server::Run() {
  startMs_ = lastHeartbeatMs_ = NowMs();
  log::Info("server running: port=%u maxConnections=%zu idleTimeoutMs=%d rssKb=%ld", port_,
            config_.maxConnections, config_.idleTimeoutMs, RssKb());
  while (!stop_) RunOnce(250);
  log::Info("server stopping: connections=%zu rooms=%zu uptimeS=%lld", conns_.size(),
            lobby_.RoomCount(), static_cast<long long>((NowMs() - startMs_) / 1000));
}

void Server::Heartbeat(int64_t now) {
  if (now - lastHeartbeatMs_ < config_.heartbeatMs) return;
  lastHeartbeatMs_ = now;
  size_t open = 0, closing = 0, pendingOut = 0;
  for (const auto& [id, c] : conns_) {
    open += c.open;
    closing += c.closing;
    pendingOut += c.outbuf.size();
  }
  log::Info(
      "stats: conns=%zu open=%zu closing=%zu rooms=%zu accepted=%llu dropped=%llu "
      "outbufBytes=%zu fds=%d rssKb=%ld maxPassMs=%lld uptimeS=%lld jevBusy=%d",
      conns_.size(), open, closing, lobby_.RoomCount(), static_cast<unsigned long long>(accepted_),
      static_cast<unsigned long long>(dropped_), pendingOut, OpenFds(), RssKb(),
      static_cast<long long>(maxPassMs_), static_cast<long long>((now - startMs_) / 1000),
      lobby_.BotsBusy() ? 1 : 0);
  maxPassMs_ = 0;
}

void Server::BeginClose(Conn& c, const char* reason, bool discardOutput) {
  if (!c.closing) {
    c.closing = true;
    c.closingSinceMs = NowMs();
    c.closeReason = reason;
  }
  if (discardOutput) c.outbuf.clear();
}

void Server::Accept(int listenFd, bool control) {
  while (true) {
    const int fd = accept(listenFd, nullptr, nullptr);
    if (fd < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        // EMFILE/ENFILE here means the process ran out of fds: the listener
        // stays readable and nothing new can connect.
        log::Error("accept failed: %s (conns=%zu fds=%d)", std::strerror(errno), conns_.size(),
                   OpenFds());
      }
      return;
    }
    if (conns_.size() >= config_.maxConnections) {
      log::Warn("connection refused: at the %zu connection limit", config_.maxConnections);
      close(fd);
      continue;
    }
    SetNonBlocking(fd);
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
    Conn c;
    c.fd = fd;
    c.connectedMs = c.lastActivityMs = c.lastPingMs = c.windowStartMs = NowMs();
    c.control = control;
    c.parser = std::make_unique<ws::FrameParser>(control ? kMaxControlBytes : net::kMaxMessageBytes);
    ++accepted_;
    const int id = nextId_++;
    log::Info("conn %d accepted%s (conns=%zu)", id, control ? " [control]" : "", conns_.size() + 1);
    conns_.emplace(id, std::move(c));
  }
}

void Server::Flush(Conn& c) {
  while (!c.outbuf.empty()) {
    const ssize_t n = send(c.fd, c.outbuf.data(), c.outbuf.size(), MSG_NOSIGNAL);
    if (n > 0) {
      c.bytesOut += static_cast<size_t>(n);
      c.outbuf.erase(0, static_cast<size_t>(n));
    } else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return;
    } else {
      log::Info("send failed: %s", n < 0 ? std::strerror(errno) : "wrote 0 bytes");
      BeginClose(c, "send-failed", true);
      return;
    }
  }
}

void Server::Drop(int id) {
  auto it = conns_.find(id);
  if (it == conns_.end()) return;
  const Conn& c = it->second;
  ++dropped_;
  log::Info("conn %d closed: reason=%s ageS=%lld in=%zuB out=%zuB msgs=%llu (conns=%zu)", id,
            c.closeReason, static_cast<long long>((NowMs() - c.connectedMs) / 1000), c.bytesIn,
            c.bytesOut, static_cast<unsigned long long>(c.messages), conns_.size() - 1);
  close(it->second.fd);
  conns_.erase(it);
  lobby_.OnDisconnect(id);
}

void Server::ProcessHandshake(int id, Conn& c) {
  (void)id;
  ws::HttpRequest req;
  size_t consumed = 0;
  const ws::ParseStatus status = ws::ParseHttpRequest(c.inbuf, kMaxHandshakeBytes, &req, &consumed);
  if (status == ws::ParseStatus::NeedMore) return;
  BeginClose(c, "http-response");
  if (status == ws::ParseStatus::Bad) {
    log::Warn("bad handshake request (%zu bytes)", c.inbuf.size());
    Queue(c, ws::HttpTextResponse(400, "Bad Request", "bad request"));
    return;
  }
  if (req.method == "GET" && req.upgrade == "websocket" && !req.key.empty() && req.version == "13") {
    Queue(c, ws::HandshakeResponse(req.key));
    c.open = true;
    c.closing = false;
    c.closeReason = "";
    c.inbuf.erase(0, consumed);
    return;
  }
  if (req.method == "GET" && req.path == "/health") {
    Queue(c, ws::HttpTextResponse(200, "OK", "ok"));
  } else {
    Queue(c, ws::HttpTextResponse(426, "Upgrade Required", "WebSocket endpoint"));
  }
}

void Server::ProcessFrames(int id, Conn& c, int64_t now) {
  if (!c.inbuf.empty()) {
    c.parser->Feed(c.inbuf.data(), c.inbuf.size());
    c.inbuf.clear();
  }
  while (auto msg = c.parser->Next()) {
    switch (msg->op) {
      case ws::Opcode::Ping: Queue(c, ws::EncodeFrame(ws::Opcode::Pong, msg->payload)); break;
      case ws::Opcode::Pong: break;
      case ws::Opcode::Close:
        Queue(c, ws::EncodeClose(1000));
        BeginClose(c, "client-close");
        return;
      case ws::Opcode::Text: {
        if (c.control) {
          HandleControl(c, msg->payload);
          break;
        }
        if (now - c.windowStartMs >= 1000) {
          c.windowStartMs = now;
          c.windowCount = 0;
        }
        if (++c.windowCount > config_.maxMessagesPerSecond) {
          log::Warn("conn %d exceeded %d messages/s", id, config_.maxMessagesPerSecond);
          Queue(c, ws::EncodeClose(1008));
          BeginClose(c, "rate-limit");
          return;
        }
        ++c.messages;
        try {
          lobby_.OnMessage(id, msg->payload);
        } catch (const std::exception& e) {
          // Don't let one bad message take the whole server (and every match) down.
          log::Error("conn %d: exception handling a message (%zu bytes): %s", id,
                     msg->payload.size(), e.what());
          Queue(c, ws::EncodeClose(1011));
          BeginClose(c, "handler-exception");
          return;
        }
        break;
      }
      default:  // Binary / stray continuation: not part of the protocol.
        Queue(c, ws::EncodeClose(1003));
        BeginClose(c, "unsupported-frame");
        return;
    }
  }
  if (c.parser->failed()) {
    log::Warn("conn %d sent a bad frame (code %d)", id, static_cast<int>(c.parser->failure_code()));
    Queue(c, ws::EncodeClose(c.parser->failure_code()));
    BeginClose(c, "protocol-error");
  }
}

void Server::HandleControl(Conn& c, const std::string& text) {
  net::Json message;
  net::Json reply;
  if (!net::Json::Parse(text, &message) || !message.IsObject()) {
    reply.Set("t", "control");
    reply.Set("ok", net::Json(false));
    reply.Set("error", "malformed message");
  } else if (message["t"].IsString() && message["t"].AsString() == "shutdown") {
    stop_ = true;
    reply.Set("t", "control");
    reply.Set("ok", net::Json(true));
  } else {
    reply = lobby_.OnControl(message);
  }
  Queue(c, ws::EncodeFrame(ws::Opcode::Text, reply.Dump()));
}

void Server::OnReadable(int id, Conn& c, int64_t now) {
  char buf[4096];
  while (true) {
    const ssize_t n = recv(c.fd, buf, sizeof buf, 0);
    if (n > 0) {
      c.lastActivityMs = now;
      c.bytesIn += static_cast<size_t>(n);
      c.inbuf.append(buf, static_cast<size_t>(n));
      // Bound buffering before a handshake completes.
      if (!c.open && c.inbuf.size() > kMaxHandshakeBytes + sizeof buf) {
        BeginClose(c, "handshake-too-large");
        return;
      }
    } else if (n == 0) {
      BeginClose(c, "peer-closed", true);
      return;
    } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
      break;
    } else {
      log::Info("conn %d recv failed: %s", id, std::strerror(errno));
      BeginClose(c, "recv-failed", true);
      return;
    }
  }
  if (!c.open) ProcessHandshake(id, c);
  if (c.open) ProcessFrames(id, c, now);
}

void Server::RunOnce(int timeoutMs) {
  std::vector<pollfd> fds;
  std::vector<int> ids;
  fds.push_back({listenFd_, POLLIN, 0});
  ids.push_back(0);
  if (controlFd_ >= 0) {
    fds.push_back({controlFd_, POLLIN, 0});
    ids.push_back(-1);
  }
  for (auto& [id, c] : conns_) {
    short events = c.closing ? 0 : POLLIN;
    if (!c.outbuf.empty()) events |= POLLOUT;
    fds.push_back({c.fd, events, 0});
    ids.push_back(id);
  }
  // While Jev is thinking, wake often enough to deliver its answer promptly.
  phase_ = "poll";
  passStartMs_ = 0;
  poll(fds.data(), fds.size(), lobby_.BotsBusy() ? std::min(timeoutMs, 50) : timeoutMs);
  const int64_t now = NowMs();
  passStartMs_ = now;
  phase_ = "accept";

  if (fds[0].revents & POLLIN) Accept(listenFd_, false);
  size_t first = 1;
  if (controlFd_ >= 0) {
    if (fds[1].revents & POLLIN) Accept(controlFd_, true);
    first = 2;
  }
  phase_ = "read-and-dispatch";
  for (size_t i = first; i < fds.size(); ++i) {
    auto it = conns_.find(ids[i]);
    if (it == conns_.end()) continue;
    Conn& c = it->second;
    if (fds[i].revents & (POLLERR | POLLNVAL)) {
      BeginClose(c, "poll-error", true);
    } else if (fds[i].revents & (POLLIN | POLLHUP)) {
      OnReadable(ids[i], c, now);
    }
  }

  phase_ = "lobby-tick";
  try {
    lobby_.Tick();
  } catch (const std::exception& e) {
    log::Error("exception in lobby tick: %s", e.what());
  }
  phase_ = "flush";

  // Lobby callbacks may have queued output on any connection: flush all,
  // run keepalive/timeouts, and reap finished ones.
  std::vector<int> dead;
  for (auto& [id, c] : conns_) {
    if (c.open && !c.closing) {
      if (now - c.lastActivityMs > config_.idleTimeoutMs) {
        BeginClose(c, "idle-timeout", true);
      } else if (now - c.lastPingMs > config_.idleTimeoutMs / 2) {
        c.lastPingMs = now;
        Queue(c, ws::EncodeFrame(ws::Opcode::Ping, ""));
      }
    } else if (!c.open && !c.closing && now - c.connectedMs > config_.handshakeTimeoutMs) {
      BeginClose(c, "handshake-timeout");
    }
    if (c.outbuf.size() > config_.maxOutbufBytes) {
      log::Warn("conn %d is not reading: %zu bytes queued", id, c.outbuf.size());
      BeginClose(c, "output-backlog", true);
    }
    Flush(c);
    // A closing connection whose peer never drains its buffer would otherwise
    // hold a slot forever; enough of them exhaust maxConnections and the
    // server then refuses everyone, /health included.
    if (c.closing && !c.outbuf.empty() && now - c.closingSinceMs > config_.closeTimeoutMs) {
      c.outbuf.clear();
    }
    if (c.closing && c.outbuf.empty()) dead.push_back(id);
  }
  for (int id : dead) Drop(id);
  const int64_t passMs = NowMs() - now;
  maxPassMs_ = std::max(maxPassMs_, passMs);
  ++passes_;
  if (passMs >= config_.stallWarnMs / 6) {
    log::Warn("slow loop pass: %lld ms (conns=%zu rooms=%zu)", static_cast<long long>(passMs),
              conns_.size(), lobby_.RoomCount());
  }
  Heartbeat(now);
  passStartMs_ = 0;
  phase_ = "poll";
}

}  // namespace tactics::server
