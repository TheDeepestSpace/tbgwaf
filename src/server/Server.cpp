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
#include <random>
#include <vector>

#include "net/Protocol.h"

namespace tactics::server {

namespace {
constexpr size_t kMaxHandshakeBytes = 8192;

void SetNonBlocking(int fd) { fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK); }

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
      lobby_(
          [this](int conn, const std::string& text) {
            auto it = conns_.find(conn);
            if (it == conns_.end() || !it->second.open) return;
            Queue(it->second, ws::EncodeFrame(ws::Opcode::Text, text));
          },
          RandomSeed) {}

Server::~Server() {
  for (auto& [id, c] : conns_) close(c.fd);
  if (listenFd_ >= 0) close(listenFd_);
}

bool Server::Listen(std::string* error) {
  listenFd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (listenFd_ < 0) {
    *error = std::string("socket: ") + std::strerror(errno);
    return false;
  }
  int one = 1;
  setsockopt(listenFd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(config_.port);
  if (inet_pton(AF_INET, config_.bindAddress.c_str(), &addr.sin_addr) != 1) {
    *error = "bad bind address: " + config_.bindAddress;
    return false;
  }
  if (bind(listenFd_, reinterpret_cast<sockaddr*>(&addr), sizeof addr) != 0 ||
      listen(listenFd_, 64) != 0) {
    *error = std::string("bind/listen: ") + std::strerror(errno);
    return false;
  }
  socklen_t len = sizeof addr;
  getsockname(listenFd_, reinterpret_cast<sockaddr*>(&addr), &len);
  port_ = ntohs(addr.sin_port);
  SetNonBlocking(listenFd_);
  return true;
}

void Server::Run() {
  while (!stop_) RunOnce(250);
}

void Server::Accept() {
  while (true) {
    const int fd = accept(listenFd_, nullptr, nullptr);
    if (fd < 0) return;
    if (conns_.size() >= config_.maxConnections) {
      close(fd);
      continue;
    }
    SetNonBlocking(fd);
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
    Conn c;
    c.fd = fd;
    c.connectedMs = c.lastActivityMs = c.lastPingMs = c.windowStartMs = NowMs();
    c.parser = std::make_unique<ws::FrameParser>(net::kMaxMessageBytes);
    conns_.emplace(nextId_++, std::move(c));
  }
}

void Server::Flush(Conn& c) {
  while (!c.outbuf.empty()) {
    const ssize_t n = send(c.fd, c.outbuf.data(), c.outbuf.size(), MSG_NOSIGNAL);
    if (n > 0) {
      c.outbuf.erase(0, static_cast<size_t>(n));
    } else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return;
    } else {
      c.closing = true;
      c.outbuf.clear();
      return;
    }
  }
}

void Server::Drop(int id) {
  auto it = conns_.find(id);
  if (it == conns_.end()) return;
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
  c.closing = true;
  if (status == ws::ParseStatus::Bad) {
    Queue(c, ws::HttpTextResponse(400, "Bad Request", "bad request"));
    return;
  }
  if (req.method == "GET" && req.upgrade == "websocket" && !req.key.empty() && req.version == "13") {
    Queue(c, ws::HandshakeResponse(req.key));
    c.open = true;
    c.closing = false;
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
        c.closing = true;
        return;
      case ws::Opcode::Text: {
        if (now - c.windowStartMs >= 1000) {
          c.windowStartMs = now;
          c.windowCount = 0;
        }
        if (++c.windowCount > config_.maxMessagesPerSecond) {
          Queue(c, ws::EncodeClose(1008));
          c.closing = true;
          return;
        }
        lobby_.OnMessage(id, msg->payload);
        break;
      }
      default:  // Binary / stray continuation: not part of the protocol.
        Queue(c, ws::EncodeClose(1003));
        c.closing = true;
        return;
    }
  }
  if (c.parser->failed()) {
    Queue(c, ws::EncodeClose(c.parser->failure_code()));
    c.closing = true;
  }
}

void Server::OnReadable(int id, Conn& c, int64_t now) {
  char buf[4096];
  while (true) {
    const ssize_t n = recv(c.fd, buf, sizeof buf, 0);
    if (n > 0) {
      c.lastActivityMs = now;
      c.inbuf.append(buf, static_cast<size_t>(n));
      // Bound buffering before a handshake completes.
      if (!c.open && c.inbuf.size() > kMaxHandshakeBytes + sizeof buf) {
        c.closing = true;
        return;
      }
    } else if (n == 0) {
      c.closing = true;
      c.outbuf.clear();
      return;
    } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
      break;
    } else {
      c.closing = true;
      c.outbuf.clear();
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
  for (auto& [id, c] : conns_) {
    short events = c.closing ? 0 : POLLIN;
    if (!c.outbuf.empty()) events |= POLLOUT;
    fds.push_back({c.fd, events, 0});
    ids.push_back(id);
  }
  poll(fds.data(), fds.size(), timeoutMs);
  const int64_t now = NowMs();

  if (fds[0].revents & POLLIN) Accept();
  for (size_t i = 1; i < fds.size(); ++i) {
    auto it = conns_.find(ids[i]);
    if (it == conns_.end()) continue;
    Conn& c = it->second;
    if (fds[i].revents & (POLLERR | POLLNVAL)) {
      c.closing = true;
      c.outbuf.clear();
    } else if (fds[i].revents & (POLLIN | POLLHUP)) {
      OnReadable(ids[i], c, now);
    }
  }

  // Lobby callbacks may have queued output on any connection: flush all,
  // run keepalive/timeouts, and reap finished ones.
  std::vector<int> dead;
  for (auto& [id, c] : conns_) {
    if (c.open && !c.closing) {
      if (now - c.lastActivityMs > config_.idleTimeoutMs) {
        c.closing = true;
        c.outbuf.clear();
      } else if (now - c.lastPingMs > config_.idleTimeoutMs / 2) {
        c.lastPingMs = now;
        Queue(c, ws::EncodeFrame(ws::Opcode::Ping, ""));
      }
    } else if (!c.open && !c.closing && now - c.connectedMs > config_.handshakeTimeoutMs) {
      c.closing = true;
    }
    Flush(c);
    if (c.closing && c.outbuf.empty()) dead.push_back(id);
  }
  for (int id : dead) Drop(id);
}

}  // namespace tactics::server
