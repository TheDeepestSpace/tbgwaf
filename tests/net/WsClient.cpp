#include "WsClient.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <random>

#include "server/WebSocket.h"

namespace tactics::testing {

bool WsClient::Connect(uint16_t port, std::string* error) {
  fd_ = socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
  if (fd_ < 0 || connect(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof addr) != 0) {
    *error = std::string("connect: ") + std::strerror(errno);
    Close();
    return false;
  }
  int one = 1;
  setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);

  const std::string key = "dGhlIHNhbXBsZSBub25jZQ==";
  const std::string request = "GET / HTTP/1.1\r\nHost: 127.0.0.1:" + std::to_string(port) +
                              "\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                              "Sec-WebSocket-Key: " + key + "\r\nSec-WebSocket-Version: 13\r\n\r\n";
  if (!SendRaw(request)) {
    *error = "handshake send failed";
    Close();
    return false;
  }
  // Read the response head (bounded wait), keeping any bytes that follow it.
  for (int waited = 0; inbuf_.find("\r\n\r\n") == std::string::npos; waited += 50) {
    if (waited > 5000) {
      *error = "handshake timed out";
      Close();
      return false;
    }
    pollfd p{fd_, POLLIN, 0};
    if (poll(&p, 1, 50) > 0) {
      char buf[2048];
      const ssize_t n = recv(fd_, buf, sizeof buf, 0);
      if (n <= 0) {
        *error = "closed during handshake";
        Close();
        return false;
      }
      inbuf_.append(buf, static_cast<size_t>(n));
    }
  }
  const size_t end = inbuf_.find("\r\n\r\n") + 4;
  const std::string head = inbuf_.substr(0, end);
  inbuf_.erase(0, end);
  if (head.find(" 101 ") == std::string::npos ||
      head.find(server::ws::AcceptKey(key)) == std::string::npos) {
    *error = "bad handshake response";
    Close();
    return false;
  }
  return true;
}

bool WsClient::SendRaw(const std::string& bytes) {
  size_t sent = 0;
  while (sent < bytes.size()) {
    const ssize_t n = send(fd_, bytes.data() + sent, bytes.size() - sent, MSG_NOSIGNAL);
    if (n > 0) {
      sent += static_cast<size_t>(n);
    } else if (n < 0 && (errno == EAGAIN || errno == EINTR)) {
      pollfd p{fd_, POLLOUT, 0};
      poll(&p, 1, 100);
    } else {
      return false;
    }
  }
  return true;
}

bool WsClient::SendFrame(uint8_t opcode, const std::string& payload) {
  static std::mt19937 rng{12345};
  std::string out;
  out.push_back(static_cast<char>(0x80 | opcode));
  const size_t n = payload.size();
  if (n < 126) {
    out.push_back(static_cast<char>(0x80 | n));
  } else if (n <= 0xFFFF) {
    out.push_back(static_cast<char>(0x80 | 126));
    out.push_back(static_cast<char>(n >> 8));
    out.push_back(static_cast<char>(n & 0xFF));
  } else {
    out.push_back(static_cast<char>(0x80 | 127));
    for (int i = 7; i >= 0; --i) out.push_back(static_cast<char>((static_cast<uint64_t>(n) >> (i * 8)) & 0xFF));
  }
  char mask[4];
  for (char& m : mask) m = static_cast<char>(rng() & 0xFF);
  out.append(mask, 4);
  for (size_t i = 0; i < n; ++i) out.push_back(static_cast<char>(payload[i] ^ mask[i % 4]));
  return SendRaw(out);
}

bool WsClient::Send(const std::string& text) { return open() && SendFrame(1, text); }

void WsClient::Parse(std::vector<std::string>* out) {
  while (inbuf_.size() >= 2) {
    const uint8_t b0 = static_cast<uint8_t>(inbuf_[0]);
    uint64_t len = static_cast<uint8_t>(inbuf_[1]) & 0x7F;
    size_t header = 2;
    if (len == 126) {
      if (inbuf_.size() < 4) return;
      len = (static_cast<uint8_t>(inbuf_[2]) << 8) | static_cast<uint8_t>(inbuf_[3]);
      header = 4;
    } else if (len == 127) {
      if (inbuf_.size() < 10) return;
      len = 0;
      for (int i = 0; i < 8; ++i) len = (len << 8) | static_cast<uint8_t>(inbuf_[2 + i]);
      header = 10;
    }
    if (inbuf_.size() < header + len) return;
    const std::string payload = inbuf_.substr(header, static_cast<size_t>(len));
    inbuf_.erase(0, header + static_cast<size_t>(len));
    switch (b0 & 0x0F) {
      case 1: out->push_back(payload); break;  // The server never fragments.
      case 9: SendFrame(10, payload); break;
      case 8: closed_ = true; return;
      default: break;
    }
  }
}

bool WsClient::Poll(int timeoutMs, std::vector<std::string>* out) {
  if (!open()) return false;
  pollfd p{fd_, POLLIN, 0};
  if (poll(&p, 1, timeoutMs) > 0) {
    char buf[8192];
    while (true) {
      const ssize_t n = recv(fd_, buf, sizeof buf, MSG_DONTWAIT);
      if (n > 0) {
        inbuf_.append(buf, static_cast<size_t>(n));
      } else if (n == 0) {
        closed_ = true;
        break;
      } else {
        break;  // EAGAIN (or an error that the next poll will surface).
      }
    }
  }
  Parse(out);
  return open();
}

void WsClient::Close() {
  if (fd_ >= 0) close(fd_);
  fd_ = -1;
}

}  // namespace tactics::testing
