#include "server/HttpClient.h"

#include <fcntl.h>
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstdlib>

namespace tactics::server {

namespace {

using Clock = std::chrono::steady_clock;

struct Fd {
  int fd = -1;
  ~Fd() {
    if (fd >= 0) close(fd);
  }
};

// Waits for `events` on fd until the deadline, in short slices so `cancelled`
// is honoured. Returns >0 ready, 0 deadline, <0 cancelled/error.
int WaitFor(int fd, short events, Clock::time_point deadline,
            const std::function<bool()>& cancelled) {
  for (;;) {
    if (cancelled && cancelled()) return -1;
    const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now());
    if (left.count() <= 0) return 0;
    pollfd p{fd, events, 0};
    const int n = poll(&p, 1, static_cast<int>(std::min<long long>(left.count(), 200)));
    if (n > 0) return 1;
    if (n < 0 && errno != EINTR) return -1;
  }
}

std::string Lower(std::string s) {
  for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

}  // namespace

bool ParseHttpUrl(const std::string& url, HttpUrl* out) {
  static const std::string kScheme = "http://";
  if (url.compare(0, kScheme.size(), kScheme) != 0) return false;
  std::string rest = url.substr(kScheme.size());
  const size_t slash = rest.find('/');
  std::string authority = rest.substr(0, slash);
  std::string path = slash == std::string::npos ? "" : rest.substr(slash);
  while (!path.empty() && path.back() == '/') path.pop_back();
  if (authority.empty() || authority.find('@') != std::string::npos) return false;
  HttpUrl parsed;
  if (authority.front() == '[') {  // [v6]:port
    const size_t close = authority.find(']');
    if (close == std::string::npos) return false;
    parsed.host = authority.substr(1, close - 1);
    if (close + 1 < authority.size()) {
      if (authority[close + 1] != ':') return false;
      parsed.port = authority.substr(close + 2);
    }
  } else {
    const size_t colon = authority.rfind(':');
    parsed.host = authority.substr(0, colon);
    if (colon != std::string::npos) parsed.port = authority.substr(colon + 1);
  }
  if (parsed.host.empty() || parsed.port.empty() ||
      !std::all_of(parsed.port.begin(), parsed.port.end(),
                   [](char c) { return std::isdigit(static_cast<unsigned char>(c)); })) {
    return false;
  }
  parsed.basePath = path;
  *out = parsed;
  return true;
}

bool HttpPostJson(const HttpUrl& url, const std::string& path, const std::string& body,
                  int timeoutMs, HttpResponse* response, std::string* error,
                  const std::function<bool()>& cancelled) {
  const auto deadline = Clock::now() + std::chrono::milliseconds(timeoutMs);
  auto Fail = [&](const char* why) {
    *error = why;
    return false;
  };

  addrinfo hints{};
  hints.ai_socktype = SOCK_STREAM;
  addrinfo* found = nullptr;
  if (getaddrinfo(url.host.c_str(), url.port.c_str(), &hints, &found) != 0 || !found) {
    return Fail("cannot resolve the Jev adapter host");
  }
  Fd sock;
  bool connected = false;
  for (addrinfo* a = found; a && !connected; a = a->ai_next) {
    Fd candidate;
    candidate.fd = socket(a->ai_family, a->ai_socktype, a->ai_protocol);
    if (candidate.fd < 0) continue;
    fcntl(candidate.fd, F_SETFL, fcntl(candidate.fd, F_GETFL, 0) | O_NONBLOCK);
    int rc = connect(candidate.fd, a->ai_addr, a->ai_addrlen);
    if (rc < 0 && errno == EINPROGRESS) {
      if (WaitFor(candidate.fd, POLLOUT, deadline, cancelled) <= 0) continue;
      int soError = 0;
      socklen_t len = sizeof(soError);
      getsockopt(candidate.fd, SOL_SOCKET, SO_ERROR, &soError, &len);
      rc = soError == 0 ? 0 : -1;
    }
    if (rc == 0) {
      sock.fd = candidate.fd;
      candidate.fd = -1;
      connected = true;
    }
  }
  freeaddrinfo(found);
  if (!connected) return Fail("cannot connect to the Jev adapter");

  std::string request = "POST " + url.basePath + path + " HTTP/1.1\r\nHost: " + url.host +
                        "\r\nContent-Type: application/json\r\nContent-Length: " +
                        std::to_string(body.size()) + "\r\nConnection: close\r\n\r\n" + body;
  size_t sent = 0;
  while (sent < request.size()) {
    if (WaitFor(sock.fd, POLLOUT, deadline, cancelled) <= 0) return Fail("Jev adapter request timed out");
    const ssize_t n = send(sock.fd, request.data() + sent, request.size() - sent, MSG_NOSIGNAL);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
      return Fail("Jev adapter connection failed");
    }
    sent += static_cast<size_t>(n);
  }

  std::string raw;
  constexpr size_t kMaxResponse = 1 << 20;
  for (;;) {
    if (WaitFor(sock.fd, POLLIN, deadline, cancelled) <= 0) return Fail("Jev adapter request timed out");
    char buf[4096];
    const ssize_t n = recv(sock.fd, buf, sizeof(buf), 0);
    if (n == 0) break;
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
      return Fail("Jev adapter connection failed");
    }
    raw.append(buf, static_cast<size_t>(n));
    if (raw.size() > kMaxResponse) return Fail("Jev adapter response is too large");
  }

  const size_t headEnd = raw.find("\r\n\r\n");
  if (raw.compare(0, 5, "HTTP/") != 0 || headEnd == std::string::npos) {
    return Fail("Jev adapter returned a malformed response");
  }
  const size_t sp = raw.find(' ');
  response->status = sp == std::string::npos ? 0 : std::atoi(raw.c_str() + sp + 1);
  const std::string head = Lower(raw.substr(0, headEnd));
  std::string payload = raw.substr(headEnd + 4);
  if (head.find("transfer-encoding: chunked") != std::string::npos) {
    std::string decoded;
    size_t pos = 0;
    for (;;) {
      const size_t eol = payload.find("\r\n", pos);
      if (eol == std::string::npos) return Fail("Jev adapter returned a malformed response");
      const unsigned long size = std::strtoul(payload.c_str() + pos, nullptr, 16);
      pos = eol + 2;
      if (size == 0) break;
      if (pos + size > payload.size()) return Fail("Jev adapter returned a malformed response");
      decoded.append(payload, pos, size);
      pos += size + 2;
    }
    payload = std::move(decoded);
  }
  response->body = std::move(payload);
  return true;
}

}  // namespace tactics::server
