#pragma once

#include <functional>
#include <string>

namespace tactics::server {

struct HttpResponse {
  int status = 0;
  std::string body;
};

struct HttpUrl {
  std::string host;
  std::string port = "80";
  std::string basePath;  // No trailing slash; "" for the root.
};

// Parses "http://host[:port][/base]". Plain HTTP only: the Jev adapter runs
// next to the server (loopback or the platform's private network), and TLS
// to Jev's API is terminated by the adapter. Returns false for anything else.
bool ParseHttpUrl(const std::string& url, HttpUrl* out);

// Blocking JSON POST of `body` to `url.basePath + path` with an overall
// deadline. `cancelled` (optional) is polled so a shutdown never waits out
// the deadline. Returns false with *error set on transport failure; any HTTP
// status (including 4xx/5xx) is a successful transport and is reported in
// *response. Call from a worker thread, never the poll loop.
bool HttpPostJson(const HttpUrl& url, const std::string& path, const std::string& body,
                  int timeoutMs, HttpResponse* response, std::string* error,
                  const std::function<bool()>& cancelled = nullptr);

}  // namespace tactics::server
