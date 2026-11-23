#pragma once

#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <ctime>
#include <mutex>

namespace tactics::server::log {

// Line-oriented, timestamped logging to stdout (Fly.io captures stdout/stderr
// and serves it through `fly logs`). Silent unless server_main enables it, so
// the library stays quiet inside the unit tests.
inline std::atomic<bool>& Enabled() {
  static std::atomic<bool> enabled{false};
  return enabled;
}
inline void SetEnabled(bool on) { Enabled() = on; }

// Usage: log::Info("room %s started", name.c_str()); level is a short tag.
inline void Write(const char* level, const char* fmt, va_list args) {
  if (!Enabled()) return;
  static std::mutex mutex;
  timespec ts{};
  clock_gettime(CLOCK_REALTIME, &ts);
  tm utc{};
  gmtime_r(&ts.tv_sec, &utc);
  char stamp[64];
  std::snprintf(stamp, sizeof stamp, "%04d-%02d-%02dT%02d:%02d:%02d.%03ldZ", utc.tm_year + 1900,
                utc.tm_mon + 1, utc.tm_mday, utc.tm_hour, utc.tm_min, utc.tm_sec,
                ts.tv_nsec / 1000000);
  char line[1024];
  std::vsnprintf(line, sizeof line, fmt, args);
  std::lock_guard<std::mutex> lock(mutex);
  std::fprintf(stdout, "%s %-5s %s\n", stamp, level, line);
  std::fflush(stdout);
}

#if defined(__GNUC__)
#define TBGWAF_LOG_FMT __attribute__((format(printf, 1, 2)))
#else
#define TBGWAF_LOG_FMT
#endif

TBGWAF_LOG_FMT inline void Info(const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  Write("INFO", fmt, args);
  va_end(args);
}
TBGWAF_LOG_FMT inline void Warn(const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  Write("WARN", fmt, args);
  va_end(args);
}
TBGWAF_LOG_FMT inline void Error(const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  Write("ERROR", fmt, args);
  va_end(args);
}

}  // namespace tactics::server::log
