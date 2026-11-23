// tbgwaf authoritative game server: pairs two WebSocket clients per room
// and runs the match on the headless tactics_logic library.
//
//   tbgwaf_server [--port N] [--bind ADDR]      (PORT env var also honored)
//
// Server-side Jev (Blue vs Jev / Jev vs Jev): set TBGWAF_JEV_ADAPTER_URL to the
// Jev adapter (http://host:port; the shipped image runs it on 127.0.0.1:8787
// when TBGWAF_JEV_API_KEY is set). TBGWAF_JEV_MAX_ROOMS caps concurrent Jev matches.
//
// Game persistence: set TBGWAF_DB_PATH (or --db FILE) to a SQLite file; matches
// between two players who joined with a device token are stored there and
// restored on startup. Unset, matches live in memory only.
//
// Builder scene storage: set TBGWAF_SCENES_DIR to keep scenes on disk (one JSON
// file each, survives restarts); unset, they live in memory until restart.
//
// Built with -DTBGWAF_TEST_CONTROL (the tbgwaf_server_testctl target, used by
// the networked scenario tests) it also accepts --control-port N (the loopback-only
// test control tap) and --rate-limit N (messages/s per connection) and
// --round-limit N (matches end as a draw after N rounds). The shipped binary has no such flag.
#include <execinfo.h>
#include <unistd.h>

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <string>

#include "server/Log.h"
#include "server/Server.h"

namespace {
tactics::server::Server* g_server = nullptr;
void WriteErr(const char* text) {
  const ssize_t ignored = write(STDOUT_FILENO, text, std::strlen(text));
  (void)ignored;
}
void HandleSignal(int sig) {
  // Fly sends SIGTERM when it stops or replaces the machine (autostop, deploy,
  // OOM kills arrive as SIGKILL and leave no trace here): leave a marker.
  WriteErr(sig == SIGTERM ? "SIGTERM received: shutting down\n" : "SIGINT received: shutting down\n");
  if (g_server) g_server->Stop();
}
// Fatal signals: print a backtrace (async-signal-safe calls only) so a crash
// shows up in `fly logs` instead of just a machine restart.
void HandleCrash(int sig) {
  WriteErr("FATAL: crashed with signal ");
  WriteErr(strsignal(sig));
  WriteErr("\n");
  void* frames[48];
  const int n = backtrace(frames, 48);
  backtrace_symbols_fd(frames, n, STDOUT_FILENO);
  std::signal(sig, SIG_DFL);
  std::raise(sig);
}
void HandleTerminate() {
  try {
    if (auto e = std::current_exception()) std::rethrow_exception(e);
    tactics::server::log::Error("FATAL: std::terminate without an exception");
  } catch (const std::exception& e) {
    tactics::server::log::Error("FATAL: uncaught exception: %s", e.what());
  } catch (...) {
    tactics::server::log::Error("FATAL: uncaught non-standard exception");
  }
  void* frames[48];
  const int n = backtrace(frames, 48);
  backtrace_symbols_fd(frames, n, STDOUT_FILENO);
  std::abort();
}
}  // namespace

int main(int argc, char** argv) {
  tactics::server::log::SetEnabled(true);
  std::set_terminate(HandleTerminate);
  for (int sig : {SIGSEGV, SIGABRT, SIGBUS, SIGFPE, SIGILL}) std::signal(sig, HandleCrash);
  tactics::server::ServerConfig config;
  if (const char* env = std::getenv("PORT")) config.port = static_cast<uint16_t>(std::atoi(env));
  if (const char* jev = std::getenv("TBGWAF_JEV_ADAPTER_URL")) config.jevAdapterUrl = jev;
  if (const char* scenes = std::getenv("TBGWAF_SCENES_DIR")) config.scenesDir = scenes;
  if (const char* db = std::getenv("TBGWAF_DB_PATH")) config.dbPath = db;
  if (const char* rooms = std::getenv("TBGWAF_JEV_MAX_ROOMS")) config.maxBotRooms = std::atoi(rooms);
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--port" && i + 1 < argc) {
      config.port = static_cast<uint16_t>(std::atoi(argv[++i]));
    } else if (arg == "--bind" && i + 1 < argc) {
      config.bindAddress = argv[++i];
    } else if (arg == "--db" && i + 1 < argc) {
      config.dbPath = argv[++i];
#ifdef TBGWAF_TEST_CONTROL
    } else if (arg == "--control-port" && i + 1 < argc) {
      config.controlPort = std::atoi(argv[++i]);
    } else if (arg == "--rate-limit" && i + 1 < argc) {
      config.maxMessagesPerSecond = std::atoi(argv[++i]);
    } else if (arg == "--round-limit" && i + 1 < argc) {
      config.roundLimit = std::atoi(argv[++i]);
#endif
    } else {
      std::fprintf(stderr, "usage: %s [--port N] [--bind ADDR] [--db FILE]\n", argv[0]);
      return arg == "--help" ? 0 : 2;
    }
  }

  tactics::server::Server server(config);
  std::string error;
  if (!server.Listen(&error)) {
    std::fprintf(stderr, "tbgwaf_server: %s\n", error.c_str());
    return 1;
  }
  g_server = &server;
  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);
  std::signal(SIGPIPE, SIG_IGN);
  std::printf("tbgwaf_server listening on %s:%u\n", config.bindAddress.c_str(), server.port());
  if (config.controlPort >= 0) {
    std::printf("tbgwaf_server control tap on 127.0.0.1:%u\n", server.control_port());
  }
  std::fflush(stdout);
  server.StartWatchdog();
  server.Run();
  tactics::server::log::Info("tbgwaf_server exiting cleanly");
  return 0;
}
