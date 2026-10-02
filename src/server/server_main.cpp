// tbgwaf authoritative game server: pairs two WebSocket clients per room
// and runs the match on the headless tactics_logic library.
//
//   tbgwaf_server [--port N] [--bind ADDR]      (PORT env var also honored)
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "server/Server.h"

namespace {
tactics::server::Server* g_server = nullptr;
void HandleSignal(int) {
  if (g_server) g_server->Stop();
}
}  // namespace

int main(int argc, char** argv) {
  tactics::server::ServerConfig config;
  if (const char* env = std::getenv("PORT")) config.port = static_cast<uint16_t>(std::atoi(env));
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--port" && i + 1 < argc) {
      config.port = static_cast<uint16_t>(std::atoi(argv[++i]));
    } else if (arg == "--bind" && i + 1 < argc) {
      config.bindAddress = argv[++i];
    } else {
      std::fprintf(stderr, "usage: %s [--port N] [--bind ADDR]\n", argv[0]);
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
  std::fflush(stdout);
  server.Run();
  return 0;
}
