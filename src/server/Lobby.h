#pragma once

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "server/GameSession.h"

namespace tactics::server {

// Pairs connections into games: "a session is just two sockets, one game".
// The first connection to join a room waits (Blue); the second is paired
// (Red) and the authoritative GameSession starts. Transport-agnostic: the
// socket layer feeds it messages and supplies a send callback. A third kind of
// connection, a spectator ({"t":"join","room":..,"spectate":true}), watches a
// room read-only with unfiltered state; it is not one of the two players.
class Lobby {
 public:
  using SendFn = std::function<void(int conn, const std::string& text)>;
  using SeedFn = std::function<uint32_t()>;
  Lobby(SendFn send, SeedFn seed) : send_(std::move(send)), seed_(std::move(seed)) {}

  // One inbound text message from `conn`. Never throws on bad input.
  void OnMessage(int conn, const std::string& text);
  void OnDisconnect(int conn);

  // Test-only control tap (see Server). `load_scenario` pre-registers a room
  // whose match runs on an explicit scene; `dump_state` returns the room's
  // unfiltered server-side state. Always returns the reply to send.
  net::Json OnControl(const net::Json& message);

  size_t RoomCount() const { return rooms_.size(); }

 private:
  struct Room {
    int conns[2] = {-1, -1};  // Indexed by Team.
    std::vector<int> spectators;  // Read-only, unfiltered viewers.
    std::unique_ptr<GameSession> session;
    std::optional<Scene> scene;  // Set by load_scenario.
    net::Json sceneSpec;         // The same scene, sent to clients in `start`.
    bool friendlyFire = true;    // Test knobs from load_scenario.
    std::vector<float> shotRolls;
  };
  struct Member {
    std::string room;
    Team team = Team::Blue;
    bool spectator = false;
  };

  std::string NewRoomCode();  // A code no existing room uses.
  void Join(int conn, const net::Json& message);
  void HandleAction(int conn, Member& member, const net::Json& message);
  void StartMatch(Room& room);
  void SpectatorJoin(int conn, const net::Json& message);
  void SendSpectatorView(int conn, const Room& room);  // start + plans + state.
  void Send(int conn, const net::Json& message) { send_(conn, message.Dump()); }
  void SendError(int conn, const std::string& why);

  SendFn send_;
  SeedFn seed_;
  std::map<std::string, Room> rooms_;
  std::map<int, Member> members_;
  std::optional<std::mt19937> codeRng_;
};

}  // namespace tactics::server
