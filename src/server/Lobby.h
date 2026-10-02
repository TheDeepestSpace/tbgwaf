#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "server/GameSession.h"

namespace tactics::server {

// Pairs connections into games: "a session is just two sockets, one game".
// The first connection to join a room waits (Blue); the second is paired
// (Red) and the authoritative GameSession starts. Transport-agnostic: the
// socket layer feeds it messages and supplies a send callback.
class Lobby {
 public:
  using SendFn = std::function<void(int conn, const std::string& text)>;
  using SeedFn = std::function<uint32_t()>;
  Lobby(SendFn send, SeedFn seed) : send_(std::move(send)), seed_(std::move(seed)) {}

  // One inbound text message from `conn`. Never throws on bad input.
  void OnMessage(int conn, const std::string& text);
  void OnDisconnect(int conn);

  size_t RoomCount() const { return rooms_.size(); }

 private:
  struct Room {
    int conns[2] = {-1, -1};  // Indexed by Team.
    std::unique_ptr<GameSession> session;
  };
  struct Member {
    std::string room;
    Team team = Team::Blue;
  };

  void Join(int conn, const net::Json& message);
  void HandleAction(int conn, Member& member, const net::Json& message);
  void StartMatch(Room& room);
  void Send(int conn, const net::Json& message) { send_(conn, message.Dump()); }
  void SendError(int conn, const std::string& why);

  SendFn send_;
  SeedFn seed_;
  std::map<std::string, Room> rooms_;
  std::map<int, Member> members_;
};

}  // namespace tactics::server
