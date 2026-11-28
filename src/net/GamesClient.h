#pragma once

#include <string>
#include <vector>

namespace tactics::net {

// Client side of "My games": asks the server for this device's stored
// two-player matches (own token only) and keeps the list. Transport-agnostic
// like BuilderSceneSync: the host moves frames between TakeOutgoing()/
// OnMessage() and a short-lived WebSocket.
class GamesClient {
 public:
  struct Entry {
    std::string id;
    bool finished = false;
    std::string team;    // "blue" | "red"
    int round = 1;       // Next round to play (active) / rounds played + 1.
    std::string winner;  // Finished games: "blue" | "red" | "none".
    double updatedMs = 0;
  };

  explicit GamesClient(std::string token) : token_(std::move(token)) {}

  void OnOpen();   // Asks for the list.
  void OnClose() { open_ = false; }
  // True if `text` was the games list (then loaded() and entries() are fresh).
  bool OnMessage(const std::string& text);
  std::vector<std::string> TakeOutgoing();

  bool loaded() const { return loaded_; }
  const std::vector<Entry>& entries() const { return entries_; }
  const std::string& lastError() const { return error_; }
  // Most recently updated unfinished game, or nullptr.
  const Entry* LatestActive() const;
  // "Blue - round 4 - active" / "Red - finished, Blue won".
  static std::string Title(const Entry& entry);

  // A fresh device token: 32 hex digits from `a` and `b`.
  static std::string NewToken(unsigned long long a, unsigned long long b);

 private:
  std::string token_;
  bool open_ = false, loaded_ = false;
  std::vector<Entry> entries_;
  std::vector<std::string> outgoing_;
  std::string error_;
};

}  // namespace tactics::net
