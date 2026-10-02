#pragma once

#include <array>
#include <cstdint>
#include <string>

#include "game/GameLogic.h"
#include "net/Json.h"
#include "net/Protocol.h"

namespace tactics::server {

// One authoritative match: a GameLogic instance plus the per-team "ready to
// commit" handshake. Transport-agnostic (no sockets), so it is unit-testable
// and the WebSocket layer stays a thin pipe.
class GameSession {
 public:
  explicit GameSession(uint32_t seed);

  uint32_t seed() const { return seed_; }
  const GameLogic& game() const { return game_; }

  struct Result {
    bool ok = true;
    std::string error;
  };
  // Validates and applies one client action on behalf of `team`. Plan edits
  // clear that team's ready flag; Commit marks it ready (all of the team's
  // living figures must have a plan); NewMatch restarts after game over with
  // `newSeed`.
  Result Apply(Team team, const net::Action& action, uint32_t newSeed = 0);

  bool Ready(Team team) const { return ready_[static_cast<int>(team)]; }
  bool BothReady() const { return Ready(Team::Blue) && Ready(Team::Red); }

  // Commits and fully simulates the round (call when BothReady()). Returns
  // one "round" message per team, indexed by Team: a fog-filtered timeline
  // (only what that team could see) plus the resulting rule state.
  std::array<net::Json, 2> RunRound();

  // Rule-state views for one team (fog-filtered; own plans included).
  net::Json StateMessage(Team team) const;
  net::Json PlansMessage(Team team) const;

 private:
  net::Json StateBody(Team team) const;

  uint32_t seed_;
  GameLogic game_;
  std::array<bool, 2> ready_{false, false};
};

constexpr float kSimStep = 1.0f / 30.0f;
constexpr int kFrameEvery = 3;  // Timeline sampled at 10 Hz.

}  // namespace tactics::server
