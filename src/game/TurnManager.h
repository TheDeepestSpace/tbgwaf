#pragma once

#include <optional>
#include <vector>

#include "game/Types.h"
#include "game/Unit.h"

namespace tactics {

// Drives the strict, alternating turn order described in the Stage-A spec:
// exactly one figure acts at a time, action order alternates Blue/Red as
// far as each team's remaining figures allow, and a new round begins once
// every living figure has acted once.
class TurnManager {
 public:
  // (Re)builds the round order from the currently alive units, interleaved
  // by team (Blue0, Red0, Blue1, Red1, ...). Call once at game start and
  // again whenever a new round needs to begin.
  void StartRound(const std::vector<Unit>& units);

  // Unit id whose turn it currently is, or std::nullopt if no living units
  // remain (shouldn't happen once a winner is declared).
  std::optional<int> CurrentActorId(const std::vector<Unit>& units) const;

  // Call after the current actor has finished its action (move/shoot/pass).
  // Advances to the next living actor, starting a new round automatically
  // when the current round is exhausted.
  void AdvanceTurn(const std::vector<Unit>& units);

  int RoundNumber() const { return roundNumber_; }

 private:
  void SkipDead(const std::vector<Unit>& units);

  std::vector<int> order_;
  size_t cursor_ = 0;
  int roundNumber_ = 0;
};

// Returns the winning team if exactly one side has living units, or
// std::nullopt if the match is still undecided (both sides have survivors;
// this does not happen once StartRound/AdvanceTurn have been driven, since
// the game should stop advancing turns once a side is eliminated).
std::optional<Team> CheckWinner(const std::vector<Unit>& units);

}  // namespace tactics
