#pragma once

#include <optional>
#include <vector>

#include "game/Types.h"
#include "game/Unit.h"

namespace tactics {

// Drives the plan-then-commit round structure: one "turn" is a whole team
// planning and then committing actions for every one of its living figures,
// not a single figure acting. Turns alternate Blue/Red; a new round begins
// once both teams have committed a turn.
class TurnManager {
 public:
  // (Re)starts the round at Blue's turn. Call once at game start.
  void StartRound();

  // The team whose planning/commit phase it currently is.
  Team CurrentTeam() const { return currentTeam_; }

  // Call after the current team's committed turn has fully resolved.
  // Switches to the other team, starting a new round once Blue is back up.
  void AdvanceTurn();

  int RoundNumber() const { return roundNumber_; }

 private:
  Team currentTeam_ = Team::Blue;
  int roundNumber_ = 0;
};

// Returns the winning team if exactly one side has living units, or
// std::nullopt if the match is still undecided (both sides have survivors;
// this does not happen once StartRound/AdvanceTurn have been driven, since
// the game should stop advancing turns once a side is eliminated).
std::optional<Team> CheckWinner(const std::vector<Unit>& units);

}  // namespace tactics
