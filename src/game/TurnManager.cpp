#include "game/TurnManager.h"

namespace tactics {

void TurnManager::StartRound() {
  currentTeam_ = Team::Blue;
  ++roundNumber_;
}

void TurnManager::AdvanceTurn() {
  if (currentTeam_ == Team::Blue) {
    currentTeam_ = Team::Red;
  } else {
    currentTeam_ = Team::Blue;
    ++roundNumber_;
  }
}

std::optional<Team> CheckWinner(const std::vector<Unit>& units) {
  bool blueAlive = false, redAlive = false;
  for (const auto& unit : units) {
    if (!unit.alive) continue;
    if (unit.team == Team::Blue) blueAlive = true;
    if (unit.team == Team::Red) redAlive = true;
  }
  if (blueAlive && !redAlive) return Team::Blue;
  if (redAlive && !blueAlive) return Team::Red;
  return std::nullopt;
}

}  // namespace tactics
