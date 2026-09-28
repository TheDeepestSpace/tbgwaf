#include "game/TurnManager.h"

namespace tactics {

void TurnManager::StartRound(const std::vector<Unit>& units) {
  std::vector<int> blueIds, redIds;
  for (const auto& unit : units) {
    if (!unit.alive) continue;
    (unit.team == Team::Blue ? blueIds : redIds).push_back(unit.id);
  }

  order_.clear();
  const size_t maxCount = std::max(blueIds.size(), redIds.size());
  for (size_t i = 0; i < maxCount; ++i) {
    if (i < blueIds.size()) order_.push_back(blueIds[i]);
    if (i < redIds.size()) order_.push_back(redIds[i]);
  }

  cursor_ = 0;
  ++roundNumber_;
  SkipDead(units);
}

void TurnManager::SkipDead(const std::vector<Unit>& units) {
  auto isAlive = [&](int id) {
    for (const auto& unit : units) {
      if (unit.id == id) return unit.alive;
    }
    return false;
  };
  while (cursor_ < order_.size() && !isAlive(order_[cursor_])) {
    ++cursor_;
  }
}

std::optional<int> TurnManager::CurrentActorId(const std::vector<Unit>& units) const {
  TurnManager copy = *this;
  copy.SkipDead(units);
  if (copy.cursor_ >= copy.order_.size()) return std::nullopt;
  return copy.order_[copy.cursor_];
}

void TurnManager::AdvanceTurn(const std::vector<Unit>& units) {
  if (cursor_ < order_.size()) ++cursor_;
  SkipDead(units);
  if (cursor_ >= order_.size()) {
    StartRound(units);
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
