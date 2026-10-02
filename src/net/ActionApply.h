#pragma once

#include <string>

#include "game/GameLogic.h"
#include "net/Protocol.h"

namespace tactics::net {

// Applies one planning action for `team` by driving the same team-tagged
// click/choose flow the interactive game and the scenario harness use, so
// every rule check lives in GameLogic. Atomic: on failure *error is set and
// the unit's previous plan is restored. Handles Move/Shoot/Pass/Overwatch/
// Cancel/Focus/Reaction; Commit/NewMatch are session-level and rejected here.
// The server runs this to validate untrusted input; the client runs it to
// replay the server's authoritative plans into its local mirror.
bool ApplyPlanAction(tactics::GameLogic& game, tactics::Team team, const Action& action,
                     std::string* error);

// The discrete action that reproduces `unit`'s current plan (Cancel if it has none).
Action PlanToAction(const tactics::Unit& unit);

}  // namespace tactics::net
