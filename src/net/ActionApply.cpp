#include "net/ActionApply.h"

#include <cmath>

namespace tactics::net {

using tactics::GameLogic;
using tactics::InputMode;
using tactics::PlannedAction;
using tactics::PlannedActionType;
using tactics::Team;
using tactics::Unit;

namespace {

// Leaves the click-flow state machine idle (no selection, no pending target).
void ResetInteraction(GameLogic& game) {
  for (int i = 0; i < 3 && game.Mode() != InputMode::AwaitingSelection &&
                  game.Mode() != InputMode::Executing && game.Mode() != InputMode::GameOver;
       ++i) {
    game.CancelAction();
  }
}

constexpr float kWaypointTolerance = 0.05f;  // Wire rounding slack.

}  // namespace

bool ApplyPlanAction(GameLogic& game, Team team, const Action& action, std::string* error) {
  auto Fail = [&](const char* why) {
    *error = why;
    return false;
  };
  if (action.kind == ActionKind::Commit || action.kind == ActionKind::NewMatch) {
    return Fail("not a planning action");
  }
  if (game.Mode() == InputMode::Executing || game.Mode() == InputMode::GameOver) {
    return Fail("planning is closed");
  }
  Unit* unit = game.FindUnit(action.unit);
  if (!unit) return Fail("no such unit");
  if (unit->team != team) return Fail("not your unit");
  if (!unit->alive) return Fail("unit is dead");

  if (action.kind == ActionKind::Focus) return true;
  if (action.kind == ActionKind::Reaction) {
    unit->reactionOnStationary = action.rule;
    return true;
  }
  if (action.kind == ActionKind::Cancel) {
    unit->plan = PlannedAction{};
    return true;
  }

  ResetInteraction(game);
  const PlannedAction saved = unit->plan;
  auto Rollback = [&](const char* why) {
    ResetInteraction(game);
    unit->plan = saved;
    return Fail(why);
  };

  game.ClickUnit(unit->id, team);
  if (game.SelectedUnitId() != unit->id || game.Mode() != InputMode::ActionMenu) {
    return Rollback("could not select unit");
  }

  switch (action.kind) {
    case ActionKind::Pass:
      game.ChoosePass();
      return true;
    case ActionKind::Overwatch:
      game.ChooseOverwatch();
      return true;
    case ActionKind::Shoot: {
      game.ChooseShoot();
      game.ClickUnit(action.target, team);
      if (game.Mode() != InputMode::AwaitingSelection) {
        return Rollback("invalid shot target (not an enemy in your team's view)");
      }
      return true;
    }
    case ActionKind::Move: {
      game.ChooseMove();
      for (const glm::vec3& waypoint : action.waypoints) {
        game.ClickGround(waypoint, team);
        if (glm::distance(game.MoveChainEnd(), waypoint) > kWaypointTolerance) {
          return Rollback("waypoint unreachable (no path, or beyond one round's move budget)");
        }
      }
      game.FinishMovePlan();
      if (game.Mode() != InputMode::AwaitingSelection) return Rollback("move was not accepted");
      if (action.facing) game.SetPlannedMoveFacing(unit->id, *action.facing, team);
      return true;
    }
    default:
      return Rollback("unsupported action");
  }
}

Action PlanToAction(const Unit& unit) {
  Action a;
  a.unit = unit.id;
  switch (unit.plan.type) {
    case PlannedActionType::Move:
      a.kind = ActionKind::Move;
      if (!unit.plan.movePath.empty()) a.waypoints.push_back(unit.plan.movePath.back());
      for (const auto& leg : unit.plan.queuedLegs) {
        if (!leg.empty()) a.waypoints.push_back(leg.back());
      }
      a.facing = unit.plan.endFacingYaw;
      if (a.waypoints.empty()) a.kind = ActionKind::Cancel;
      break;
    case PlannedActionType::Shoot:
      a.kind = ActionKind::Shoot;
      a.target = unit.plan.shootTargetId;
      break;
    case PlannedActionType::Pass: a.kind = ActionKind::Pass; break;
    case PlannedActionType::Overwatch: a.kind = ActionKind::Overwatch; break;
    case PlannedActionType::None: a.kind = ActionKind::Cancel; break;
  }
  return a;
}

}  // namespace tactics::net
