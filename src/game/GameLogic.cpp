#include "game/GameLogic.h"

#include <cmath>

#include "game/Raycast.h"

namespace tactics {

void GameLogic::Reset() {
  scene_ = BuildDefaultScene();
  obstacleBounds_ = ObstacleBounds(scene_.obstacles);
  navMesh_.Build(scene_.obstacles, constants::kMapHalfExtent, constants::kAgentRadius);
  turnManager_.StartRound(scene_.units);

  mode_ = InputMode::AwaitingSelection;
  selectedUnitId_.reset();
  winner_.reset();
  movePreviewPath_.clear();
  movePreviewValid_ = false;
}

Unit* GameLogic::FindUnit(int id) {
  for (auto& unit : scene_.units) {
    if (unit.id == id) return &unit;
  }
  return nullptr;
}

const Unit* GameLogic::FindUnit(int id) const {
  for (const auto& unit : scene_.units) {
    if (unit.id == id) return &unit;
  }
  return nullptr;
}

void GameLogic::ClickUnit(int unitId) {
  if (winner_) return;
  Unit* unit = FindUnit(unitId);
  if (!unit || !unit->alive) return;

  if (mode_ == InputMode::AwaitingSelection) {
    const auto current = turnManager_.CurrentActorId(scene_.units);
    if (current && *current == unitId) {
      selectedUnitId_ = unitId;
      mode_ = InputMode::ActionMenu;
    }
    return;
  }

  if (mode_ == InputMode::AwaitingShootTarget) {
    Unit* shooter = FindUnit(selectedUnitId_.value_or(-1));
    if (!shooter) return;
    if (unit->team == shooter->team) return;  // Can only shoot enemies.
    // Stage-B fog-of-war: a figure outside the shooter's team's current
    // combined FOV isn't a valid target at all (the click is a no-op, not a
    // guaranteed miss) -- distinct from an in-FOV shot that misses due to
    // the shooter's own cone/LOS in ResolveShot below.
    if (!ComputeVisibility(shooter->team).UnitVisible(unit->id)) return;
    ResolveShot(*shooter, *unit);
    CompleteAction();
  }
}

void GameLogic::ClickGround(const glm::vec3& point) {
  if (mode_ != InputMode::AwaitingMoveDestination) return;
  Unit* mover = FindUnit(selectedUnitId_.value_or(-1));
  if (!mover) return;

  std::vector<glm::vec3> path;
  if (!navMesh_.FindPath(mover->position, point, &path)) return;

  const glm::vec3 origin = mover->position;
  mover->position = path.back();
  const glm::vec3 delta = mover->position - origin;
  if (glm::length(glm::vec2(delta.x, delta.z)) > 1e-4f) {
    mover->facingYaw = std::atan2(delta.z, delta.x);
  }
  CompleteAction();
}

void GameLogic::HoverGround(const glm::vec3& point) {
  movePreviewPath_.clear();
  movePreviewValid_ = false;
  if (mode_ != InputMode::AwaitingMoveDestination) return;
  const Unit* mover = FindUnit(selectedUnitId_.value_or(-1));
  if (!mover) return;
  movePreviewValid_ = navMesh_.FindPath(mover->position, point, &movePreviewPath_);
}

void GameLogic::ChooseMove() {
  if (mode_ != InputMode::ActionMenu) return;
  mode_ = InputMode::AwaitingMoveDestination;
  movePreviewPath_.clear();
  movePreviewValid_ = false;
}

void GameLogic::ChooseShoot() {
  if (mode_ != InputMode::ActionMenu) return;
  mode_ = InputMode::AwaitingShootTarget;
}

void GameLogic::ChoosePass() {
  if (mode_ != InputMode::ActionMenu) return;
  CompleteAction();
}

void GameLogic::CancelAction() {
  if (mode_ == InputMode::AwaitingMoveDestination || mode_ == InputMode::AwaitingShootTarget) {
    mode_ = InputMode::ActionMenu;
    movePreviewPath_.clear();
    movePreviewValid_ = false;
  } else if (mode_ == InputMode::ActionMenu) {
    mode_ = InputMode::AwaitingSelection;
    selectedUnitId_.reset();
  }
}

bool GameLogic::ResolveShot(Unit& shooter, Unit& target) {
  const bool hit =
      InFovCone(shooter.EyePosition(), shooter.FacingDirection(), target.EyePosition(),
                constants::kShootHalfFovDegrees, constants::kShootRange) &&
      LineOfSightClear(shooter.EyePosition(), target.EyePosition(), obstacleBounds_);
  if (hit) target.alive = false;
  return hit;
}

void GameLogic::CompleteAction() {
  selectedUnitId_.reset();
  mode_ = InputMode::AwaitingSelection;
  movePreviewPath_.clear();
  movePreviewValid_ = false;

  const auto winner = CheckWinner(scene_.units);
  if (winner) {
    winner_ = winner;
    mode_ = InputMode::GameOver;
    return;
  }
  turnManager_.AdvanceTurn(scene_.units);
}

}  // namespace tactics
