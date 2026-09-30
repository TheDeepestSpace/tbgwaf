#include "game/GameLogic.h"

#include <algorithm>
#include <cmath>

#include "game/Raycast.h"

namespace tactics {

void GameLogic::Reset() { Reset(BuildDefaultScene()); }

void GameLogic::Reset(Scene scene) {
  scene_ = std::move(scene);
  obstacleBounds_ = ObstacleBounds(scene_.obstacles);
  navMesh_.Build(scene_.obstacles, constants::kMapHalfExtent, constants::kAgentRadius);
  turnManager_.StartRound();

  mode_ = InputMode::AwaitingSelection;
  selectedUnitId_.reset();
  winner_.reset();
  movePreviewPath_.clear();
  movePreviewValid_ = false;
  activeMoves_.clear();
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

bool GameLogic::CanCommitTurn() const {
  bool anyLiving = false;
  for (const auto& unit : scene_.units) {
    if (!unit.alive || unit.team != turnManager_.CurrentTeam()) continue;
    anyLiving = true;
    if (unit.plan.type == PlannedActionType::None) return false;
  }
  return anyLiving;
}

void GameLogic::ClickUnit(int unitId) {
  if (winner_) return;
  if (mode_ == InputMode::Moving) return;  // A commit is animating; input is inert.
  Unit* unit = FindUnit(unitId);
  if (!unit || !unit->alive) return;

  if (mode_ == InputMode::AwaitingSelection) {
    // Any living figure on the currently acting team can be (re)selected to
    // set or revise its plan, regardless of whether it already has one.
    if (unit->team != turnManager_.CurrentTeam()) return;
    selectedUnitId_ = unitId;
    mode_ = InputMode::ActionMenu;
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
    shooter->plan.type = PlannedActionType::Shoot;
    shooter->plan.shootTargetId = unit->id;
    shooter->plan.movePath.clear();
    selectedUnitId_.reset();
    mode_ = InputMode::AwaitingSelection;
  }
}

void GameLogic::ClickGround(const glm::vec3& point) {
  if (mode_ != InputMode::AwaitingMoveDestination) return;
  Unit* mover = FindUnit(selectedUnitId_.value_or(-1));
  if (!mover) return;

  std::vector<glm::vec3> path;
  if (!navMesh_.FindPath(mover->position, point, &path)) return;

  // Plan only: NavMesh::FindPath always returns at least [start, goal] on
  // success. Nothing moves until this plan is executed by CommitTurn().
  mover->plan.type = PlannedActionType::Move;
  mover->plan.movePath = std::move(path);
  mover->plan.shootTargetId = -1;
  selectedUnitId_.reset();
  mode_ = InputMode::AwaitingSelection;
  movePreviewPath_.clear();
  movePreviewValid_ = false;
}

void GameLogic::Update(float dtSeconds) {
  // Knockdowns advance regardless of mode: an overwatch kill can happen
  // mid-enemy-turn while this team isn't the one animating.
  for (Unit& unit : scene_.units) {
    if (unit.alive || unit.knockdownElapsed < 0.0f) continue;
    unit.knockdownElapsed =
        std::min(unit.knockdownElapsed + dtSeconds, constants::kKnockdownDuration);
  }

  if (mode_ != InputMode::Moving) return;

  const float distance = dtSeconds * constants::kMoveSpeed;
  for (ActiveMove& move : activeMoves_) {
    Unit* mover = FindUnit(move.unitId);
    if (!mover || move.path.size() < 2) continue;

    float remaining = distance;
    while (remaining > 0.0f && move.segment + 1 < move.path.size()) {
      const glm::vec3& segStart = move.path[move.segment];
      const glm::vec3& segEnd = move.path[move.segment + 1];

      const glm::vec3 segDelta = segEnd - segStart;
      if (glm::length(glm::vec2(segDelta.x, segDelta.z)) > 1e-4f) {
        mover->facingYaw = std::atan2(segDelta.z, segDelta.x);
      }

      const glm::vec3 toEnd = segEnd - mover->position;
      const float distToEnd = glm::length(toEnd);
      if (distToEnd <= remaining) {
        mover->position = segEnd;
        remaining -= distToEnd;
        ++move.segment;
      } else {
        mover->position += (toEnd / distToEnd) * remaining;
        remaining = 0.0f;
      }
    }

    if (TriggerOverwatch(*mover)) {
      // Force this mover's removal below without disturbing the others,
      // which keep animating their own planned moves this commit.
      move.segment = move.path.size();
    }
  }

  // Every mover advances together above; drop whichever ones just finished
  // their path, and once none are left the whole commit is done.
  activeMoves_.erase(std::remove_if(activeMoves_.begin(), activeMoves_.end(),
                                     [](const ActiveMove& move) {
                                       return move.path.size() < 2 ||
                                              move.segment + 1 >= move.path.size();
                                     }),
                      activeMoves_.end());

  if (activeMoves_.empty()) FinishCommit();
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
  Unit* unit = FindUnit(selectedUnitId_.value_or(-1));
  if (!unit) return;
  unit->plan.type = PlannedActionType::Pass;
  unit->plan.movePath.clear();
  unit->plan.shootTargetId = -1;
  selectedUnitId_.reset();
  mode_ = InputMode::AwaitingSelection;
}

void GameLogic::ChooseOverwatch() {
  if (mode_ != InputMode::ActionMenu) return;
  Unit* unit = FindUnit(selectedUnitId_.value_or(-1));
  if (!unit) return;
  unit->plan.type = PlannedActionType::Overwatch;
  unit->plan.movePath.clear();
  unit->plan.shootTargetId = -1;
  selectedUnitId_.reset();
  mode_ = InputMode::AwaitingSelection;
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
  if (hit) {
    target.alive = false;
    glm::vec3 dir = target.position - shooter.position;
    dir.y = 0.0f;
    if (glm::length(dir) < 1e-4f) dir = shooter.FacingDirection();
    dir = glm::normalize(dir);
    // up x dir: tipping around this axis leans the figure toward dir.
    target.knockdownAxis = glm::vec3(dir.z, 0.0f, -dir.x);
    target.knockdownElapsed = 0.0f;
  }
  return hit;
}

bool GameLogic::IsUnitMoving(int unitId) const {
  for (const ActiveMove& move : activeMoves_) {
    if (move.unitId == unitId) return true;
  }
  return false;
}

bool GameLogic::TriggerOverwatch(Unit& mover) {
  for (auto& watcher : scene_.units) {
    if (!watcher.alive || watcher.team == mover.team) continue;
    if (watcher.triggerAction != TriggerAction::Shoot) continue;
    if (ResolveShot(watcher, mover)) {
      watcher.triggerAction = TriggerAction::None;
      return true;
    }
  }
  return false;
}

void GameLogic::CommitTurn() {
  if (!CanCommitTurn()) return;
  if (mode_ != InputMode::AwaitingSelection && mode_ != InputMode::ActionMenu &&
      mode_ != InputMode::AwaitingMoveDestination && mode_ != InputMode::AwaitingShootTarget) {
    return;
  }

  selectedUnitId_.reset();
  movePreviewPath_.clear();
  movePreviewValid_ = false;

  // Snapshot who's alive before any of this commit's shots resolve, so a
  // shot's outcome never depends on whether an ally's shot earlier in the
  // same commit already landed on the same target -- every planned shot is
  // judged against the same pre-commit world state.
  std::vector<bool> aliveAtCommit(scene_.units.size());
  for (const auto& unit : scene_.units) aliveAtCommit[unit.id] = unit.alive;

  const Team committingTeam = turnManager_.CurrentTeam();
  activeMoves_.clear();
  for (auto& unit : scene_.units) {
    if (!unit.alive || unit.team != committingTeam) continue;
    const PlannedAction plan = unit.plan;
    unit.plan = PlannedAction{};

    if (plan.type == PlannedActionType::Move) {
      activeMoves_.push_back(ActiveMove{unit.id, plan.movePath, 0});
    } else if (plan.type == PlannedActionType::Shoot) {
      Unit* target = FindUnit(plan.shootTargetId);
      if (target && aliveAtCommit[target->id]) {
        ResolveShot(unit, *target);
      }
    } else if (plan.type == PlannedActionType::Overwatch) {
      unit.triggerAction = TriggerAction::Shoot;
    }
  }

  if (activeMoves_.empty()) {
    FinishCommit();
  } else {
    mode_ = InputMode::Moving;  // Update() animates every planned move concurrently.
  }
}

void GameLogic::FinishCommit() {
  selectedUnitId_.reset();
  activeMoves_.clear();
  mode_ = InputMode::AwaitingSelection;

  const auto winner = CheckWinner(scene_.units);
  if (winner) {
    winner_ = winner;
    mode_ = InputMode::GameOver;
    return;
  }
  turnManager_.AdvanceTurn();
}

}  // namespace tactics
