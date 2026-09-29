#include "game/GameLogic.h"

#include <cmath>

#include "game/Raycast.h"

namespace tactics {

void GameLogic::Reset() {
  scene_ = BuildDefaultScene();
  obstacleBounds_ = ObstacleBounds(scene_.obstacles);
  navMesh_.Build(scene_.obstacles, constants::kMapHalfExtent, constants::kAgentRadius);
  turnManager_.StartRound();

  mode_ = InputMode::AwaitingSelection;
  selectedUnitId_.reset();
  winner_.reset();
  movePreviewPath_.clear();
  movePreviewValid_ = false;
  moveAnimPath_.clear();
  moveAnimSegment_ = 0;
  commitOrder_.clear();
  commitIndex_ = 0;
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
  if (mode_ != InputMode::Moving) return;
  Unit* mover = FindUnit(selectedUnitId_.value_or(-1));
  if (!mover || moveAnimPath_.size() < 2) {
    moveAnimPath_.clear();
    ++commitIndex_;
    ContinueCommit();
    return;
  }

  float remaining = dtSeconds * constants::kMoveSpeed;
  while (remaining > 0.0f && moveAnimSegment_ + 1 < moveAnimPath_.size()) {
    const glm::vec3& segStart = moveAnimPath_[moveAnimSegment_];
    const glm::vec3& segEnd = moveAnimPath_[moveAnimSegment_ + 1];

    const glm::vec3 segDelta = segEnd - segStart;
    if (glm::length(glm::vec2(segDelta.x, segDelta.z)) > 1e-4f) {
      mover->facingYaw = std::atan2(segDelta.z, segDelta.x);
    }

    const glm::vec3 toEnd = segEnd - mover->position;
    const float distToEnd = glm::length(toEnd);
    if (distToEnd <= remaining) {
      mover->position = segEnd;
      remaining -= distToEnd;
      ++moveAnimSegment_;
    } else {
      mover->position += (toEnd / distToEnd) * remaining;
      remaining = 0.0f;
    }
  }

  if (moveAnimSegment_ + 1 >= moveAnimPath_.size()) {
    moveAnimPath_.clear();
    ++commitIndex_;
    ContinueCommit();
  }
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

void GameLogic::CommitTurn() {
  if (!CanCommitTurn()) return;
  if (mode_ != InputMode::AwaitingSelection && mode_ != InputMode::ActionMenu &&
      mode_ != InputMode::AwaitingMoveDestination && mode_ != InputMode::AwaitingShootTarget) {
    return;
  }

  selectedUnitId_.reset();
  movePreviewPath_.clear();
  movePreviewValid_ = false;

  commitOrder_.clear();
  const Team committingTeam = turnManager_.CurrentTeam();
  for (const auto& unit : scene_.units) {
    if (unit.alive && unit.team == committingTeam) commitOrder_.push_back(unit.id);
  }
  commitIndex_ = 0;
  mode_ = InputMode::AwaitingSelection;
  ContinueCommit();
}

void GameLogic::ContinueCommit() {
  while (commitIndex_ < commitOrder_.size()) {
    Unit* unit = FindUnit(commitOrder_[commitIndex_]);
    if (!unit || !unit->alive) {
      ++commitIndex_;
      continue;
    }

    const PlannedAction plan = unit->plan;
    unit->plan = PlannedAction{};

    if (plan.type == PlannedActionType::Move) {
      moveAnimPath_ = plan.movePath;
      moveAnimSegment_ = 0;
      selectedUnitId_ = unit->id;
      mode_ = InputMode::Moving;
      return;  // Update() drives the animation and resumes the commit.
    }

    if (plan.type == PlannedActionType::Shoot) {
      // A known PoC scope cut: actions in a commit resolve in squad order
      // against world state as it stands after each prior action, not
      // simultaneously. If an earlier action in this same commit already
      // killed the planned target, this shot simply has nothing to hit.
      Unit* target = FindUnit(plan.shootTargetId);
      if (target && target->alive) {
        ResolveShot(*unit, *target);
      }
    }

    ++commitIndex_;
  }

  FinishCommit();
}

void GameLogic::FinishCommit() {
  selectedUnitId_.reset();
  commitOrder_.clear();
  commitIndex_ = 0;
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
