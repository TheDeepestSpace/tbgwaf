#include "game/GameLogic.h"

#include <algorithm>
#include <cmath>

#include "game/Raycast.h"

namespace tactics {

namespace {

float PathLength(const std::vector<glm::vec3>& path) {
  float length = 0.0f;
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    length += glm::distance(path[i], path[i + 1]);
  }
  return length;
}

}  // namespace

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

void GameLogic::Reset() { Reset(BuildDefaultScene()); }

void GameLogic::Reset(Scene scene) {
  scene_ = std::move(scene);
  obstacleBounds_ = ObstacleBounds(scene_.obstacles);
  navMesh_.Build(scene_.obstacles, constants::kMapHalfExtent, constants::kAgentRadius);
  roundNumber_ = 1;

  mode_ = InputMode::AwaitingSelection;
  selectedUnitId_.reset();
  winner_.reset();
  movePreviewPath_.clear();
  movePreviewValid_ = false;
  activeMoves_.clear();
  pendingShots_.clear();
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

bool GameLogic::CanCommitRound() const {
  if (mode_ == InputMode::Executing || mode_ == InputMode::GameOver) return false;
  bool anyLiving = false;
  for (const auto& unit : scene_.units) {
    if (!unit.alive) continue;
    anyLiving = true;
    if (unit.plan.type == PlannedActionType::None) return false;
  }
  return anyLiving;
}

void GameLogic::ClickUnit(int unitId, Team byTeam) {
  if (winner_) return;
  if (mode_ == InputMode::Executing) return;  // The round is animating; input is inert.
  Unit* unit = FindUnit(unitId);
  if (!unit || !unit->alive) return;

  if (mode_ == InputMode::AwaitingSelection) {
    // Both teams plan concurrently, but a player only ever plans their own
    // side: a click coming from `byTeam`'s pane can only (re)select a living
    // figure on that team, regardless of whether it already has a plan.
    if (unit->team != byTeam) return;
    selectedUnitId_ = unitId;
    mode_ = InputMode::ActionMenu;
    return;
  }

  if (mode_ == InputMode::AwaitingShootTarget) {
    Unit* shooter = FindUnit(selectedUnitId_.value_or(-1));
    if (!shooter) return;
    if (shooter->team != byTeam) return;  // Only the shooter's own side aims its shot.
    if (unit->team == shooter->team) return;  // Can only shoot enemies.
    // Stage-B fog-of-war: a figure outside the shooter's team's current
    // combined FOV isn't a valid target at all (the click is a no-op, not a
    // guaranteed miss) -- distinct from an in-FOV shot that misses due to
    // the shooter's own cone/LOS once the round executes.
    if (!ComputeVisibility(shooter->team).UnitVisible(unit->id)) return;
    shooter->plan.type = PlannedActionType::Shoot;
    shooter->plan.shootTargetId = unit->id;
    shooter->plan.movePath.clear();
    selectedUnitId_.reset();
    mode_ = InputMode::AwaitingSelection;
  }
}

void GameLogic::ClickGround(const glm::vec3& point, Team byTeam) {
  if (mode_ != InputMode::AwaitingMoveDestination) return;
  Unit* mover = FindUnit(selectedUnitId_.value_or(-1));
  if (!mover || mover->team != byTeam) return;

  std::vector<glm::vec3> path;
  if (!navMesh_.FindPath(mover->position, point, &path)) return;
  // The round executes over a fixed window, so a figure can only plan as far
  // as it can actually run in that time.
  if (PathLength(path) > mover->MoveBudget()) return;

  // Plan only: NavMesh::FindPath always returns at least [start, goal] on
  // success. Nothing moves until this plan is executed by CommitRound().
  mover->plan.type = PlannedActionType::Move;
  mover->plan.movePath = std::move(path);
  mover->plan.shootTargetId = -1;
  selectedUnitId_.reset();
  mode_ = InputMode::AwaitingSelection;
  movePreviewPath_.clear();
  movePreviewValid_ = false;
}

void GameLogic::Update(float dtSeconds) {
  // Knockdowns advance regardless of mode: they're purely visual and can
  // outlast the round that caused them.
  for (Unit& unit : scene_.units) {
    if (unit.alive || unit.knockdownElapsed < 0.0f) continue;
    unit.knockdownElapsed =
        std::min(unit.knockdownElapsed + dtSeconds, constants::kKnockdownDuration);
  }

  if (mode_ != InputMode::Executing) return;

  for (ActiveMove& move : activeMoves_) {
    Unit* mover = FindUnit(move.unitId);
    if (!mover || move.path.size() < 2) continue;
    if (!mover->alive) {
      // Killed mid-round (pending shot or overwatch): the move stops where
      // the figure fell.
      move.segment = move.path.size();
      continue;
    }

    float remaining = dtSeconds * mover->runSpeed;
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
      // which keep animating their own planned moves this round.
      move.segment = move.path.size();
    }
  }

  // Continuous shot resolution: with everyone's position advanced for this
  // tick, held shots get their per-tick FOV/LOS re-check -- this is what
  // lets a shooter hit a target that only walks into its cone mid-round.
  ResolvePendingShots();

  // Every mover advances together above; drop whichever ones just finished
  // their path (or died to a shot this tick), and once none are left the
  // whole round is done -- any still-pending shot's geometry can no longer
  // change, so it expires.
  activeMoves_.erase(std::remove_if(activeMoves_.begin(), activeMoves_.end(),
                                     [this](const ActiveMove& move) {
                                       const Unit* mover = FindUnit(move.unitId);
                                       return !mover || !mover->alive ||
                                              move.path.size() < 2 ||
                                              move.segment + 1 >= move.path.size();
                                     }),
                      activeMoves_.end());

  if (activeMoves_.empty()) FinishRound();
}

void GameLogic::HoverGround(const glm::vec3& point, Team byTeam) {
  movePreviewPath_.clear();
  movePreviewValid_ = false;
  if (mode_ != InputMode::AwaitingMoveDestination) return;
  const Unit* mover = FindUnit(selectedUnitId_.value_or(-1));
  if (!mover || mover->team != byTeam) return;
  movePreviewValid_ = navMesh_.FindPath(mover->position, point, &movePreviewPath_) &&
                      PathLength(movePreviewPath_) <= mover->MoveBudget();
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

bool GameLogic::ShotConnects(const Unit& shooter, const Unit& target) const {
  return InFovCone(shooter.EyePosition(), shooter.FacingDirection(), target.EyePosition(),
                   constants::kShootHalfFovDegrees, constants::kShootRange) &&
         LineOfSightClear(shooter.EyePosition(), target.EyePosition(), obstacleBounds_);
}

bool GameLogic::ResolveShot(Unit& shooter, Unit& target) {
  const bool hit = ShotConnects(shooter, target);
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

void GameLogic::ResolvePendingShots() {
  // Judge every held shot against the same snapshot (positions don't change
  // during resolution, and hits are applied only after all are judged), so
  // two figures whose shots connect on the same tick both fire: a mutual
  // kill downs both, rather than whichever happens to resolve first
  // silencing the other.
  std::vector<std::pair<Unit*, Unit*>> firing;
  for (const PendingShot& shot : pendingShots_) {
    Unit* shooter = FindUnit(shot.shooterId);
    Unit* target = FindUnit(shot.targetId);
    if (!shooter || !target || !shooter->alive || !target->alive) continue;
    if (ShotConnects(*shooter, *target)) firing.emplace_back(shooter, target);
  }
  for (auto& [shooter, target] : firing) ResolveShot(*shooter, *target);

  // Drop everything that fired or can no longer fire (dead shooter holds
  // its fire from here on; a downed target stops being worth a bullet).
  pendingShots_.erase(
      std::remove_if(pendingShots_.begin(), pendingShots_.end(),
                     [this](const PendingShot& shot) {
                       const Unit* shooter = FindUnit(shot.shooterId);
                       const Unit* target = FindUnit(shot.targetId);
                       return !shooter || !target || !shooter->alive || !target->alive;
                     }),
      pendingShots_.end());
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

void GameLogic::CommitRound() {
  if (!CanCommitRound()) return;

  selectedUnitId_.reset();
  movePreviewPath_.clear();
  movePreviewValid_ = false;

  activeMoves_.clear();
  pendingShots_.clear();
  for (auto& unit : scene_.units) {
    if (!unit.alive) continue;
    const PlannedAction plan = unit.plan;
    unit.plan = PlannedAction{};

    if (plan.type == PlannedActionType::Move) {
      activeMoves_.push_back(ActiveMove{unit.id, plan.movePath, 0});
    } else if (plan.type == PlannedActionType::Shoot) {
      pendingShots_.push_back(PendingShot{unit.id, plan.shootTargetId});
    } else if (plan.type == PlannedActionType::Overwatch) {
      unit.triggerAction = TriggerAction::Shoot;
    }
  }

  mode_ = InputMode::Executing;

  // Tick 0: shots whose FOV/LOS is already valid at the pre-move positions
  // fire the instant the round starts (simultaneously, snapshot-judged);
  // blocked ones stay pending and re-check as the round's movement unfolds.
  ResolvePendingShots();

  // Anyone killed at tick 0 never starts walking.
  activeMoves_.erase(std::remove_if(activeMoves_.begin(), activeMoves_.end(),
                                     [this](const ActiveMove& move) {
                                       const Unit* mover = FindUnit(move.unitId);
                                       return !mover || !mover->alive;
                                     }),
                      activeMoves_.end());

  // With no movement in flight, nothing can change a still-blocked shot's
  // geometry: the round is already over.
  if (activeMoves_.empty()) FinishRound();
}

void GameLogic::FinishRound() {
  selectedUnitId_.reset();
  activeMoves_.clear();
  pendingShots_.clear();
  mode_ = InputMode::AwaitingSelection;

  const auto winner = CheckWinner(scene_.units);
  bool anyAlive = false;
  for (const auto& unit : scene_.units) anyAlive |= unit.alive;
  if (winner || !anyAlive) {
    // Simultaneous execution can wipe out both sides in the same round;
    // that's a draw (GameOver with no winner).
    winner_ = winner;
    mode_ = InputMode::GameOver;
    return;
  }
  ++roundNumber_;
}

}  // namespace tactics
