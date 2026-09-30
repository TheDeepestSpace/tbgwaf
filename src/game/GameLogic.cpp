#include "game/GameLogic.h"

#include <cmath>
#include <sstream>

#include "game/Raycast.h"

namespace tactics {

void GameLogic::Reset() { Reset(BuildDefaultScene()); }

void GameLogic::Reset(Scene scene) {
  scene_ = std::move(scene);
  obstacleBounds_ = ObstacleBounds(scene_.obstacles);
  navMesh_.Build(scene_.obstacles, constants::kMapHalfExtent, constants::kAgentRadius);
  turnManager_.StartRound(scene_.units);

  mode_ = InputMode::AwaitingSelection;
  selectedUnitId_.reset();
  winner_.reset();
  movePreviewPath_.clear();
  movePreviewValid_ = false;
  moveAnimPath_.clear();
  moveAnimSegment_ = 0;
}

GameSnapshot GameLogic::ExportState() const {
  GameSnapshot snap;
  for (const auto& unit : scene_.units) {
    snap.units.push_back({unit.id, unit.position, unit.facingYaw, unit.alive, unit.triggerAction});
  }
  snap.turn = turnManager_.GetState();
  snap.mode = mode_;
  snap.selectedUnitId = selectedUnitId_.value_or(-1);
  snap.winner = winner_ ? static_cast<int>(*winner_) : -1;
  return snap;
}

bool GameLogic::ImportState(const GameSnapshot& snap) {
  if (snap.units.size() != scene_.units.size()) return false;
  for (const auto& u : snap.units) {
    if (!FindUnit(u.id)) return false;
  }
  for (const auto& u : snap.units) {
    Unit* unit = FindUnit(u.id);
    unit->position = u.position;
    unit->facingYaw = u.facingYaw;
    unit->alive = u.alive;
    unit->triggerAction = u.triggerAction;
  }
  turnManager_.SetState(snap.turn);
  mode_ = snap.mode;
  if (snap.selectedUnitId >= 0) {
    selectedUnitId_ = snap.selectedUnitId;
  } else {
    selectedUnitId_.reset();
  }
  if (snap.winner >= 0) {
    winner_ = static_cast<Team>(snap.winner);
  } else {
    winner_.reset();
  }
  movePreviewPath_.clear();
  movePreviewValid_ = false;
  moveAnimPath_.clear();
  moveAnimSegment_ = 0;
  return true;
}

std::string SerializeSnapshot(const GameSnapshot& snap) {
  std::ostringstream out;
  out.precision(9);
  out << static_cast<int>(snap.mode) << ' ' << snap.selectedUnitId << ' ' << snap.winner << ' '
      << snap.turn.roundNumber << ' ' << snap.turn.cursor << ' ' << snap.turn.order.size();
  for (int id : snap.turn.order) out << ' ' << id;
  out << ' ' << snap.units.size();
  for (const auto& u : snap.units) {
    out << ' ' << u.id << ' ' << u.position.x << ' ' << u.position.y << ' ' << u.position.z << ' '
        << u.facingYaw << ' ' << (u.alive ? 1 : 0) << ' ' << static_cast<int>(u.triggerAction);
  }
  return out.str();
}

bool DeserializeSnapshot(const std::string& text, GameSnapshot* outSnap) {
  std::istringstream in(text);
  GameSnapshot snap;
  int mode = 0;
  size_t orderCount = 0, unitCount = 0;
  if (!(in >> mode >> snap.selectedUnitId >> snap.winner >> snap.turn.roundNumber >>
        snap.turn.cursor >> orderCount)) {
    return false;
  }
  if (mode < 0 || mode > static_cast<int>(InputMode::GameOver)) return false;
  if (snap.winner < -1 || snap.winner > 1) return false;
  constexpr size_t kMaxEntries = 1024;
  if (orderCount > kMaxEntries) return false;
  snap.mode = static_cast<InputMode>(mode);
  snap.turn.order.resize(orderCount);
  for (int& id : snap.turn.order) {
    if (!(in >> id)) return false;
  }
  if (!(in >> unitCount) || unitCount > kMaxEntries) return false;
  snap.units.resize(unitCount);
  for (auto& u : snap.units) {
    int alive = 0, trigger = 0;
    if (!(in >> u.id >> u.position.x >> u.position.y >> u.position.z >> u.facingYaw >> alive >>
          trigger)) {
      return false;
    }
    if (trigger < 0 || trigger > static_cast<int>(TriggerAction::Shoot)) return false;
    u.alive = alive != 0;
    u.triggerAction = static_cast<TriggerAction>(trigger);
  }
  *outSnap = std::move(snap);
  return true;
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

  // Animate rather than teleport: hand the resolved path off to Update(),
  // which walks the mover along it at constant speed and completes the
  // action once the path is consumed. NavMesh::FindPath always returns at
  // least [start, goal] on success.
  moveAnimPath_ = std::move(path);
  moveAnimSegment_ = 0;
  mode_ = InputMode::Moving;
  movePreviewPath_.clear();
  movePreviewValid_ = false;
}

void GameLogic::Update(float dtSeconds) {
  if (mode_ != InputMode::Moving) return;
  Unit* mover = FindUnit(selectedUnitId_.value_or(-1));
  if (!mover || moveAnimPath_.size() < 2) {
    moveAnimPath_.clear();
    CompleteAction();
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

    if (TriggerOverwatch(*mover)) {
      moveAnimPath_.clear();
      CompleteAction();
      return;
    }
  }

  if (moveAnimSegment_ + 1 >= moveAnimPath_.size()) {
    moveAnimPath_.clear();
    CompleteAction();
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
  CompleteAction();
}

void GameLogic::ChooseOverwatch() {
  if (mode_ != InputMode::ActionMenu) return;
  Unit* unit = FindUnit(selectedUnitId_.value_or(-1));
  if (!unit) return;
  unit->triggerAction = TriggerAction::Shoot;
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
