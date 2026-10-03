#include "game/GameLogic.h"

#include <algorithm>
#include <cmath>
#include <sstream>

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

float FinalYaw(const std::vector<glm::vec3>& path, float fallback) {
  for (size_t i = path.size(); i-- > 1;) {
    const glm::vec3 delta = path[i] - path[i - 1];
    if (glm::length(glm::vec2(delta.x, delta.z)) > 1e-4f) return std::atan2(delta.z, delta.x);
  }
  return fallback;
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
  // On hilly terrain every figure stands on the sampled ground, so scenario
  // files can place units by XZ alone. Flat scenes (empty field) keep their
  // authored Y (e.g. crate-top starts).
  if (!scene_.ground.Empty()) {
    for (Unit& unit : scene_.units) {
      // Y=0 is the scenario shorthand for "place on terrain". Preserve an
      // explicitly authored elevated Y so a stacked surface above hilly
      // ground is not collapsed onto the heightfield during Reset/import.
      if (std::fabs(unit.position.y) < 1e-4f) {
        unit.position.y = scene_.ground.HeightAt(unit.position.x, unit.position.z);
      }
    }
  }
  navMesh_ = NavMesh();
  navMeshUnitId_ = -1;
  roundNumber_ = 1;

  mode_ = InputMode::AwaitingSelection;
  selectedUnitId_.reset();
  winner_.reset();
  movePreviewPath_.clear();
  movePreviewValid_ = false;
  activeMoves_.clear();
  pendingShots_.clear();
  mirroredMoving_.clear();

  const size_t unitSlots = scene_.units.size();
  for (int t = 0; t < 2; ++t) {
    sightings_[t].assign(unitSlots, {});
    sightedLastFrame_[t].assign(unitSlots, false);
    sightingTimer_[t].assign(unitSlots, 0.0f);
  }
  lastUnitPosition_.assign(unitSlots, glm::vec3(0.0f));
  hasLastUnitPosition_ = false;
  lastSightingRound_ = roundNumber_;
}

const std::vector<GameLogic::EnemySighting>& GameLogic::Sightings(Team viewingTeam,
                                                                  int targetUnitId) const {
  static const std::vector<EnemySighting> kEmpty;
  const auto& perUnit = sightings_[static_cast<int>(viewingTeam)];
  if (targetUnitId < 0 || static_cast<size_t>(targetUnitId) >= perUnit.size()) return kEmpty;
  return perUnit[targetUnitId];
}

void GameLogic::UpdateSightingMemory(float dtSeconds) {
  const int elapsedRounds = std::max(0, roundNumber_ - lastSightingRound_);
  lastSightingRound_ = roundNumber_;
  for (int t = 0; t < 2; ++t) {
    const Team viewer = static_cast<Team>(t);
    if (elapsedRounds > 0) {
      for (auto& list : sightings_[t]) {
        for (EnemySighting& s : list) s.ageRounds += elapsedRounds;
        list.erase(std::remove_if(list.begin(), list.end(),
                                  [](const EnemySighting& s) {
                                    return s.ageRounds >= constants::kSightingMemoryRounds;
                                  }),
                   list.end());
      }
    }

    const TeamVisibility visibility = ComputeVisibility(viewer);
    for (const Unit& unit : scene_.units) {
      if (unit.team == viewer || unit.id < 0 ||
          static_cast<size_t>(unit.id) >= sightings_[t].size()) {
        continue;
      }
      const bool visible = visibility.UnitVisible(unit.id);
      bool sample = false;
      if (visible && !sightedLastFrame_[t][unit.id]) {
        sample = true;  // Just entered FOV: record immediately.
        sightingTimer_[t][unit.id] = 0.0f;
      } else if (visible) {
        sightingTimer_[t][unit.id] += dtSeconds;
        if (sightingTimer_[t][unit.id] >= constants::kSightingSampleInterval) {
          sightingTimer_[t][unit.id] =
              std::fmod(sightingTimer_[t][unit.id], constants::kSightingSampleInterval);
          // Skip stationary figures: stacked identical ghosts brighten via blending.
          const auto& existing = sightings_[t][unit.id];
          sample = existing.empty();
          if (!sample) {
            glm::vec3 delta = unit.position - existing.back().position;
            delta.y = 0.0f;
            sample = glm::length(delta) > 1e-4f;
          }
        }
      }
      sightedLastFrame_[t][unit.id] = visible;
      if (!sample) continue;

      EnemySighting s;
      s.position = unit.position;
      s.facingYaw = unit.facingYaw;
      s.walkPhase = unit.walkPhase;
      s.walkBlend = unit.walkBlend;
      s.idleElapsed = unit.idleElapsed;
      if (hasLastUnitPosition_) {
        glm::vec3 delta = unit.position - lastUnitPosition_[unit.id];
        delta.y = 0.0f;
        if (glm::length(delta) > 1e-4f) s.moveDirection = glm::normalize(delta);
      }
      sightings_[t][unit.id].push_back(s);
    }
  }
  for (const Unit& unit : scene_.units) {
    if (unit.id >= 0 && static_cast<size_t>(unit.id) < lastUnitPosition_.size()) {
      lastUnitPosition_[unit.id] = unit.position;
    }
  }
  hasLastUnitPosition_ = true;
}

void GameLogic::EnsureNavMeshFor(const Unit& mover, const glm::vec3& origin) {
  if (navMeshUnitId_ == mover.id && navMeshOrigin_ == origin) return;

  // Any path of length <= MoveBudget stays within Euclidean distance
  // MoveBudget of the start, hence inside this window, so the window's
  // shortest path equals the global one whenever that one is affordable;
  // when the global shortest exceeds the budget, the windowed result can only
  // be longer or absent -- rejected by the budget check either way. So no
  // margin is needed for correctness; the small one keeps the goal-side
  // padded obstacle footprints from being clipped at the window edge and
  // absorbs float error.
  const float reach = mover.MoveBudget() + 2.0f * constants::kAgentRadius;
  const float half = scene_.mapHalfExtent;
  NavRegion region;
  region.xMin = std::max(-half, origin.x - reach);
  region.xMax = std::min(half, origin.x + reach);
  region.zMin = std::max(-half, origin.z - reach);
  region.zMax = std::min(half, origin.z + reach);
  navMesh_.Build(scene_.obstacles, region, constants::kAgentRadius, &scene_.ground,
                 &scene_.walkSurfaces);
  navMeshUnitId_ = mover.id;
  navMeshOrigin_ = origin;
}

GameSnapshot GameLogic::ExportState() const {
  GameSnapshot snap;
  for (const auto& unit : scene_.units) {
    GameSnapshot::UnitState u;
    u.id = unit.id;
    u.position = unit.position;
    u.facingYaw = unit.facingYaw;
    u.alive = unit.alive;
    u.planType = unit.plan.type;
    u.planShootTargetId = unit.plan.shootTargetId;
    u.planPath = unit.plan.movePath;
    u.planQueuedLegs = unit.plan.queuedLegs;
    u.planEndFacingYaw = unit.plan.endFacingYaw;
    u.knockdownAxis = unit.knockdownAxis;
    u.knockdownElapsed = unit.knockdownElapsed;
    u.walkPhase = unit.walkPhase;
    u.walkBlend = unit.walkBlend;
    u.idleElapsed = unit.idleElapsed;
    u.shootElapsed = unit.shootElapsed;
    u.shootAimYaw = unit.shootAimYaw;
    u.moving = IsUnitMoving(unit.id);
    snap.units.push_back(std::move(u));
  }
  snap.playbooks[0] = playbooks_[0];
  snap.playbooks[1] = playbooks_[1];
  snap.mode = mode_;
  snap.roundNumber = roundNumber_;
  snap.winner = winner_ ? static_cast<int>(*winner_) : -1;
  return snap;
}

namespace {

bool SnapshotMatchesUnits(const GameSnapshot& snap, const std::vector<Unit>& units) {
  if (snap.units.size() != units.size()) return false;
  for (const auto& u : snap.units) {
    bool found = false;
    for (const auto& unit : units) found |= unit.id == u.id;
    if (!found) return false;
  }
  return true;
}

void ApplyPlan(const GameSnapshot::UnitState& u, Unit* unit) {
  unit->plan = PlannedAction{};
  unit->plan.type = u.planType;
  unit->plan.shootTargetId = u.planShootTargetId;
  unit->plan.movePath = u.planPath;
  unit->plan.queuedLegs = u.planQueuedLegs;
  unit->plan.endFacingYaw = u.planEndFacingYaw;
}

}  // namespace

bool GameLogic::ImportState(const GameSnapshot& snap) {
  if (!SnapshotMatchesUnits(snap, scene_.units)) return false;
  mirroredMoving_.clear();
  for (const auto& u : snap.units) {
    Unit* unit = FindUnit(u.id);
    unit->position = u.position;
    unit->facingYaw = u.facingYaw;
    unit->alive = u.alive;
    unit->knockdownAxis = u.knockdownAxis;
    unit->knockdownElapsed = u.knockdownElapsed;
    unit->walkPhase = u.walkPhase;
    unit->walkBlend = u.walkBlend;
    unit->idleElapsed = u.idleElapsed;
    unit->shootElapsed = u.shootElapsed;
    unit->shootAimYaw = u.shootAimYaw;
    ApplyPlan(u, unit);
    if (u.moving) mirroredMoving_.push_back(u.id);
  }
  playbooks_[0] = snap.playbooks[0];
  playbooks_[1] = snap.playbooks[1];
  mode_ = snap.mode;
  roundNumber_ = snap.roundNumber;
  if (snap.winner >= 0) {
    winner_ = static_cast<Team>(snap.winner);
  } else {
    winner_.reset();
  }
  selectedUnitId_.reset();
  movePreviewPath_.clear();
  movePreviewValid_ = false;
  activeMoves_.clear();
  pendingShots_.clear();
  return true;
}

bool GameLogic::ImportTeamPlans(const GameSnapshot& snap, Team team) {
  if (!SnapshotMatchesUnits(snap, scene_.units)) return false;
  for (const auto& u : snap.units) {
    Unit* unit = FindUnit(u.id);
    if (unit->team != team) continue;
    ApplyPlan(u, unit);
  }
  return true;
}

std::string SerializeSnapshot(const GameSnapshot& snap) {
  std::ostringstream out;
  out.precision(9);
  out << static_cast<int>(snap.mode) << ' ' << snap.winner << ' ' << snap.roundNumber << ' '
      << snap.units.size();
  for (const auto& u : snap.units) {
    out << ' ' << u.id << ' ' << u.position.x << ' ' << u.position.y << ' ' << u.position.z << ' '
        << u.facingYaw << ' ' << (u.alive ? 1 : 0) << ' '
        << static_cast<int>(u.planType) << ' ' << u.planShootTargetId << ' ' << u.planEndFacingYaw << ' '
        << u.knockdownAxis.x << ' ' << u.knockdownAxis.y << ' ' << u.knockdownAxis.z << ' '
        << u.knockdownElapsed << ' ' << u.walkPhase << ' ' << u.walkBlend << ' '
        << u.idleElapsed << ' ' << u.shootElapsed << ' ' << u.shootAimYaw << ' '
        << (u.moving ? 1 : 0) << ' '
        << u.planPath.size();
    for (const auto& p : u.planPath) out << ' ' << p.x << ' ' << p.y << ' ' << p.z;
    out << ' ' << u.planQueuedLegs.size();
    for (const auto& leg : u.planQueuedLegs) {
      out << ' ' << leg.size();
      for (const auto& p : leg) out << ' ' << p.x << ' ' << p.y << ' ' << p.z;
    }
  }
  for (const auto& pb : snap.playbooks)
    for (int m = 0; m < 2; ++m)
      for (int s = 0; s < 2; ++s) out << ' ' << static_cast<int>(pb.table[m][s]);
  return out.str();
}

bool DeserializeSnapshot(const std::string& text, GameSnapshot* outSnap) {
  std::istringstream in(text);
  GameSnapshot snap;
  int mode = 0;
  size_t unitCount = 0;
  constexpr size_t kMaxEntries = 1024;
  if (!(in >> mode >> snap.winner >> snap.roundNumber >> unitCount)) return false;
  if (mode < 0 || mode > static_cast<int>(InputMode::GameOver)) return false;
  if (snap.winner < -1 || snap.winner > 1) return false;
  if (unitCount > kMaxEntries) return false;
  snap.mode = static_cast<InputMode>(mode);
  snap.units.resize(unitCount);
  for (auto& u : snap.units) {
    int alive = 0, plan = 0, moving = 0;
    size_t pathCount = 0;
    if (!(in >> u.id >> u.position.x >> u.position.y >> u.position.z >> u.facingYaw >> alive >>
          plan >> u.planShootTargetId >> u.planEndFacingYaw >> u.knockdownAxis.x >>
          u.knockdownAxis.y >> u.knockdownAxis.z >> u.knockdownElapsed >> u.walkPhase >>
          u.walkBlend >> u.idleElapsed >> u.shootElapsed >> u.shootAimYaw >> moving >>
          pathCount)) {
      return false;
    }
    if (plan < 0 || plan > static_cast<int>(PlannedActionType::Pass)) return false;
    if (pathCount > kMaxEntries) return false;
    u.planType = static_cast<PlannedActionType>(plan);
    u.alive = alive != 0;
    u.moving = moving != 0;
    u.planPath.resize(pathCount);
    for (auto& p : u.planPath) {
      if (!(in >> p.x >> p.y >> p.z)) return false;
    }
    size_t legCount = 0;
    if (!(in >> legCount) || legCount > kMaxEntries) return false;
    u.planQueuedLegs.resize(legCount);
    for (auto& leg : u.planQueuedLegs) {
      size_t legSize = 0;
      if (!(in >> legSize) || legSize > kMaxEntries) return false;
      leg.resize(legSize);
      for (auto& p : leg) {
        if (!(in >> p.x >> p.y >> p.z)) return false;
      }
    }
  }
  for (auto& pb : snap.playbooks) {
    for (int m = 0; m < 2; ++m) {
      for (int s = 0; s < 2; ++s) {
        int action = 0;
        if (!(in >> action)) return false;
        if (action < 0 || action > static_cast<int>(ReactionAction::ShootContinue)) return false;
        pb.table[m][s] = static_cast<ReactionAction>(action);
      }
    }
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
    shooter->plan.queuedLegs.clear();
    selectedUnitId_.reset();
    mode_ = InputMode::AwaitingSelection;
  }
}

glm::vec3 GameLogic::MoveChainEnd() const {
  const Unit* mover = FindUnit(selectedUnitId_.value_or(-1));
  if (!mover) return glm::vec3(0.0f);
  if (mover->plan.type != PlannedActionType::Move || mover->plan.movePath.empty()) {
    return mover->position;
  }
  if (!mover->plan.queuedLegs.empty()) return mover->plan.queuedLegs.back().back();
  return mover->plan.movePath.back();
}

void GameLogic::ClickGround(const glm::vec3& point, Team byTeam) {
  if (mode_ != InputMode::AwaitingMoveDestination) return;
  Unit* mover = FindUnit(selectedUnitId_.value_or(-1));
  if (!mover || mover->team != byTeam) return;

  EnsureNavMeshFor(*mover, MoveChainEnd());
  const bool chaining = mover->plan.type == PlannedActionType::Move && !mover->plan.movePath.empty();
  std::vector<glm::vec3> path;
  if (!navMesh_.FindPath(MoveChainEnd(), point, &path)) return;
  // The round executes over a fixed window, so each click (leg) can only
  // reach as far as the figure can run in one round.
  if (PathLength(path) > mover->MoveBudget()) return;

  // NavMesh::FindPath always returns at least [start, goal] on success.
  if (chaining) {
    mover->plan.queuedLegs.push_back(std::move(path));
  } else {
    // Default the final facing to the leg's last non-degenerate segment
    // direction (what the walk animation would leave the figure facing), so
    // an untouched ghost costs nothing extra.
    mover->plan.endFacingYaw = FinalYaw(path, mover->facingYaw);
    // Plan only: nothing moves until this plan is executed by CommitRound();
    // the facing stays adjustable via SetPlannedMoveFacing() until then.
    mover->plan.type = PlannedActionType::Move;
    mover->plan.movePath = std::move(path);
    mover->plan.queuedLegs.clear();
    mover->plan.shootTargetId = -1;
  }
  // Stay in this mode with the figure selected: the next click chains
  // another leg, FinishMovePlan() ends it.
  RefreshMoveFrontier();
  movePreviewPath_.clear();
  movePreviewValid_ = false;
}

void GameLogic::FinishMovePlan() {
  if (mode_ != InputMode::AwaitingMoveDestination) return;
  const Unit* mover = FindUnit(selectedUnitId_.value_or(-1));
  if (!mover || mover->plan.type != PlannedActionType::Move) return;
  selectedUnitId_.reset();
  moveFrontier_ = ReachField();
  movePreviewPath_.clear();
  movePreviewValid_ = false;
  mode_ = InputMode::AwaitingSelection;
}

void GameLogic::SetPlannedMoveFacing(int unitId, float yaw, Team byTeam) {
  if (mode_ == InputMode::Executing || mode_ == InputMode::GameOver) return;
  Unit* unit = FindUnit(unitId);
  if (!unit || !unit->alive || unit->team != byTeam) return;
  if (unit->plan.type != PlannedActionType::Move || unit->plan.movePath.empty()) return;
  unit->plan.endFacingYaw = std::remainder(yaw, 2.0f * 3.14159265358979f);
}

bool GameLogic::HasActiveKnockdown() const {
  for (const Unit& unit : scene_.units) {
    if (!unit.alive && unit.knockdownElapsed >= 0.0f &&
        unit.knockdownElapsed < constants::kKnockdownDuration) {
      return true;
    }
  }
  return false;
}

void GameLogic::Update(float dtSeconds) {
  // Knockdowns and the quick-draw beat advance regardless of mode: they're
  // purely visual and can outlast the round that caused them. A knockdown
  // holds its final pose; the shoot beat returns to idle (<0) once done.
  for (Unit& unit : scene_.units) {
    if (unit.alive) {
      unit.idleElapsed =
          std::fmod(unit.idleElapsed + dtSeconds, constants::kIdleAnimDuration);
    }
    if (!unit.alive && unit.knockdownElapsed >= 0.0f) {
      unit.knockdownElapsed =
          std::min(unit.knockdownElapsed + dtSeconds, constants::kKnockdownDuration);
    }
    if (unit.shootElapsed >= 0.0f) {
      unit.shootElapsed += dtSeconds;
      if (unit.shootElapsed >= constants::kShootAnimDuration) unit.shootElapsed = -1.0f;
    }
  }

  if (mode_ == InputMode::Executing) AdvanceExecutingRound(dtSeconds);

  // Walk-cycle blend, judged *after* the movement step so a move that
  // finished (or a mover that died) this very tick already starts easing
  // back to the rest pose. Exponential-decay lerp, same form as
  // OrbitCamera::Update's zoom damping: a large dt snaps straight to the
  // target, so a fast-forwarded round leaves everyone at rest.
  for (Unit& unit : scene_.units) {
    const float target = unit.alive && IsUnitMoving(unit.id) ? 1.0f : 0.0f;
    unit.walkBlend +=
        (target - unit.walkBlend) * std::min(1.0f, constants::kWalkBlendRate * dtSeconds);
  }
}

void GameLogic::AdvanceExecutingRound(float dtSeconds) {
  for (ActiveMove& move : activeMoves_) {
    Unit* mover = FindUnit(move.unitId);
    if (!mover || move.path.size() < 2) continue;
    if (!mover->alive) {
      // Killed mid-round (pending shot): the move stops where
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
      const float step = std::min(distToEnd, remaining);
      if (distToEnd <= remaining) {
        mover->position = segEnd;
        remaining -= distToEnd;
        ++move.segment;
      } else {
        mover->position += (toEnd / distToEnd) * remaining;
        remaining = 0.0f;
      }
      // Distance-driven walk cycle: feet stay in step with the ground
      // regardless of run speed or frame rate. Wrapped so the phase can't
      // grow without bound over a long match.
      constexpr float kTwoPi = 6.28318530717958647692f;
      mover->walkPhase =
          std::fmod(mover->walkPhase + step * (kTwoPi / constants::kWalkStrideLength), kTwoPi);
    }

    if (mover->alive && move.segment + 1 >= move.path.size()) {
      mover->facingYaw = move.endFacingYaw;
    }
  }

  ApplyPlaybookReactions();

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
  EnsureNavMeshFor(*mover, MoveChainEnd());
  movePreviewValid_ = navMesh_.FindPath(MoveChainEnd(), point, &movePreviewPath_) &&
                      PathLength(movePreviewPath_) <= mover->MoveBudget();
}

// Any manual touch of a figure's plan drops its queued multi-round route.
void GameLogic::ClearQueuedLegs(std::optional<int> unitId) {
  if (Unit* unit = FindUnit(unitId.value_or(-1))) unit->plan.queuedLegs.clear();
}

// Frontier of where the next leg can reach: built around the chain end.
void GameLogic::RefreshMoveFrontier() {
  moveFrontier_ = ReachField();
  if (const Unit* mover = FindUnit(selectedUnitId_.value_or(-1))) {
    const glm::vec3 origin = MoveChainEnd();
    EnsureNavMeshFor(*mover, origin);
    moveFrontier_ = navMesh_.ComputeReachField(origin, mover->MoveBudget());
  }
}

void GameLogic::ChooseMove() {
  if (mode_ != InputMode::ActionMenu) return;
  // Start a fresh chain: drop any earlier move plan.
  if (Unit* unit = FindUnit(selectedUnitId_.value_or(-1))) {
    if (unit->plan.type == PlannedActionType::Move) unit->plan = PlannedAction{};
  }
  mode_ = InputMode::AwaitingMoveDestination;
  RefreshMoveFrontier();
  movePreviewPath_.clear();
  movePreviewValid_ = false;
}

void GameLogic::ChooseShoot() {
  if (mode_ != InputMode::ActionMenu) return;
  ClearQueuedLegs(selectedUnitId_);
  mode_ = InputMode::AwaitingShootTarget;
}

void GameLogic::ChoosePass() {
  if (mode_ != InputMode::ActionMenu) return;
  Unit* unit = FindUnit(selectedUnitId_.value_or(-1));
  if (!unit) return;
  unit->plan.type = PlannedActionType::Pass;
  unit->plan.movePath.clear();
  unit->plan.queuedLegs.clear();
  unit->plan.shootTargetId = -1;
  selectedUnitId_.reset();
  mode_ = InputMode::AwaitingSelection;
}

void GameLogic::CancelAction() {
  if (mode_ == InputMode::AwaitingMoveDestination) {
    // Esc abandons the whole chain being planned.
    if (Unit* unit = FindUnit(selectedUnitId_.value_or(-1))) {
      if (unit->plan.type == PlannedActionType::Move) unit->plan = PlannedAction{};
    }
  }
  if (mode_ == InputMode::AwaitingMoveDestination || mode_ == InputMode::AwaitingShootTarget) {
    mode_ = InputMode::ActionMenu;
    moveFrontier_ = ReachField();
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
         LineOfSightClear(shooter.EyePosition(), target.EyePosition(), scene_.obstacles,
                          scene_.walkSurfaces, scene_.ground);
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
    // Presentation only: start the shooter's quick-draw beat, aimed at the
    // target's actual bearing (which may sit anywhere inside the FOV cone).
    shooter.shootElapsed = 0.0f;
    shooter.shootAimYaw = std::atan2(dir.z, dir.x);
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
  for (int id : mirroredMoving_) {
    if (id == unitId) return true;
  }
  for (const ActiveMove& move : activeMoves_) {
    if (move.unitId == unitId) return true;
  }
  return false;
}

void GameLogic::ApplyPlaybookReactions() {
  struct Reaction {
    Unit* actor;
    Unit* target;  // Nearest sighted enemy; only fired on when `shoot`.
    bool shoot;
    bool stop;
  };
  std::vector<Reaction> reactions;
  for (Unit& unit : scene_.units) {
    if (!unit.alive) continue;
    auto isMidMove = [&](int id) {
      return std::any_of(activeMoves_.begin(), activeMoves_.end(), [&](const ActiveMove& m) {
        return m.unitId == id && m.path.size() >= 2 && m.segment + 1 < m.path.size();
      });
    };
    const bool moving = isMidMove(unit.id);
    Unit* nearest = nullptr;
    float nearestDist = 0.0f;
    bool canSeeMe = false;
    for (Unit& enemy : scene_.units) {
      if (!enemy.alive || enemy.team == unit.team) continue;
      // A stationary figure reacts to enemies *moving* into its view (the
      // watcher-on-mover case), not to everyone idling in its cone.
      if (!moving && !isMidMove(enemy.id)) continue;
      if (!CanUnitSee(unit, enemy, scene_.obstacles, scene_.walkSurfaces, scene_.ground)) continue;
      const float dist = glm::distance(unit.position, enemy.position);
      if (!nearest || dist < nearestDist) {
        nearest = &enemy;
        nearestDist = dist;
      }
      // Most-cautious tie-break: any sighted enemy that sees back counts.
      canSeeMe |= CanUnitSee(enemy, unit, scene_.obstacles, scene_.walkSurfaces, scene_.ground);
    }
    if (!nearest) continue;
    const ReactionAction action = Playbook(unit.team).At(moving, canSeeMe);
    const bool shoot = ReactionShoots(action);
    const bool stop = moving && ReactionStops(action);
    if (shoot || stop) reactions.push_back(Reaction{&unit, nearest, shoot, stop});
  }
  for (const Reaction& r : reactions) {
    if (r.shoot) ResolveShot(*r.actor, *r.target);
    if (r.stop) {
      for (ActiveMove& move : activeMoves_) {
        if (move.unitId == r.actor->id) move.segment = move.path.size();
      }
    }
  }
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
    // Carry the rest of a multi-round route across the commit; FinishRound
    // arms its next leg.
    if (plan.type == PlannedActionType::Move) unit.plan.queuedLegs = plan.queuedLegs;

    if (plan.type == PlannedActionType::Move) {
      activeMoves_.push_back(ActiveMove{unit.id, plan.movePath, 0, plan.endFacingYaw});
    } else if (plan.type == PlannedActionType::Shoot) {
      pendingShots_.push_back(PendingShot{unit.id, plan.shootTargetId});
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

  // Auto-arm the next leg of any multi-round route; the player can still
  // replan (which clears the queue) before committing.
  for (auto& unit : scene_.units) {
    if (unit.plan.queuedLegs.empty()) continue;
    if (!unit.alive) {
      unit.plan = PlannedAction{};
      continue;
    }
    unit.plan.type = PlannedActionType::Move;
    unit.plan.movePath = std::move(unit.plan.queuedLegs.front());
    unit.plan.queuedLegs.erase(unit.plan.queuedLegs.begin());
    unit.plan.endFacingYaw = FinalYaw(unit.plan.movePath, unit.facingYaw);
  }

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
