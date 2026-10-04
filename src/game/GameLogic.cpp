#include "game/GameLogic.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

#include "game/Geometry.h"
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
  aimPreview_.reset();

  ResetSightingMemory();
}

void GameLogic::ResetSightingMemory() {
  const size_t unitSlots = scene_.units.size();
  for (int t = 0; t < 2; ++t) {
    sightings_[t].assign(unitSlots, {});
    sightedLastFrame_[t].assign(unitSlots, false);
    sightingTimer_[t].assign(unitSlots, 0.0f);
  }
  lastUnitPosition_.assign(unitSlots, glm::vec3(0.0f));
  hasLastUnitPosition_ = false;
  lastSightingRound_ = roundNumber_;
  tracers_.clear();
}

void GameLogic::RecordTracer(const Unit& shooter, const glm::vec3& from, const glm::vec3& to) {
  tracers_.erase(std::remove_if(tracers_.begin(), tracers_.end(),
                                [this](const Tracer& t) {
                                  return roundNumber_ - t.birthRound >=
                                         constants::kTracerMemoryRounds;
                                }),
                 tracers_.end());
  tracers_.push_back(Tracer{shooter.team, from, to, roundNumber_});
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
    u.planHasAimPoint = unit.plan.hasAimPoint;
    u.planAimPoint = unit.plan.aimPoint;
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
  snap.tracers = tracers_;
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
  unit->plan.hasAimPoint = u.planHasAimPoint;
  unit->plan.aimPoint = u.planAimPoint;
  unit->plan.movePath = u.planPath;
  unit->plan.queuedLegs = u.planQueuedLegs;
  unit->plan.endFacingYaw = u.planEndFacingYaw;
}

}  // namespace

bool GameLogic::ImportState(const GameSnapshot& snap) {
  if (!SnapshotMatchesUnits(snap, scene_.units)) return false;
  // Deaths arriving in this snapshot whose victim the opposing team cannot
  // see happened outside its FOV: a free-aim blind hit resolved on the
  // simulating peer. Sighting memory is per-page derived state (never
  // shipped in snapshots), so mirror here the reveal ApplyAimShot recorded
  // on the simulator -- the one bit of info a connecting blind shot earns.
  std::vector<int> newlyDeadIds;
  for (const auto& u : snap.units) {
    const Unit* unit = FindUnit(u.id);
    if (unit && unit->alive && !u.alive) newlyDeadIds.push_back(u.id);
  }
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
  for (int deadId : newlyDeadIds) {
    const Unit* dead = FindUnit(deadId);
    const Team opponent = OpposingTeam(dead->team);
    if (ComputeVisibility(opponent).UnitVisible(deadId)) continue;
    const int viewerIndex = static_cast<int>(opponent);
    if (deadId >= 0 && static_cast<size_t>(deadId) < sightings_[viewerIndex].size()) {
      EnemySighting s;
      s.position = dead->position;
      s.facingYaw = dead->facingYaw;
      s.walkPhase = dead->walkPhase;
      s.walkBlend = dead->walkBlend;
      s.idleElapsed = dead->idleElapsed;
      sightings_[viewerIndex][deadId].push_back(s);
    }
  }
  // A new game on the simulator arrives as a snapshot with the same unit ids,
  // so it is not rejected above; detect it (round counter went backwards, or
  // a finished match is back in play) and drop the previous game's memory.
  const bool newGame = snap.roundNumber < roundNumber_ ||
                       (mode_ == InputMode::GameOver && snap.mode != InputMode::GameOver);
  mode_ = snap.mode;
  roundNumber_ = snap.roundNumber;
  if (newGame) ResetSightingMemory();
  tracers_ = snap.tracers;
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
  aimPreview_.reset();
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
        << static_cast<int>(u.planType) << ' ' << u.planShootTargetId << ' '
        << (u.planHasAimPoint ? 1 : 0) << ' ' << u.planAimPoint.x << ' ' << u.planAimPoint.y
        << ' ' << u.planAimPoint.z << ' ' << u.planEndFacingYaw << ' '
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
  out << ' ' << snap.tracers.size();
  for (const Tracer& t : snap.tracers) {
    out << ' ' << static_cast<int>(t.team) << ' ' << t.from.x << ' ' << t.from.y << ' ' << t.from.z
        << ' ' << t.to.x << ' ' << t.to.y << ' ' << t.to.z << ' ' << t.birthRound;
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
    int alive = 0, plan = 0, hasAim = 0, moving = 0;
    size_t pathCount = 0;
    if (!(in >> u.id >> u.position.x >> u.position.y >> u.position.z >> u.facingYaw >> alive >>
          plan >> u.planShootTargetId >> hasAim >> u.planAimPoint.x >> u.planAimPoint.y >>
          u.planAimPoint.z >> u.planEndFacingYaw >> u.knockdownAxis.x >>
          u.knockdownAxis.y >> u.knockdownAxis.z >> u.knockdownElapsed >> u.walkPhase >>
          u.walkBlend >> u.idleElapsed >> u.shootElapsed >> u.shootAimYaw >> moving >>
          pathCount)) {
      return false;
    }
    if (plan < 0 || plan > static_cast<int>(PlannedActionType::Pass)) return false;
    if (pathCount > kMaxEntries) return false;
    u.planType = static_cast<PlannedActionType>(plan);
    u.alive = alive != 0;
    u.planHasAimPoint = hasAim != 0;
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
  size_t tracerCount = 0;
  if (!(in >> tracerCount) || tracerCount > kMaxEntries) return false;
  snap.tracers.resize(tracerCount);
  for (Tracer& t : snap.tracers) {
    int team = 0;
    if (!(in >> team >> t.from.x >> t.from.y >> t.from.z >> t.to.x >> t.to.y >> t.to.z >>
          t.birthRound) ||
        team < 0 || team > 1) {
      return false;
    }
    t.team = static_cast<Team>(team);
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
    shooter->plan.hasAimPoint = false;
    shooter->plan.movePath.clear();
    shooter->plan.queuedLegs.clear();
    aimPreview_.reset();
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
    mover->plan.hasAimPoint = false;
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
  aimPreview_.reset();
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
  unit->plan.hasAimPoint = false;
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
    aimPreview_.reset();
  } else if (mode_ == InputMode::ActionMenu) {
    mode_ = InputMode::AwaitingSelection;
    selectedUnitId_.reset();
  }
}

float ShotProfileHitChance(const ShotProfile& profile, float angleDegrees, float distance) {
  const float absAngle = std::fabs(angleDegrees);
  if (absAngle >= profile.halfAngleDegrees) return 0.0f;
  constexpr float kHalfPi = 1.57079632679489661923f;
  const float angleFalloff = std::cos(glm::radians(absAngle) / glm::radians(profile.halfAngleDegrees) * kHalfPi);
  const float rangeFalloff = 1.0f / (1.0f + std::max(distance, 0.0f) / profile.range);
  return profile.maxChance * angleFalloff * rangeFalloff;
}

float ShotConeAlpha(const ShotProfile& profile, float distance) {
  const float t = glm::clamp(distance / profile.range, 0.0f, 1.0f);
  return constants::kConeStartAlpha * (1.0f - t);
}

float GameLogic::ShotHitChance(const Unit& shooter, const Unit& target) const {
  const ShotProfile& profile = kDefaultShotProfile;  // Future: derive from shooter's role.
  const glm::vec3 eye = shooter.EyePosition();
  const glm::vec3 targetEye = target.EyePosition();
  if (!InFovCone(eye, shooter.FacingDirection(), targetEye, profile.halfAngleDegrees,
                 std::numeric_limits<float>::infinity()) ||
      !LineOfSightClear(eye, targetEye, scene_.obstacles, scene_.walkSurfaces, scene_.ground)) {
    return 0.0f;
  }
  const glm::vec3 toTarget = targetEye - eye;
  const float distance = glm::length(toTarget);
  const glm::vec3 fwd = glm::normalize(glm::vec3(shooter.FacingDirection().x, 0.0f, shooter.FacingDirection().z));
  float angle = 0.0f;
  if (glm::length(glm::vec2(toTarget.x, toTarget.z)) > 1e-6f) {
    const glm::vec3 dir = glm::normalize(glm::vec3(toTarget.x, 0.0f, toTarget.z));
    angle = glm::degrees(std::acos(glm::clamp(glm::dot(fwd, dir), -1.0f, 1.0f)));
  }
  return ShotProfileHitChance(profile, angle, distance);
}

bool GameLogic::ShotConnects(const Unit& shooter, const Unit& target) const {
  return ShotHitChance(shooter, target) > 0.0f;
}

float GameLogic::RollShot() {
  if (shotRollSource_) return shotRollSource_();
  // Drawn by hand: <random> distributions aren't specified across stdlibs.
  return static_cast<float>(shotRng_() >> 8) / 16777216.0f;
}

bool GameLogic::ResolveShot(Unit& shooter, Unit& target, bool* fired) {
  const float chance = ShotHitChance(shooter, target);
  if (fired) *fired = chance > 0.0f;
  if (chance <= 0.0f) return false;
  const bool hit = RollShot() < chance;
  ApplyShot(shooter, target, hit);
  return hit;
}

void GameLogic::ApplyShot(Unit& shooter, Unit& target, bool hit) {
  glm::vec3 dir = target.position - shooter.position;
  dir.y = 0.0f;
  if (glm::length(dir) < 1e-4f) dir = shooter.FacingDirection();
  dir = glm::normalize(dir);
  // Presentation only: start the shooter's quick-draw beat, aimed at the
  // target's actual bearing (which may sit anywhere inside the FOV cone).
  // Played for misses too -- the shot was taken.
  shooter.shootElapsed = 0.0f;
  shooter.shootAimYaw = std::atan2(dir.z, dir.x);
  {
    Unit aimed = shooter;
    aimed.facingYaw = shooter.shootAimYaw;
    RecordTracer(shooter, aimed.MuzzlePosition(), target.EyePosition());
  }
  if (hit) {
    target.alive = false;
    // up x dir: tipping around this axis leans the figure toward dir.
    target.knockdownAxis = glm::vec3(dir.z, 0.0f, -dir.x);
    target.knockdownElapsed = 0.0f;
  }
}

namespace {

// Nearest aimable-surface hit along a camera ray: the ground (flat plane or
// sampled terrain, inside the map square), slab tops, walk surfaces
// (decks/ramps) and obstacle faces -- a wall is a legitimate aim surface
// (area denial). Mirrors what the renderer draws, so the "+" selector lands
// where the player actually pointed.
bool IntersectAimSurface(const glm::vec3& origin, const glm::vec3& dir, const Scene& scene,
                         glm::vec3* outPoint) {
  bool found = false;
  float bestT = std::numeric_limits<float>::infinity();
  const auto consider = [&](float t) {
    if (t >= 0.0f && t < bestT) {
      bestT = t;
      found = true;
    }
  };

  if (scene.ground.Empty()) {
    if (std::fabs(dir.y) > 1e-6f) {
      const float t = -origin.y / dir.y;
      if (t >= 0.0f) {
        const glm::vec3 p = origin + dir * t;
        if (std::fabs(p.x) <= scene.mapHalfExtent && std::fabs(p.z) <= scene.mapHalfExtent) {
          consider(t);
        }
      }
    }
  } else if (origin.y > scene.ground.HeightAt(origin.x, origin.z)) {
    // Coarse ray-march refined by bisection, same approach as the app's
    // terrain click picking.
    constexpr float kCoarseStep = 0.5f;
    constexpr float kMaxDistance = 2000.0f;
    float prevT = 0.0f;
    for (float t = kCoarseStep; t <= kMaxDistance; t += kCoarseStep) {
      const glm::vec3 p = origin + dir * t;
      if (p.y - scene.ground.HeightAt(p.x, p.z) <= 0.0f) {
        float lo = prevT, hi = t;
        for (int i = 0; i < 24; ++i) {
          const float mid = 0.5f * (lo + hi);
          const glm::vec3 q = origin + dir * mid;
          (q.y - scene.ground.HeightAt(q.x, q.z) <= 0.0f ? hi : lo) = mid;
        }
        const float hitT = 0.5f * (lo + hi);
        const glm::vec3 hit = origin + dir * hitT;
        if (std::fabs(hit.x) <= scene.mapHalfExtent && std::fabs(hit.z) <= scene.mapHalfExtent) {
          consider(hitT);
        }
        break;
      }
      prevT = t;
    }
  }

  float t = 0.0f;
  for (const Obstacle& obstacle : scene.obstacles) {
    if (RayIntersectsObstacle(origin, dir, obstacle, &t)) consider(t);
  }
  for (const AABB& slab : scene.sidewalks) {
    if (RayIntersectsAABB(origin, dir, slab, &t)) consider(t);
  }
  // Same deck thickness as the LOS raycast, so aimable decks and occluding
  // decks are one and the same slab.
  constexpr float kDeckThickness = 0.45f;
  for (const WalkSurface& surface : scene.walkSurfaces) {
    if (RayIntersectsWalkSurface(origin, dir, surface, kDeckThickness, &t)) consider(t);
  }

  if (found && outPoint) *outPoint = origin + dir * bestT;
  return found;
}

}  // namespace

bool GameLogic::IsAimSurfaceVisible(const glm::vec3& point) const {
  const Unit* shooter = FindUnit(selectedUnitId_.value_or(-1));
  if (!shooter) return false;
  const glm::vec3 eye = shooter->EyePosition();
  if (glm::distance(eye, point) > constants::kSightRange) return false;
  // Lifted a hair off the surface so the LOS segment doesn't graze the very
  // triangle/face the point sits on. No FOV cone: the figure turns to shoot.
  const glm::vec3 probe = point + glm::vec3(0.0f, 0.05f, 0.0f);
  return LineOfSightClear(eye, probe, scene_.obstacles, scene_.walkSurfaces, scene_.ground);
}

GameLogic::AimRayResult GameLogic::ResolveAimRay(const glm::vec3& origin,
                                                 const glm::vec3& direction) const {
  AimRayResult result;
  const Unit* shooter = FindUnit(selectedUnitId_.value_or(-1));
  if (!shooter) return result;
  if (glm::length(direction) < 1e-6f) return result;
  const glm::vec3 dir = glm::normalize(direction);

  {
    // A figure under the cursor beats the surface behind it, keeping the
    // existing lock-on flow. Only enemies the shooter's team can currently
    // see are pickable -- the same fog rule as rendering.
    const TeamVisibility visibility = ComputeVisibility(shooter->team);
    float bestT = std::numeric_limits<float>::infinity();
    for (const Unit& unit : scene_.units) {
      if (!unit.alive || unit.team == shooter->team || !visibility.UnitVisible(unit.id)) continue;
      float t = 0.0f;
      if (RayIntersectsAABB(origin, dir, unit.Bounds(), &t) && t < bestT) {
        bestT = t;
        result.unitId = unit.id;
      }
    }
    if (result.unitId >= 0) {
      result.kind = AimRayResult::Kind::Unit;
      return result;
    }

    glm::vec3 surface;
    if (IntersectAimSurface(origin, dir, scene_, &surface) && IsAimSurfaceVisible(surface)) {
      result.kind = AimRayResult::Kind::Surface;
      result.point = surface;
      return result;
    }
  }

  // Nothing aimable under the cursor (sky, or ground the shooter can't see).
  return result;
}

void GameLogic::ClickAimRay(const glm::vec3& origin, const glm::vec3& direction, Team byTeam) {
  if (mode_ != InputMode::AwaitingShootTarget) return;
  const Unit* shooter = FindUnit(selectedUnitId_.value_or(-1));
  if (!shooter || shooter->team != byTeam) return;
  const AimRayResult aim = ResolveAimRay(origin, direction);
  if (aim.kind == AimRayResult::Kind::Unit) {
    ClickUnit(aim.unitId, byTeam);
    return;
  }
  if (aim.kind == AimRayResult::Kind::Surface) PlaceAimPoint(aim.point, byTeam);
}

void GameLogic::PlaceAimPoint(const glm::vec3& point, Team byTeam) {
  if (mode_ != InputMode::AwaitingShootTarget) return;
  const Unit* shooter = FindUnit(selectedUnitId_.value_or(-1));
  if (!shooter || shooter->team != byTeam) return;
  aimPreview_ = AimPreview{point};
}

void GameLogic::ConfirmAim(Team byTeam) {
  if (mode_ != InputMode::AwaitingShootTarget || !aimPreview_) return;
  Unit* shooter = FindUnit(selectedUnitId_.value_or(-1));
  if (!shooter || shooter->team != byTeam) return;
  shooter->plan.type = PlannedActionType::Shoot;
  shooter->plan.shootTargetId = -1;
  shooter->plan.hasAimPoint = true;
  shooter->plan.aimPoint = aimPreview_->point;
  shooter->plan.movePath.clear();
  shooter->plan.queuedLegs.clear();
  aimPreview_.reset();
  selectedUnitId_.reset();
  mode_ = InputMode::AwaitingSelection;
}

std::vector<GameLogic::AimTraceCandidate> GameLogic::AimTraceCandidates(
    const Unit& shooter, const glm::vec3& aimPoint) const {
  // The shot leaves the muzzle of the figure already turned to its aim
  // point, and keeps flying past it until something stops it.
  Unit aimed = shooter;
  const glm::vec3 flat(aimPoint.x - shooter.position.x, 0.0f, aimPoint.z - shooter.position.z);
  if (glm::length(flat) > 1e-4f) aimed.facingYaw = std::atan2(flat.z, flat.x);
  const glm::vec3 muzzle = aimed.MuzzlePosition();
  glm::vec3 dir = aimPoint - muzzle;
  if (glm::length(dir) < 1e-4f) {
    dir = aimed.FacingDirection();
  } else {
    dir = glm::normalize(dir);
  }
  const glm::vec3 fwd = aimed.FacingDirection();

  std::vector<AimTraceCandidate> candidates;
  for (const Unit& unit : scene_.units) {
    if (!unit.alive || unit.id == shooter.id) continue;
    if (!friendlyFire_ && unit.team == shooter.team) continue;  // Transparent to the trace.
    float t = 0.0f;
    if (!RayIntersectsAABB(muzzle, dir, unit.Bounds(), &t)) continue;
    if (t < 0.0f || t > constants::kAimTraceRange) continue;
    // Walls/decks/terrain between the muzzle and the figure stop the bullet
    // before it gets there -- and, the ray being straight, before anything
    // beyond it too.
    const glm::vec3 entry = muzzle + dir * t;
    if (!LineOfSightClear(muzzle, entry, scene_.obstacles, scene_.walkSurfaces, scene_.ground)) {
      continue;
    }
    // Same profile falloff as a locked shot, at the struck figure's actual
    // bearing/distance (the bearing is near-zero for anything on the ray).
    const glm::vec3 toEye = unit.EyePosition() - muzzle;
    float angle = 0.0f;
    if (glm::length(glm::vec2(toEye.x, toEye.z)) > 1e-6f) {
      const glm::vec3 to = glm::normalize(glm::vec3(toEye.x, 0.0f, toEye.z));
      angle = glm::degrees(std::acos(glm::clamp(glm::dot(fwd, to), -1.0f, 1.0f)));
    }
    AimTraceCandidate candidate;
    candidate.unitId = unit.id;
    candidate.chance = ShotProfileHitChance(kDefaultShotProfile, angle, glm::length(toEye));
    candidate.rayT = t;
    candidates.push_back(candidate);
  }
  std::sort(candidates.begin(), candidates.end(),
            [](const AimTraceCandidate& a, const AimTraceCandidate& b) { return a.rayT < b.rayT; });
  return candidates;
}

void GameLogic::ApplyAimShot(Unit& shooter, Unit* hitTarget, const glm::vec3& aimPoint) {
  // The one bit of fog-of-war info a blind shot earns: a connecting hit on a
  // figure unseen *when the trigger was pulled* (before the turn below)
  // records a sighting. Misses reveal nothing.
  const bool wasVisible =
      hitTarget && ComputeVisibility(shooter.team).UnitVisible(hitTarget->id);

  glm::vec3 dir(aimPoint.x - shooter.position.x, 0.0f, aimPoint.z - shooter.position.z);
  if (glm::length(dir) < 1e-4f) dir = shooter.FacingDirection();
  dir = glm::normalize(dir);
  const float aimYaw = std::atan2(dir.z, dir.x);
  // The turn outlives the shot: it is this figure's facing for the next
  // rounds' FOV and overwatch reactions.
  shooter.facingYaw = aimYaw;
  shooter.shootElapsed = 0.0f;
  shooter.shootAimYaw = aimYaw;

  // The bullet flies from the muzzle along the aim ray until it strikes the
  // hit figure, a wall/deck, the ground, or runs out of range.
  const glm::vec3 muzzle = shooter.MuzzlePosition();
  glm::vec3 ray = aimPoint - muzzle;
  ray = glm::length(ray) < 1e-4f ? shooter.FacingDirection() : glm::normalize(ray);
  float reach = constants::kAimTraceRange;
  if (hitTarget) {
    float t = 0.0f;
    if (RayIntersectsAABB(muzzle, ray, hitTarget->Bounds(), &t)) reach = t;
  } else {
    for (const Obstacle& obstacle : scene_.obstacles) {
      float t = 0.0f;
      if (RayIntersectsObstacle(muzzle, ray, obstacle, &t) && t > 0.0f) reach = std::min(reach, t);
    }
    for (const WalkSurface& surface : scene_.walkSurfaces) {
      float t = 0.0f;
      if (RayIntersectsWalkSurface(muzzle, ray, surface, 0.45f, &t) && t > 0.0f) {
        reach = std::min(reach, t);
      }
    }
    if (ray.y < -1e-4f && muzzle.y > 0.0f) reach = std::min(reach, -muzzle.y / ray.y);
  }
  RecordTracer(shooter, muzzle, muzzle + ray * reach);
  if (!hitTarget) return;

  glm::vec3 fall(hitTarget->position.x - shooter.position.x, 0.0f,
                 hitTarget->position.z - shooter.position.z);
  fall = glm::length(fall) < 1e-4f ? dir : glm::normalize(fall);
  hitTarget->alive = false;
  hitTarget->knockdownAxis = glm::vec3(fall.z, 0.0f, -fall.x);
  hitTarget->knockdownElapsed = 0.0f;

  const int viewerIndex = static_cast<int>(shooter.team);
  if (!wasVisible && hitTarget->id >= 0 &&
      static_cast<size_t>(hitTarget->id) < sightings_[viewerIndex].size()) {
    EnemySighting s;
    s.position = hitTarget->position;
    s.facingYaw = hitTarget->facingYaw;
    s.walkPhase = hitTarget->walkPhase;
    s.walkBlend = hitTarget->walkBlend;
    s.idleElapsed = hitTarget->idleElapsed;
    sightings_[viewerIndex][hitTarget->id].push_back(s);
  }
}

void GameLogic::ResolvePendingShots() {
  // Judge every held shot against the same snapshot (positions don't change
  // during resolution, and hits are applied only after all are judged), so
  // two figures whose shots connect on the same tick both fire: a mutual
  // kill downs both, rather than whichever happens to resolve first
  // silencing the other.
  // A held locked-target shot is taken (and consumed, hit or miss) the first
  // tick it passes the hard gates. A free-aim shot has no gates at all --
  // the figure turns to the point and fires on its first resolution tick;
  // the ballistic trace decides what (if anything) the bullet meets.
  struct Firing {
    Unit* shooter = nullptr;
    Unit* target = nullptr;  // Locked target; null for a free-aim shot.
    bool aim = false;
    glm::vec3 aimPoint{0.0f};
    std::vector<AimTraceCandidate> trace;
  };
  std::vector<Firing> firing;
  for (const PendingShot& shot : pendingShots_) {
    Unit* shooter = FindUnit(shot.shooterId);
    if (!shooter || !shooter->alive) continue;
    if (shot.hasAimPoint) {
      Firing f;
      f.shooter = shooter;
      f.aim = true;
      f.aimPoint = shot.aimPoint;
      f.trace = AimTraceCandidates(*shooter, shot.aimPoint);
      firing.push_back(std::move(f));
      continue;
    }
    Unit* target = FindUnit(shot.targetId);
    if (!target || !target->alive) continue;
    if (ShotConnects(*shooter, *target)) firing.push_back(Firing{shooter, target});
  }
  // Rolls are applied in order but each hit only flips the target's alive
  // flag after all were judged gate-wise, so mutual shots still both fire.
  // A free-aim trace rolls once per figure in the bullet's path, nearest
  // first; the first success absorbs the bullet.
  struct Outcome {
    Unit* traceHit = nullptr;
    bool lockedHit = false;
  };
  std::vector<Outcome> outcomes;
  for (const Firing& f : firing) {
    Outcome o;
    if (f.aim) {
      for (const AimTraceCandidate& c : f.trace) {
        if (RollShot() < c.chance) {
          o.traceHit = FindUnit(c.unitId);
          break;
        }
      }
    } else {
      o.lockedHit = RollShot() < ShotHitChance(*f.shooter, *f.target);
    }
    outcomes.push_back(o);
  }
  for (size_t i = 0; i < firing.size(); ++i) {
    if (firing[i].aim) {
      ApplyAimShot(*firing[i].shooter, outcomes[i].traceHit, firing[i].aimPoint);
    } else {
      ApplyShot(*firing[i].shooter, *firing[i].target, outcomes[i].lockedHit);
    }
  }

  // Drop everything that fired or can no longer fire (dead shooter holds
  // its fire from here on; a downed target stops being worth a bullet).
  pendingShots_.erase(
      std::remove_if(pendingShots_.begin(), pendingShots_.end(),
                     [this](const PendingShot& shot) {
                       if (shot.hasAimPoint) return true;  // Fired above, hit or miss.
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
      pendingShots_.push_back(
          PendingShot{unit.id, plan.shootTargetId, plan.hasAimPoint, plan.aimPoint});
    }
  }
  aimPreview_.reset();

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
