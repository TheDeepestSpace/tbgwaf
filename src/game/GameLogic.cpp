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

namespace {

// Earliest fraction t in [0,1] along p0->p1 at which the XZ distance to
// `center` is within `radius` (0 if p0 already is); false if the segment
// never gets that close.
bool SegmentEntersDisc(const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& center,
                       float radius, float* outT) {
  const glm::vec2 a(p0.x - center.x, p0.z - center.z);
  const glm::vec2 v(p1.x - p0.x, p1.z - p0.z);
  const float c = glm::dot(a, a) - radius * radius;
  if (c <= 0.0f) {
    *outT = 0.0f;
    return true;
  }
  const float qa = glm::dot(v, v);
  if (qa < 1e-12f) return false;
  const float qb = 2.0f * glm::dot(v, a);
  const float disc = qb * qb - 4.0f * qa * c;
  if (disc < 0.0f) return false;
  const float t = (-qb - std::sqrt(disc)) / (2.0f * qa);
  if (t < 0.0f || t > 1.0f) return false;
  *outT = t;
  return true;
}

}  // namespace

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
  flagWinner_.reset();
  InitFlag();
  movePreviewPath_.clear();
  movePreviewValid_ = false;
  activeMoves_.clear();
  pendingShots_.clear();
  mirroredMoving_.clear();
  aimPreview_.reset();
  lockPreviewId_.reset();
  plannedShots_ = 1;
  executionElapsed_ = 0.0f;

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

void GameLogic::RecordTracer(const Unit& shooter, const glm::vec3& from, const glm::vec3& to,
                             float age) {
  tracers_.erase(std::remove_if(tracers_.begin(), tracers_.end(),
                                [this](const Tracer& t) {
                                  return roundNumber_ - t.birthRound >=
                                         constants::kTracerMemoryRounds;
                                }),
                 tracers_.end());
  tracers_.push_back(Tracer{shooter.team, from, to, roundNumber_, age});
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
    u.planShots = unit.plan.shots;
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
    u.grabElapsed = unit.grabElapsed;
    u.moving = IsUnitMoving(unit.id);
    snap.units.push_back(std::move(u));
  }
  snap.tracers = tracers_;
  snap.playbooks[0] = playbooks_[0];
  snap.playbooks[1] = playbooks_[1];
  snap.mode = mode_;
  snap.roundNumber = roundNumber_;
  snap.winner = winner_ ? static_cast<int>(*winner_) : -1;
  snap.flag = flag_;
  if (flag_.carrierId >= 0) snap.flag.position = FlagPosition();
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
  // Re-clamped against this unit's own weapon so a peer can never ship a
  // plan that fires past the magazine/round-window cap.
  unit->plan.shots =
      std::clamp(u.planShots, 1, MaxShotsPerAction(unit->weapon, constants::kRoundDuration));
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
    unit->grabElapsed = u.grabElapsed;
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
  flag_ = snap.flag;
  for (Unit& unit : scene_.units) unit.carryingFlag = flag_.enabled && unit.id == flag_.carrierId;
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
  lockPreviewId_.reset();
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
        << ' ' << u.planAimPoint.z << ' ' << u.planShots << ' ' << u.planEndFacingYaw << ' '
        << u.knockdownAxis.x << ' ' << u.knockdownAxis.y << ' ' << u.knockdownAxis.z << ' '
        << u.knockdownElapsed << ' ' << u.walkPhase << ' ' << u.walkBlend << ' '
        << u.idleElapsed << ' ' << u.shootElapsed << ' ' << u.shootAimYaw << ' '
        << u.grabElapsed << ' ' << (u.moving ? 1 : 0) << ' '
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
        << ' ' << t.to.x << ' ' << t.to.y << ' ' << t.to.z << ' ' << t.birthRound << ' ' << t.age;
  }
  for (const auto& pb : snap.playbooks)
    for (int m = 0; m < 2; ++m)
      for (int s = 0; s < 2; ++s) out << ' ' << static_cast<int>(pb.table[m][s]);
  out << ' ' << (snap.flag.enabled ? 1 : 0) << ' ' << snap.flag.carrierId << ' '
      << snap.flag.position.x << ' ' << snap.flag.position.y << ' ' << snap.flag.position.z << ' '
      << snap.flag.dropElapsed;
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
          u.planAimPoint.z >> u.planShots >> u.planEndFacingYaw >> u.knockdownAxis.x >>
          u.knockdownAxis.y >> u.knockdownAxis.z >> u.knockdownElapsed >> u.walkPhase >>
          u.walkBlend >> u.idleElapsed >> u.shootElapsed >> u.shootAimYaw >> u.grabElapsed >>
          moving >>
          pathCount)) {
      return false;
    }
    if (plan < 0 || plan > static_cast<int>(PlannedActionType::Pass)) return false;
    if (u.planShots < 1 || u.planShots > static_cast<int>(kMaxEntries)) return false;
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
          t.birthRound >> t.age) ||
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
  int flagEnabled = 0;
  if (!(in >> flagEnabled >> snap.flag.carrierId >> snap.flag.position.x >>
        snap.flag.position.y >> snap.flag.position.z >> snap.flag.dropElapsed)) {
    return false;
  }
  snap.flag.enabled = flagEnabled != 0;
  bool carrierKnown = snap.flag.carrierId < 0;
  for (const auto& u : snap.units) carrierKnown |= u.id == snap.flag.carrierId;
  if (!carrierKnown) return false;
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
    CommitLockedShot(*shooter, unit->id);
  }
}

void GameLogic::CommitLockedShot(Unit& shooter, int targetId) {
  shooter.plan.type = PlannedActionType::Shoot;
  shooter.plan.shootTargetId = targetId;
  shooter.plan.hasAimPoint = false;
  shooter.plan.shots = plannedShots_;
  shooter.plan.movePath.clear();
  shooter.plan.queuedLegs.clear();
  aimPreview_.reset();
  lockPreviewId_.reset();
  plannedShots_ = 1;
  selectedUnitId_.reset();
  mode_ = InputMode::AwaitingSelection;
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
    if (unit.grabElapsed >= 0.0f) {
      unit.grabElapsed += dtSeconds;
      if (unit.grabElapsed >= constants::kGrabAnimDuration) unit.grabElapsed = -1.0f;
    }
  }
  if (flag_.dropElapsed >= 0.0f) {
    flag_.dropElapsed = std::min(flag_.dropElapsed + dtSeconds, constants::kFlagDropDuration);
  }
  SyncFlag();  // Also catches a carrier felled outside a round (direct ResolveShot).

  for (Tracer& tracer : tracers_) tracer.age = std::min(tracer.age + dtSeconds, 60.0f);

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
  const float tickStart = executionElapsed_;
  executionElapsed_ += dtSeconds;
  std::vector<FlagTouch> flagTouches;
  CollectStationaryFlagTouches(tickStart, &flagTouches);
  const bool flagLoose = flag_.enabled && flag_.carrierId < 0;
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
    float walked = 0.0f;
    bool touched = false;
    auto NoteTouch = [&](float distanceWalked) {
      touched = true;
      flagTouches.push_back({mover->id, tickStart + distanceWalked / mover->runSpeed});
    };
    if (flagLoose && TouchesFlag(mover->position)) NoteTouch(0.0f);
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
      if (flagLoose && !touched) {
        // Passing *through* the flag's spot counts, even mid-tick.
        const glm::vec3 pieceEnd =
            distToEnd <= remaining ? segEnd : mover->position + (toEnd / distToEnd) * remaining;
        float t = 0.0f;
        if (SegmentEntersDisc(mover->position, pieceEnd, flag_.position,
                              constants::kFlagGrabRadius, &t) &&
            std::abs(glm::mix(mover->position.y, pieceEnd.y, t) - flag_.position.y) <=
                constants::kFlagGrabHeight) {
          NoteTouch(walked + t * step);
        }
      }
      walked += step;
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

  // Grabs happen where the figures physically are, before reactions or shots
  // this tick can change anything (a grabber shot a moment later still held it).
  GrabFlagFrom(flagTouches);

  ApplyPlaybookReactions();

  // Continuous shot resolution: with everyone's position advanced for this
  // tick, held shots get their per-tick FOV/LOS re-check -- this is what
  // lets a shooter hit a target that only walks into its cone mid-round.
  ResolvePendingShots();
  SyncFlag();

  // Every mover advances together above; drop whichever ones just finished
  // their path (or died to a shot this tick). Once none are left, a shot
  // still waiting on its gates can never pass them (its geometry is
  // frozen), so it expires -- but a burst already firing keeps the round
  // executing until its last scheduled shot.
  activeMoves_.erase(std::remove_if(activeMoves_.begin(), activeMoves_.end(),
                                     [this](const ActiveMove& move) {
                                       const Unit* mover = FindUnit(move.unitId);
                                       return !mover || !mover->alive ||
                                              move.path.size() < 2 ||
                                              move.segment + 1 >= move.path.size();
                                     }),
                      activeMoves_.end());

  if (activeMoves_.empty()) {
    pendingShots_.erase(std::remove_if(pendingShots_.begin(), pendingShots_.end(),
                                       [](const PendingShot& shot) { return !shot.started; }),
                        pendingShots_.end());
    if (pendingShots_.empty()) FinishRound();
  }
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
  if (!moveFrontierEnabled_) return;
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
  lockPreviewId_.reset();
  plannedShots_ = 1;
  mode_ = InputMode::AwaitingShootTarget;
}

int GameLogic::MaxShotsForSelected() const {
  const Unit* shooter = FindUnit(selectedUnitId_.value_or(-1));
  if (!shooter) return 1;
  return MaxShotsPerAction(shooter->weapon, constants::kRoundDuration);
}

void GameLogic::SetPlannedShotCount(int count, Team byTeam) {
  if (mode_ != InputMode::AwaitingShootTarget) return;
  const Unit* shooter = FindUnit(selectedUnitId_.value_or(-1));
  if (!shooter || shooter->team != byTeam) return;
  // Clamp: never 0 (an action always fires at least one shot), never past
  // the weapon's magazine/round-window cap.
  plannedShots_ = std::clamp(count, 1, MaxShotsForSelected());
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
    lockPreviewId_.reset();
    plannedShots_ = 1;
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

void GameLogic::PlayShootBeat(Unit& shooter, float aimYaw, float sinceShot) {
  // Presentation only: the shooter's quick-draw beat, aimed at the actual
  // bearing (which may sit anywhere inside the FOV cone). Played for misses
  // too -- the shot was taken. A follow-up shot of a burst lands while the
  // beat is still playing: it re-triggers just the recoil kick (the weapon
  // stays shouldered) instead of re-drawing from the carry pose.
  shooter.shootElapsed =
      (shooter.shootElapsed >= 0.0f ? constants::kShootRecoilStart : 0.0f) + sinceShot;
  shooter.shootAimYaw = aimYaw;
}

void GameLogic::KnockDown(Unit& target, const glm::vec3& dir) {
  target.alive = false;
  // up x dir: tipping around this axis leans the figure toward dir.
  target.knockdownAxis = glm::vec3(dir.z, 0.0f, -dir.x);
  target.knockdownElapsed = 0.0f;
}

void GameLogic::ApplyShot(Unit& shooter, Unit& target, bool hit) {
  glm::vec3 dir = target.position - shooter.position;
  dir.y = 0.0f;
  if (glm::length(dir) < 1e-4f) dir = shooter.FacingDirection();
  dir = glm::normalize(dir);
  PlayShootBeat(shooter, std::atan2(dir.z, dir.x));
  Unit aimed = shooter;
  aimed.facingYaw = shooter.shootAimYaw;
  RecordTracer(shooter, aimed.MuzzlePosition(), target.EyePosition());
  if (hit) KnockDown(target, dir);
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
    // Stage the lock; the shot-level bar stays up until Fire confirms.
    aimPreview_.reset();
    lockPreviewId_ = aim.unitId;
    return;
  }
  if (aim.kind == AimRayResult::Kind::Surface) PlaceAimPoint(aim.point, byTeam);
}

void GameLogic::PlaceAimPoint(const glm::vec3& point, Team byTeam) {
  if (mode_ != InputMode::AwaitingShootTarget) return;
  const Unit* shooter = FindUnit(selectedUnitId_.value_or(-1));
  if (!shooter || shooter->team != byTeam) return;
  lockPreviewId_.reset();
  aimPreview_ = AimPreview{point};
}

void GameLogic::ConfirmAim(Team byTeam) {
  if (mode_ != InputMode::AwaitingShootTarget || (!aimPreview_ && !lockPreviewId_)) return;
  Unit* shooter = FindUnit(selectedUnitId_.value_or(-1));
  if (!shooter || shooter->team != byTeam) return;
  if (lockPreviewId_) {
    const Unit* target = FindUnit(*lockPreviewId_);
    if (!target || !target->alive) {
      lockPreviewId_.reset();
      return;
    }
    CommitLockedShot(*shooter, *lockPreviewId_);
    return;
  }
  shooter->plan.type = PlannedActionType::Shoot;
  shooter->plan.shootTargetId = -1;
  shooter->plan.hasAimPoint = true;
  shooter->plan.aimPoint = aimPreview_->point;
  shooter->plan.shots = plannedShots_;
  shooter->plan.movePath.clear();
  shooter->plan.queuedLegs.clear();
  aimPreview_.reset();
  lockPreviewId_.reset();
  plannedShots_ = 1;
  selectedUnitId_.reset();
  mode_ = InputMode::AwaitingSelection;
}

Unit GameLogic::AimedAt(const Unit& shooter, const glm::vec3& point) {
  // The shot leaves the muzzle of the figure already turned to its aim
  // point.
  Unit aimed = shooter;
  const glm::vec3 flat(point.x - shooter.position.x, 0.0f, point.z - shooter.position.z);
  if (glm::length(flat) > 1e-4f) aimed.facingYaw = std::atan2(flat.z, flat.x);
  return aimed;
}

std::vector<GameLogic::AimTraceCandidate> GameLogic::AimTraceCandidates(
    const Unit& shooter, const glm::vec3& aimPoint) const {
  const Unit aimed = AimedAt(shooter, aimPoint);
  const glm::vec3 muzzle = aimed.MuzzlePosition();
  glm::vec3 dir = aimPoint - muzzle;
  if (glm::length(dir) < 1e-4f) {
    dir = aimed.FacingDirection();
  } else {
    dir = glm::normalize(dir);
  }
  return RayTraceCandidates(shooter, muzzle, dir, aimed.FacingDirection());
}

std::vector<GameLogic::AimTraceCandidate> GameLogic::RayTraceCandidates(
    const Unit& shooter, const glm::vec3& muzzle, const glm::vec3& dir,
    const glm::vec3& fwd) const {
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

float GameLogic::RayReach(const glm::vec3& muzzle, const glm::vec3& ray) const {
  // No weapon range cap: a bullet flies until an obstacle, the ground or the
  // map edge stops it (kAimTraceRange only bounds a ray that never does).
  float reach = constants::kAimTraceRange;
  const float half = scene_.mapHalfExtent;
  for (const auto [origin, d] : {std::pair{muzzle.x, ray.x}, std::pair{muzzle.z, ray.z}}) {
    if (d > 1e-6f) reach = std::min(reach, (half - origin) / d);
    if (d < -1e-6f) reach = std::min(reach, (-half - origin) / d);
  }
  reach = std::max(reach, 0.0f);
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
  return reach;
}

namespace {

// `dir` turned by yaw/pitch offsets (radians).
glm::vec3 Deflect(const glm::vec3& dir, float yawOffset, float pitchOffset) {
  const float yaw = std::atan2(dir.z, dir.x) + yawOffset;
  const float pitch =
      glm::clamp(std::asin(glm::clamp(dir.y, -1.0f, 1.0f)) + pitchOffset, -1.5f, 1.5f);
  return glm::vec3(std::cos(pitch) * std::cos(yaw), std::sin(pitch), std::cos(pitch) * std::sin(yaw));
}

// Half-width a deflected bullet must clear to visibly pass beside a figure.
constexpr float kBodyClearance = 0.6f;

}  // namespace

void GameLogic::ApplyAimShot(Unit& shooter, const std::vector<Unit*>& hitTargets,
                             const glm::vec3& aimPoint, float sinceShot) {
  // The one bit of fog-of-war info a blind shot earns: a connecting hit on a
  // figure unseen *when the trigger was pulled* (before the turn below)
  // records a sighting. Misses reveal nothing.
  const TeamVisibility visibility = ComputeVisibility(shooter.team);

  glm::vec3 dir(aimPoint.x - shooter.position.x, 0.0f, aimPoint.z - shooter.position.z);
  if (glm::length(dir) < 1e-4f) dir = shooter.FacingDirection();
  dir = glm::normalize(dir);
  const float aimYaw = std::atan2(dir.z, dir.x);
  // The turn outlives the shot: it is this figure's facing for the next
  // rounds' FOV and overwatch reactions.
  shooter.facingYaw = aimYaw;
  PlayShootBeat(shooter, aimYaw, sinceShot);

  for (Unit* hitTarget : hitTargets) {
    const bool wasVisible = visibility.UnitVisible(hitTarget->id);
    glm::vec3 fall(hitTarget->position.x - shooter.position.x, 0.0f,
                   hitTarget->position.z - shooter.position.z);
    fall = glm::length(fall) < 1e-4f ? dir : glm::normalize(fall);
    KnockDown(*hitTarget, fall);

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
}

void GameLogic::ResolvePendingShots() {
  // A held locked-target burst opens fire the first tick it passes the hard
  // gates; a free-aim burst has no gates at all and opens fire on its first
  // resolution tick. From then on the burst is paced in real time (issue
  // #140): shot k is scheduled at startTime + k * the weapon's interval,
  // and every requested shot is taken -- a kill does not cut the burst
  // short -- until the magazine level is spent or the round window closes.
  for (PendingShot& shot : pendingShots_) {
    if (shot.started) continue;
    Unit* shooter = FindUnit(shot.shooterId);
    if (!shooter || !shooter->alive) continue;
    if (!shot.hasAimPoint) {
      Unit* target = FindUnit(shot.targetId);
      if (!target || !target->alive) continue;
      if (!ShotConnects(*shooter, *target)) continue;
      // The bullets fly muzzle -> torso, so the hold-fire gate waits for
      // that very line too: a burst must not open up while chest-high cover
      // still blocks the trajectory the eyes already see over.
      const glm::vec3 aimPoint =
          target->position + glm::vec3(0.0f, constants::kTorsoAimHeight, 0.0f);
      const Unit aimed = AimedAt(*shooter, aimPoint);
      if (!LineOfSightClear(aimed.MuzzlePosition(), aimPoint, scene_.obstacles,
                            scene_.walkSurfaces, scene_.ground)) {
        continue;
      }
    }
    shot.started = true;
    shot.startTime = executionElapsed_;
  }

  // Everything with at least one bullet due at the current execution clock.
  struct Burst {
    PendingShot* shot = nullptr;
    Unit* shooter = nullptr;
    Unit* target = nullptr;  // Locked target; null for a free-aim burst.
    int bulletsDue = 0;
    // Judged against this tick's snapshot, lazily on the burst's first due
    // bullet (positions don't change during resolution).
    bool judged = false;
    // Where this tick's bullets are aimed: the staged free-aim point, or --
    // for a locked target -- its torso wherever the figure stands this tick.
    glm::vec3 aimPoint{0.0f};
    // Outcome of this tick's bullets, applied only after every due bullet
    // (across all bursts) was judged: mutual shots in a tick both land.
    std::vector<Unit*> traceHits;
    float lastAt = 0.0f;
  };
  std::vector<Burst> bursts;
  // One entry per due bullet; judged in schedule order below so the roll
  // stream does not depend on how time is sliced into ticks.
  struct Bullet {
    size_t burst;
    float at;
    int index;  // Position within the burst (0-based).
    glm::vec3 from{0.0f};
    glm::vec3 to{0.0f};  // Where this bullet's own trajectory ends.
  };
  std::vector<Bullet> bullets;
  for (PendingShot& shot : pendingShots_) {
    if (!shot.started || shot.shotsFired >= shot.shots) continue;
    Unit* shooter = FindUnit(shot.shooterId);
    if (!shooter || !shooter->alive) continue;
    const float interval = StatsOf(shooter->weapon).shotIntervalSeconds;
    Burst burst;
    burst.shot = &shot;
    burst.shooter = shooter;
    burst.target = shot.hasAimPoint ? nullptr : FindUnit(shot.targetId);
    for (int k = shot.shotsFired; k < shot.shots; ++k) {
      const float at = shot.startTime + k * interval;
      if (at > executionElapsed_ + 1e-4f || at >= constants::kRoundDuration) break;
      bullets.push_back(Bullet{bursts.size(), at, k});
      burst.lastAt = at;
      ++burst.bulletsDue;
    }
    if (burst.bulletsDue > 0) bursts.push_back(std::move(burst));
  }
  std::stable_sort(bullets.begin(), bullets.end(),
                   [](const Bullet& a, const Bullet& b) { return a.at < b.at; });

  // Judge each bullet in schedule order; every bullet flies its own line,
  // scattered inside the weapon's cone (a pure hash, see ScatterUnit), and
  // its ray traces the figures in its path nearest first (the first success
  // absorbs the bullet; figures already downed by an earlier bullet of the
  // same burst are skipped), so one burst can drop several figures. A bullet
  // that connects with nobody flies on to the first wall/deck/ground it
  // meets, bent clear of any figure whose roll it just won. A locked target
  // only aims the burst -- at its torso, wherever the figure stands this
  // tick -- the bullets themselves fly exactly like free-aim ones (the
  // weapon's precision does not change with what is under the cursor), and
  // once the target is down the rest of the magazine still fires, scattered
  // around the body.
  for (Bullet& bullet : bullets) {
    Burst& b = bursts[bullet.burst];
    const int sid = b.shooter->id;
    const float scatter = glm::radians(StatsOf(b.shooter->weapon).scatterHalfAngleDegrees);
    if (!b.judged) {
      b.judged = true;
      b.aimPoint = b.target ? b.target->position +
                                  glm::vec3(0.0f, constants::kTorsoAimHeight, 0.0f)
                            : b.shot->aimPoint;
    }
    const Unit aimed = AimedAt(*b.shooter, b.aimPoint);
    bullet.from = aimed.MuzzlePosition();
    glm::vec3 base = b.aimPoint - bullet.from;
    base = glm::length(base) < 1e-4f ? aimed.FacingDirection() : glm::normalize(base);
    const float radius = std::sqrt(ScatterUnit(sid, roundNumber_, bullet.index, 1)) * scatter;
    const float theta = 6.2831853f * ScatterUnit(sid, roundNumber_, bullet.index, 2);
    glm::vec3 ray = Deflect(base, radius * std::cos(theta), radius * std::sin(theta));
    bool absorbed = false;
    float nearestMiss = 0.0f;
    for (const AimTraceCandidate& c :
         RayTraceCandidates(*b.shooter, bullet.from, ray, aimed.FacingDirection())) {
      Unit* unit = FindUnit(c.unitId);
      if (std::find(b.traceHits.begin(), b.traceHits.end(), unit) != b.traceHits.end()) {
        continue;  // Downed by an earlier bullet of this burst.
      }
      if (RollShot() < c.chance) {
        b.traceHits.push_back(unit);
        bullet.to = bullet.from + ray * c.rayT;
        absorbed = true;
        break;
      }
      if (nearestMiss == 0.0f) nearestMiss = c.rayT;
    }
    if (absorbed) continue;
    if (nearestMiss > 0.0f) {
      // Rolled a miss on a figure the line passes through: bend it clear.
      const float sign = ScatterUnit(sid, roundNumber_, bullet.index, 3) < 0.5f ? -1.0f : 1.0f;
      ray = Deflect(ray, sign * std::atan(kBodyClearance / std::max(nearestMiss, 1.0f)), 0.0f);
    }
    bullet.to = bullet.from + ray * RayReach(bullet.from, ray);
  }

  // Apply: every bullet leaves its own tracer (aged by how long ago within
  // this tick it left the barrel), then each burst's shooter plays the beat
  // and its hits go down.
  for (const Bullet& bullet : bullets) {
    RecordTracer(*bursts[bullet.burst].shooter, bullet.from, bullet.to,
                 std::max(0.0f, executionElapsed_ - bullet.at));
  }
  for (Burst& b : bursts) {
    const float sinceShot = std::max(0.0f, executionElapsed_ - b.lastAt);
    ApplyAimShot(*b.shooter, b.traceHits, b.aimPoint, sinceShot);
    b.shot->shotsFired += b.bulletsDue;
    if (b.shot->reaction) reactionAmmo_[b.shooter->id] -= b.bulletsDue;
  }

  // Drop every burst that finished (magazine level spent, or the round
  // window closed on its remaining shots) or can no longer fire: a dead
  // shooter releases the trigger, and a locked target that dies before the
  // burst could start stops being worth a bullet.
  pendingShots_.erase(
      std::remove_if(pendingShots_.begin(), pendingShots_.end(),
                     [this](const PendingShot& shot) {
                       const Unit* shooter = FindUnit(shot.shooterId);
                       if (!shooter || !shooter->alive) return true;
                       if (shot.started) {
                         if (shot.shotsFired >= shot.shots) return true;
                         const float next =
                             shot.startTime +
                             shot.shotsFired * StatsOf(shooter->weapon).shotIntervalSeconds;
                         return next >= constants::kRoundDuration;
                       }
                       if (shot.hasAimPoint) return false;
                       const Unit* target = FindUnit(shot.targetId);
                       return !target || !target->alive;
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
  // A reaction burst keeps firing only while its target stays in the
  // shooter's FOV+LOS; the unspent rounds stay in reactionAmmo_ for when the
  // enemy is sighted again.
  pendingShots_.erase(
      std::remove_if(pendingShots_.begin(), pendingShots_.end(),
                     [this](const PendingShot& shot) {
                       if (!shot.reaction) return false;
                       const Unit* shooter = FindUnit(shot.shooterId);
                       const Unit* target = FindUnit(shot.targetId);
                       return !shooter || !target || !shooter->alive || !target->alive ||
                              !CanUnitSee(*shooter, *target, scene_.obstacles,
                                          scene_.walkSurfaces, scene_.ground);
                     }),
      pendingShots_.end());

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
    if (r.shoot) StartReactionBurst(*r.actor, *r.target);
    if (r.stop) {
      for (ActiveMove& move : activeMoves_) {
        if (move.unitId == r.actor->id) move.segment = move.path.size();
      }
    }
  }
}

void GameLogic::StartReactionBurst(const Unit& shooter, const Unit& target) {
  for (const PendingShot& shot : pendingShots_) {
    if (shot.reaction && shot.shooterId == shooter.id) return;  // Already firing.
  }
  auto ammo = reactionAmmo_.find(shooter.id);
  if (ammo == reactionAmmo_.end()) {
    ammo = reactionAmmo_.emplace(shooter.id, StatsOf(shooter.weapon).magazineSize).first;
  }
  if (ammo->second <= 0) return;  // Magazine spent this round.
  PendingShot shot{shooter.id, target.id, false, glm::vec3(0.0f), ammo->second};
  shot.reaction = true;
  shot.started = true;
  shot.startTime = executionElapsed_;
  pendingShots_.push_back(shot);
}

void GameLogic::CommitRound() {
  if (!CanCommitRound()) return;
  reactionAmmo_.clear();

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
          PendingShot{unit.id, plan.shootTargetId, plan.hasAimPoint, plan.aimPoint, plan.shots});
    }
  }
  aimPreview_.reset();
  lockPreviewId_.reset();

  mode_ = InputMode::Executing;
  executionElapsed_ = 0.0f;

  // A figure already standing on the flag (e.g. a dropped one) grabs it now.
  std::vector<FlagTouch> startTouches;
  CollectStationaryFlagTouches(0.0f, &startTouches);
  GrabFlagFrom(startTouches);

  // Tick 0: bursts whose FOV/LOS is already valid at the pre-move positions
  // open fire the instant the round starts (simultaneously,
  // snapshot-judged); blocked ones stay pending and re-check as the round's
  // movement unfolds.
  ResolvePendingShots();
  SyncFlag();

  // Anyone killed at tick 0 never starts walking.
  activeMoves_.erase(std::remove_if(activeMoves_.begin(), activeMoves_.end(),
                                     [this](const ActiveMove& move) {
                                       const Unit* mover = FindUnit(move.unitId);
                                       return !mover || !mover->alive;
                                     }),
                      activeMoves_.end());

  if (flagWinner_) {
    FinishRound();
    return;
  }

  // With no movement in flight, nothing can change a still-blocked shot's
  // geometry: it expires now. The round stays executing while any opened
  // burst still has shots scheduled; otherwise it is already over.
  if (activeMoves_.empty()) {
    pendingShots_.erase(std::remove_if(pendingShots_.begin(), pendingShots_.end(),
                                       [](const PendingShot& shot) { return !shot.started; }),
                        pendingShots_.end());
    if (pendingShots_.empty()) FinishRound();
  }
}

void GameLogic::InitFlag() {
  flag_ = FlagState{};
  for (Unit& unit : scene_.units) {
    unit.carryingFlag = false;
    unit.grabElapsed = -1.0f;
  }
  const FlagConfig& config = scene_.flag;
  if (!config.enabled) return;
  flag_.enabled = true;
  glm::vec3 pos(0.0f);
  if (config.position) {
    pos = *config.position;
  } else {
    // Map center, or the nearest spot to it that no obstacle covers
    // (searched in expanding rings, so the result is deterministic).
    auto Free = [this](float x, float z) {
      if (std::abs(x) > scene_.mapHalfExtent || std::abs(z) > scene_.mapHalfExtent) return false;
      for (const Obstacle& o : scene_.obstacles) {
        const float m = constants::kAgentRadius;
        if (x > o.bounds.min.x - m && x < o.bounds.max.x + m && z > o.bounds.min.z - m &&
            z < o.bounds.max.z + m) {
          return false;
        }
      }
      return true;
    };
    constexpr float kStep = 0.5f;
    bool found = Free(0.0f, 0.0f);
    for (int ring = 1; !found && ring * kStep <= scene_.mapHalfExtent * 1.5f; ++ring) {
      float bestD = 1e30f;
      for (int ix = -ring; ix <= ring; ++ix) {
        for (int iz = -ring; iz <= ring; ++iz) {
          if (std::max(std::abs(ix), std::abs(iz)) != ring) continue;
          const float x = ix * kStep, z = iz * kStep;
          const float d = x * x + z * z;
          if (d < bestD && Free(x, z)) {
            bestD = d;
            pos = glm::vec3(x, 0.0f, z);
            found = true;
          }
        }
      }
    }
  }
  if (std::fabs(pos.y) < 1e-4f) pos.y = scene_.ground.HeightAt(pos.x, pos.z);
  flag_.position = pos;
}

glm::vec3 GameLogic::FlagPosition() const {
  if (flag_.carrierId >= 0) {
    if (const Unit* carrier = FindUnit(flag_.carrierId)) return carrier->position;
  }
  return flag_.position;
}

bool GameLogic::FlagVisibleTo(Team team) const {
  if (!flag_.enabled) return false;
  if (flag_.carrierId < 0) return true;
  const Unit* carrier = FindUnit(flag_.carrierId);
  if (!carrier) return false;
  return carrier->team == team || ComputeVisibility(team).UnitVisible(carrier->id);
}

void GameLogic::SyncFlag() {
  if (!flag_.enabled) return;
  if (flag_.carrierId >= 0) {
    const Unit* carrier = FindUnit(flag_.carrierId);
    if (carrier && carrier->alive) {
      flag_.position = carrier->position;
    } else {
      // Carrier down: the flag stays where they fell.
      if (carrier) flag_.position = carrier->position;
      flag_.carrierId = -1;
      flag_.dropElapsed = 0.0f;
    }
  }
  for (Unit& unit : scene_.units) unit.carryingFlag = unit.id == flag_.carrierId;
}

bool GameLogic::TouchesFlag(const glm::vec3& position) const {
  return std::abs(position.y - flag_.position.y) <= constants::kFlagGrabHeight &&
         glm::length(glm::vec2(position.x - flag_.position.x, position.z - flag_.position.z)) <=
             constants::kFlagGrabRadius;
}

void GameLogic::CollectStationaryFlagTouches(float arrival, std::vector<FlagTouch>* touches) const {
  if (!flag_.enabled || flag_.carrierId >= 0) return;
  for (const Unit& unit : scene_.units) {
    if (!unit.alive || !TouchesFlag(unit.position)) continue;
    bool moving = false;
    for (const ActiveMove& move : activeMoves_) {
      moving |= move.unitId == unit.id && move.segment + 1 < move.path.size();
    }
    if (!moving) touches->push_back({unit.id, arrival});
  }
}

void GameLogic::GrabFlagFrom(const std::vector<FlagTouch>& touches) {
  if (!flag_.enabled || flag_.carrierId >= 0 || touches.empty()) return;
  const FlagTouch* best = nullptr;
  for (const FlagTouch& t : touches) {
    constexpr float kTieSeconds = 1e-4f;
    if (!best || t.arrival < best->arrival - kTieSeconds ||
        (std::abs(t.arrival - best->arrival) <= kTieSeconds && t.unitId < best->unitId)) {
      best = &t;
    }
  }
  Unit* grabber = FindUnit(best->unitId);
  if (!grabber) return;
  flag_.carrierId = grabber->id;
  flag_.dropElapsed = -1.0f;
  flag_.position = grabber->position;
  grabber->grabElapsed = 0.0f;
  if (scene_.flag.winOnGrab) flagWinner_ = grabber->team;
  SyncFlag();
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

  SyncFlag();
  if (flagWinner_) {
    // Win-on-grab beats elimination: the grab happened first.
    winner_ = flagWinner_;
    flagWinner_.reset();
    mode_ = InputMode::GameOver;
    return;
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
  if (scene_.roundLimit > 0 && roundNumber_ >= scene_.roundLimit) {
    // Round limit reached with no winner: a draw (GameOver, no winner).
    winner_.reset();
    mode_ = InputMode::GameOver;
    return;
  }
  ++roundNumber_;
}

}  // namespace tactics
