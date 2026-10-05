#include "game/GameLogic.h"

#include <algorithm>
#include <cmath>
#include <limits>
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

const PathRide* RideAtSegment(const std::vector<PathRide>& rides, size_t segment) {
  for (const PathRide& ride : rides) {
    if (ride.segment >= 0 && static_cast<size_t>(ride.segment) == segment) return &ride;
  }
  return nullptr;
}

// Move-budget cost of a path: walked segments cost their length, ride
// segments the zipline's length-proportional ride cost.
float MoveCost(const std::vector<glm::vec3>& path, const std::vector<PathRide>& rides) {
  float cost = 0.0f;
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    const float length = glm::distance(path[i], path[i + 1]);
    cost += RideAtSegment(rides, i) ? length * constants::kZiplineCostFactor : length;
  }
  return cost;
}

// Every place that rewrites a plan's legs keeps the parallel ride lists in step.
void ClearPlannedMove(PlannedAction* plan) {
  plan->movePath.clear();
  plan->moveRides.clear();
  plan->queuedLegs.clear();
  plan->queuedLegRides.clear();
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
  ClearMoveOverlays();
  activeMoves_.clear();
  pendingShots_.clear();
  mirroredMoving_.clear();

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
    u.planRides = unit.plan.moveRides;
    u.planQueuedLegRides = unit.plan.queuedLegRides;
    u.planEndFacingYaw = unit.plan.endFacingYaw;
    u.knockdownAxis = unit.knockdownAxis;
    u.knockdownElapsed = unit.knockdownElapsed;
    u.walkPhase = unit.walkPhase;
    u.walkBlend = unit.walkBlend;
    u.idleElapsed = unit.idleElapsed;
    u.shootElapsed = unit.shootElapsed;
    u.shootAimYaw = unit.shootAimYaw;
    u.rideTravel = unit.rideTravel;
    u.rideLength = unit.rideLength;
    u.rideSlope = unit.rideSlope;
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
  unit->plan.moveRides = u.planRides;
  unit->plan.queuedLegRides = u.planQueuedLegRides;
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
    unit->rideTravel = u.rideTravel;
    unit->rideLength = u.rideLength;
    unit->rideSlope = u.rideSlope;
    ApplyPlan(u, unit);
    if (u.moving) mirroredMoving_.push_back(u.id);
  }
  playbooks_[0] = snap.playbooks[0];
  playbooks_[1] = snap.playbooks[1];
  // A new game on the simulator arrives as a snapshot with the same unit ids,
  // so it is not rejected above; detect it (round counter went backwards, or
  // a finished match is back in play) and drop the previous game's memory.
  const bool newGame = snap.roundNumber < roundNumber_ ||
                       (mode_ == InputMode::GameOver && snap.mode != InputMode::GameOver);
  mode_ = snap.mode;
  roundNumber_ = snap.roundNumber;
  if (newGame) ResetSightingMemory();
  if (snap.winner >= 0) {
    winner_ = static_cast<Team>(snap.winner);
  } else {
    winner_.reset();
  }
  selectedUnitId_.reset();
  ClearMoveOverlays();
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
        << u.rideTravel << ' ' << u.rideLength << ' ' << u.rideSlope << ' '
        << (u.moving ? 1 : 0) << ' '
        << u.planPath.size();
    for (const auto& p : u.planPath) out << ' ' << p.x << ' ' << p.y << ' ' << p.z;
    out << ' ' << u.planQueuedLegs.size();
    for (const auto& leg : u.planQueuedLegs) {
      out << ' ' << leg.size();
      for (const auto& p : leg) out << ' ' << p.x << ' ' << p.y << ' ' << p.z;
    }
    auto writeRides = [&out](const std::vector<PathRide>& rides) {
      out << ' ' << rides.size();
      for (const PathRide& r : rides) out << ' ' << r.segment << ' ' << r.zipline;
    };
    writeRides(u.planRides);
    out << ' ' << u.planQueuedLegRides.size();
    for (const auto& rides : u.planQueuedLegRides) writeRides(rides);
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
          u.walkBlend >> u.idleElapsed >> u.shootElapsed >> u.shootAimYaw >> u.rideTravel >> u.rideLength >>
          u.rideSlope >> moving >>
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
    auto readRides = [&in](std::vector<PathRide>* rides) {
      size_t count = 0;
      if (!(in >> count) || count > kMaxEntries) return false;
      rides->resize(count);
      for (PathRide& r : *rides) {
        if (!(in >> r.segment >> r.zipline)) return false;
      }
      return true;
    };
    if (!readRides(&u.planRides)) return false;
    size_t legRideCount = 0;
    if (!(in >> legRideCount) || legRideCount > kMaxEntries) return false;
    u.planQueuedLegRides.resize(legRideCount);
    for (auto& rides : u.planQueuedLegRides) {
      if (!readRides(&rides)) return false;
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
    ClearPlannedMove(&shooter->plan);
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

  const bool chaining = mover->plan.type == PlannedActionType::Move && !mover->plan.movePath.empty();
  std::vector<glm::vec3> path;
  std::vector<PathRide> rides;
  // The round executes over a fixed window, so each click (leg) can only
  // reach as far as the figure can run in one round (a zipline ride costs
  // ZiplineRideCost of that, not its length).
  if (!PlanLeg(*mover, MoveChainEnd(), point, &path, &rides)) return;

  // NavMesh::FindPath always returns at least [start, goal] on success.
  if (chaining) {
    mover->plan.queuedLegRides.resize(mover->plan.queuedLegs.size());
    mover->plan.queuedLegs.push_back(std::move(path));
    mover->plan.queuedLegRides.push_back(std::move(rides));
  } else {
    // Default the final facing to the leg's last non-degenerate segment
    // direction (what the walk animation would leave the figure facing), so
    // an untouched ghost costs nothing extra.
    mover->plan.endFacingYaw = FinalYaw(path, mover->facingYaw);
    // Plan only: nothing moves until this plan is executed by CommitRound();
    // the facing stays adjustable via SetPlannedMoveFacing() until then.
    mover->plan.type = PlannedActionType::Move;
    mover->plan.movePath = std::move(path);
    mover->plan.moveRides = std::move(rides);
    mover->plan.queuedLegs.clear();
    mover->plan.queuedLegRides.clear();
    mover->plan.shootTargetId = -1;
  }
  // Stay in this mode with the figure selected: the next click chains
  // another leg, FinishMovePlan() ends it.
  RefreshMoveFrontier();
  movePreviewPath_.clear();
  movePreviewRides_.clear();
  movePreviewValid_ = false;
}

void GameLogic::FinishMovePlan() {
  if (mode_ != InputMode::AwaitingMoveDestination) return;
  const Unit* mover = FindUnit(selectedUnitId_.value_or(-1));
  if (!mover || mover->plan.type != PlannedActionType::Move) return;
  selectedUnitId_.reset();
  ClearMoveOverlays();
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
    const float target =
        unit.alive && IsUnitMoving(unit.id) && !IsUnitRiding(unit.id) ? 1.0f : 0.0f;
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
      move.ridingZipline = -1;
      mover->rideTravel = -1.0f;
      continue;
    }

    // `remaining` is move budget, the same currency a plan was priced in:
    // walking spends one unit per world unit, a zipline ride only
    // kZiplineCostFactor per world unit (so it covers ground faster).
    float remaining = dtSeconds * mover->runSpeed;
    while (remaining > 0.0f && move.segment + 1 < move.path.size()) {
      const glm::vec3& segStart = move.path[move.segment];
      const glm::vec3& segEnd = move.path[move.segment + 1];
      const PathRide* ride = RideAtSegment(move.rides, move.segment);
      if (ride && move.ridingZipline != ride->zipline) {
        // One rider at a time: wait at the anchor until the line is free.
        const bool occupied = std::any_of(
            activeMoves_.begin(), activeMoves_.end(), [&](const ActiveMove& other) {
              return other.unitId != move.unitId && other.ridingZipline == ride->zipline;
            });
        if (occupied) break;
        move.ridingZipline = ride->zipline;
      }
      const float costScale = ride ? constants::kZiplineCostFactor : 1.0f;

      const glm::vec3 segDelta = segEnd - segStart;
      if (glm::length(glm::vec2(segDelta.x, segDelta.z)) > 1e-4f) {
        mover->facingYaw = std::atan2(segDelta.z, segDelta.x);
      }

      const glm::vec3 toEnd = segEnd - mover->position;
      const float distToEnd = glm::length(toEnd);
      const float costToEnd = distToEnd * costScale;
      float step;
      if (costToEnd <= remaining) {
        step = distToEnd;
        mover->position = segEnd;
        remaining -= costToEnd;
        ++move.segment;
        if (ride) {
          move.ridingZipline = -1;
          mover->rideTravel = -1.0f;
        }
      } else {
        step = remaining / costScale;
        mover->position += (toEnd / distToEnd) * step;
        remaining = 0.0f;
      }
      if (ride) {
        // Hanging from the cable: no stride, the rig poses off the ride state.
        if (move.ridingZipline >= 0) {
          const float run = glm::length(glm::vec2(segDelta.x, segDelta.z));
          mover->rideTravel = glm::distance(segStart, mover->position);
          mover->rideLength = glm::length(segDelta);
          mover->rideSlope = run > 1e-4f ? segDelta.y / run : 0.0f;
        }
        continue;
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
  movePreviewRides_.clear();
  movePreviewValid_ = false;
  if (mode_ != InputMode::AwaitingMoveDestination) return;
  const Unit* mover = FindUnit(selectedUnitId_.value_or(-1));
  if (!mover || mover->team != byTeam) return;
  movePreviewValid_ = PlanLeg(*mover, MoveChainEnd(), point, &movePreviewPath_, &movePreviewRides_);
}

// Any manual touch of a figure's plan drops its queued multi-round route.
void GameLogic::ClearQueuedLegs(std::optional<int> unitId) {
  if (Unit* unit = FindUnit(unitId.value_or(-1))) {
    unit->plan.queuedLegs.clear();
    unit->plan.queuedLegRides.clear();
  }
}

// Frontier of where the next leg can reach: built around the chain end.
void GameLogic::RefreshMoveFrontier() {
  moveFrontier_ = ReachField();
  ziplineFrontiers_.clear();
  ziplineMeshes_.clear();
  ziplineUnitId_ = -1;
  if (const Unit* mover = FindUnit(selectedUnitId_.value_or(-1))) {
    const glm::vec3 origin = MoveChainEnd();
    EnsureNavMeshFor(*mover, origin);
    moveFrontier_ = navMesh_.ComputeReachField(origin, mover->MoveBudget());
    BuildZiplineFrontiers(*mover, origin);
  }
}

const std::vector<GameLogic::ZiplineFrontier>& GameLogic::ZiplineFrontiers() const {
  static const std::vector<ZiplineFrontier> kNone;
  return mode_ == InputMode::AwaitingMoveDestination ? ziplineFrontiers_ : kNone;
}

bool GameLogic::IsUnitRiding(int unitId) const {
  for (const ActiveMove& move : activeMoves_) {
    if (move.unitId == unitId && move.ridingZipline >= 0) return true;
  }
  return false;
}

void GameLogic::ClearMoveOverlays() {
  moveFrontier_ = ReachField();
  ziplineFrontiers_.clear();
  ziplineMeshes_.clear();
  ziplineUnitId_ = -1;
  movePreviewPath_.clear();
  movePreviewRides_.clear();
  movePreviewValid_ = false;
}

// Zipline edges of the nav graph for a leg starting at `origin`: for each
// line end the mover can walk to with budget to spare for the ride, the walk
// region around the opposite end with whatever budget is left.
void GameLogic::BuildZiplineFrontiers(const Unit& mover, const glm::vec3& origin) {
  ziplineFrontiers_.clear();
  ziplineMeshes_.clear();
  ziplineUnitId_ = mover.id;
  ziplineOrigin_ = origin;
  EnsureNavMeshFor(mover, origin);
  const float budget = mover.MoveBudget();
  const float half = scene_.mapHalfExtent;
  for (size_t i = 0; i < scene_.ziplines.size(); ++i) {
    const Zipline& line = scene_.ziplines[i];
    const float rideCost = ZiplineRideCost(line);
    for (int end = 0; end < 2; ++end) {
      const glm::vec3 entry = end == 0 ? line.a : line.b;
      const glm::vec3 exit = end == 0 ? line.b : line.a;
      std::vector<glm::vec3> walk;
      if (!navMesh_.FindPath(origin, entry, &walk)) continue;
      const float walkCost = PathLength(walk);
      const float remaining = budget - walkCost - rideCost;
      if (remaining < 0.0f) continue;

      // Window the onward mesh around the exit, as EnsureNavMeshFor does.
      const float reach = remaining + 2.0f * constants::kAgentRadius;
      NavRegion region;
      region.xMin = std::max(-half, exit.x - reach);
      region.xMax = std::min(half, exit.x + reach);
      region.zMin = std::max(-half, exit.z - reach);
      region.zMax = std::min(half, exit.z + reach);
      NavMesh mesh;
      mesh.Build(scene_.obstacles, region, constants::kAgentRadius, &scene_.ground,
                 &scene_.walkSurfaces);
      ZiplineFrontier frontier;
      frontier.zipline = static_cast<int>(i);
      frontier.entry = entry;
      frontier.exit = exit;
      frontier.walkCost = walkCost;
      frontier.rideCost = rideCost;
      frontier.field = mesh.ComputeReachField(exit, remaining);
      ziplineFrontiers_.push_back(std::move(frontier));
      ziplineMeshes_.push_back(std::move(mesh));
    }
  }
}

bool GameLogic::PlanLeg(const Unit& mover, const glm::vec3& from, const glm::vec3& goal,
                        std::vector<glm::vec3>* path, std::vector<PathRide>* rides) {
  path->clear();
  rides->clear();
  const float budget = mover.MoveBudget();
  EnsureNavMeshFor(mover, from);
  // Walking wins whenever it fits the budget; ziplines only extend reach.
  if (navMesh_.FindPath(from, goal, path) && PathLength(*path) <= budget) return true;
  path->clear();

  if (ziplineUnitId_ != mover.id || ziplineOrigin_ != from) BuildZiplineFrontiers(mover, from);
  float bestCost = std::numeric_limits<float>::infinity();
  for (size_t i = 0; i < ziplineFrontiers_.size(); ++i) {
    const ZiplineFrontier& frontier = ziplineFrontiers_[i];
    std::vector<glm::vec3> walkIn, walkOut;
    if (!ziplineMeshes_[i].FindPath(frontier.exit, goal, &walkOut)) continue;
    const float total = frontier.walkCost + frontier.rideCost + PathLength(walkOut);
    if (total > budget || total >= bestCost) continue;
    if (!navMesh_.FindPath(from, frontier.entry, &walkIn)) continue;
    bestCost = total;
    walkIn.back() = frontier.entry;
    walkOut.front() = frontier.exit;
    rides->assign(1, PathRide{static_cast<int>(walkIn.size()) - 1, frontier.zipline});
    *path = std::move(walkIn);
    path->insert(path->end(), walkOut.begin(), walkOut.end());
  }
  return std::isfinite(bestCost);
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
  movePreviewRides_.clear();
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
  ClearPlannedMove(&unit->plan);
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
    ClearMoveOverlays();
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
  if (IsUnitRiding(shooter.id)) return 0.0f;  // Hands on the cable: no shooting while riding.
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
  if (hit) {
    target.alive = false;
    // up x dir: tipping around this axis leans the figure toward dir.
    target.knockdownAxis = glm::vec3(dir.z, 0.0f, -dir.x);
    target.knockdownElapsed = 0.0f;
  }
}

void GameLogic::ResolvePendingShots() {
  // Judge every held shot against the same snapshot (positions don't change
  // during resolution, and hits are applied only after all are judged), so
  // two figures whose shots connect on the same tick both fire: a mutual
  // kill downs both, rather than whichever happens to resolve first
  // silencing the other.
  // A held shot is taken (and consumed, hit or miss) the first tick it
  // passes the hard gates.
  std::vector<std::pair<Unit*, Unit*>> firing;
  for (const PendingShot& shot : pendingShots_) {
    Unit* shooter = FindUnit(shot.shooterId);
    Unit* target = FindUnit(shot.targetId);
    if (!shooter || !target || !shooter->alive || !target->alive) continue;
    if (ShotConnects(*shooter, *target)) firing.emplace_back(shooter, target);
  }
  // Rolls are applied in order but each hit only flips the target's alive
  // flag after all were judged gate-wise, so mutual shots still both fire.
  std::vector<bool> hits;
  for (auto& [shooter, target] : firing) hits.push_back(RollShot() < ShotHitChance(*shooter, *target));
  for (size_t i = 0; i < firing.size(); ++i) ApplyShot(*firing[i].first, *firing[i].second, hits[i]);

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
    // A rider is committed to the line: it neither reacts nor halts mid-air.
    if (IsUnitRiding(unit.id)) continue;
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
    if (plan.type == PlannedActionType::Move) {
      unit.plan.queuedLegs = plan.queuedLegs;
      unit.plan.queuedLegRides = plan.queuedLegRides;
    }

    if (plan.type == PlannedActionType::Move) {
      activeMoves_.push_back(
          ActiveMove{unit.id, plan.movePath, 0, plan.endFacingYaw, plan.moveRides, -1});
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
    unit.plan.queuedLegRides.resize(unit.plan.queuedLegs.size());
    unit.plan.movePath = std::move(unit.plan.queuedLegs.front());
    unit.plan.moveRides = std::move(unit.plan.queuedLegRides.front());
    unit.plan.queuedLegs.erase(unit.plan.queuedLegs.begin());
    unit.plan.queuedLegRides.erase(unit.plan.queuedLegRides.begin());
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
