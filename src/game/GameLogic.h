#pragma once

#include <functional>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "game/NavMesh.h"
#include "game/Scene.h"
#include "game/Types.h"
#include "game/Unit.h"
#include "game/Visibility.h"

namespace tactics {

enum class InputMode {
  AwaitingSelection,      // Waiting for a click on one of either team's living figures.
  ActionMenu,             // A figure is selected; waiting for Move/Shoot/Pass to plan its action.
  AwaitingMoveDestination,  // Waiting for a ground click to plan a move to.
  AwaitingShootTarget,    // Waiting for a click on an enemy figure to plan a shot at.
  Executing,              // A committed round is playing out; planning input is inert.
  GameOver,
};

// Returns the winning team if exactly one side has living units, or
// std::nullopt if neither side has won outright (both sides have survivors,
// or -- a genuine possibility under simultaneous execution -- both sides
// were wiped out in the same round, which GameLogic treats as a draw).
std::optional<Team> CheckWinner(const std::vector<Unit>& units);

// Serializable dynamic match state (everything that changes after Reset()
// on a fixed scene), used by two-canvas web play: each canvas runs its own
// GameLogic instance for one team. The instance for Blue is the simulator
// (it alone commits and executes rounds); the Red instance mirrors its
// snapshots instead of re-simulating so float divergence can't desync the
// two. During planning each side only ships its own team's plans.
struct GameSnapshot {
  struct UnitState {
    int id = -1;
    glm::vec3 position{0.0f};
    float facingYaw = 0.0f;
    bool alive = true;
    PlannedActionType planType = PlannedActionType::None;
    int planShootTargetId = -1;
    std::vector<glm::vec3> planPath;
    std::vector<std::vector<glm::vec3>> planQueuedLegs;
    std::vector<PathRide> planRides;
    std::vector<std::vector<PathRide>> planQueuedLegRides;
    float planEndFacingYaw = 0.0f;
    glm::vec3 knockdownAxis{1.0f, 0.0f, 0.0f};
    float knockdownElapsed = -1.0f;
    // Figure animation state (see Unit): a follower mirrors these rather
    // than re-simulating, so its panes play the same walk/shoot beats.
    float walkPhase = 0.0f;
    float walkBlend = 0.0f;
    float idleElapsed = 0.0f;
    float shootElapsed = -1.0f;
    float shootAimYaw = 0.0f;
    float rideTravel = -1.0f;
    float rideLength = 0.0f;
    float rideSlope = 0.0f;
    bool moving = false;  // Has an in-flight move in the executing round.
  };
  std::vector<UnitState> units;
  SquadPlaybook playbooks[2];  // Indexed by Team.
  InputMode mode = InputMode::AwaitingSelection;
  int roundNumber = 1;
  int winner = -1;  // -1 = none, else static_cast<int>(Team).
};

// Text encoding of a snapshot (for the page-level message bus). Deserialize
// returns false on malformed input.
std::string SerializeSnapshot(const GameSnapshot& snapshot);
bool DeserializeSnapshot(const std::string& text, GameSnapshot* out);

// Owns the whole game state machine: scene, navmesh, round phases, and the
// click-driven plan-then-commit flow. Deliberately free of any SDL/GL/ImGui
// dependency so it can be driven and verified headlessly.
//
// WEGO round model: each round has two phases. During *planning*, both teams
// concurrently assign one plan (Move/Shoot/Pass) to each of their
// living figures -- nothing happens yet, and either team may revise its own
// figures' plans at any time (input calls carry the acting team, so a player
// can only ever plan their own side). Once every living figure on both teams
// has a plan, CommitRound() starts the *executing* phase: every planned move
// on both teams animates concurrently (bounded by each figure's MoveBudget()
// so it fits the round's fixed kRoundDuration window), and every planned
// shot re-checks the shooter's live FOV/LOS each tick, firing at the first
// instant it connects -- so a target that walks into a shooter's cone
// mid-round can be hit. All shots that connect in the same tick are judged
// against the same tick-start snapshot, so two figures shooting each other
// simultaneously both die. The round ends once nothing is left in flight,
// and the next round's planning begins.
// Shape of a shooter's hit-probability cone. Only one profile is wired up
// today (kDefaultShotProfile); the seam for per-role shapes (sniper: narrow,
// long, accurate; infantry: wide, short, forgiving) is that ShotHitChance
// takes its numbers from a profile rather than from the constants directly.
struct ShotProfile {
  float halfAngleDegrees;  // Hard cone edge; chance is 0 at and beyond it.
  float range;             // Falloff scale, not a cap: distance falloff is 0.5 here, never 0.
  float maxChance;         // Chance at point-blank on the centerline.
};
inline constexpr ShotProfile kDefaultShotProfile{constants::kShootHalfFovDegrees,
                                                 constants::kShootRange, 0.95f};

// Pure falloff function. Chance = maxChance * angleFalloff * rangeFalloff:
// cosine falloff on angle (1 on axis, 0 at the cone edge) times linear
// falloff on distance, 1/(1 + d/range): 1 point-blank, 0.5 at range, no hard cap.
// Returns 0 outside the cone.
// Opacity of the shot-cone overlay at `distance` from the gun tip: kConeStartAlpha
// at the tip, falling linearly to 0 at the profile's range (and 0 beyond).
float ShotConeAlpha(const ShotProfile& profile, float distance);

float ShotProfileHitChance(const ShotProfile& profile, float angleDegrees, float distance);

class GameLogic {
 public:
  GameLogic() { Reset(); }
  // Drives the same state machine over a caller-supplied scene instead of
  // BuildDefaultScene(), so tests (e.g. YAML gameplay scenarios) can exercise
  // arbitrary maps/unit layouts without duplicating any game logic.
  explicit GameLogic(Scene scene) { Reset(std::move(scene)); }

  void Reset();
  void Reset(Scene scene);

  const Scene& GetScene() const { return scene_; }
  // The navmesh currently in use: windowed around the figure most recently
  // planned for (see EnsureNavMeshFor), empty until one has been.
  const NavMesh& GetNavMesh() const { return navMesh_; }
  InputMode Mode() const { return mode_; }
  std::optional<int> SelectedUnitId() const { return selectedUnitId_; }
  std::optional<Team> Winner() const { return winner_; }
  int RoundNumber() const { return roundNumber_; }

  // True once every living figure on *both* teams has a non-None plan,
  // i.e. CommitRound() is ready to be called.
  bool CanCommitRound() const;
  // True while any killed unit is still mid-fall (knockdown animation running).
  bool HasActiveKnockdown() const;

  const std::vector<glm::vec3>& MovePreviewPath() const { return movePreviewPath_; }
  bool MovePreviewValid() const { return movePreviewValid_; }

  // Reachable-area field for the selected figure's move budget, computed once
  // when entering move-destination mode (null otherwise).
  const ReachField* MoveFrontier() const {
    return mode_ == InputMode::AwaitingMoveDestination && moveFrontier_.nx > 0 ? &moveFrontier_
                                                                              : nullptr;
  }

  // Zipline region of the move frontier: where the selected figure can get by
  // walking to one end of a zipline (`walkCost`), riding it (`rideCost`) and
  // walking on from the other end with whatever budget remains. `field` is
  // that onward walk region around `exit`. One entry per (zipline, entry
  // end) whose entry is affordable.
  struct ZiplineFrontier {
    int zipline = -1;
    glm::vec3 entry{0.0f};
    glm::vec3 exit{0.0f};
    float walkCost = 0.0f;
    float rideCost = 0.0f;
    ReachField field;
  };
  // Valid while planning a move (empty otherwise).
  const std::vector<ZiplineFrontier>& ZiplineFrontiers() const;

  // Rides in the current hover preview (MovePreviewPath()'s traverse steps).
  const std::vector<PathRide>& MovePreviewRides() const { return movePreviewRides_; }

  // True while `unitId` is mid-ride on a zipline in the executing round.
  bool IsUnitRiding(int unitId) const;

  Unit* FindUnit(int id);
  const Unit* FindUnit(int id) const;

  // Stage-B fog-of-war: which enemy figures/obstacles are currently inside
  // the combined FOV of `team`'s living figures. Recomputed on demand from
  // the current scene state (cheap given the handful of units/obstacles
  // here), so callers always see a result consistent with the latest move.
  TeamVisibility ComputeVisibility(Team team) const {
    return tactics::ComputeTeamVisibility(team, scene_.units, scene_.obstacles,
                                          scene_.walkSurfaces, scene_.ground);
  }

  // One remembered glimpse of an enemy figure in `viewingTeam`'s FOV.
  // moveDirection is a unit vector on the XZ plane, or zero if the figure
  // was stationary when sighted.
  struct EnemySighting {
    glm::vec3 position{0.0f};
    float facingYaw = 0.0f;
    glm::vec3 moveDirection{0.0f};
    // Animation state at sample time, so the ghost holds the captured pose.
    float walkPhase = 0.0f;
    float walkBlend = 0.0f;
    float idleElapsed = 0.0f;
    int ageRounds = 0;  // Completed rounds since the sample was taken.
  };
  // Oldest-first samples of `targetUnitId` as seen by `viewingTeam`; empty
  // once all have aged past kSightingMemoryRounds.
  const std::vector<EnemySighting>& Sightings(Team viewingTeam, int targetUnitId) const;

  // Ages sighting memory by completed rounds (tracked via the round number, so
  // followers age too) and samples newly visible enemies. Must be called
  // every frame on every page regardless of mode or simulator/follower role
  // (unlike Update(), a follower never runs the physics tick during
  // Executing, yet still needs its own memory built from imported state).
  void UpdateSightingMemory(float dtSeconds);
  void ResetSightingMemory();

  // Input events, driven by the input/render layer after it has resolved a
  // screen click into either a unit id or a ground-plane world point.
  // `byTeam` is the side the input came from (in split-screen, the clicked
  // pane's team): a player can only select/plan their own figures, even
  // though both teams plan at once. These only ever record/modify a figure's
  // plan; nothing executes until CommitRound().
  void ClickUnit(int unitId, Team byTeam);
  // In AwaitingMoveDestination each click adds one more leg (at most one
  // round's MoveBudget() from the previous leg's end; farther clicks are
  // ignored). The figure stays selected so the next click chains another leg,
  // until FinishMovePlan().
  void ClickGround(const glm::vec3& point, Team byTeam);
  // Ends the chaining started by ChooseMove(); a no-op unless a leg is planned.
  void FinishMovePlan();
  // End of the chain planned so far (the figure's position if none yet): the
  // origin of the next leg.
  glm::vec3 MoveChainEnd() const;
  void HoverGround(const glm::vec3& point, Team byTeam);

  // Advances the executing round (Mode() == InputMode::Executing) by
  // `dtSeconds`: every in-flight planned move on both teams advances along
  // its own path at the mover's run speed, and every not-yet-fired planned
  // shot re-checks its live FOV/LOS, firing the first tick it connects. The
  // round finishes once no moves remain in flight (unfired shots then expire
  // as misses -- with nobody moving, their geometry can no longer change).
  // A no-op in any other mode. `main.cpp`'s frame loop drives this with real
  // frame delta; tests can pass a large dt to fast-forward to completion,
  // though mid-path events (shots connecting mid-move) then only
  // sample the coarse positions that dt steps through.
  void Update(float dtSeconds);

  // Action menu choices, valid only while Mode() == ActionMenu. Each records
  // a plan on the selected figure and returns to unit selection within the
  // still-active planning phase.
  void ChooseMove();
  void ChooseShoot();
  void ChoosePass();

  // Re-aims a planned move's final facing. Allowed at any time before the
  // round is committed, by the unit's own team; no-op for units without a planned move.
  void SetPlannedMoveFacing(int unitId, float yaw, Team byTeam);

  // Steps back one level: AwaitingMove/ShootTarget -> ActionMenu -> AwaitingSelection.
  void CancelAction();

  // Starts the round's executing phase: resolves every planned shot that
  // already connects at the pre-move positions, kicks off every planned
  // move on both teams concurrently. Held
  // shots keep re-checking each Update() tick. Finishes immediately (in
  // this same call) if nobody planned a move. No-op unless CanCommitRound().
  void CommitRound();

  // Snapshot of the dynamic match state. ImportState overwrites the whole
  // match (a follower mirroring the simulator) and returns false, leaving
  // state untouched, if the snapshot doesn't match this scene's units.
  // ImportTeamPlans only copies `team`'s figures' plans (planning phase:
  // learning what the other side has planned) and leaves everything else,
  // including local selection, alone; it too returns false on mismatch.
  GameSnapshot ExportState() const;
  bool ImportState(const GameSnapshot& snapshot);
  bool ImportTeamPlans(const GameSnapshot& snapshot, Team team);

  // True while `unitId` has an in-flight planned move animating as part of
  // the executing round (figures from both teams can be animating at once).
  bool IsUnitMoving(int unitId) const;

  // Squad-wide standing reaction table for `team` (see SquadPlaybook).
  const SquadPlaybook& Playbook(Team team) const { return playbooks_[static_cast<int>(team)]; }
  void SetPlaybook(Team team, const SquadPlaybook& playbook) {
    playbooks_[static_cast<int>(team)] = playbook;
  }

  // Probabilistic hit resolution, applied immediately: if the shot passes
  // the hard gates (cone, LOS) it is fired (shooter animates) and a
  // roll against ShotHitChance decides whether the target goes down.
  // Returns true on a hit. If `fired` is non-null it is set to whether a
  // shot was actually taken (gates passed), hit or miss. Exposed directly so
  // it can be unit tested without going through the click flow; also the
  // playbook reaction path.
  bool ResolveShot(Unit& shooter, Unit& target, bool* fired = nullptr);

  // Hit probability in [0,1]; 0 for out-of-cone or LOS-blocked
  // (the hard gates, unchanged). Otherwise ShotProfileHitChance of the
  // shooter's profile at the target's bearing/distance.
  float ShotHitChance(const Unit& shooter, const Unit& target) const;

  // Test seam: replaces the uniform [0,1) roll used by ResolveShot. Pass an
  // empty function to restore the default seeded RNG.
  void SetShotRollSource(std::function<float()> source) { shotRollSource_ = std::move(source); }

 private:
  void ClearQueuedLegs(std::optional<int> unitId);
  // One figure's in-flight planned move; multiple can be active at once
  // since a commit animates both teams' planned moves concurrently.
  // path[segment] is the waypoint the mover last passed through;
  // path[segment + 1] is the one it's walking toward.
  struct ActiveMove {
    int unitId = -1;
    std::vector<glm::vec3> path;
    size_t segment = 0;
    float endFacingYaw = 0.0f;  // Snapped to once the path is consumed.
    std::vector<PathRide> rides;
    int ridingZipline = -1;  // Zipline currently occupied by this rider, else -1.
  };

  // A planned shot waiting for its first tick with valid FOV+LOS. Expires
  // (as a miss / hold-fire) if the shooter or target dies first, or if the
  // round ends with it still blocked.
  struct PendingShot {
    int shooterId = -1;
    int targetId = -1;
  };

  // Gate check only, no side effects (ShotHitChance > 0): "can this shooter
  // take the shot at all". Split out so a tick's simultaneous shots can all
  // be judged against the same snapshot before any of them is applied.
  bool ShotConnects(const Unit& shooter, const Unit& target) const;

  float RollShot();
  // Applies a taken shot: shooter animation, plus knockdown if `hit`.
  void ApplyShot(Unit& shooter, Unit& target, bool hit);

  // Judges every pending shot against the current (start-of-resolution)
  // state, then applies all connecting hits at once: mutual shots in the
  // same tick both land. Fired shots and shots whose shooter/target died
  // are removed from pendingShots_.
  void ResolvePendingShots();

  void FinishRound();

  // The Executing-mode body of Update(): advances every in-flight move,
  // re-checks held shots, and finishes the round once nothing is in flight.
  void AdvanceExecutingRound(float dtSeconds);

  // (Re)builds navMesh_ over the area `mover` can reach from `origin` (its
  // position, or the end of its planned move chain) in one leg, unless
  // the cached one already covers this figure at this origin.
  void EnsureNavMeshFor(const Unit& mover, const glm::vec3& origin);
  void RefreshMoveFrontier();
  // Rebuilds ziplineFrontiers_ (and their navmeshes) for a leg from `origin`.
  void BuildZiplineFrontiers(const Unit& mover, const glm::vec3& origin);
  // Plans one leg from `from` to `goal`: a plain walk when affordable within
  // the mover's budget, otherwise walk -> ride -> walk over the cheapest
  // zipline. False if neither fits. Fills `rides` (empty for a walk).
  bool PlanLeg(const Unit& mover, const glm::vec3& from, const glm::vec3& goal,
               std::vector<glm::vec3>* path, std::vector<PathRide>* rides);
  void ClearMoveOverlays();

  // Squad-playbook pass, run once per executing tick after every mover has
  // advanced: for each living figure with a living enemy in its FOV+LOS,
  // looks up its team's table by (moving?, any sighted enemy sees it back?)
  // and shoots the nearest sighted enemy and/or cuts the figure's move short.
  // All reactions are judged against the same snapshot, then applied, so
  // mutual shots both land.
  void ApplyPlaybookReactions();

  Scene scene_;
  SquadPlaybook playbooks_[2];  // Indexed by Team; survives Reset().
  // Range-scoped: covers only [origin +/- (MoveBudget + margin)],
  // clipped to the map, not the whole map. Cached per (unit id, position).
  NavMesh navMesh_;
  int navMeshUnitId_ = -1;
  glm::vec3 navMeshOrigin_{0.0f};
  int roundNumber_ = 1;
  std::vector<AABB> obstacleBounds_;  // Retained for compatibility/debug broad-phase inspection.

  InputMode mode_ = InputMode::AwaitingSelection;
  std::optional<int> selectedUnitId_;
  std::optional<Team> winner_;

  std::vector<glm::vec3> movePreviewPath_;
  bool movePreviewValid_ = false;
  ReachField moveFrontier_;
  std::vector<ZiplineFrontier> ziplineFrontiers_;
  std::vector<NavMesh> ziplineMeshes_;  // Parallel to ziplineFrontiers_: mesh around each exit.
  int ziplineUnitId_ = -1;
  glm::vec3 ziplineOrigin_{0.0f};
  std::vector<PathRide> movePreviewRides_;

  // Every figure's in-flight planned move / not-yet-fired planned shot for
  // the executing round, valid only while mode_ == Executing; empty once
  // the round finishes.
  std::vector<ActiveMove> activeMoves_;
  std::vector<PendingShot> pendingShots_;
  std::mt19937 shotRng_{0x5eedu};
  std::function<float()> shotRollSource_;
  // Ids of figures a simulating peer reports as mid-move (ImportState only;
  // a follower has no activeMoves_ of its own).
  std::vector<int> mirroredMoving_;

  // Sighting memory, indexed [viewing team][target unit id].
  std::vector<std::vector<EnemySighting>> sightings_[2];
  int lastSightingRound_ = 1;
  std::vector<bool> sightedLastFrame_[2];
  std::vector<float> sightingTimer_[2];
  std::vector<glm::vec3> lastUnitPosition_;  // Previous frame's position per unit id.
  bool hasLastUnitPosition_ = false;
};

}  // namespace tactics
