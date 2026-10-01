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
    TriggerAction triggerAction = TriggerAction::None;
    PlannedActionType planType = PlannedActionType::None;
    int planShootTargetId = -1;
    std::vector<glm::vec3> planPath;
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
    bool moving = false;  // Has an in-flight move in the executing round.
    ReactionRule reactionOnStationary = ReactionRule::DoNothing;
  };
  std::vector<UnitState> units;
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
// concurrently assign one plan (Move/Shoot/Pass/Overwatch) to each of their
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
  float range;             // Hard range cap; chance is 0 at and beyond it.
  float maxChance;         // Chance at point-blank on the centerline.
};
inline constexpr ShotProfile kDefaultShotProfile{constants::kShootHalfFovDegrees,
                                                 constants::kShootRange, 0.95f};

// Pure falloff function. Chance = maxChance * angleFalloff * rangeFalloff:
// cosine falloff on angle (1 on axis, 0 at the cone edge) times linear
// falloff on distance (1 point-blank, 0 at range). Returns 0 outside the cone/range.
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

  Unit* FindUnit(int id);
  const Unit* FindUnit(int id) const;

  // Stage-B fog-of-war: which enemy figures/obstacles are currently inside
  // the combined FOV of `team`'s living figures. Recomputed on demand from
  // the current scene state (cheap given the handful of units/obstacles
  // here), so callers always see a result consistent with the latest move.
  TeamVisibility ComputeVisibility(Team team) const {
    return tactics::ComputeTeamVisibility(team, scene_.units, obstacleBounds_);
  }

  // One remembered glimpse of an enemy figure in `viewingTeam`'s FOV.
  // moveDirection is a unit vector on the XZ plane, or zero if the figure
  // was stationary when sighted.
  struct EnemySighting {
    glm::vec3 position{0.0f};
    float facingYaw = 0.0f;
    glm::vec3 moveDirection{0.0f};
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

  // Input events, driven by the input/render layer after it has resolved a
  // screen click into either a unit id or a ground-plane world point.
  // `byTeam` is the side the input came from (in split-screen, the clicked
  // pane's team): a player can only select/plan their own figures, even
  // though both teams plan at once. These only ever record/modify a figure's
  // plan; nothing executes until CommitRound().
  void ClickUnit(int unitId, Team byTeam);
  void ClickGround(const glm::vec3& point, Team byTeam);
  void HoverGround(const glm::vec3& point, Team byTeam);

  // Advances the executing round (Mode() == InputMode::Executing) by
  // `dtSeconds`: every in-flight planned move on both teams advances along
  // its own path at the mover's run speed, and every not-yet-fired planned
  // shot re-checks its live FOV/LOS, firing the first tick it connects. The
  // round finishes once no moves remain in flight (unfired shots then expire
  // as misses -- with nobody moving, their geometry can no longer change).
  // A no-op in any other mode. `main.cpp`'s frame loop drives this with real
  // frame delta; tests can pass a large dt to fast-forward to completion,
  // though mid-path events (overwatch, shots connecting mid-move) then only
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

  // Plans overwatch (triggerAction = Shoot, armed once this commits) on the
  // acting unit, the same way ChoosePass() plans a pass.
  void ChooseOverwatch();

  // Steps back one level: AwaitingMove/ShootTarget -> ActionMenu -> AwaitingSelection.
  void CancelAction();

  // Starts the round's executing phase: resolves every planned shot that
  // already connects at the pre-move positions, kicks off every planned
  // move on both teams concurrently, and arms planned overwatches. Held
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

  // Probabilistic hit resolution, applied immediately: if the shot passes
  // the hard gates (cone, range, LOS) it is fired (shooter animates) and a
  // roll against ShotHitChance decides whether the target goes down.
  // Returns true on a hit. If `fired` is non-null it is set to whether a
  // shot was actually taken (gates passed), hit or miss. Exposed directly so
  // it can be unit tested without going through the click flow; also the
  // overwatch trigger path.
  bool ResolveShot(Unit& shooter, Unit& target, bool* fired = nullptr);

  // Hit probability in [0,1]; 0 for out-of-cone, out-of-range or LOS-blocked
  // (the hard gates, unchanged). Otherwise ShotProfileHitChance of the
  // shooter's profile at the target's bearing/distance.
  float ShotHitChance(const Unit& shooter, const Unit& target) const;

  // Test seam: replaces the uniform [0,1) roll used by ResolveShot. Pass an
  // empty function to restore the default seeded RNG.
  void SetShotRollSource(std::function<float()> source) { shotRollSource_ = std::move(source); }

 private:
  // One figure's in-flight planned move; multiple can be active at once
  // since a commit animates both teams' planned moves concurrently.
  // path[segment] is the waypoint the mover last passed through;
  // path[segment + 1] is the one it's walking toward.
  struct ActiveMove {
    int unitId = -1;
    std::vector<glm::vec3> path;
    size_t segment = 0;
    float endFacingYaw = 0.0f;  // Snapped to once the path is consumed.
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

  // Checks every living enemy of `mover` armed with triggerAction == Shoot
  // for FOV+LOS on `mover`'s current (mid-move) position. On the first
  // watcher that has a shot, resolves it (killing `mover`), consumes that
  // watcher's trigger, and returns true so Update() can interrupt the move.
  bool TriggerOverwatch(Unit& mover);

  // (Re)builds navMesh_ over the area `mover` can reach this round, unless
  // the cached one already covers this figure at this position.
  void EnsureNavMeshFor(const Unit& mover);

  // Playbook reaction check, called after every per-frame position advance
  // of a moving unit: resolves a shot from any living enemy `watcher` of
  // `mover` with reactionOnStationary == Shoot that currently has `mover`
  // in FOV+LOS. Returns true (and kills `mover`) on the first such hit.
  // Unlike a one-shot trigger, the watcher's rule is never cleared here, so
  // it stays armed for future moves/rounds.
  bool CheckPlaybookReactions(Unit& mover);

  Scene scene_;
  // Range-scoped: covers only [mover.position +/- (MoveBudget + margin)],
  // clipped to the map, not the whole map. Cached per (unit id, position).
  NavMesh navMesh_;
  int navMeshUnitId_ = -1;
  glm::vec3 navMeshOrigin_{0.0f};
  int roundNumber_ = 1;
  std::vector<AABB> obstacleBounds_;  // Cached flat bounds of scene_.obstacles for LOS/FOV checks.

  InputMode mode_ = InputMode::AwaitingSelection;
  std::optional<int> selectedUnitId_;
  std::optional<Team> winner_;

  std::vector<glm::vec3> movePreviewPath_;
  bool movePreviewValid_ = false;
  ReachField moveFrontier_;

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
