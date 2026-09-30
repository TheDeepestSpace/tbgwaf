#pragma once

#include <optional>
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
  const NavMesh& GetNavMesh() const { return navMesh_; }
  InputMode Mode() const { return mode_; }
  std::optional<int> SelectedUnitId() const { return selectedUnitId_; }
  std::optional<Team> Winner() const { return winner_; }
  int RoundNumber() const { return roundNumber_; }

  // True once every living figure on *both* teams has a non-None plan,
  // i.e. CommitRound() is ready to be called.
  bool CanCommitRound() const;

  const std::vector<glm::vec3>& MovePreviewPath() const { return movePreviewPath_; }
  bool MovePreviewValid() const { return movePreviewValid_; }

  Unit* FindUnit(int id);
  const Unit* FindUnit(int id) const;

  // Stage-B fog-of-war: which enemy figures/obstacles are currently inside
  // the combined FOV of `team`'s living figures. Recomputed on demand from
  // the current scene state (cheap given the handful of units/obstacles
  // here), so callers always see a result consistent with the latest move.
  TeamVisibility ComputeVisibility(Team team) const {
    return tactics::ComputeTeamVisibility(team, scene_.units, obstacleBounds_);
  }

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

  // True while `unitId` has an in-flight planned move animating as part of
  // the executing round (figures from both teams can be animating at once).
  bool IsUnitMoving(int unitId) const;

  // Deterministic hit resolution: FOV cone + clear line-of-sight, applied
  // immediately. Exposed directly so it can be unit tested without going
  // through the click flow; also the overwatch trigger path.
  bool ResolveShot(Unit& shooter, Unit& target);

 private:
  // One figure's in-flight planned move; multiple can be active at once
  // since a commit animates both teams' planned moves concurrently.
  // path[segment] is the waypoint the mover last passed through;
  // path[segment + 1] is the one it's walking toward.
  struct ActiveMove {
    int unitId = -1;
    std::vector<glm::vec3> path;
    size_t segment = 0;
  };

  // A planned shot waiting for its first tick with valid FOV+LOS. Expires
  // (as a miss / hold-fire) if the shooter or target dies first, or if the
  // round ends with it still blocked.
  struct PendingShot {
    int shooterId = -1;
    int targetId = -1;
  };

  // FOV+LOS check only, no side effects -- the "would ResolveShot hit"
  // predicate, split out so a tick's simultaneous shots can all be judged
  // against the same snapshot before any of them is applied.
  bool ShotConnects(const Unit& shooter, const Unit& target) const;

  // Judges every pending shot against the current (start-of-resolution)
  // state, then applies all connecting hits at once: mutual shots in the
  // same tick both land. Fired shots and shots whose shooter/target died
  // are removed from pendingShots_.
  void ResolvePendingShots();

  void FinishRound();

  // Checks every living enemy of `mover` armed with triggerAction == Shoot
  // for FOV+LOS on `mover`'s current (mid-move) position. On the first
  // watcher that has a shot, resolves it (killing `mover`), consumes that
  // watcher's trigger, and returns true so Update() can interrupt the move.
  bool TriggerOverwatch(Unit& mover);

  Scene scene_;
  NavMesh navMesh_;
  int roundNumber_ = 1;
  std::vector<AABB> obstacleBounds_;  // Cached flat bounds of scene_.obstacles for LOS/FOV checks.

  InputMode mode_ = InputMode::AwaitingSelection;
  std::optional<int> selectedUnitId_;
  std::optional<Team> winner_;

  std::vector<glm::vec3> movePreviewPath_;
  bool movePreviewValid_ = false;

  // Every figure's in-flight planned move / not-yet-fired planned shot for
  // the executing round, valid only while mode_ == Executing; empty once
  // the round finishes.
  std::vector<ActiveMove> activeMoves_;
  std::vector<PendingShot> pendingShots_;
};

}  // namespace tactics
