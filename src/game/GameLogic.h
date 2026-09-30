#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "game/NavMesh.h"
#include "game/Scene.h"
#include "game/TurnManager.h"
#include "game/Types.h"
#include "game/Unit.h"
#include "game/Visibility.h"

namespace tactics {

enum class InputMode {
  AwaitingSelection,      // Waiting for a click on one of the acting team's living figures.
  ActionMenu,             // A figure is selected; waiting for Move/Shoot/Pass to plan its action.
  AwaitingMoveDestination,  // Waiting for a ground click to plan a move to.
  AwaitingShootTarget,    // Waiting for a click on an enemy figure to plan a shot at.
  Moving,                 // A planned move is animating as part of a turn commit.
  GameOver,
};

// Serializable dynamic match state (everything that changes after Reset()
// on a fixed scene). Used to mirror one authoritative match into a second,
// non-simulating instance (two-pane play): the follower imports snapshots
// instead of re-simulating, so float divergence can't desync the two.
struct GameSnapshot {
  struct UnitState {
    int id = -1;
    glm::vec3 position{0.0f};
    float facingYaw = 0.0f;
    bool alive = true;
    TriggerAction triggerAction = TriggerAction::None;
    PlannedActionType planType = PlannedActionType::None;  // Move paths are not mirrored.
    int planShootTargetId = -1;
    glm::vec3 knockdownAxis{1.0f, 0.0f, 0.0f};
    float knockdownElapsed = -1.0f;
  };
  std::vector<UnitState> units;
  TurnManager::State turn;
  InputMode mode = InputMode::AwaitingSelection;
  int selectedUnitId = -1;  // -1 = none.
  int winner = -1;          // -1 = none, else static_cast<int>(Team).
};

// Text encoding of a snapshot (for the page-level message bus). Deserialize
// returns false on malformed input.
std::string SerializeSnapshot(const GameSnapshot& snapshot);
bool DeserializeSnapshot(const std::string& text, GameSnapshot* out);

// Owns the whole game state machine: scene, navmesh, turn order, and the
// click-driven plan-then-commit flow. Deliberately free of any SDL/GL/ImGui
// dependency so it can be driven and verified headlessly.
//
// Turn model: on its turn, a team assigns one plan (Move/Shoot/Pass/Overwatch)
// to each of its living figures -- nothing happens yet. Once every living figure on
// the team has a plan, CommitTurn() resolves them all at once: every planned
// shot is judged against the same pre-commit snapshot of the enemy team (so
// one figure's shot never depends on whether an ally's shot in the same
// commit already landed), and every planned move animates concurrently
// rather than one figure waiting for the last to finish. The turn then
// passes to the other team once every planned move has finished animating.
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
  Team CurrentTeam() const { return turnManager_.CurrentTeam(); }
  std::optional<Team> Winner() const { return winner_; }
  int RoundNumber() const { return turnManager_.RoundNumber(); }

  // True once every living figure on the current team has a non-None plan,
  // i.e. CommitTurn() is ready to be called.
  bool CanCommitTurn() const;
  // True while any killed unit is still mid-fall (knockdown animation running).
  bool HasActiveKnockdown() const;

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
  // screen click into either a unit id or a ground-plane world point. These
  // only ever record/modify a figure's plan; nothing executes until
  // CommitTurn().
  void ClickUnit(int unitId);
  void ClickGround(const glm::vec3& point);
  void HoverGround(const glm::vec3& point);

  // Advances every in-flight planned move (Mode() == InputMode::Moving) by
  // `dtSeconds` at once, moving each animating figure along its own planned
  // path at constant speed, and finishes the commit once every move's path
  // is consumed. A no-op in any other mode. `main.cpp`'s frame loop drives
  // this with real frame delta; tests can pass a large dt to fast-forward to
  // completion.
  void Update(float dtSeconds);

  // Action menu choices, valid only while Mode() == ActionMenu. Each records
  // a plan on the selected figure and returns to unit selection within the
  // still-active team's turn.
  void ChooseMove();
  void ChooseShoot();
  void ChoosePass();

  // Plans overwatch (triggerAction = Shoot, armed once this commits) on the
  // acting unit, the same way ChoosePass() plans a pass.
  void ChooseOverwatch();

  // Snapshot of the dynamic match state. ImportState overwrites this
  // instance's state with it (any in-flight move/preview is dropped) and
  // returns false, leaving state untouched, if the snapshot doesn't match
  // this scene's units.
  GameSnapshot ExportState() const;
  bool ImportState(const GameSnapshot& snapshot);

  // Steps back one level: AwaitingMove/ShootTarget -> ActionMenu -> AwaitingSelection.
  void CancelAction();

  // Resolves every one of the current team's planned shots simultaneously
  // and kicks off every planned move's animation concurrently, then hands
  // the turn to the other team once all move animations finish (immediately,
  // in this same call, if nobody planned a move). No-op unless
  // CanCommitTurn().
  void CommitTurn();

  // True while `unitId` has an in-flight planned move animating as part of
  // the current commit (multiple figures can be animating at once).
  bool IsUnitMoving(int unitId) const;

  // Deterministic hit resolution: FOV cone + clear line-of-sight. Exposed
  // directly so it can be unit tested without going through the click flow.
  bool ResolveShot(Unit& shooter, Unit& target);

 private:
  // One figure's in-flight planned move; multiple can be active at once
  // since a commit animates the whole team's planned moves concurrently.
  // path[segment] is the waypoint the mover last passed through;
  // path[segment + 1] is the one it's walking toward.
  struct ActiveMove {
    int unitId = -1;
    std::vector<glm::vec3> path;
    size_t segment = 0;
  };

  void FinishCommit();

  // Checks every living enemy of `mover` armed with triggerAction == Shoot
  // for FOV+LOS on `mover`'s current (mid-move) position. On the first
  // watcher that has a shot, resolves it (killing `mover`), consumes that
  // watcher's trigger, and returns true so Update() can interrupt the move.
  bool TriggerOverwatch(Unit& mover);

  Scene scene_;
  NavMesh navMesh_;
  TurnManager turnManager_;
  std::vector<AABB> obstacleBounds_;  // Cached flat bounds of scene_.obstacles for LOS/FOV checks.

  InputMode mode_ = InputMode::AwaitingSelection;
  std::optional<int> selectedUnitId_;
  std::optional<Team> winner_;

  std::vector<glm::vec3> movePreviewPath_;
  bool movePreviewValid_ = false;

  // Every figure's in-flight planned move for the current commit, valid only
  // while mode_ == Moving; empty once all of them finish.
  std::vector<ActiveMove> activeMoves_;
};

}  // namespace tactics
