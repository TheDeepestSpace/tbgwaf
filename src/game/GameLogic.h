#pragma once

#include <optional>
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

// Owns the whole game state machine: scene, navmesh, turn order, and the
// click-driven plan-then-commit flow. Deliberately free of any SDL/GL/ImGui
// dependency so it can be driven and verified headlessly.
//
// Turn model: on its turn, a team assigns one plan (Move/Shoot/Pass) to each
// of its living figures -- nothing happens yet. Once every living figure on
// the team has a plan, CommitTurn() executes them in squad order, one at a
// time (each resolves against world state as it stands after the previous
// one in the same commit -- a deliberate PoC simplification, not true
// simultaneous WEGO resolution). The turn then passes to the other team.
class GameLogic {
 public:
  GameLogic() { Reset(); }

  void Reset();

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

  // Advances an in-flight commit's move animation (Mode() == InputMode::Moving)
  // by `dtSeconds`, moving the animating unit along its planned path at
  // constant speed and continuing on to the rest of the committed turn's
  // planned actions once the path is consumed. A no-op in any other mode.
  // `main.cpp`'s frame loop drives this with real frame delta; tests can
  // pass a large dt to fast-forward to completion.
  void Update(float dtSeconds);

  // Action menu choices, valid only while Mode() == ActionMenu. Each records
  // a plan on the selected figure and returns to unit selection within the
  // still-active team's turn.
  void ChooseMove();
  void ChooseShoot();
  void ChoosePass();

  // Steps back one level: AwaitingMove/ShootTarget -> ActionMenu -> AwaitingSelection.
  void CancelAction();

  // Executes the current team's planned actions in squad order, one at a
  // time, then hands the turn to the other team. No-op unless
  // CanCommitTurn().
  void CommitTurn();

  // Deterministic hit resolution: FOV cone + clear line-of-sight. Exposed
  // directly so it can be unit tested without going through the click flow.
  bool ResolveShot(Unit& shooter, Unit& target);

 private:
  // Executes commitOrder_[commitIndex_] onward, stopping to let Update()
  // animate a planned move, or falling through to FinishCommit() once the
  // queue is exhausted.
  void ContinueCommit();
  void FinishCommit();

  Scene scene_;
  NavMesh navMesh_;
  TurnManager turnManager_;
  std::vector<AABB> obstacleBounds_;  // Cached flat bounds of scene_.obstacles for LOS/FOV checks.

  InputMode mode_ = InputMode::AwaitingSelection;
  std::optional<int> selectedUnitId_;
  std::optional<Team> winner_;

  std::vector<glm::vec3> movePreviewPath_;
  bool movePreviewValid_ = false;

  // In-flight move animation state, valid only while mode_ == Moving.
  // moveAnimPath_[moveAnimSegment_] is the waypoint the mover last passed
  // through; moveAnimPath_[moveAnimSegment_ + 1] is the one it's walking
  // toward.
  std::vector<glm::vec3> moveAnimPath_;
  size_t moveAnimSegment_ = 0;

  // Turn-commit execution state: the committing team's living figures, in
  // squad order, and how far through that order the commit has progressed.
  std::vector<int> commitOrder_;
  size_t commitIndex_ = 0;
};

}  // namespace tactics
