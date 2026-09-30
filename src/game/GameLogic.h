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
  AwaitingSelection,      // Waiting for the click on the current actor.
  ActionMenu,             // Current actor selected; waiting for Move/Shoot/Pass.
  AwaitingMoveDestination,  // Waiting for a ground click to move to.
  AwaitingShootTarget,    // Waiting for a click on an enemy figure to shoot.
  Moving,                 // Selected actor is animating along its resolved move path.
  GameOver,
};

// Serializable dynamic match state (everything that changes after Reset()
// on a fixed scene). Used to mirror one authoritative match into a second,
// non-simulating instance (two-tab play): the follower imports snapshots
// instead of re-simulating, so float divergence can't desync the two.
struct GameSnapshot {
  struct UnitState {
    int id = -1;
    glm::vec3 position{0.0f};
    float facingYaw = 0.0f;
    bool alive = true;
    TriggerAction triggerAction = TriggerAction::None;
  };
  std::vector<UnitState> units;
  TurnManager::State turn;
  InputMode mode = InputMode::AwaitingSelection;
  int selectedUnitId = -1;  // -1 = none.
  int winner = -1;          // -1 = none, else static_cast<int>(Team).
};

// Text encoding of a snapshot (for BroadcastChannel). Deserialize returns
// false on malformed input.
std::string SerializeSnapshot(const GameSnapshot& snapshot);
bool DeserializeSnapshot(const std::string& text, GameSnapshot* out);

// Owns the whole Stage-A game state machine: scene, navmesh, turn order, and
// the click-driven selection/action flow. Deliberately free of any
// SDL/GL/ImGui dependency so it can be driven and verified headlessly.
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
  std::optional<int> CurrentActorId() const { return turnManager_.CurrentActorId(scene_.units); }
  std::optional<Team> Winner() const { return winner_; }
  int RoundNumber() const { return turnManager_.RoundNumber(); }

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
  void ClickUnit(int unitId);
  void ClickGround(const glm::vec3& point);
  void HoverGround(const glm::vec3& point);

  // Advances an in-flight move animation (Mode() == InputMode::Moving) by
  // `dtSeconds`, moving the selected unit along its resolved path at
  // constant speed and completing the action once the path is consumed. A
  // no-op in any other mode. `main.cpp`'s frame loop drives this with real
  // frame delta; tests can pass a large dt to fast-forward to completion.
  void Update(float dtSeconds);

  // Action menu choices, valid only while Mode() == ActionMenu.
  void ChooseMove();
  void ChooseShoot();
  void ChoosePass();

  // Arms overwatch (triggerAction = Shoot) on the acting unit and ends its
  // turn, the same way ChoosePass() does today.
  void ChooseOverwatch();

  // Snapshot of the dynamic match state. ImportState overwrites this
  // instance's state with it (any in-flight move/preview is dropped) and
  // returns false, leaving state untouched, if the snapshot doesn't match
  // this scene's units.
  GameSnapshot ExportState() const;
  bool ImportState(const GameSnapshot& snapshot);

  // Steps back one level: AwaitingMove/ShootTarget -> ActionMenu -> AwaitingSelection.
  void CancelAction();

  // Deterministic hit resolution: FOV cone + clear line-of-sight. Exposed
  // directly so it can be unit tested without going through the click flow.
  bool ResolveShot(Unit& shooter, Unit& target);

 private:
  void CompleteAction();

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

  // In-flight move animation state, valid only while mode_ == Moving.
  // moveAnimPath_[moveAnimSegment_] is the waypoint the mover last passed
  // through; moveAnimPath_[moveAnimSegment_ + 1] is the one it's walking
  // toward.
  std::vector<glm::vec3> moveAnimPath_;
  size_t moveAnimSegment_ = 0;
};

}  // namespace tactics
