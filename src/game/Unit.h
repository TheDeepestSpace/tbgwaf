#pragma once

#include <cmath>
#include <vector>

#include <glm/glm.hpp>

#include "game/Types.h"

namespace tactics {

// WEGO plan-then-commit round model: during the planning phase both teams
// choose an action for each of their figures, which only records what the
// figure *will* do -- nothing executes until the whole round is committed
// (see GameLogic::CommitRound), at which point every plan on both teams
// plays out together. Overwatch doesn't act immediately either: it just
// arms triggerAction below as part of the same commit.
enum class PlannedActionType { None, Move, Shoot, Pass, Overwatch };

struct PlannedAction {
  PlannedActionType type = PlannedActionType::None;
  std::vector<glm::vec3> movePath;  // Resolved via NavMesh::FindPath, for type == Move.
  float endFacingYaw = 0.0f;        // Final facing once the path ends, for type == Move.
  int shootTargetId = -1;           // For type == Shoot.
};

// A standing order a figure can arm on its turn, to react automatically
// during an enemy's move instead of acting immediately. `None` is the
// default (no reaction); `Shoot` is the overwatch PoC. Left room to extend
// with more reactions later.
enum class TriggerAction { None, Shoot };

// What a figure does, on its own, on a tick where at least one living enemy
// is inside its FOV+LOS. A persistent config set outside the turn economy
// (unlike a one-shot Overwatch trigger): it stays in force across rounds
// until the player changes it. Stop/Continue/ShootStop/ShootContinue only
// mean something to a moving figure -- "continue" just keeps executing the
// already-committed path this round; for a stationary figure they behave as
// DoNothing / Shoot.
enum class ReactionAction { DoNothing, Shoot, Stop, Continue, ShootStop, ShootContinue };

inline bool ReactionShoots(ReactionAction a) {
  return a == ReactionAction::Shoot || a == ReactionAction::ShootStop ||
         a == ReactionAction::ShootContinue;
}
inline bool ReactionStops(ReactionAction a) {
  return a == ReactionAction::Stop || a == ReactionAction::ShootStop;
}

// Squad-wide reaction lookup: (moving | stationary) x (some sighted enemy can
// see me back | none can) -> action. One table per team, shared by all of its
// figures.
struct SquadPlaybook {
  ReactionAction table[2][2] = {
      // [moving][canSeeMe]
      {ReactionAction::DoNothing, ReactionAction::DoNothing},  // Stationary.
      {ReactionAction::Continue, ReactionAction::Continue},    // Moving.
  };

  ReactionAction& At(bool moving, bool canSeeMe) { return table[moving ? 1 : 0][canSeeMe ? 1 : 0]; }
  ReactionAction At(bool moving, bool canSeeMe) const {
    return table[moving ? 1 : 0][canSeeMe ? 1 : 0];
  }
  bool operator==(const SquadPlaybook& o) const {
    for (int m = 0; m < 2; ++m)
      for (int s = 0; s < 2; ++s)
        if (table[m][s] != o.table[m][s]) return false;
    return true;
  }
  bool operator!=(const SquadPlaybook& o) const { return !(*this == o); }
};

struct Unit {
  int id = -1;
  Team team = Team::Blue;
  glm::vec3 position{0.0f};  // Feet position; unit box spans [position, position + (0,height,0)].
  float facingYaw = 0.0f;    // Radians, measured from +X axis in the XZ plane.
  bool alive = true;
  // Visual-only fall state for a downed unit. Axis is horizontal; tipping
  // around it topples the figure in the shot's direction of travel.
  glm::vec3 knockdownAxis{1.0f, 0.0f, 0.0f};
  float knockdownElapsed = -1.0f;  // Seconds since hit; <0 = not falling.
  // Visual-only walk cycle: phase in radians (advances with distance moved,
  // 2*pi per kWalkStrideLength) and a 0..1 blend of how much the walk pose
  // overrides the rest pose (eases toward 1 while a planned move is in
  // flight, back to 0 once it ends).
  float walkPhase = 0.0f;
  float walkBlend = 0.0f;
  // Looping clock for the subtle standing pose. Stored per unit so mirrored
  // clients render the same sampled rig pose during execution.
  float idleElapsed = 0.0f;
  // Visual-only quick-draw beat: seconds since this figure's shot resolved
  // (<0 = idle) and the world yaw toward the target it fired at, so the gun
  // arm can swing onto the target even when it sits off-center in the FOV.
  float shootElapsed = -1.0f;
  float shootAimYaw = 0.0f;
  float runSpeed = constants::kMoveSpeed;  // World units per second while moving.
  PlannedAction plan;  // This figure's plan for the current/upcoming round commit.
  TriggerAction triggerAction = TriggerAction::None;

  // How far this figure can move in one round's fixed execution window --
  // the plannable path-length cap enforced by GameLogic::ClickGround.
  float MoveBudget() const { return runSpeed * constants::kRoundDuration; }

  glm::vec3 EyePosition() const {
    return position + glm::vec3(0.0f, constants::kEyeHeight, 0.0f);
  }

  glm::vec3 FacingDirection() const {
    return glm::vec3(std::cos(facingYaw), 0.0f, std::sin(facingYaw));
  }

  AABB Bounds() const {
    glm::vec3 halfExtents(constants::kUnitHalfWidth, 0.0f, constants::kUnitHalfWidth);
    glm::vec3 base = position - halfExtents;
    glm::vec3 top = position + halfExtents + glm::vec3(0.0f, constants::kUnitHeight, 0.0f);
    return AABB{base, top};
  }
};

}  // namespace tactics
