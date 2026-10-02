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
  // Further legs the player chained after movePath, one per later round.
  // Each leg is a polyline of at most MoveBudget() length that starts where
  // the previous leg (movePath for the first) ends; FinishRound arms the next.
  std::vector<std::vector<glm::vec3>> queuedLegs;
  float endFacingYaw = 0.0f;        // Final facing once the path ends, for type == Move.
  int shootTargetId = -1;           // For type == Shoot.
};

// A standing order a figure can arm on its turn, to react automatically
// during an enemy's move instead of acting immediately. `None` is the
// default (no reaction); `Shoot` is the overwatch PoC. Left room to extend
// with more reactions later.
enum class TriggerAction { None, Shoot };

// Standing per-figure playbook rule: what a figure does, on its own,
// whenever it is stationary (not itself the one moving) and an enemy enters
// its FOV+LOS. Distinct from a one-shot action-menu trigger (e.g. an
// Overwatch-style ability that costs a turn and is consumed on first use):
// this is a persistent config set outside the turn economy, and it stays
// armed across rounds until the player changes it.
enum class ReactionRule { DoNothing, Shoot };

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
  ReactionRule reactionOnStationary = ReactionRule::DoNothing;

  // How far this figure can move in one round's fixed execution window --
  // the length of one planned leg (longer routes are chained leg by leg, see
  // PlannedAction::queuedLegs).
  float MoveBudget() const { return runSpeed * constants::kRoundDuration; }

  glm::vec3 EyePosition() const {
    return position + glm::vec3(0.0f, constants::kEyeHeight, 0.0f);
  }

  // World position of the gun tip when aiming (see constants::kMuzzle*).
  glm::vec3 MuzzlePosition() const {
    const glm::vec3 fwd = FacingDirection();
    const glm::vec3 right(-fwd.z, 0.0f, fwd.x);
    return position + fwd * constants::kMuzzleForward + right * constants::kMuzzleSide +
           glm::vec3(0.0f, constants::kMuzzleHeight, 0.0f);
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
