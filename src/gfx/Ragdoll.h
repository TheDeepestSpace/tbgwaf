#pragma once

#include <array>

#include <glm/glm.hpp>

#include "game/Unit.h"

namespace gfx {

// Render-side ragdoll for downed figures (issue #217). A small Verlet
// particle skeleton (head, torso, limbs) is launched from the unit's standing
// pose with an impulse along the shot direction and falls under gravity onto
// the ground plane until it comes to rest.
//
// Purely visual and stateless from the caller's view: the pose is a pure
// function of (id, position, facingYaw, knockdownAxis, knockdownElapsed), all
// of which are already replicated/replayed state, so every client and every
// replay reproduces the same result. Nothing here reads or writes game logic.
// Collision is against the ground plane at unit.position.y only (no
// buildings: the sim has no access to the scene).

enum RagdollJoint {
  kRagHead,
  kRagNeck,
  kRagPelvis,
  kRagShoulderL,
  kRagShoulderR,
  kRagElbowL,
  kRagElbowR,
  kRagHandL,
  kRagHandR,
  kRagHipL,
  kRagHipR,
  kRagKneeL,
  kRagKneeR,
  kRagFootL,
  kRagFootR,
  kRagJointCount,
};

using RagdollPose = std::array<glm::vec3, kRagJointCount>;  // World positions.

// Fixed simulation step, number of steps from hit to rest, and the cap on
// simultaneously cached (i.e. actively evaluated) ragdolls.
constexpr float kRagdollStep = 1.0f / 60.0f;
constexpr int kRagdollSteps = 60;
constexpr int kRagdollMaxCached = 16;

// True for a downed unit whose knockdown animation has started: such units
// are drawn as a ragdoll instead of a rigid tip-over.
bool RagdollActive(const tactics::Unit& unit);

// The full-fall trajectory frame `step` (0..kRagdollSteps) for this unit.
RagdollPose SampleRagdollStep(const tactics::Unit& unit, int step);

// The pose for the unit's current knockdownElapsed (progress through
// kKnockdownDuration maps onto the full simulated fall, so the ragdoll is at
// rest exactly when the knockdown timer finishes).
RagdollPose SampleRagdoll(const tactics::Unit& unit);

}  // namespace gfx
