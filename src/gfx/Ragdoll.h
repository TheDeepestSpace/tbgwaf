#pragma once

#include <array>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

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
// Collision is against the ground plane at unit.position.y plus the map's
// obstacle boxes, ramps/decks and terrain (see SetRagdollObstacles,
// SetRagdollGround): a figure shot on top of an
// obstacle falls off its edge to the ground, and obstacle sides block it.

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

// Fixed simulation step, the most steps a fall is simulated for (it stops
// earlier once the body is at rest; later frames repeat the last), and the
// cap on simultaneously cached (i.e. actively evaluated) ragdolls. Playback is
// real time: knockdownElapsed seconds = elapsed / kRagdollStep steps.
constexpr float kRagdollStep = 1.0f / 60.0f;
constexpr int kRagdollSteps = 300;
constexpr int kRagdollMaxCached = 16;

// The weapon of a downed unit. Roughly half of all figures (decided
// deterministically from the unit id) let go of it when shot: it then tumbles
// as a free rigid body, bounces and settles on the ground. Otherwise it
// stays gripped in the right hand and `dropped` is false (pos/rot unused).
struct RagdollGun {
  bool dropped = false;
  glm::vec3 pos{0.0f};  // World position of the weapon-local origin (grip top).
  glm::quat rot{1.0f, 0.0f, 0.0f, 0.0f};  // Weapon-local -> world (+X is the barrel).
};

// Obstacles the ragdolls collide with. Call before sampling (cheap when
// unchanged; a change drops all cached trajectories).
void SetRagdollObstacles(const std::vector<tactics::Obstacle>& obstacles);

// Ramps/decks and the terrain heightfield the ragdolls rest on, so a body on
// a ramp slides down it and one on hilly ground lies on the slope. Same
// calling convention as SetRagdollObstacles.
void SetRagdollGround(const std::vector<tactics::WalkSurface>& surfaces,
                      const tactics::HeightField& terrain);

// A living figure as an abstracted collider (issue #266): a vertical capsule
// standing on `base` (feet). Falling bodies are pushed out of it (bounded per
// step) so they rest on / slide off a standing figure instead of passing
// through it. Visual only: the figure itself is never moved.
struct RagdollFigureCollider {
  glm::vec3 base{0.0f};
  float radius = 0.30f;
  float height = 1.75f;
};

// Living figures at the moment a fall is first simulated. Unlike obstacles,
// changing these does NOT drop cached trajectories (figures move every
// frame); a fall uses the figures as they were when it was first evaluated.
void SetRagdollFigures(const std::vector<RagdollFigureCollider>& figures);

// True for a downed unit whose knockdown animation has started: such units
// are drawn as a ragdoll instead of a rigid tip-over.
bool RagdollActive(const tactics::Unit& unit);

// The full-fall trajectory frame `step` for this unit (clamped to the
// frames simulated, i.e. the rest pose once the fall is over).
RagdollPose SampleRagdollStep(const tactics::Unit& unit, int step);

// The weapon state matching SampleRagdollStep / SampleRagdoll.
RagdollGun SampleRagdollGunStep(const tactics::Unit& unit, int step);
RagdollGun SampleRagdollGun(const tactics::Unit& unit);

// The pose for the unit's current knockdownElapsed, in real time. The timer
// only advances while a round executes, so a fall cut off by the end of the
// round freezes mid-air and resumes (same state) in the next one.
RagdollPose SampleRagdoll(const tactics::Unit& unit);

}  // namespace gfx
