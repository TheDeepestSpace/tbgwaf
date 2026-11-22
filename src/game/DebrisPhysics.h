#pragma once

// Rigid-body simulation of explosion debris (issue #354). Every chunk is a
// convex prism (a polygon outline extruded through a thickness) that tumbles,
// collides with the other chunks, the standing geometry, the ground, walk
// surfaces and figures, and comes to rest in whatever pose the contacts leave
// it in -- tilted, leaning, piled on its neighbours.
//
// It runs once, on whatever authority plans the blast (the server in a
// networked match), and its output is a sampled pose track per chunk that
// clients replay verbatim (see DebrisFlight): clients never integrate
// anything, so a chunk that hits a figure or a wall does so identically
// everywhere.
//
// Determinism and calm: fixed time step, a fixed iteration order, no clock.
// To keep stacks from "seizing" (resting bodies jittering against each other)
// contacts are cold-started and solved by sequential impulses with penetration
// removed by moving bodies apart rather than adding velocity, whatever is in
// contact is damped, and a body that has gone still is put to sleep -- frozen
// at its exact rest pose; its track simply ends there, and it wakes only if
// something hits it hard. A pile that has not settled after a few seconds is
// damped ever harder, so every blast terminates.

#include <vector>

#include <glm/glm.hpp>

#include "game/Types.h"
#include "game/WorldChange.h"

namespace tactics {

// A convex prism: `outline` (counter-clockwise, local x/y) extruded +-thickness/2
// along local z, placed by `pose` (local -> world, proper rotation).
struct DebrisShape {
  std::vector<glm::vec2> outline;
  float thickness = 0.1f;
  glm::mat4 pose{1.0f};
};

struct DebrisBodyDesc {
  DebrisShape shape;
  glm::vec3 velocity{0.0f};         // Of the centre of mass, m/s.
  glm::vec3 angularVelocity{0.0f};  // World frame, rad/s.
  // Litter: collides with everything but never pushes a heavier chunk (so a
  // window bar caught under a roof slab cannot make the pile jump).
  bool light = false;
};

// What chunks collide with besides each other. Everything is borrowed.
struct DebrisWorldDesc {
  const HeightField* ground = nullptr;
  std::vector<DebrisShape> statics;           // Standing geometry (immovable prisms).
  std::vector<const WalkSurface*> decks;      // Floors, roofs, ramps (slabs of kDeckThickness).
  float extent = 1.0e9f;                      // Map half-extent: chunks stay inside it.
};

constexpr float kDeckThickness = 0.45f;       // Solid below a walk surface.

// One track per body: its centre of mass and orientation every kDebrisKeyDt
// from t = 0 (keys[0] is the launch pose) until the moment it came to rest;
// the last key is its exact rest pose. `settled` is false for a body that was
// still moving when `maxSeconds` ran out (or fell out of the world).
struct DebrisTrack {
  std::vector<DebrisKey> keys;
  bool settled = true;
};

std::vector<DebrisTrack> SimulateDebris(const DebrisWorldDesc& world,
                                        const std::vector<DebrisBodyDesc>& bodies,
                                        float maxSeconds = 8.0f);

// Centre of mass (in the outline's own coordinates, z = 0) of a prism.
glm::vec2 DebrisCentroid(const std::vector<glm::vec2>& outline);

}  // namespace tactics
