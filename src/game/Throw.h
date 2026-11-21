#pragma once

#include <vector>

#include <glm/glm.hpp>

#include "game/Types.h"
#include "game/Unit.h"

namespace tactics {

// One grenade throw's simulated flight (issue #349): the ballistic arc from
// the thrower's hand, every bounce, and where/when it detonates. Produced by
// SimulateThrow, a pure function of (thrower, aim point, static geometry), so
// the planning preview and the round's execution share the exact same flight
// ("drawn == simulated").
struct ThrowPreview {
  // Grenade position every constants::kSimStepSeconds (30 Hz) from release
  // (arc[0], t = 0) to detonation (arc.back(), t = fuse).
  std::vector<glm::vec3> arc;
  // Impact points where the grenade rebounded off an obstacle, a walk
  // surface or the terrain, in flight order.
  std::vector<glm::vec3> bounces;
  glm::vec3 landing{0.0f};  // Where it detonates (== arc.back()).
  float blastRadius = constants::kGrenadeBlastRadius;
  float fuse = constants::kGrenadeFuseSeconds;  // Seconds from release to blast.
};

// Horizontal distance from `unit` to `point`.
float ThrowDistance(const Unit& unit, const glm::vec3& point);

// True if `unit` may plan a throw at `point`: alive, has supply, in range.
bool CanThrowAt(const Unit& unit, const glm::vec3& point);

// Deterministic 30 Hz flight of a grenade thrown by `unit` at `point`:
// a gravity arc tuned to pass through `point`, bounced off the given
// geometry on each short step segment, detonating at the fixed fuse.
ThrowPreview SimulateThrow(const Unit& unit, const glm::vec3& point,
                           const std::vector<Obstacle>& obstacles,
                           const std::vector<WalkSurface>& walkSurfaces,
                           const HeightField& terrain);

// Flight position `t` seconds after release (clamped to [0, fuse]),
// interpolated between the 30 Hz samples.
glm::vec3 ThrowPositionAt(const ThrowPreview& flight, float t);

}  // namespace tactics
