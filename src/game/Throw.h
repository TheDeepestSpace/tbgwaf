#pragma once

#include <vector>

#include <glm/glm.hpp>

#include "game/Types.h"
#include "game/Unit.h"

namespace tactics {

// Planning-time view of a grenade throw: the predicted flight, where it
// lands, and its blast radius. Pure function of (thrower, aim point), so the
// planning UI and any later simulation can share it ("drawn == simulated").
struct ThrowPreview {
  std::vector<glm::vec3> arc;  // Hand -> landing polyline (kThrowArcSamples + 1 points).
  // Surfaces the grenade rebounds off after the first impact. Empty until
  // the throw simulation lands; the field exists so the UI/protocol shape
  // does not change then.
  std::vector<glm::vec3> bounces;
  glm::vec3 landing{0.0f};
  float blastRadius = constants::kGrenadeBlastRadius;
};

// Horizontal distance from `unit` to `point`.
float ThrowDistance(const Unit& unit, const glm::vec3& point);

// True if `unit` may plan a throw at `point`: alive, has supply, in range.
bool CanThrowAt(const Unit& unit, const glm::vec3& point);

// Deterministic parabolic arc from the unit's hand to `point`.
ThrowPreview PredictThrow(const Unit& unit, const glm::vec3& point);

}  // namespace tactics
