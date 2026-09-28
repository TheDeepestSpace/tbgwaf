#pragma once

#include <vector>

#include <glm/glm.hpp>

#include "game/Types.h"

namespace tactics {

// Slab-method ray/AABB intersection. If the ray (origin + t*direction, t>=0)
// intersects `box`, returns true and sets `outT` to the entry distance.
bool RayIntersectsAABB(const glm::vec3& origin, const glm::vec3& direction, const AABB& box,
                        float* outT = nullptr);

// True if the closed segment [from, to] is not blocked by any obstacle. A
// blocking hit exactly at the endpoints (t<=0 or t>=1) does not count, so
// units standing next to an obstacle can still see past its own edge.
bool LineOfSightClear(const glm::vec3& from, const glm::vec3& to,
                       const std::vector<AABB>& obstacles);

// True if `target` lies within a cone from `origin` centered on `forward`
// with the given half-angle (degrees) and max range.
bool InFovCone(const glm::vec3& origin, const glm::vec3& forward, const glm::vec3& target,
               float halfAngleDegrees, float maxRange);

}  // namespace tactics
