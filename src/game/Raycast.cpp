#include "game/Raycast.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "game/Geometry.h"

namespace tactics {

bool RayIntersectsAABB(const glm::vec3& origin, const glm::vec3& direction, const AABB& box,
                        float* outT) {
  float tMin = 0.0f;
  float tMax = std::numeric_limits<float>::infinity();

  for (int axis = 0; axis < 3; ++axis) {
    const float o = origin[axis];
    const float d = direction[axis];
    const float lo = box.min[axis];
    const float hi = box.max[axis];

    if (std::fabs(d) < 1e-8f) {
      if (o < lo || o > hi) return false;
      continue;
    }
    float t1 = (lo - o) / d;
    float t2 = (hi - o) / d;
    if (t1 > t2) std::swap(t1, t2);
    tMin = std::max(tMin, t1);
    tMax = std::min(tMax, t2);
    if (tMin > tMax) return false;
  }

  if (outT) *outT = tMin;
  return true;
}

bool LineOfSightClear(const glm::vec3& from, const glm::vec3& to,
                       const std::vector<AABB>& obstacles) {
  const glm::vec3 segment = to - from;
  const float segmentLength = glm::length(segment);
  if (segmentLength < 1e-6f) return true;
  const glm::vec3 direction = segment / segmentLength;

  constexpr float kEndpointEpsilon = 1e-3f;
  for (const auto& obstacle : obstacles) {
    float t = 0.0f;
    if (RayIntersectsAABB(from, direction, obstacle, &t)) {
      if (t > kEndpointEpsilon && t < segmentLength - kEndpointEpsilon) {
        return false;
      }
    }
  }
  return true;
}

bool LineOfSightClear(const glm::vec3& from, const glm::vec3& to,
                      const std::vector<Obstacle>& obstacles) {
  const glm::vec3 segment = to - from;
  const float segmentLength = glm::length(segment);
  if (segmentLength < 1e-6f) return true;
  const glm::vec3 direction = segment / segmentLength;
  constexpr float kEndpointEpsilon = 1e-3f;
  for (const Obstacle& obstacle : obstacles) {
    float t = 0.0f;
    if (RayIntersectsObstacle(from, direction, obstacle, &t) && t > kEndpointEpsilon &&
        t < segmentLength - kEndpointEpsilon) {
      return false;
    }
  }
  return true;
}

bool LineOfSightClear(const glm::vec3& from, const glm::vec3& to,
                      const std::vector<Obstacle>& obstacles,
                      const std::vector<WalkSurface>& walkSurfaces) {
  if (!LineOfSightClear(from, to, obstacles)) return false;
  const glm::vec3 segment = to - from;
  const float segmentLength = glm::length(segment);
  if (segmentLength < 1e-6f) return true;
  const glm::vec3 direction = segment / segmentLength;
  constexpr float kEndpointEpsilon = 1e-3f;
  for (const WalkSurface& surface : walkSurfaces) {
    float t = 0.0f;
    if (RayIntersectsWalkSurface(from, direction, surface, 0.45f, &t) &&
        t > kEndpointEpsilon && t < segmentLength - kEndpointEpsilon) {
      return false;
    }
  }
  return true;
}

bool InFovCone(const glm::vec3& origin, const glm::vec3& forward, const glm::vec3& target,
               float halfAngleDegrees, float maxRange) {
  const glm::vec3 toTarget = target - origin;
  const float distance = glm::length(toTarget);
  if (distance < 1e-6f) return true;
  if (distance > maxRange) return false;

  const glm::vec3 forwardXZ = glm::normalize(glm::vec3(forward.x, 0.0f, forward.z));
  const glm::vec3 toTargetXZ = glm::normalize(glm::vec3(toTarget.x, 0.0f, toTarget.z));

  const float cosAngle = glm::clamp(glm::dot(forwardXZ, toTargetXZ), -1.0f, 1.0f);
  const float angleDegrees = glm::degrees(std::acos(cosAngle));
  return angleDegrees <= halfAngleDegrees;
}

}  // namespace tactics
