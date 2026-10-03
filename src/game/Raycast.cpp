#include "game/Raycast.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "game/Geometry.h"

namespace tactics {

namespace {

// Clips one parameter interval to [lo, hi] along an axis. The segment is
// p(t)=origin+t*delta, t in [0,1].
bool ClipSegmentAxis(float origin, float delta, float lo, float hi, float* enter,
                     float* exit) {
  if (std::fabs(delta) < 1e-8f) return origin >= lo && origin <= hi;
  float a = (lo - origin) / delta;
  float b = (hi - origin) / delta;
  if (a > b) std::swap(a, b);
  *enter = std::max(*enter, a);
  *exit = std::min(*exit, b);
  return *enter <= *exit;
}

}  // namespace

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

bool LineOfSightClear(const glm::vec3& from, const glm::vec3& to,
                      const HeightField& terrain) {
  if (terrain.Empty() || terrain.nx < 2 || terrain.nz < 2 || terrain.step <= 0.0f) return true;
  const glm::vec3 segment = to - from;
  const float segmentLength = glm::length(segment);
  if (segmentLength < 1e-6f) return true;

  // Restrict checks to the rendered heightfield rectangle. Its border skirt
  // only closes the visual map edge and is not playable terrain.
  float enter = 0.0f, exit = 1.0f;
  const float maxX = terrain.minX + (terrain.nx - 1) * terrain.step;
  const float maxZ = terrain.minZ + (terrain.nz - 1) * terrain.step;
  if (!ClipSegmentAxis(from.x, segment.x, terrain.minX, maxX, &enter, &exit) ||
      !ClipSegmentAxis(from.z, segment.z, terrain.minZ, maxZ, &enter, &exit)) {
    return true;
  }
  const float endpointEpsilon = std::min(0.25f, 1e-3f / segmentLength);
  enter = std::max(enter, endpointEpsilon);
  exit = std::min(exit, 1.0f - endpointEpsilon);
  if (enter > exit) return true;

  // Split the sightline wherever it crosses a grid edge. Within each such
  // interval it crosses at most one cell diagonal, and terrain height is
  // linear on either side of that diagonal. Checking those boundaries is
  // therefore an exact segment-vs-triangulated-heightfield test rather than
  // a sampling approximation.
  std::vector<float> breaks{enter, exit};
  if (std::fabs(segment.x) >= 1e-8f) {
    for (int ix = 0; ix < terrain.nx; ++ix) {
      const float x = terrain.minX + ix * terrain.step;
      const float t = (x - from.x) / segment.x;
      if (t > enter && t < exit) breaks.push_back(t);
    }
  }
  if (std::fabs(segment.z) >= 1e-8f) {
    for (int iz = 0; iz < terrain.nz; ++iz) {
      const float z = terrain.minZ + iz * terrain.step;
      const float t = (z - from.z) / segment.z;
      if (t > enter && t < exit) breaks.push_back(t);
    }
  }
  std::sort(breaks.begin(), breaks.end());
  breaks.erase(std::unique(breaks.begin(), breaks.end(), [](float a, float b) {
                 return std::fabs(a - b) < 1e-6f;
               }),
               breaks.end());

  constexpr float kTerrainEpsilon = 1e-3f;
  const auto blockedAt = [&](float t) {
    const glm::vec3 p = from + segment * t;
    return p.y < terrain.MeshHeightAt(p.x, p.z) - kTerrainEpsilon;
  };

  for (float t : breaks) {
    if (blockedAt(t)) return false;
  }
  for (size_t i = 0; i + 1 < breaks.size(); ++i) {
    const float t0 = breaks[i], t1 = breaks[i + 1];
    if (t1 - t0 < 1e-7f) continue;
    const glm::vec3 mid = from + segment * ((t0 + t1) * 0.5f);
    const int ix = std::clamp(static_cast<int>(std::floor((mid.x - terrain.minX) / terrain.step)),
                              0, terrain.nx - 2);
    const int iz = std::clamp(static_cast<int>(std::floor((mid.z - terrain.minZ) / terrain.step)),
                              0, terrain.nz - 2);
    const float cellX = terrain.minX + ix * terrain.step;
    const float cellZ = terrain.minZ + iz * terrain.step;
    const auto diagonalSide = [&](float t) {
      const glm::vec3 p = from + segment * t;
      return (p.x - cellX) - (p.z - cellZ);
    };
    const float side0 = diagonalSide(t0), side1 = diagonalSide(t1);
    if ((side0 < 0.0f && side1 > 0.0f) || (side0 > 0.0f && side1 < 0.0f)) {
      const float diagonalT = t0 + (t1 - t0) * side0 / (side0 - side1);
      if (blockedAt(diagonalT)) return false;
    }
  }
  return true;
}

bool LineOfSightClear(const glm::vec3& from, const glm::vec3& to,
                      const std::vector<Obstacle>& obstacles,
                      const std::vector<WalkSurface>& walkSurfaces,
                      const HeightField& terrain) {
  return LineOfSightClear(from, to, obstacles, walkSurfaces) &&
         LineOfSightClear(from, to, terrain);
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
