#include "game/Cover.h"

#include <cmath>

#include "game/Geometry.h"

namespace tactics {
namespace {

constexpr int kDirs = 8;

bool Blocked(const std::vector<Obstacle>& obstacles, const glm::vec2& p, float feetY) {
  for (const Obstacle& o : obstacles) {
    if (o.navOnly) continue;
    const AABB& b = o.bounds;
    // Must stand on the feet's level (not a crown overhead) and be tall enough.
    if (b.max.y < feetY + kCoverMinHeight || b.min.y > feetY + 0.3f) continue;
    if (p.x < b.min.x || p.x > b.max.x || p.y < b.min.z || p.y > b.max.z) continue;
    if (!o.footprint.empty() && !PointInConvexPolygon(p, o.footprint)) continue;
    return true;
  }
  return false;
}

}  // namespace

CoverTile ClassifyCover(const std::vector<Obstacle>& obstacles, const glm::vec3& feet,
                        float* yaw) {
  CoverTile tile;
  const glm::vec2 c(feet.x, feet.z);
  if (Blocked(obstacles, c, feet.y)) return tile;  // Inside a blocker: not a standing spot.
  glm::vec2 sum(0.0f);
  bool tactical = false, cardinal = false;
  for (int k = 0; k < kDirs; ++k) {
    const float a = static_cast<float>(k) * (3.14159265358979f / 4.0f);
    const glm::vec2 d(std::cos(a), std::sin(a));
    // Diagonal probes reach the same face distance as cardinal ones.
    const float reach = (k % 2 == 0) ? kCoverProbeDistance : kCoverProbeDistance * 1.2f;
    if (!Blocked(obstacles, c + d * reach, feet.y)) continue;
    tile.dirMask |= static_cast<uint8_t>(1u << k);
    sum += d;
    if (k % 2 != 0) continue;  // Diagonals only add direction; edges are judged on faces.
    cardinal = true;
    // Edge test along the wall on both sides: the blocker has ended (open
    // edge) if the side point is free ground and the probe beyond it is empty.
    const glm::vec2 t(-d.y, d.x);
    for (float side : {-1.0f, 1.0f}) {
      const glm::vec2 s = c + t * (side * kCoverEdgeReach);
      if (!Blocked(obstacles, s, feet.y) && !Blocked(obstacles, s + d * reach, feet.y)) {
        tactical = true;
      }
    }
  }
  if (tile.dirMask == 0) return tile;
  // Only a diagonal probe hit: the tile touches a blocker's corner.
  if (!cardinal) tactical = true;
  tile.kind = tactical ? CoverKind::Tactical : CoverKind::Flat;
  if (yaw) {
    *yaw = glm::length(sum) > 1e-4f ? std::atan2(sum.y, sum.x) : 0.0f;
  }
  return tile;
}

CoverMap ComputeCoverMap(const std::vector<Obstacle>& obstacles, const ReachField& field) {
  CoverMap map;
  map.nx = field.nx;
  map.nz = field.nz;
  map.tiles.assign(static_cast<size_t>(field.nx) * field.nz, CoverTile{});
  if (field.nx <= 0 || field.nz <= 0) return map;
  // Only blockers near the field can matter; the full list can be huge.
  const float margin = kCoverProbeDistance * 1.2f + kCoverEdgeReach + 0.1f;
  const float x0 = field.minX - margin, x1 = field.minX + field.nx * field.step + margin;
  const float z0 = field.minZ - margin, z1 = field.minZ + field.nz * field.step + margin;
  std::vector<Obstacle> nearby;
  for (const Obstacle& o : obstacles) {
    const AABB& b = o.bounds;
    if (b.max.x >= x0 && b.min.x <= x1 && b.max.z >= z0 && b.min.z <= z1) nearby.push_back(o);
  }
  for (int iz = 0; iz < field.nz; ++iz) {
    for (int ix = 0; ix < field.nx; ++ix) {
      if (!field.Reached(ix, iz)) continue;
      map.tiles[iz * field.nx + ix] = ClassifyCover(nearby, field.Node(ix, iz));
    }
  }
  return map;
}

}  // namespace tactics
