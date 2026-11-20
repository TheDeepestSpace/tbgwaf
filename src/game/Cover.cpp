#include "game/Cover.h"

#include <algorithm>
#include <cmath>

#include "game/Geometry.h"

namespace tactics {
namespace {

constexpr int kDirs = 8;
// Openness probes at a wall's end: how far past the corner they look, and
// how deep behind the face plane the continuation probe sits.
constexpr float kCoverEdgeProbeOut = 0.3f;
constexpr float kCoverEdgeProbeIn = 0.1f;

// Height of the blocker above the feet at `p`, or a negative value if none.
float BlockHeight(const std::vector<Obstacle>& obstacles, const glm::vec2& p, float feetY) {
  for (const Obstacle& o : obstacles) {
    if (o.navOnly) continue;
    const AABB& b = o.bounds;
    // Must stand on the feet's level (not a crown overhead) and be tall enough.
    if (b.max.y < feetY + kCoverMinHeight || b.min.y > feetY + 0.3f) continue;
    if (p.x < b.min.x || p.x > b.max.x || p.y < b.min.z || p.y > b.max.z) continue;
    if (!o.footprint.empty() && !PointInConvexPolygon(p, o.footprint)) continue;
    return b.max.y - feetY;
  }
  return -1.0f;
}

// Narrowest horizontal width of the obstacle (min over footprint edges of
// the polygon's extent across that edge). Thin props -- street poles, lamps --
// are narrower than a figure and cannot hide one (issue #292).
bool WideEnoughForCover(const Obstacle& ob) {
  if (ob.kind == ObstacleKind::Wall) return true;  // Thin wall pieces of buildings stay cover.
  const AABB& b = ob.bounds;
  std::vector<glm::vec2> poly;
  if (!ob.footprint.empty()) {
    poly = ob.footprint;
  } else {
    poly = {{b.min.x, b.min.z}, {b.max.x, b.min.z}, {b.max.x, b.max.z}, {b.min.x, b.max.z}};
  }
  float narrowest = 1e9f;
  for (size_t i = 0; i < poly.size(); ++i) {
    const glm::vec2 d = poly[(i + 1) % poly.size()] - poly[i];
    const float len = glm::length(d);
    if (len < 1e-4f) continue;
    const glm::vec2 n(d.y / len, -d.x / len);
    float lo = 1e9f, hi = -1e9f;
    for (const glm::vec2& v : poly) {
      const float h = glm::dot(v - poly[i], n);
      lo = std::min(lo, h);
      hi = std::max(hi, h);
    }
    narrowest = std::min(narrowest, hi - lo);
  }
  return narrowest >= 2.0f * constants::kUnitHalfWidth - 1e-4f;
}

bool Blocked(const std::vector<Obstacle>& obstacles, const glm::vec2& p, float feetY) {
  return BlockHeight(obstacles, p, feetY) >= 0.0f;
}

// One wall face giving cover to a point: the face plane is within the band
// and the point projects inside the face's span. A point past the span --
// diagonally off a corner -- gets nothing from the face.
struct Wall {
  glm::vec2 toWall{0.0f};  // Unit XZ direction from the point to the face.
  float height = 0.0f;     // Blocker top above the feet.
  bool tactical = false;   // Inside the 45-degree wedge of an open end.
};

// Collects the wall faces within `band` of `c`, with the tactical wedge rule
// of issue #292: a face with an open end (a corner to peek around) is
// tactical only on the stretch of wall leading up to that corner, cut at 45
// degrees -- the point's distance to the corner along the wall must be at
// most kCoverEdgeReach and at least its distance from the wall. The corner
// itself and anything diagonally beyond the face's span get no cover.
std::vector<Wall> CollectWalls(const std::vector<Obstacle>& obstacles, const glm::vec2& c,
                               float feetY, float band) {
  std::vector<Wall> walls;
  std::vector<glm::vec2> poly;
  for (const Obstacle& ob : obstacles) {
    if (ob.navOnly) continue;
    const AABB& b = ob.bounds;
    if (b.max.y < feetY + kCoverMinHeight || b.min.y > feetY + 0.3f) continue;
    if (c.x < b.min.x - band || c.x > b.max.x + band || c.y < b.min.z - band ||
        c.y > b.max.z + band) {
      continue;
    }
    // Last: the width test walks the footprint, the checks above are compares.
    if (!WideEnoughForCover(ob)) continue;
    if (!ob.footprint.empty()) {
      poly = ob.footprint;
    } else {
      poly = {{b.min.x, b.min.z}, {b.max.x, b.min.z}, {b.max.x, b.max.z}, {b.min.x, b.max.z}};
    }
    glm::vec2 centroid(0.0f);
    for (const glm::vec2& v : poly) centroid += v;
    centroid /= static_cast<float>(poly.size());
    for (size_t i = 0; i < poly.size(); ++i) {
      const glm::vec2 v0 = poly[i], v1 = poly[(i + 1) % poly.size()];
      const float len = glm::length(v1 - v0);
      if (len < 1e-4f) continue;
      const glm::vec2 along = (v1 - v0) / len;
      glm::vec2 n(along.y, -along.x);
      if (glm::dot(centroid - v0, n) > 0.0f) n = -n;  // Outward.
      const float o = glm::dot(c - v0, n);
      if (o <= 1e-4f || o > band) continue;
      const float t = glm::dot(c - v0, along);
      if (t < 0.0f || t > len) continue;  // Past the face's span.
      Wall w;
      w.toWall = -n;
      w.height = b.max.y - feetY;
      const struct {
        glm::vec2 corner, out;
        float a;  // Distance from the point to this end along the wall.
      } ends[2] = {{v0, -along, t}, {v1, along, len - t}};
      for (const auto& end : ends) {
        if (end.a > kCoverEdgeReach || o > end.a) continue;  // Outside the wedge.
        // Open end: nothing continues the wall just past the corner behind
        // the face plane, and the ground around the corner is free to peek
        // from. A joint between collinear boxes stays one flat wall.
        const glm::vec2 base = end.corner + end.out * kCoverEdgeProbeOut;
        if (Blocked(obstacles, base - n * kCoverEdgeProbeIn, feetY)) continue;
        if (Blocked(obstacles, base + n * kCoverEdgeProbeOut, feetY)) continue;
        w.tactical = true;
        break;
      }
      walls.push_back(w);
    }
  }
  return walls;
}

}  // namespace

CoverTile ClassifyCover(const std::vector<Obstacle>& obstacles, const glm::vec3& feet,
                        float* yaw) {
  CoverTile tile;
  const glm::vec2 c(feet.x, feet.z);
  if (Blocked(obstacles, c, feet.y)) return tile;  // Inside a blocker: not a standing spot.
  const std::vector<Wall> walls = CollectWalls(obstacles, c, feet.y, kCoverProbeDistance);
  if (walls.empty()) return tile;
  glm::vec2 sum(0.0f);
  bool tactical = false;
  float tallest = 0.0f;
  for (const Wall& w : walls) {
    tallest = std::max(tallest, w.height);
    tactical |= w.tactical;
    sum += w.toWall;
    const float angle = std::atan2(w.toWall.y, w.toWall.x);
    const int k = (static_cast<int>(std::lround(angle / (3.14159265358979f / 4.0f))) + kDirs) %
                  kDirs;
    tile.dirMask |= static_cast<uint8_t>(1u << k);
  }
  tile.crouch = tallest < kCoverStandHeight;
  tile.kind = tactical ? CoverKind::Tactical : CoverKind::Flat;
  if (yaw) {
    *yaw = glm::length(sum) > 1e-4f ? std::atan2(sum.y, sum.x) : 0.0f;
  }
  return tile;
}

std::vector<CoverZoneRect> CoverZoneRects(const std::vector<Obstacle>& obstacles, float feetY,
                                          float band) {
  std::vector<CoverZoneRect> rects;
  std::vector<glm::vec2> poly;
  for (const Obstacle& ob : obstacles) {
    if (ob.navOnly) continue;
    const AABB& b = ob.bounds;
    if (b.max.y < feetY + kCoverMinHeight || b.min.y > feetY + 0.3f) continue;
    if (!WideEnoughForCover(ob)) continue;
    if (!ob.footprint.empty()) {
      poly = ob.footprint;
    } else {
      poly = {{b.min.x, b.min.z}, {b.max.x, b.min.z}, {b.max.x, b.max.z}, {b.min.x, b.max.z}};
    }
    glm::vec2 centroid(0.0f);
    for (const glm::vec2& v : poly) centroid += v;
    centroid /= static_cast<float>(poly.size());
    for (size_t i = 0; i < poly.size(); ++i) {
      const glm::vec2 v0 = poly[i], v1 = poly[(i + 1) % poly.size()];
      const float len = glm::length(v1 - v0);
      if (len < 1e-4f) continue;
      const glm::vec2 along = (v1 - v0) / len;
      glm::vec2 n(along.y, -along.x);
      if (glm::dot(centroid - v0, n) > 0.0f) n = -n;  // Outward.
      // How far each open end's tactical stretch reaches along the face,
      // with the same openness probes as CollectWalls above.
      const struct {
        glm::vec2 corner, dirOut;
      } ends[2] = {{v0, -along}, {v1, along}};
      float reach[2] = {0.0f, 0.0f};
      for (int e = 0; e < 2; ++e) {
        const glm::vec2 base = ends[e].corner + ends[e].dirOut * kCoverEdgeProbeOut;
        if (Blocked(obstacles, base - n * kCoverEdgeProbeIn, feetY)) continue;
        if (Blocked(obstacles, base + n * kCoverEdgeProbeOut, feetY)) continue;
        reach[e] = std::min(kCoverEdgeReach, len);
      }
      const auto add = [&](float t0, float t1, bool tactical) {
        if (t1 - t0 < 1e-4f) return;
        rects.push_back({v0 + along * t0, along, n, t1 - t0, band, tactical});
      };
      if (reach[0] + reach[1] >= len) {
        // The open ends' tactical stretches cover the whole face.
        add(0.0f, len, true);
      } else {
        add(0.0f, reach[0], true);
        add(reach[0], len - reach[1], false);
        add(len - reach[1], len, true);
      }
    }
  }
  return rects;
}

CoverMap ComputeCoverMap(const std::vector<Obstacle>& obstacles, const ReachField& field) {
  CoverMap map;
  map.nx = field.nx;
  map.nz = field.nz;
  map.tiles.assign(static_cast<size_t>(field.nx) * field.nz, CoverTile{});
  if (field.nx <= 0 || field.nz <= 0) return map;
  // Only blockers near the field can matter; the full list can be huge. The
  // margin covers the cover band plus the corner openness probes.
  const float margin = kCoverProbeDistance + kCoverEdgeReach + kCoverEdgeProbeOut + 0.1f;
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
