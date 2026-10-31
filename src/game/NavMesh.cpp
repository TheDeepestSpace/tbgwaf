#include "game/NavMesh.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <utility>

#include "game/Raycast.h"
#include "game/Geometry.h"

namespace tactics {
namespace {

constexpr float kEps = 1e-4f;

// Climb-top cells are inset from the obstacle's own footprint so an agent
// standing on top doesn't have its center placed right at (or past) the
// edge. Smaller than `kAgentRadius` so even a 1.2x1.2 crate keeps a usable
// top surface.
constexpr float kClimbTopInset = 0.15f;

struct ZInterval {
  float lo, hi;
};

std::vector<ZInterval> MergeIntervals(std::vector<ZInterval> intervals) {
  std::sort(intervals.begin(), intervals.end(),
            [](const ZInterval& a, const ZInterval& b) { return a.lo < b.lo; });
  std::vector<ZInterval> merged;
  for (const auto& iv : intervals) {
    if (!merged.empty() && iv.lo <= merged.back().hi + kEps) {
      merged.back().hi = std::max(merged.back().hi, iv.hi);
    } else {
      merged.push_back(iv);
    }
  }
  return merged;
}

struct Portal {
  glm::vec2 left, right;
};

// Twice the signed area of triangle (a, b, c) in the XZ plane.
float TriArea2(glm::vec2 a, glm::vec2 b, glm::vec2 c) {
  const glm::vec2 ab = b - a;
  const glm::vec2 ac = c - a;
  return ac.x * ab.y - ab.x * ac.y;
}

// Simple stupid funnel algorithm: pulls a path from `start` to `goal` taut
// through a sequence of portals, bending only at portal endpoints (i.e.
// obstacle corners). Appends the corners and `goal` (not `start`) to `out`.
void Funnel(glm::vec2 start, glm::vec2 goal, const std::vector<Portal>& inner,
            std::vector<glm::vec2>* out) {
  std::vector<Portal> portals;
  portals.push_back({start, start});
  portals.insert(portals.end(), inner.begin(), inner.end());
  portals.push_back({goal, goal});

  glm::vec2 apex = start, left = start, right = start;
  size_t leftIdx = 0, rightIdx = 0;
  for (size_t i = 1; i < portals.size(); ++i) {
    const glm::vec2 newLeft = portals[i].left;
    const glm::vec2 newRight = portals[i].right;

    if (TriArea2(apex, right, newRight) <= 0.0f) {
      if (apex == right || TriArea2(apex, left, newRight) > 0.0f) {
        right = newRight;
        rightIdx = i;
      } else {
        out->push_back(left);
        apex = left;
        right = left;
        rightIdx = leftIdx;
        i = leftIdx;
        continue;
      }
    }
    if (TriArea2(apex, left, newLeft) >= 0.0f) {
      if (apex == left || TriArea2(apex, right, newLeft) < 0.0f) {
        left = newLeft;
        leftIdx = i;
      } else {
        out->push_back(right);
        apex = right;
        left = right;
        leftIdx = rightIdx;
        i = rightIdx;
        continue;
      }
    }
  }
  out->push_back(goal);
}

// True if axis-aligned rectangle `cell` and `rect` share a border segment of
// positive length (touching along one axis while overlapping on the other).
bool RectsAdjacent(const NavCell& cell, const AABB& rect) {
  const bool xTouch =
      std::fabs(cell.xMax - rect.min.x) < kEps || std::fabs(cell.xMin - rect.max.x) < kEps;
  const bool zOverlap = cell.zMin < rect.max.z - kEps && cell.zMax > rect.min.z + kEps;
  if (xTouch && zOverlap) return true;

  const bool zTouch =
      std::fabs(cell.zMax - rect.min.z) < kEps || std::fabs(cell.zMin - rect.max.z) < kEps;
  const bool xOverlap = cell.xMin < rect.max.x - kEps && cell.xMax > rect.min.x + kEps;
  return zTouch && xOverlap;
}

}  // namespace

namespace {
NavRegion SquareRegion(float halfExtent) {
  return NavRegion{-halfExtent, halfExtent, -halfExtent, halfExtent};
}
}  // namespace

void NavMesh::Build(const std::vector<AABB>& obstacles, float mapHalfExtent, float agentRadius) {
  std::vector<Obstacle> typed;
  typed.reserve(obstacles.size());
  for (const AABB& bounds : obstacles) typed.push_back(Obstacle{bounds, false});
  Build(typed, SquareRegion(mapHalfExtent), agentRadius, nullptr, nullptr);
}

void NavMesh::Build(const std::vector<Obstacle>& obstacles, float mapHalfExtent,
                     float agentRadius, const HeightField* ground,
                     const std::vector<WalkSurface>* walkSurfaces) {
  Build(obstacles, SquareRegion(mapHalfExtent), agentRadius, ground, walkSurfaces);
}

void NavMesh::Build(const std::vector<Obstacle>& obstacles, const NavRegion& region,
                     float agentRadius, const HeightField* ground,
                     const std::vector<WalkSurface>* walkSurfaces) {
  ground_ = ground ? *ground : HeightField{};
  walkSurfaces_ = walkSurfaces ? *walkSurfaces : std::vector<WalkSurface>{};
  BuildSurfaceCaches(agentRadius);
  polygonMode_ = false;
  paddedPolygons_.clear();
  paddedPolygons_.reserve(obstacles.size());
  for (const Obstacle& obstacle : obstacles) {
    polygonMode_ |= !obstacle.footprint.empty();
    paddedPolygons_.push_back(ExpandConvexPolygon(ObstacleFootprint(obstacle), agentRadius));
  }
  AddLowDeckBlockers(agentRadius);
  BuildGroundMesh(ObstacleBounds(obstacles), region, agentRadius);
  AddClimbConnections(obstacles, agentRadius);
}

void NavMesh::BuildGroundMesh(const std::vector<AABB>& obstacles, const NavRegion& region,
                               float agentRadius) {
  cells_.clear();
  neighbors_.clear();
  paddedFootprints_.clear();
  region_ = region;

  struct Footprint {
    float xMin, xMax, zMin, zMax;
  };
  std::vector<Footprint> footprints;
  footprints.reserve(obstacles.size());
  for (const auto& obstacle : obstacles) {
    Footprint fp;
    fp.xMin = std::max(region.xMin, obstacle.min.x - agentRadius);
    fp.xMax = std::min(region.xMax, obstacle.max.x + agentRadius);
    fp.zMin = std::max(region.zMin, obstacle.min.z - agentRadius);
    fp.zMax = std::min(region.zMax, obstacle.max.z + agentRadius);
    if (fp.xMax > fp.xMin + kEps && fp.zMax > fp.zMin + kEps) {
      footprints.push_back(fp);
      // Give the padded footprint a generous Y range so it can be reused
      // directly with the 3D LineOfSightClear ray/AABB test below.
      paddedFootprints_.push_back(
          AABB{glm::vec3(fp.xMin, -1.0f, fp.zMin), glm::vec3(fp.xMax, 1000.0f, fp.zMax)});
    }
  }

  std::vector<float> xs = {region.xMin, region.xMax};
  for (const auto& fp : footprints) {
    xs.push_back(fp.xMin);
    xs.push_back(fp.xMax);
  }
  std::sort(xs.begin(), xs.end());
  xs.erase(std::unique(xs.begin(), xs.end(),
                        [](float a, float b) { return std::fabs(a - b) < kEps; }),
           xs.end());

  std::vector<std::vector<int>> cellsByStrip(xs.empty() ? 0 : xs.size() - 1);

  for (size_t i = 0; i + 1 < xs.size(); ++i) {
    const float stripXMin = xs[i];
    const float stripXMax = xs[i + 1];
    if (stripXMax - stripXMin < kEps) continue;
    const float stripCenterX = (stripXMin + stripXMax) * 0.5f;

    std::vector<ZInterval> blocked;
    for (const auto& fp : footprints) {
      if (fp.xMin <= stripCenterX && fp.xMax >= stripCenterX) {
        blocked.push_back({fp.zMin, fp.zMax});
      }
    }
    std::vector<ZInterval> merged = MergeIntervals(std::move(blocked));

    float cursor = region.zMin;
    for (const auto& iv : merged) {
      if (iv.lo - cursor > kEps) {
        cellsByStrip[i].push_back(static_cast<int>(cells_.size()));
        cells_.push_back(NavCell{stripXMin, stripXMax, cursor, iv.lo, 0.0f});
      }
      cursor = std::max(cursor, iv.hi);
    }
    if (region.zMax - cursor > kEps) {
      cellsByStrip[i].push_back(static_cast<int>(cells_.size()));
      cells_.push_back(NavCell{stripXMin, stripXMax, cursor, region.zMax, 0.0f});
    }
  }

  neighbors_.resize(cells_.size());
  for (size_t i = 0; i + 1 < cellsByStrip.size(); ++i) {
    for (int aIdx : cellsByStrip[i]) {
      for (int bIdx : cellsByStrip[i + 1]) {
        const NavCell& a = cells_[aIdx];
        const NavCell& b = cells_[bIdx];
        const float zLo = std::max(a.zMin, b.zMin);
        const float zHi = std::min(a.zMax, b.zMax);
        if (zHi - zLo < kEps) continue;
        neighbors_[aIdx].push_back(bIdx);
        neighbors_[bIdx].push_back(aIdx);
      }
    }
  }
}

void NavMesh::AddClimbConnections(const std::vector<Obstacle>& obstacles, float agentRadius) {
  const int groundCellCount = static_cast<int>(cells_.size());

  for (const auto& obstacle : obstacles) {
    if (!obstacle.climbable) continue;
    const AABB& bounds = obstacle.bounds;
    // Windowed builds: skip obstacles entirely outside the region.
    if (bounds.max.x < region_.xMin || bounds.min.x > region_.xMax ||
        bounds.max.z < region_.zMin || bounds.min.z > region_.zMax) {
      continue;
    }

    const float xMin = bounds.min.x + kClimbTopInset;
    const float xMax = bounds.max.x - kClimbTopInset;
    const float zMin = bounds.min.z + kClimbTopInset;
    const float zMax = bounds.max.z - kClimbTopInset;
    if (xMax <= xMin + kEps || zMax <= zMin + kEps) continue;  // Too small to stand on.

    const int topIdx = static_cast<int>(cells_.size());
    cells_.push_back(NavCell{xMin, xMax, zMin, zMax, bounds.max.y, /*climbTop=*/true});
    neighbors_.emplace_back();

    const AABB padded{
        glm::vec3(std::max(region_.xMin, bounds.min.x - agentRadius), bounds.min.y,
                   std::max(region_.zMin, bounds.min.z - agentRadius)),
        glm::vec3(std::min(region_.xMax, bounds.max.x + agentRadius), bounds.min.y,
                   std::min(region_.zMax, bounds.max.z + agentRadius))};

    for (int groundIdx = 0; groundIdx < groundCellCount; ++groundIdx) {
      if (!RectsAdjacent(cells_[groundIdx], padded)) continue;
      neighbors_[groundIdx].push_back(topIdx);
      neighbors_[topIdx].push_back(groundIdx);
    }
  }
}

void NavMesh::BuildSurfaceCaches(float agentRadius) {
  const size_t n = walkSurfaces_.size();
  surfaceBlockers_.assign(n, {});
  for (size_t i = 0; i < n; ++i) {
    for (const auto& blocker : walkSurfaces_[i].blockers) {
      // Half the usual obstacle padding: roof furniture is knee-to-waist-high
      // scenery, so a figure may stand close enough to brush it.
      std::vector<glm::vec2> padded = ExpandConvexPolygon(blocker, 0.5f * agentRadius);
      if (!padded.empty()) surfaceBlockers_[i].push_back(std::move(padded));
    }
  }
  surfaceFootprints_.assign(n, {});
  surfaceBounds_.assign(n, glm::vec4(0.0f));
  surfaceCenters_.assign(n, glm::vec3(0.0f));
  surfaceParent_.clear();
  surfaceGrid_.clear();
  gridNx_ = gridNz_ = 0;
  if (n == 0) return;

  const float inf = std::numeric_limits<float>::infinity();
  float minX = inf, minZ = inf, maxX = -inf, maxZ = -inf;
  float meanSpan = 0.0f;
  for (size_t i = 0; i < n; ++i) {
    glm::vec4 bounds(inf, inf, -inf, -inf);
    for (const glm::vec3& v : walkSurfaces_[i].vertices) {
      surfaceFootprints_[i].emplace_back(v.x, v.z);
      bounds.x = std::min(bounds.x, v.x);
      bounds.y = std::min(bounds.y, v.z);
      bounds.z = std::max(bounds.z, v.x);
      bounds.w = std::max(bounds.w, v.z);
    }
    // Padded to cover PointInConvexPolygon's boundary epsilon.
    constexpr float kBoundsPad = 1e-3f;
    bounds += glm::vec4(-kBoundsPad, -kBoundsPad, kBoundsPad, kBoundsPad);
    surfaceBounds_[i] = bounds;
    surfaceCenters_[i] = SurfaceCenter(walkSurfaces_[i]);
    if (walkSurfaces_[i].vertices.empty()) continue;
    minX = std::min(minX, bounds.x);
    minZ = std::min(minZ, bounds.y);
    maxX = std::max(maxX, bounds.z);
    maxZ = std::max(maxZ, bounds.w);
    meanSpan += std::max(bounds.z - bounds.x, bounds.w - bounds.y);
  }

  // Uniform grid sized to the typical surface so each covers a few cells.
  if (minX <= maxX) {
    gridCell_ = std::max(meanSpan / static_cast<float>(n), 0.5f);
    gridMinX_ = minX;
    gridMinZ_ = minZ;
    gridNx_ = std::min(256, static_cast<int>((maxX - minX) / gridCell_) + 1);
    gridNz_ = std::min(256, static_cast<int>((maxZ - minZ) / gridCell_) + 1);
    gridCell_ = std::max(gridCell_, std::max((maxX - minX) / gridNx_, (maxZ - minZ) / gridNz_));
    surfaceGrid_.assign(static_cast<size_t>(gridNx_) * gridNz_, {});
    const auto clampCell = [](int v, int count) { return std::max(0, std::min(count - 1, v)); };
    for (size_t i = 0; i < n; ++i) {
      if (walkSurfaces_[i].vertices.empty()) continue;
      const glm::vec4& b = surfaceBounds_[i];
      const int x0 = clampCell(static_cast<int>(std::floor((b.x - gridMinX_) / gridCell_)), gridNx_);
      const int x1 = clampCell(static_cast<int>(std::floor((b.z - gridMinX_) / gridCell_)), gridNx_);
      const int z0 = clampCell(static_cast<int>(std::floor((b.y - gridMinZ_) / gridCell_)), gridNz_);
      const int z1 = clampCell(static_cast<int>(std::floor((b.w - gridMinZ_) / gridCell_)), gridNz_);
      for (int gz = z0; gz <= z1; ++gz) {
        for (int gx = x0; gx <= x1; ++gx) {
          surfaceGrid_[static_cast<size_t>(gz) * gridNx_ + gx].push_back(static_cast<int>(i));
        }
      }
    }
  }

  // All-pairs shortest routes over the center-to-center surface graph: one
  // Dijkstra per source, done once here instead of per FindSurfacePath call.
  surfaceParent_.assign(n, std::vector<int>(n, -1));
  using Entry = std::pair<float, int>;
  std::vector<float> distance(n);
  for (size_t src = 0; src < n; ++src) {
    std::fill(distance.begin(), distance.end(), inf);
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
    distance[src] = 0.0f;
    open.push({0.0f, static_cast<int>(src)});
    while (!open.empty()) {
      const auto [d, current] = open.top();
      open.pop();
      if (d != distance[current]) continue;
      for (int next : walkSurfaces_[current].neighbors) {
        if (next < 0 || static_cast<size_t>(next) >= n) continue;
        const float edge = glm::distance(surfaceCenters_[current], surfaceCenters_[next]);
        if (d + edge < distance[next]) {
          distance[next] = d + edge;
          surfaceParent_[src][next] = current;
          open.push({distance[next], next});
        }
      }
    }
    surfaceParent_[src][src] = -1;
  }
}

bool NavMesh::SurfaceContains(int surface, float x, float z) const {
  const glm::vec4& b = surfaceBounds_[surface];
  if (x < b.x || x > b.z || z < b.y || z > b.w) return false;
  return PointInConvexPolygon({x, z}, surfaceFootprints_[surface]);
}

bool NavMesh::SurfaceBlocked(int surface, float x, float z) const {
  for (const auto& polygon : surfaceBlockers_[surface]) {
    if (PointInConvexPolygon({x, z}, polygon)) return true;
  }
  return false;
}

const std::vector<int>& NavMesh::SurfaceCandidates(float x, float z) const {
  static const std::vector<int> kNone;
  if (surfaceGrid_.empty()) return kNone;
  const int gx = static_cast<int>(std::floor((x - gridMinX_) / gridCell_));
  const int gz = static_cast<int>(std::floor((z - gridMinZ_) / gridCell_));
  if (gx < 0 || gz < 0 || gx >= gridNx_ || gz >= gridNz_) return kNone;
  return surfaceGrid_[static_cast<size_t>(gz) * gridNx_ + gx];
}

ReachField NavMesh::ComputeReachField(const glm::vec3& start, float budget, float step,
                                      const MovingBlockers* blockers) const {
  ReachField field;
  field.step = step;
  field.budget = budget;
  if (budget <= 0.0f || step <= 0.0f) return field;
  if (blockers && (blockers->trajectories.empty() || blockers->dt <= 0.0f ||
                   blockers->speed <= 0.0f)) {
    blockers = nullptr;
  }
  // Blocker position when the mover has walked `distance` of the field.
  const auto blockerSampleAt = [&](const std::vector<glm::vec3>& traj, float distance) {
    const float t = (blockers->distanceOffset + distance) / blockers->speed;
    const size_t i = static_cast<size_t>(std::max(0L, std::lround(t / blockers->dt)));
    return traj[std::min(i, traj.size() - 1)];
  };
  const auto blockedAt = [&](const glm::vec3& p, float distance) {
    for (const auto& traj : blockers->trajectories) {
      if (traj.empty()) continue;
      const glm::vec3 s = blockerSampleAt(traj, distance);
      if (glm::length(glm::vec2(p.x - s.x, p.z - s.z)) < blockers->radius &&
          std::fabs(p.y - s.y) < blockers->height) {
        return true;
      }
    }
    return false;
  };
  const int startSurface = FindWalkSurfaceContaining(start.x, start.z, start.y);
  if (startSurface < 0 && !IsWalkable(start.x, start.z)) return field;

  // Align the grid so the source is exactly a node.
  const int radius = static_cast<int>(std::ceil(budget / step));
  field.minX = start.x - radius * step;
  field.minZ = start.z - radius * step;
  field.nx = field.nz = 2 * radius + 1;
  const float inf = std::numeric_limits<float>::infinity();
  field.dist.assign(static_cast<size_t>(field.nx) * field.nz, inf);
  field.surfaceY.resize(field.dist.size(), 0.0f);
  field.parent.assign(field.dist.size(), -1);
  for (int iz = 0; iz < field.nz; ++iz) {
    for (int ix = 0; ix < field.nx; ++ix) {
      const float x = field.minX + ix * field.step;
      const float z = field.minZ + iz * field.step;
      field.surfaceY[iz * field.nx + ix] = ground_.HeightAt(x, z);
    }
  }

  // Broadphase for the Dijkstra below (issue #256): testing every node and
  // every short edge against every padded blocker polygon is
  // O(nodes x polygons) and cost ~0.5 s per move click on urban maps. Bucket
  // the polygons into a coarse grid over the field window so each query only
  // tests the polygons whose bounding box overlaps its cell(s); skipping
  // polygons whose box cannot touch the query leaves every result unchanged.
  const float span = (field.nx - 1) * step;
  const float gridCell = std::max(1.0f, 4.0f * step);
  const int gridN = std::max(1, static_cast<int>(std::ceil(span / gridCell)));
  std::vector<std::vector<int>> gridBuckets(static_cast<size_t>(gridN) * gridN);
  const auto gridIndex = [&](float v, float minV) {
    return std::clamp(static_cast<int>((v - minV) / gridCell), 0, gridN - 1);
  };
  if (polygonMode_) {
    for (size_t p = 0; p < paddedPolygons_.size(); ++p) {
      const std::vector<glm::vec2>& polygon = paddedPolygons_[p];
      if (polygon.empty()) continue;
      glm::vec2 lo = polygon[0], hi = polygon[0];
      for (const glm::vec2& v : polygon) {
        lo = glm::min(lo, v);
        hi = glm::max(hi, v);
      }
      if (hi.x < field.minX || hi.y < field.minZ || lo.x > field.minX + span ||
          lo.y > field.minZ + span) {
        continue;
      }
      const int x0 = gridIndex(lo.x, field.minX), x1 = gridIndex(hi.x, field.minX);
      const int z0 = gridIndex(lo.y, field.minZ), z1 = gridIndex(hi.y, field.minZ);
      for (int gz = z0; gz <= z1; ++gz) {
        for (int gx = x0; gx <= x1; ++gx) {
          gridBuckets[static_cast<size_t>(gz) * gridN + gx].push_back(static_cast<int>(p));
        }
      }
    }
  }
  // True if `test` holds for some polygon bucketed near the XZ rectangle
  // [lo, hi] (which must lie inside the field window).
  const auto anyPolygonNear = [&](const glm::vec2& lo, const glm::vec2& hi, const auto& test) {
    const int x0 = gridIndex(lo.x, field.minX), x1 = gridIndex(hi.x, field.minX);
    const int z0 = gridIndex(lo.y, field.minZ), z1 = gridIndex(hi.y, field.minZ);
    for (int gz = z0; gz <= z1; ++gz) {
      for (int gx = x0; gx <= x1; ++gx) {
        for (int p : gridBuckets[static_cast<size_t>(gz) * gridN + gx]) {
          if (test(paddedPolygons_[p])) return true;
        }
      }
    }
    return false;
  };

  // 16-neighbour moves (8 king + 8 knight) keep grid distance close to
  // Euclidean (< ~2% error) while still routing around obstacles.
  static const int kMoves[16][2] = {{1, 0},  {-1, 0}, {0, 1},  {0, -1}, {1, 1},  {1, -1},
                                    {-1, 1}, {-1, -1}, {2, 1},  {2, -1}, {-2, 1}, {-2, -1},
                                    {1, 2},  {1, -2}, {-1, 2}, {-1, -2}};
  std::vector<char> walkable(field.dist.size(), -1);  // -1 unknown, 0/1 cached.
  auto nodeWalkable = [&](int ix, int iz) {
    char& w = walkable[iz * field.nx + ix];
    if (w < 0) {
      const glm::vec3 p = field.Node(ix, iz);
      if (!polygonMode_) {
        w = IsWalkable(p.x, p.z) ? 1 : 0;
      } else if (p.x < region_.xMin || p.x > region_.xMax || p.z < region_.zMin ||
                 p.z > region_.zMax) {
        // Same answer as IsWalkable, through the grid broadphase.
        w = 0;
      } else {
        w = anyPolygonNear({p.x, p.z}, {p.x, p.z},
                           [&](const std::vector<glm::vec2>& polygon) {
                             return PointInConvexPolygon({p.x, p.z}, polygon);
                           })
                ? 0
                : 1;
      }
    }
    return w == 1;
  };
  // Node position on the walked surface: distances then include the vertical
  // component over hilly terrain, matching FindPath's terrain-lifted paths.
  auto nodeWorld = [&](int ix, int iz) {
    glm::vec3 p = field.Node(ix, iz);
    p.y = ground_.HeightAt(p.x, p.z);
    return p;
  };

  if (startSurface < 0) {
    using Entry = std::pair<float, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
    const int startIdx = radius * field.nx + radius;
    field.dist[startIdx] = 0.0f;
    open.push({0.0f, startIdx});
    while (!open.empty()) {
      const auto [d, idx] = open.top();
      open.pop();
      if (d > field.dist[idx]) continue;
      const int ix = idx % field.nx, iz = idx / field.nx;
      const glm::vec3 from = nodeWorld(ix, iz);
      for (const auto& m : kMoves) {
        const int jx = ix + m[0], jz = iz + m[1];
        if (jx < 0 || jz < 0 || jx >= field.nx || jz >= field.nz) continue;
        if (!nodeWalkable(jx, jz)) continue;
        const glm::vec3 to = nodeWorld(jx, jz);
        // Sheer faces (stair/ramp/level sides) are not walkable.
        const float run = std::hypot(to.x - from.x, to.z - from.z);
        const float midY = ground_.HeightAt(0.5f * (from.x + to.x), 0.5f * (from.z + to.z));
        const float half = constants::kMaxWalkSlope * 0.5f * run + 1e-3f;
        if (std::fabs(midY - from.y) > half || std::fabs(to.y - midY) > half) continue;
        const float nd = d + glm::distance(from, to);
        const int jdx = jz * field.nx + jx;
        if (nd > budget || nd >= field.dist[jdx]) continue;
        bool clear = true;
        if (polygonMode_) {
          const glm::vec2 a(from.x, from.z), b(to.x, to.z);
          clear = !anyPolygonNear(glm::min(a, b), glm::max(a, b),
                                  [&](const std::vector<glm::vec2>& polygon) {
                                    return SegmentEntersConvexPolygon(a, b, polygon);
                                  });
        } else {
          clear = LineOfSightClear(from, to, paddedFootprints_);
        }
        if (!clear) continue;
        if (blockers) {
          // Space-time gate: the step must keep clear of every moving figure
          // both at its end and midpoint, at the times the mover is there.
          const glm::vec3 mid(0.5f * (from.x + to.x), midY, 0.5f * (from.z + to.z));
          if (blockedAt(to, nd) || blockedAt(mid, 0.5f * (d + nd))) continue;
        }
        field.dist[jdx] = nd;
        field.parent[jdx] = idx;
        open.push({nd, jdx});
      }
    }
  }

  std::vector<float> entryCost(walkSurfaces_.size(), inf);
  if (startSurface < 0) {
    for (size_t entry = 0; entry < walkSurfaces_.size(); ++entry) {
      if (!walkSurfaces_[entry].connectsToGround) continue;
      const glm::vec3 connection = GroundConnectionPoint(static_cast<int>(entry));
      glm::vec3 groundConnection = connection;
      groundConnection.y = ground_.HeightAt(connection.x, connection.z);
      std::vector<glm::vec3> path;
      if (!FindGroundPath(start, groundConnection, &path)) continue;
      float length = glm::distance(groundConnection, connection);
      for (size_t i = 0; i + 1 < path.size(); ++i) length += glm::distance(path[i], path[i + 1]);
      entryCost[entry] = length;
    }
  }
  std::vector<glm::vec3> entryPoint(walkSurfaces_.size());
  for (size_t entry = 0; entry < walkSurfaces_.size(); ++entry) {
    if (std::isfinite(entryCost[entry])) {
      entryPoint[entry] = GroundConnectionPoint(static_cast<int>(entry));
    }
  }

  // Add the explicit stacked layer without ever connecting it merely because
  // it overlaps ground in XZ. A ground start must first route to a declared
  // ramp foot; a surface start remains on its connected surface component.
  const auto pathLength = [](const std::vector<glm::vec3>& path) {
    float length = 0.0f;
    for (size_t i = 0; i + 1 < path.size(); ++i) length += glm::distance(path[i], path[i + 1]);
    return length;
  };
  for (int iz = 0; iz < field.nz; ++iz) {
    for (int ix = 0; ix < field.nx; ++ix) {
      const int idx = iz * field.nx + ix;
      const float x = field.minX + ix * field.step;
      const float z = field.minZ + iz * field.step;
      for (int surface : SurfaceCandidates(x, z)) {
        if (!SurfaceContains(surface, x, z) || SurfaceBlocked(surface, x, z)) continue;
        glm::vec3 goal(x, SurfaceHeightAt(walkSurfaces_[surface], x, z), z);
        std::vector<glm::vec3> path;
        bool found = false;
        float distance = 0.0f;
        if (startSurface >= 0) {
          found = FindSurfacePath(startSurface, start, surface, goal, &path);
          if (found) distance = pathLength(path);
        } else {
          // Cheapest ground entry overall, not the first that connects: the
          // deck and its ramps form one connected graph, so the first foot
          // always "succeeds" even when it routes the long way around and
          // prices the cell out of the budget.
          for (size_t entry = 0; entry < walkSurfaces_.size(); ++entry) {
            if (!std::isfinite(entryCost[entry])) continue;
            // Unreachable from this foot (cached routes), or provably no
            // better than the best/budget (a path is never shorter than the
            // straight line from the foot).
            if (static_cast<int>(entry) != surface &&
                surfaceParent_[entry][surface] < 0) {
              continue;
            }
            const glm::vec3& connection = entryPoint[entry];
            const float bound = entryCost[entry] + glm::distance(
                glm::vec2(connection.x, connection.z), glm::vec2(x, z));
            if (bound > budget || (found && bound >= distance)) continue;
            if (!FindSurfacePath(static_cast<int>(entry), connection, surface, goal, &path)) {
              continue;
            }
            const float total = entryCost[entry] + pathLength(path);
            if (!found || total < distance) distance = total;
            found = true;
          }
        }
        if (!found) continue;
        if (distance <= budget && (startSurface >= 0 || distance < field.dist[idx] + 0.5f)) {
          field.dist[idx] = distance;
          field.surfaceY[idx] = goal.y;
          field.parent[idx] = -1;  // Priced over the surface graph, not this grid.
        }
      }
    }
  }

  if (blockers) {
    // Endpoint pass: a mover stopping on a node stands there for the rest of
    // the round, so the node is no valid destination if any blocker comes
    // within radius after the mover's arrival. Trajectory bounding boxes
    // (inflated by the radius) keep this pass to the nodes near a blocker.
    field.endpointBlocked.assign(field.dist.size(), 0);
    std::vector<glm::vec4> trajBounds;  // (xMin, zMin, xMax, zMax)
    trajBounds.reserve(blockers->trajectories.size());
    for (const auto& traj : blockers->trajectories) {
      glm::vec4 b(inf, inf, -inf, -inf);
      for (const glm::vec3& s : traj) {
        b.x = std::min(b.x, s.x - blockers->radius);
        b.y = std::min(b.y, s.z - blockers->radius);
        b.z = std::max(b.z, s.x + blockers->radius);
        b.w = std::max(b.w, s.z + blockers->radius);
      }
      trajBounds.push_back(b);
    }
    for (int iz = 0; iz < field.nz; ++iz) {
      for (int ix = 0; ix < field.nx; ++ix) {
        const int idx = iz * field.nx + ix;
        if (field.dist[idx] > budget) continue;
        const glm::vec3 p(field.minX + ix * step, field.surfaceY[idx], field.minZ + iz * step);
        const float arrival = (blockers->distanceOffset + field.dist[idx]) / blockers->speed;
        const size_t from = static_cast<size_t>(
            std::max(0L, std::lround(arrival / blockers->dt)));
        for (size_t k = 0; k < blockers->trajectories.size() && !field.endpointBlocked[idx]; ++k) {
          const auto& traj = blockers->trajectories[k];
          if (traj.empty()) continue;
          const glm::vec4& b = trajBounds[k];
          if (p.x < b.x || p.z < b.y || p.x > b.z || p.z > b.w) continue;
          for (size_t i = std::min(from, traj.size() - 1); i < traj.size(); ++i) {
            if (glm::length(glm::vec2(p.x - traj[i].x, p.z - traj[i].z)) < blockers->radius &&
                std::fabs(p.y - traj[i].y) < blockers->height) {
              field.endpointBlocked[idx] = 1;
              break;
            }
          }
        }
      }
    }
  }
  return field;
}

bool NavMesh::IsWalkable(float x, float z) const {
  if (!polygonMode_) return FindCellContaining(x, z) >= 0;
  if (x < region_.xMin || x > region_.xMax || z < region_.zMin || z > region_.zMax) return false;
  for (const auto& polygon : paddedPolygons_) {
    if (PointInConvexPolygon({x, z}, polygon)) return false;
  }
  return true;
}

// Ground cells only, so IsWalkable matches its documented "ground level"
// semantics regardless of the terrain height under (x, z).
int NavMesh::FindCellContaining(float x, float z) const {
  for (size_t i = 0; i < cells_.size(); ++i) {
    if (!cells_[i].climbTop && cells_[i].Contains(x, z)) return static_cast<int>(i);
  }
  return -1;
}

int NavMesh::FindCellContaining(float x, float z, float yHint) const {
  int best = -1;
  float bestDy = std::numeric_limits<float>::infinity();
  for (size_t i = 0; i < cells_.size(); ++i) {
    if (!cells_[i].Contains(x, z)) continue;
    const float dy = std::fabs(SurfaceY(cells_[i], x, z) - yHint);
    if (dy < bestDy) {
      bestDy = dy;
      best = static_cast<int>(i);
    }
  }
  return best;
}

int NavMesh::FindWalkSurfaceContaining(float x, float z, float yHint) const {
  int best = -1;
  float bestDy = std::fabs(ground_.HeightAt(x, z) - yHint) + 1e-4f;
  for (int i : SurfaceCandidates(x, z)) {
    if (!SurfaceContains(i, x, z)) continue;
    const float dy = std::fabs(SurfaceHeightAt(walkSurfaces_[i], x, z) - yHint);
    if (dy <= bestDy) {
      bestDy = dy;
      best = i;
    }
  }
  return best;
}

float NavMesh::ResolveSurfaceY(float x, float z, float yHint) const {
  const int surface = FindWalkSurfaceContaining(x, z, yHint);
  return surface >= 0 ? SurfaceHeightAt(walkSurfaces_[surface], x, z) : ground_.HeightAt(x, z);
}

glm::vec3 NavMesh::GroundConnectionPoint(int surface) const {
  if (surface < 0 || static_cast<size_t>(surface) >= walkSurfaces_.size()) return glm::vec3(0.0f);
  const WalkSurface& patch = walkSurfaces_[surface];
  if (patch.vertices.empty()) return glm::vec3(0.0f);
  float lowest = patch.vertices.front().y;
  for (const glm::vec3& v : patch.vertices) lowest = std::min(lowest, v.y);
  glm::vec3 sum(0.0f);
  int count = 0;
  for (const glm::vec3& v : patch.vertices) {
    if (std::fabs(v.y - lowest) < 1e-3f) {
      sum += v;
      ++count;
    }
  }
  return count ? sum / static_cast<float>(count) : SurfaceCenter(patch);
}

bool NavMesh::FindSurfacePath(int startSurface, glm::vec3 start, int goalSurface,
                              glm::vec3 goal, std::vector<glm::vec3>* outPath) const {
  outPath->clear();
  if (startSurface < 0 || goalSurface < 0 ||
      static_cast<size_t>(startSurface) >= walkSurfaces_.size() ||
      static_cast<size_t>(goalSurface) >= walkSurfaces_.size()) {
    return false;
  }
  if (startSurface == goalSurface && !surfaceBlockers_[startSurface].empty()) {
    return FindBlockedSurfacePath(startSurface, start, goal, outPath);
  }
  if (startSurface != goalSurface && surfaceParent_[startSurface][goalSurface] < 0) return false;
  std::vector<int> route;
  for (int node = goalSurface; node >= 0; node = surfaceParent_[startSurface][node]) {
    route.push_back(node);
  }
  std::reverse(route.begin(), route.end());

  start.y = SurfaceHeightAt(walkSurfaces_[startSurface], start.x, start.z);
  goal.y = SurfaceHeightAt(walkSurfaces_[goalSurface], goal.x, goal.z);

  // Height on the corridor at (x, z): the first route surface containing the
  // point. Bends sit on seam vertices shared by both neighbors, where the
  // two planes agree, so "first containing" is unambiguous there.
  auto corridorY = [&](float x, float z, float fallback) {
    for (int node : route) {
      if (SurfaceContains(node, x, z)) {
        return SurfaceHeightAt(walkSurfaces_[node], x, z);
      }
    }
    return fallback;
  };

  // Walk the surface corridor. Runs of seam-adjacent surfaces are pulled
  // taut with the funnel through the shared seam edges (instead of forcing
  // every seam midpoint, which dog-legged paths via the deck centerline);
  // transitions without a shared edge stay hard waypoints. Funneled segments
  // are densified so the path tracks the deck's changing grade between
  // seams rather than cutting straight through a crest or sag.
  outPath->push_back(start);
  glm::vec3 from = start;
  std::vector<Portal> portals;
  auto flush = [&](glm::vec3 to) {
    std::vector<glm::vec2> pts;
    Funnel({from.x, from.z}, {to.x, to.z}, portals, &pts);
    constexpr float kSurfacePathStep = 1.5f;
    glm::vec3 prev = from;
    for (const glm::vec2& pt : pts) {
      glm::vec3 p(pt.x, 0.0f, pt.y);
      p.y = corridorY(p.x, p.z, prev.y);
      const float len = glm::length(glm::vec2(p.x - prev.x, p.z - prev.z));
      const int pieces = static_cast<int>(std::ceil(len / kSurfacePathStep));
      for (int k = 1; k < pieces; ++k) {
        const float t = static_cast<float>(k) / static_cast<float>(pieces);
        glm::vec3 q = prev + (p - prev) * t;
        q.y = corridorY(q.x, q.z, q.y);
        if (glm::distance(q, outPath->back()) > kEps) outPath->push_back(q);
      }
      if (glm::distance(p, outPath->back()) > kEps) outPath->push_back(p);
      prev = p;
    }
    outPath->back().y = to.y;
    portals.clear();
    from = to;
  };
  for (size_t i = 0; i + 1 < route.size(); ++i) {
    const WalkSurface& a = walkSurfaces_[route[i]];
    const WalkSurface& b = walkSurfaces_[route[i + 1]];
    std::vector<glm::vec3> shared;
    for (const glm::vec3& av : a.vertices) {
      for (const glm::vec3& bv : b.vertices) {
        if (glm::distance(av, bv) < 1e-3f) shared.push_back((av + bv) * 0.5f);
      }
    }
    if (shared.size() == 2) {
      // Full shared seam: a funnel portal, oriented left/right for the
      // direction of travel (from the previous portal, or the funnel start).
      const glm::vec2 u(shared[0].x, shared[0].z);
      const glm::vec2 v(shared[1].x, shared[1].z);
      const glm::vec2 mid = (u + v) * 0.5f;
      const glm::vec2 ref = portals.empty()
                                ? glm::vec2(from.x, from.z)
                                : (portals.back().left + portals.back().right) * 0.5f;
      glm::vec2 d = mid - ref;
      if (glm::dot(d, d) < kEps * kEps) d = glm::vec2(goal.x, goal.z) - mid;
      const float crossU = d.x * (u.y - mid.y) - d.y * (u.x - mid.x);
      const float crossV = d.x * (v.y - mid.y) - d.y * (v.x - mid.x);
      portals.push_back(crossU >= crossV ? Portal{u, v} : Portal{v, u});
      continue;
    }
    glm::vec3 transition(0.0f);
    if (!shared.empty()) {
      for (const glm::vec3& s : shared) transition += s;
      transition /= static_cast<float>(shared.size());
    } else {
      // A ramp may terminate in the interior of a wider deck. Prefer a
      // vertex contained by both, then fall back to the midpoint of centers.
      int contained = 0;
      for (const glm::vec3& v : a.vertices) {
        if (SurfaceContainsXZ(b, v.x, v.z)) {
          transition += v;
          ++contained;
        }
      }
      if (contained) transition /= static_cast<float>(contained);
      else transition = (SurfaceCenter(a) + SurfaceCenter(b)) * 0.5f;
    }
    transition.y = 0.5f * (SurfaceHeightAt(a, transition.x, transition.z) +
                            SurfaceHeightAt(b, transition.x, transition.z));
    flush(transition);
  }
  flush(goal);
  return true;
}

// Shortest path across one patch around its padded blockers (visibility
// graph over their corners). A start inside a padded blocker (a landing next
// to a parapet) may leave it; a goal inside one is refused.
bool NavMesh::FindBlockedSurfacePath(int surface, glm::vec3 start, glm::vec3 goal,
                                     std::vector<glm::vec3>* outPath) const {
  const WalkSurface& patch = walkSurfaces_[surface];
  const auto& blockers = surfaceBlockers_[surface];
  const glm::vec2 s(start.x, start.z), g(goal.x, goal.z);
  if (SurfaceBlocked(surface, g.x, g.y)) return false;
  std::vector<glm::vec2> nodes = {s, g};
  for (const auto& polygon : blockers) {
    // Way-nodes sit a nudge outside their blocker: a corner on the boundary
    // itself would read as blocked, and a leg running exactly along an edge
    // is numerically unsafe against the strict-interior segment test.
    for (const glm::vec2& p : ExpandConvexPolygon(polygon, 1e-3f)) {
      if (SurfaceContains(surface, p.x, p.y) && !SurfaceBlocked(surface, p.x, p.y)) {
        nodes.push_back(p);
      }
    }
  }
  std::vector<char> startInside(blockers.size(), 0);
  for (size_t i = 0; i < blockers.size(); ++i) {
    startInside[i] = PointInConvexPolygon(s, blockers[i]);
  }
  const auto clear = [&](size_t from, size_t to) {
    for (size_t i = 0; i < blockers.size(); ++i) {
      if (from == 0 && startInside[i]) continue;
      if (SegmentEntersConvexPolygon(nodes[from], nodes[to], blockers[i])) return false;
    }
    return true;
  };
  const size_t n = nodes.size();
  const float inf = std::numeric_limits<float>::infinity();
  std::vector<float> distance(n, inf);
  std::vector<int> parent(n, -1);
  std::vector<char> closed(n, 0);
  distance[0] = 0.0f;
  for (;;) {
    int current = -1;
    for (size_t i = 0; i < n; ++i) {
      if (!closed[i] && distance[i] < inf && (current < 0 || distance[i] < distance[current])) {
        current = static_cast<int>(i);
      }
    }
    if (current < 0 || current == 1) break;
    closed[current] = 1;
    for (size_t next = 1; next < n; ++next) {
      if (closed[next] || !clear(current, next)) continue;
      const float candidate = distance[current] + glm::distance(nodes[current], nodes[next]);
      if (candidate < distance[next]) {
        distance[next] = candidate;
        parent[next] = current;
      }
    }
  }
  if (!std::isfinite(distance[1])) return false;
  std::vector<int> route;
  for (int node = 1; node >= 0; node = parent[node]) route.push_back(node);
  std::reverse(route.begin(), route.end());
  outPath->clear();
  for (int node : route) {
    const glm::vec2 p = nodes[node];
    outPath->emplace_back(p.x, SurfaceHeightAt(patch, p.x, p.y), p.y);
  }
  return true;
}

bool NavMesh::FindPolygonGroundPath(glm::vec3 start, glm::vec3 goal,
                                    std::vector<glm::vec3>* outPath) const {
  outPath->clear();
  if (!IsWalkable(start.x, start.z) || !IsWalkable(goal.x, goal.z)) return false;
  std::vector<glm::vec2> nodes = {{start.x, start.z}, {goal.x, goal.z}};
  for (const auto& polygon : paddedPolygons_) {
    for (const glm::vec2& p : polygon) {
      if (p.x >= region_.xMin - kEps && p.x <= region_.xMax + kEps &&
          p.y >= region_.zMin - kEps && p.y <= region_.zMax + kEps) {
        nodes.push_back(p);
      }
    }
  }
  // Broad-phase bounds keep the O(nodes^2) visibility sweep cheap on dense
  // polygon scenes: most node pairs are nowhere near most footprints.
  std::vector<glm::vec4> polygonBounds;  // (xMin, zMin, xMax, zMax)
  polygonBounds.reserve(paddedPolygons_.size());
  for (const auto& polygon : paddedPolygons_) {
    glm::vec4 bounds(std::numeric_limits<float>::infinity(),
                     std::numeric_limits<float>::infinity(),
                     -std::numeric_limits<float>::infinity(),
                     -std::numeric_limits<float>::infinity());
    for (const glm::vec2& p : polygon) {
      bounds.x = std::min(bounds.x, p.x);
      bounds.y = std::min(bounds.y, p.y);
      bounds.z = std::max(bounds.z, p.x);
      bounds.w = std::max(bounds.w, p.y);
    }
    polygonBounds.push_back(bounds);
  }
  auto clear = [&](glm::vec2 a, glm::vec2 b) {
    const float xMin = std::min(a.x, b.x), xMax = std::max(a.x, b.x);
    const float zMin = std::min(a.y, b.y), zMax = std::max(a.y, b.y);
    for (size_t i = 0; i < paddedPolygons_.size(); ++i) {
      const glm::vec4& bounds = polygonBounds[i];
      if (xMax < bounds.x || xMin > bounds.z || zMax < bounds.y || zMin > bounds.w) continue;
      if (SegmentEntersConvexPolygon(a, b, paddedPolygons_[i])) return false;
    }
    return true;
  };
  const size_t n = nodes.size();
  std::vector<float> distance(n, std::numeric_limits<float>::infinity());
  std::vector<int> parent(n, -1);
  std::vector<char> closed(n, false);
  using Entry = std::pair<float, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  distance[0] = 0.0f;
  open.push({0.0f, 0});
  while (!open.empty()) {
    const auto [d, current] = open.top();
    open.pop();
    if (closed[current]) continue;
    closed[current] = true;
    if (current == 1) break;
    for (size_t next = 0; next < n; ++next) {
      if (next == static_cast<size_t>(current) || closed[next] || !clear(nodes[current], nodes[next])) {
        continue;
      }
      const float candidate = d + glm::distance(nodes[current], nodes[next]);
      if (candidate < distance[next]) {
        distance[next] = candidate;
        parent[next] = current;
        open.push({candidate, static_cast<int>(next)});
      }
    }
  }
  if (!std::isfinite(distance[1])) return false;
  std::vector<int> route;
  for (int node = 1; node >= 0; node = parent[node]) route.push_back(node);
  std::reverse(route.begin(), route.end());
  for (int node : route) {
    const glm::vec2 p = nodes[node];
    outPath->emplace_back(p.x, 0.0f, p.y);
  }
  LiftGroundPathOntoTerrain(outPath);
  return true;
}

bool NavMesh::FindPath(glm::vec3 start, glm::vec3 goal, std::vector<glm::vec3>* outPath) const {
  if (!outPath) return false;
  outPath->clear();
  const int startSurface = FindWalkSurfaceContaining(start.x, start.z, start.y);
  const int goalSurface = FindWalkSurfaceContaining(goal.x, goal.z, goal.y);
  if (startSurface < 0 && goalSurface < 0) return FindGroundPath(start, goal, outPath);
  if (startSurface >= 0 && goalSurface >= 0) {
    return FindSurfacePath(startSurface, start, goalSurface, goal, outPath);
  }

  // The only ground/surface transition is a patch that explicitly declares
  // a ramp foot. Try every such patch and retain the shortest complete route.
  std::vector<glm::vec3> best;
  float bestLength = std::numeric_limits<float>::infinity();
  for (size_t entry = 0; entry < walkSurfaces_.size(); ++entry) {
    if (!walkSurfaces_[entry].connectsToGround) continue;
    const glm::vec3 connection = GroundConnectionPoint(static_cast<int>(entry));
    glm::vec3 groundConnection = connection;
    groundConnection.y = ground_.HeightAt(connection.x, connection.z);
    std::vector<glm::vec3> groundPath, surfacePath, candidate;
    bool found = false;
    if (startSurface < 0) {
      found = FindGroundPath(start, groundConnection, &groundPath) &&
              FindSurfacePath(static_cast<int>(entry), connection, goalSurface, goal,
                              &surfacePath);
      candidate = std::move(groundPath);
      candidate.insert(candidate.end(), surfacePath.begin(), surfacePath.end());
    } else {
      found = FindSurfacePath(startSurface, start, static_cast<int>(entry), connection,
                              &surfacePath) &&
              FindGroundPath(groundConnection, goal, &groundPath);
      candidate = std::move(surfacePath);
      candidate.insert(candidate.end(), groundPath.begin(), groundPath.end());
    }
    if (!found) continue;
    float length = 0.0f;
    for (size_t i = 0; i + 1 < candidate.size(); ++i) {
      length += glm::distance(candidate[i], candidate[i + 1]);
    }
    if (length < bestLength) {
      bestLength = length;
      best = std::move(candidate);
    }
  }
  if (best.empty()) return false;
  *outPath = std::move(best);
  return true;
}

// The ground beneath the low part of a ramp (or any deck hugging the ground)
// has no headroom, so it is impassable: block the sub-polygon of each surface
// whose clearance lies in (kFootSlack, kMinDeckHeadroom). The slab plane is
// linear, so clipping the convex footprint against the two clearance levels
// stays convex. Clearance near zero (the ramp foot itself) stays open so the
// ground-to-ramp transition remains reachable.
void NavMesh::AddLowDeckBlockers(float agentRadius) {
  for (const WalkSurface& surface : walkSurfaces_) {
    if (surface.vertices.size() < 3) continue;
    std::vector<glm::vec3> pts;
    for (const glm::vec3& v : surface.vertices) {
      pts.emplace_back(v.x, v.z, v.y - ground_.HeightAt(v.x, v.z));  // (x, z, clearance)
    }
    // Sutherland-Hodgman against clearance >= lo, then clearance <= hi.
    auto clip = [](const std::vector<glm::vec3>& in, float level, bool keepAbove) {
      std::vector<glm::vec3> out;
      for (size_t i = 0; i < in.size(); ++i) {
        const glm::vec3& a = in[i];
        const glm::vec3& b = in[(i + 1) % in.size()];
        const bool aIn = keepAbove ? a.z >= level : a.z <= level;
        const bool bIn = keepAbove ? b.z >= level : b.z <= level;
        if (aIn) out.push_back(a);
        if (aIn != bIn) {
          const float t = (level - a.z) / (b.z - a.z);
          out.push_back(a + (b - a) * t);
        }
      }
      return out;
    };
    constexpr float kFootSlack = 0.3f;
    // A declared ground transition whose lowest edge is already raised is a
    // step up onto the slab, not a ramp foot: its connection stays reachable.
    if (surface.connectsToGround) {
      float lowest = pts.front().z;
      for (const glm::vec3& p : pts) lowest = std::min(lowest, p.z);
      if (lowest > kFootSlack) continue;
    }
    std::vector<glm::vec3> band = clip(pts, kFootSlack, true);
    band = clip(band, constants::kMinDeckHeadroom, false);
    if (band.size() < 3) continue;
    std::vector<glm::vec2> polygon;
    for (const glm::vec3& p : band) polygon.emplace_back(p.x, p.y);
    std::vector<glm::vec2> padded = ExpandConvexPolygon(polygon, agentRadius);
    if (padded.size() < 3) continue;
    polygonMode_ = true;
    paddedPolygons_.push_back(std::move(padded));
  }
}

bool NavMesh::GroundUnderLowDeck(float x, float z) const {
  // Below kFootSlack the deck is effectively at grade (a ramp foot or slab
  // edge), where the ground and the deck are the same place.
  constexpr float kFootSlack = 0.3f;
  const float groundY = ground_.HeightAt(x, z);
  for (int i : SurfaceCandidates(x, z)) {
    if (!SurfaceContains(i, x, z)) continue;
    const float clearance = SurfaceHeightAt(walkSurfaces_[i], x, z) - groundY;
    if (clearance > kFootSlack && clearance < constants::kMinDeckHeadroom) return true;
  }
  return false;
}

bool NavMesh::FindPlanarGroundPath(glm::vec3 start, glm::vec3 goal,
                                   std::vector<glm::vec3>* outPath) const {
  if (!outPath) return false;
  outPath->clear();

  if (polygonMode_) return FindPolygonGroundPath(start, goal, outPath);

  const int startCell = FindCellContaining(start.x, start.z, start.y);
  const int goalCell = FindCellContaining(goal.x, goal.z, goal.y);
  if (startCell < 0 || goalCell < 0) return false;
  start.y = cells_[startCell].elevation;
  goal.y = cells_[goalCell].elevation;

  if (startCell == goalCell) {
    outPath->push_back(start);
    outPath->push_back(goal);
    // Same lift as the multi-cell exit below: without it a short move whose
    // start and goal share one slab cell would stay at flat-space y = 0 and
    // the figure would walk under the hills.
    LiftGroundPathOntoTerrain(outPath);
    return true;
  }

  // Where a path crosses from cell `a` into `b`: it walks to `approach` in
  // `a`, then continues from `landing` in `b`. Same-elevation borders use the
  // point on the border nearest the straight line from `from` to the goal; climbs walk to the point in `a` nearest `b`'s
  // top, then step straight up (or down) onto the nearest point within `b`.
  auto transition = [&](const NavCell& a, const NavCell& b, glm::vec3 from,
                        glm::vec3* approach, glm::vec3* landing) {
    if (std::fabs(a.elevation - b.elevation) > kEps) {
      const glm::vec3 bCenter = b.Center();
      *approach = glm::vec3(std::clamp(bCenter.x, a.xMin, a.xMax), a.elevation,
                            std::clamp(bCenter.z, a.zMin, a.zMax));
      *landing = glm::vec3(std::clamp(approach->x, b.xMin, b.xMax), b.elevation,
                           std::clamp(approach->z, b.zMin, b.zMax));
    } else {
      const float zLo = std::max(a.zMin, b.zMin);
      const float zHi = std::min(a.zMax, b.zMax);
      const float boundaryX = std::fabs(a.xMax - b.xMin) < kEps ? a.xMax : a.xMin;
      const float dx = goal.x - from.x;
      const float t = std::fabs(dx) > kEps ? std::clamp((boundaryX - from.x) / dx, 0.0f, 1.0f) : 0.0f;
      const float z = std::clamp(from.z + t * (goal.z - from.z), zLo, zHi);
      *approach = *landing = glm::vec3(boundaryX, a.elevation, z);
    }
  };

  // A* over cells, costed by the actual walking distance between the points
  // where the path enters each cell (not cell centers, which badly misjudge
  // large cells and can make crossing a crate look cheaper than walking
  // around it).
  const size_t n = cells_.size();
  std::vector<float> gScore(n, std::numeric_limits<float>::infinity());
  std::vector<int> cameFrom(n, -1);
  std::vector<bool> closed(n, false);
  std::vector<glm::vec3> entry(n, start);

  auto heuristic = [&](int cell) { return glm::distance(entry[cell], goal); };

  using QueueItem = std::pair<float, int>;
  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open;
  gScore[startCell] = 0.0f;
  open.push({heuristic(startCell), startCell});

  bool found = false;
  while (!open.empty()) {
    const auto [f, current] = open.top();
    open.pop();
    (void)f;
    if (closed[current]) continue;
    closed[current] = true;
    if (current == goalCell) {
      found = true;
      break;
    }

    for (int next : neighbors_[current]) {
      if (closed[next]) continue;
      glm::vec3 approach, landing;
      transition(cells_[current], cells_[next], entry[current], &approach, &landing);
      const float tentativeG = gScore[current] + glm::distance(entry[current], approach) +
                               glm::distance(approach, landing);
      if (tentativeG < gScore[next]) {
        gScore[next] = tentativeG;
        cameFrom[next] = current;
        entry[next] = landing;
        open.push({tentativeG + heuristic(next), next});
      }
    }
  }

  if (!found) return false;

  std::vector<int> cellPath;
  for (int node = goalCell; node != -1; node = cameFrom[node]) {
    cellPath.push_back(node);
  }
  std::reverse(cellPath.begin(), cellPath.end());

  // Walk the cell corridor. Each maximal run of same-elevation cells is
  // pulled taut with the funnel algorithm through the shared cell borders;
  // climb transitions are hard waypoints (approach, then landing).
  outPath->push_back(start);
  glm::vec3 from = start;
  std::vector<Portal> portals;
  auto flush = [&](glm::vec3 to) {
    std::vector<glm::vec2> pts;
    Funnel({from.x, from.z}, {to.x, to.z}, portals, &pts);
    for (const auto& pt : pts) {
      const glm::vec3 p(pt.x, from.y, pt.y);
      if (glm::distance(p, outPath->back()) > kEps) outPath->push_back(p);
    }
    if (outPath->size() == 1 || glm::distance(outPath->back(), to) > kEps) {
      outPath->push_back(to);
    }
    outPath->back().y = to.y;
    portals.clear();
  };
  for (size_t i = 0; i + 1 < cellPath.size(); ++i) {
    const NavCell& a = cells_[cellPath[i]];
    const NavCell& b = cells_[cellPath[i + 1]];
    if (std::fabs(a.elevation - b.elevation) > kEps) {
      glm::vec3 approach, landing;
      transition(a, b, from, &approach, &landing);
      flush(approach);
      outPath->push_back(landing);
      from = landing;
    } else {
      const float zLo = std::max(a.zMin, b.zMin);
      const float zHi = std::min(a.zMax, b.zMax);
      const bool movingPosX = std::fabs(a.xMax - b.xMin) < kEps;
      const float boundaryX = movingPosX ? a.xMax : a.xMin;
      // Orientation matches TriArea2's handedness: moving +x, "left" is zHi.
      const glm::vec2 lo(boundaryX, zLo), hi(boundaryX, zHi);
      portals.push_back(movingPosX ? Portal{hi, lo} : Portal{lo, hi});
    }
  }
  flush(goal);

  LiftGroundPathOntoTerrain(outPath);
  return true;
}

bool NavMesh::GroundSegmentWalkable(glm::vec2 a, glm::vec2 b) const {
  if (ground_.Empty()) return true;
  constexpr float kSlopeSample = 0.25f;
  const float length = glm::distance(a, b);
  const int pieces = std::max(1, static_cast<int>(std::ceil(length / kSlopeSample)));
  const float ds = length / static_cast<float>(pieces);
  float prev = ground_.HeightAt(a.x, a.y);
  for (int k = 1; k <= pieces; ++k) {
    const glm::vec2 p = a + (b - a) * (static_cast<float>(k) / static_cast<float>(pieces));
    const float h = ground_.HeightAt(p.x, p.y);
    if (std::fabs(h - prev) > constants::kMaxWalkSlope * ds + 1e-3f) return false;
    prev = h;
  }
  return true;
}

bool NavMesh::GroundSegmentClear(glm::vec2 a, glm::vec2 b) const {
  if (!polygonMode_) {
    return LineOfSightClear(glm::vec3(a.x, 0.0f, a.y), glm::vec3(b.x, 0.0f, b.y), paddedFootprints_);
  }
  for (const auto& polygon : paddedPolygons_) {
    if (SegmentEntersConvexPolygon(a, b, polygon)) return false;
  }
  return true;
}

// Fallback for a planar route that would climb a sheer face: Dijkstra/A* over
// a 0.25 lattice anchored at `start` (the ReachField moves), skipping any move
// onto terrain steeper than the walkable slope, then pulled taut.
bool NavMesh::FindSlopeAwareGroundPath(glm::vec3 start, glm::vec3 goal,
                                       std::vector<glm::vec3>* outPath) const {
  outPath->clear();
  constexpr float kCell = 0.25f;
  constexpr float kMargin = 8.0f;
  constexpr float kGoalReach = 0.6f;
  const glm::vec2 s(start.x, start.z), g(goal.x, goal.z);
  if (!IsWalkable(s.x, s.y) || !IsWalkable(g.x, g.y)) return false;
  const float xMin = std::max(region_.xMin, std::min(s.x, g.x) - kMargin);
  const float xMax = std::min(region_.xMax, std::max(s.x, g.x) + kMargin);
  const float zMin = std::max(region_.zMin, std::min(s.y, g.y) - kMargin);
  const float zMax = std::min(region_.zMax, std::max(s.y, g.y) + kMargin);
  const int ix0 = static_cast<int>(std::ceil((xMin - s.x) / kCell));
  const int ix1 = static_cast<int>(std::floor((xMax - s.x) / kCell));
  const int iz0 = static_cast<int>(std::ceil((zMin - s.y) / kCell));
  const int iz1 = static_cast<int>(std::floor((zMax - s.y) / kCell));
  const int nx = ix1 - ix0 + 1, nz = iz1 - iz0 + 1;
  if (nx <= 0 || nz <= 0) return false;
  const auto indexOf = [&](int ix, int iz) { return (iz - iz0) * nx + (ix - ix0); };
  const auto pointOf = [&](int ix, int iz) { return s + glm::vec2(ix, iz) * kCell; };

  static const int kMoves[16][2] = {{1, 0},  {-1, 0}, {0, 1},  {0, -1}, {1, 1},  {1, -1},
                                    {-1, 1}, {-1, -1}, {2, 1},  {2, -1}, {-2, 1}, {-2, -1},
                                    {1, 2},  {1, -2}, {-1, 2}, {-1, -2}};
  const float inf = std::numeric_limits<float>::infinity();
  std::vector<float> dist(static_cast<size_t>(nx) * nz, inf);
  std::vector<int> parent(dist.size(), -1);
  std::vector<char> walkable(dist.size(), -1);
  using Entry = std::pair<float, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  const int startIdx = indexOf(0, 0);
  dist[startIdx] = 0.0f;
  open.push({glm::distance(s, g), startIdx});
  int endIdx = -1;
  while (!open.empty()) {
    const int idx = open.top().second;
    open.pop();
    const int ix = idx % nx + ix0, iz = idx / nx + iz0;
    const glm::vec2 p = pointOf(ix, iz);
    if (glm::distance(p, g) <= kGoalReach && GroundSegmentClear(p, g) && GroundSegmentWalkable(p, g)) {
      endIdx = idx;
      break;
    }
    for (const auto& m : kMoves) {
      const int jx = ix + m[0], jz = iz + m[1];
      if (jx < ix0 || jx > ix1 || jz < iz0 || jz > iz1) continue;
      const int jdx = indexOf(jx, jz);
      const glm::vec2 q = pointOf(jx, jz);
      const float nd = dist[idx] + glm::distance(p, q);
      if (nd >= dist[jdx]) continue;
      char& w = walkable[jdx];
      if (w < 0) w = IsWalkable(q.x, q.y) ? 1 : 0;
      if (!w || !GroundSegmentWalkable(p, q) || !GroundSegmentClear(p, q)) continue;
      dist[jdx] = nd;
      parent[jdx] = idx;
      open.push({nd + glm::distance(q, g), jdx});
    }
  }
  if (endIdx < 0) return false;

  std::vector<glm::vec2> lattice;
  for (int node = endIdx; node >= 0; node = parent[node]) {
    lattice.push_back(pointOf(node % nx + ix0, node / nx + iz0));
  }
  std::reverse(lattice.begin(), lattice.end());
  lattice.front() = s;
  if (glm::distance(lattice.back(), g) > kEps) lattice.push_back(g);

  // Pull taut: from each kept point, jump to the farthest later point that is
  // still a clear, walkable straight line.
  constexpr size_t kLookahead = 48;
  outPath->push_back(glm::vec3(s.x, 0.0f, s.y));
  size_t i = 0;
  while (i + 1 < lattice.size()) {
    size_t j = std::min(lattice.size() - 1, i + kLookahead);
    while (j > i + 1 && !(GroundSegmentClear(lattice[i], lattice[j]) &&
                          GroundSegmentWalkable(lattice[i], lattice[j]))) {
      --j;
    }
    outPath->push_back(glm::vec3(lattice[j].x, 0.0f, lattice[j].y));
    i = j;
  }
  LiftGroundPathOntoTerrain(outPath);
  return true;
}

bool NavMesh::FindGroundPath(glm::vec3 start, glm::vec3 goal,
                             std::vector<glm::vec3>* outPath) const {
  if (!FindPlanarGroundPath(start, goal, outPath)) return false;
  if (ground_.Empty()) return true;
  bool walkable = true;
  for (size_t i = 0; walkable && i + 1 < outPath->size(); ++i) {
    walkable = GroundSegmentWalkable({(*outPath)[i].x, (*outPath)[i].z},
                                     {(*outPath)[i + 1].x, (*outPath)[i + 1].z});
  }
  if (walkable) return true;
  return FindSlopeAwareGroundPath(start, goal, outPath);
}

// Lift a flat-space ground path onto the terrain: every ground-level waypoint
// (y == 0 in flat space; climb-top waypoints keep their obstacle height)
// gets its Y from the heightfield, and ground segments are densified so
// linear interpolation between waypoints tracks the slope.
void NavMesh::LiftGroundPathOntoTerrain(std::vector<glm::vec3>* outPath) const {
  if (ground_.Empty()) return;
  constexpr float kTerrainPathStep = 1.0f;
  std::vector<glm::vec3> lifted;
  lifted.reserve(outPath->size() * 4);
  const auto isGroundLevel = [](const glm::vec3& p) { return std::fabs(p.y) < kEps; };
  const auto lift = [&](glm::vec3 p) {
    if (isGroundLevel(p)) p.y = ground_.HeightAt(p.x, p.z);
    return p;
  };
  for (size_t i = 0; i < outPath->size(); ++i) {
    const glm::vec3& a = (*outPath)[i];
    lifted.push_back(lift(a));
    if (i + 1 >= outPath->size()) continue;
    const glm::vec3& b = (*outPath)[i + 1];
    if (!isGroundLevel(a) || !isGroundLevel(b)) continue;  // Climb steps stay two points.
    const float len = glm::length(glm::vec2(b.x - a.x, b.z - a.z));
    const int pieces = static_cast<int>(std::ceil(len / kTerrainPathStep));
    for (int k = 1; k < pieces; ++k) {
      const float t = static_cast<float>(k) / static_cast<float>(pieces);
      glm::vec3 p = a + (b - a) * t;
      p.y = ground_.HeightAt(p.x, p.z);
      lifted.push_back(p);
    }
  }
  *outPath = std::move(lifted);
}

}  // namespace tactics
