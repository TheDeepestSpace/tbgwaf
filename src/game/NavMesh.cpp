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
  polygonMode_ = false;
  paddedPolygons_.clear();
  paddedPolygons_.reserve(obstacles.size());
  for (const Obstacle& obstacle : obstacles) {
    polygonMode_ |= !obstacle.footprint.empty();
    paddedPolygons_.push_back(ExpandConvexPolygon(ObstacleFootprint(obstacle), agentRadius));
  }
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

ReachField NavMesh::ComputeReachField(const glm::vec3& start, float budget, float step) const {
  ReachField field;
  field.step = step;
  field.budget = budget;
  if (budget <= 0.0f || step <= 0.0f) return field;
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
  for (int iz = 0; iz < field.nz; ++iz) {
    for (int ix = 0; ix < field.nx; ++ix) {
      const float x = field.minX + ix * field.step;
      const float z = field.minZ + iz * field.step;
      field.surfaceY[iz * field.nx + ix] = ground_.HeightAt(x, z);
    }
  }

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
      w = IsWalkable(p.x, p.z) ? 1 : 0;
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
        const float nd = d + glm::distance(from, to);
        const int jdx = jz * field.nx + jx;
        if (nd > budget || nd >= field.dist[jdx]) continue;
        bool clear = true;
        if (polygonMode_) {
          for (const auto& polygon : paddedPolygons_) {
            if (SegmentEntersConvexPolygon({from.x, from.z}, {to.x, to.z}, polygon)) {
              clear = false;
              break;
            }
          }
        } else {
          clear = LineOfSightClear(from, to, paddedFootprints_);
        }
        if (!clear) continue;
        field.dist[jdx] = nd;
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

  // Add the explicit stacked layer without ever connecting it merely because
  // it overlaps ground in XZ. A ground start must first route to a declared
  // ramp foot; a surface start remains on its connected surface component.
  for (int iz = 0; iz < field.nz; ++iz) {
    for (int ix = 0; ix < field.nx; ++ix) {
      const int idx = iz * field.nx + ix;
      const float x = field.minX + ix * field.step;
      const float z = field.minZ + iz * field.step;
      for (size_t surface = 0; surface < walkSurfaces_.size(); ++surface) {
        if (!SurfaceContainsXZ(walkSurfaces_[surface], x, z)) continue;
        glm::vec3 goal(x, SurfaceHeightAt(walkSurfaces_[surface], x, z), z);
        std::vector<glm::vec3> path;
        bool found = false;
        float surfaceBaseCost = 0.0f;
        if (startSurface >= 0) {
          found = FindSurfacePath(startSurface, start, static_cast<int>(surface), goal, &path);
        } else {
          for (size_t entry = 0; entry < walkSurfaces_.size() && !found; ++entry) {
            if (!std::isfinite(entryCost[entry])) continue;
            const glm::vec3 connection = GroundConnectionPoint(static_cast<int>(entry));
            if (!FindSurfacePath(static_cast<int>(entry), connection,
                                 static_cast<int>(surface), goal, &path)) {
              continue;
            }
            surfaceBaseCost = entryCost[entry];
            found = true;
          }
        }
        if (!found) continue;
        float distance = surfaceBaseCost;
        for (size_t i = 0; i + 1 < path.size(); ++i) distance += glm::distance(path[i], path[i + 1]);
        if (distance <= budget && (startSurface >= 0 || distance < field.dist[idx] + 0.5f)) {
          field.dist[idx] = distance;
          field.surfaceY[idx] = goal.y;
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
  for (size_t i = 0; i < walkSurfaces_.size(); ++i) {
    if (!SurfaceContainsXZ(walkSurfaces_[i], x, z)) continue;
    const float dy = std::fabs(SurfaceHeightAt(walkSurfaces_[i], x, z) - yHint);
    if (dy <= bestDy) {
      bestDy = dy;
      best = static_cast<int>(i);
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
  const size_t n = walkSurfaces_.size();
  std::vector<float> distance(n, std::numeric_limits<float>::infinity());
  std::vector<int> parent(n, -1);
  using Entry = std::pair<float, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  distance[startSurface] = 0.0f;
  open.push({0.0f, startSurface});
  while (!open.empty()) {
    const auto [d, current] = open.top();
    open.pop();
    if (d != distance[current]) continue;
    if (current == goalSurface) break;
    for (int next : walkSurfaces_[current].neighbors) {
      if (next < 0 || static_cast<size_t>(next) >= n) continue;
      const float edge = glm::distance(SurfaceCenter(walkSurfaces_[current]),
                                       SurfaceCenter(walkSurfaces_[next]));
      if (d + edge < distance[next]) {
        distance[next] = d + edge;
        parent[next] = current;
        open.push({distance[next], next});
      }
    }
  }
  if (!std::isfinite(distance[goalSurface])) return false;
  std::vector<int> route;
  for (int node = goalSurface; node >= 0; node = parent[node]) route.push_back(node);
  std::reverse(route.begin(), route.end());

  start.y = SurfaceHeightAt(walkSurfaces_[startSurface], start.x, start.z);
  goal.y = SurfaceHeightAt(walkSurfaces_[goalSurface], goal.x, goal.z);
  outPath->push_back(start);
  for (size_t i = 0; i + 1 < route.size(); ++i) {
    const WalkSurface& a = walkSurfaces_[route[i]];
    const WalkSurface& b = walkSurfaces_[route[i + 1]];
    glm::vec3 transition(0.0f);
    int shared = 0;
    for (const glm::vec3& av : a.vertices) {
      for (const glm::vec3& bv : b.vertices) {
        if (glm::distance(av, bv) < 1e-3f) {
          transition += (av + bv) * 0.5f;
          ++shared;
        }
      }
    }
    if (shared) {
      transition /= static_cast<float>(shared);
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
    if (glm::distance(outPath->back(), transition) > kEps) outPath->push_back(transition);
  }
  if (glm::distance(outPath->back(), goal) > kEps) outPath->push_back(goal);
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
    outPath->emplace_back(p.x, ground_.HeightAt(p.x, p.y), p.y);
  }
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

bool NavMesh::FindGroundPath(glm::vec3 start, glm::vec3 goal,
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

  // Lift the flat-space path onto the terrain: every ground-level waypoint
  // (y == 0 in flat space; climb-top waypoints keep their obstacle height)
  // gets its Y from the heightfield, and ground segments are densified so
  // linear interpolation between waypoints tracks the slope.
  if (!ground_.Empty()) {
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
  return true;
}

}  // namespace tactics
