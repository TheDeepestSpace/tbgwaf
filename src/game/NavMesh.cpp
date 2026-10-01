#include "game/NavMesh.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <utility>

#include "game/Raycast.h"

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

void NavMesh::Build(const std::vector<AABB>& obstacles, float mapHalfExtent, float agentRadius) {
  BuildGroundMesh(obstacles, mapHalfExtent, agentRadius);
}

void NavMesh::Build(const std::vector<Obstacle>& obstacles, float mapHalfExtent,
                     float agentRadius) {
  BuildGroundMesh(ObstacleBounds(obstacles), mapHalfExtent, agentRadius);
  AddClimbConnections(obstacles, agentRadius);
}

void NavMesh::BuildGroundMesh(const std::vector<AABB>& obstacles, float mapHalfExtent,
                               float agentRadius) {
  cells_.clear();
  neighbors_.clear();
  paddedFootprints_.clear();
  mapHalfExtent_ = mapHalfExtent;

  const float mapMin = -mapHalfExtent;
  const float mapMax = mapHalfExtent;

  struct Footprint {
    float xMin, xMax, zMin, zMax;
  };
  std::vector<Footprint> footprints;
  footprints.reserve(obstacles.size());
  for (const auto& obstacle : obstacles) {
    Footprint fp;
    fp.xMin = std::max(mapMin, obstacle.min.x - agentRadius);
    fp.xMax = std::min(mapMax, obstacle.max.x + agentRadius);
    fp.zMin = std::max(mapMin, obstacle.min.z - agentRadius);
    fp.zMax = std::min(mapMax, obstacle.max.z + agentRadius);
    if (fp.xMax > fp.xMin + kEps && fp.zMax > fp.zMin + kEps) {
      footprints.push_back(fp);
      // Give the padded footprint a generous Y range so it can be reused
      // directly with the 3D LineOfSightClear ray/AABB test below.
      paddedFootprints_.push_back(
          AABB{glm::vec3(fp.xMin, -1.0f, fp.zMin), glm::vec3(fp.xMax, 1000.0f, fp.zMax)});
    }
  }

  std::vector<float> xs = {mapMin, mapMax};
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

    float cursor = mapMin;
    for (const auto& iv : merged) {
      if (iv.lo - cursor > kEps) {
        cellsByStrip[i].push_back(static_cast<int>(cells_.size()));
        cells_.push_back(NavCell{stripXMin, stripXMax, cursor, iv.lo, 0.0f});
      }
      cursor = std::max(cursor, iv.hi);
    }
    if (mapMax - cursor > kEps) {
      cellsByStrip[i].push_back(static_cast<int>(cells_.size()));
      cells_.push_back(NavCell{stripXMin, stripXMax, cursor, mapMax, 0.0f});
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

    const float xMin = bounds.min.x + kClimbTopInset;
    const float xMax = bounds.max.x - kClimbTopInset;
    const float zMin = bounds.min.z + kClimbTopInset;
    const float zMax = bounds.max.z - kClimbTopInset;
    if (xMax <= xMin + kEps || zMax <= zMin + kEps) continue;  // Too small to stand on.

    const int topIdx = static_cast<int>(cells_.size());
    cells_.push_back(NavCell{xMin, xMax, zMin, zMax, bounds.max.y});
    neighbors_.emplace_back();

    const AABB padded{
        glm::vec3(std::max(-mapHalfExtent_, bounds.min.x - agentRadius), bounds.min.y,
                   std::max(-mapHalfExtent_, bounds.min.z - agentRadius)),
        glm::vec3(std::min(mapHalfExtent_, bounds.max.x + agentRadius), bounds.min.y,
                   std::min(mapHalfExtent_, bounds.max.z + agentRadius))};

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
  if (budget <= 0.0f || step <= 0.0f || !IsWalkable(start.x, start.z)) return field;

  // Align the grid so the source is exactly a node.
  const int radius = static_cast<int>(std::ceil(budget / step));
  field.minX = start.x - radius * step;
  field.minZ = start.z - radius * step;
  field.nx = field.nz = 2 * radius + 1;
  const float inf = std::numeric_limits<float>::infinity();
  field.dist.assign(static_cast<size_t>(field.nx) * field.nz, inf);

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
    const glm::vec3 from = field.Node(ix, iz);
    for (const auto& m : kMoves) {
      const int jx = ix + m[0], jz = iz + m[1];
      if (jx < 0 || jz < 0 || jx >= field.nx || jz >= field.nz) continue;
      if (!nodeWalkable(jx, jz)) continue;
      const glm::vec3 to = field.Node(jx, jz);
      const float nd = d + glm::distance(from, to);
      const int jdx = jz * field.nx + jx;
      if (nd > budget || nd >= field.dist[jdx]) continue;
      if (!LineOfSightClear(from, to, paddedFootprints_)) continue;
      field.dist[jdx] = nd;
      open.push({nd, jdx});
    }
  }
  return field;
}

bool NavMesh::IsWalkable(float x, float z) const { return FindCellContaining(x, z) >= 0; }

int NavMesh::FindCellContaining(float x, float z) const { return FindCellContaining(x, z, 0.0f); }

int NavMesh::FindCellContaining(float x, float z, float yHint) const {
  int best = -1;
  float bestDy = std::numeric_limits<float>::infinity();
  for (size_t i = 0; i < cells_.size(); ++i) {
    if (!cells_[i].Contains(x, z)) continue;
    const float dy = std::fabs(cells_[i].elevation - yHint);
    if (dy < bestDy) {
      bestDy = dy;
      best = static_cast<int>(i);
    }
  }
  return best;
}

bool NavMesh::FindPath(glm::vec3 start, glm::vec3 goal, std::vector<glm::vec3>* outPath) const {
  if (!outPath) return false;
  outPath->clear();

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
  return true;
}

}  // namespace tactics
