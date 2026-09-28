#include "game/NavMesh.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

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

// Greedily skips to the furthest waypoint reachable with a clear line of
// sight, pulling the path taut while staying outside every obstacle.
std::vector<glm::vec3> PullTaut(const std::vector<glm::vec3>& waypoints,
                                 const std::vector<AABB>& obstacles) {
  if (waypoints.size() <= 2) return waypoints;

  std::vector<glm::vec3> result;
  result.push_back(waypoints.front());
  size_t anchor = 0;
  while (anchor + 1 < waypoints.size()) {
    size_t next = anchor + 1;
    for (size_t candidate = waypoints.size() - 1; candidate > anchor; --candidate) {
      if (LineOfSightClear(waypoints[anchor], waypoints[candidate], obstacles)) {
        next = candidate;
        break;
      }
    }
    result.push_back(waypoints[next]);
    anchor = next;
  }
  return result;
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

  const size_t n = cells_.size();
  std::vector<float> gScore(n, std::numeric_limits<float>::infinity());
  std::vector<int> cameFrom(n, -1);
  std::vector<bool> closed(n, false);

  auto heuristic = [&](int cell) {
    return glm::distance(cells_[cell].Center(), cells_[goalCell].Center());
  };

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
      const float tentativeG =
          gScore[current] + glm::distance(cells_[current].Center(), cells_[next].Center());
      if (tentativeG < gScore[next]) {
        gScore[next] = tentativeG;
        cameFrom[next] = current;
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

  std::vector<glm::vec3> waypoints;
  waypoints.push_back(start);
  for (size_t i = 0; i + 1 < cellPath.size(); ++i) {
    const NavCell& a = cells_[cellPath[i]];
    const NavCell& b = cells_[cellPath[i + 1]];
    if (std::fabs(a.elevation - b.elevation) > kEps) {
      // Climb transition: walk to the point in `a` nearest `b`'s top, then
      // step straight up (or down) onto the nearest point within `b`.
      const glm::vec3 bCenter = b.Center();
      const glm::vec3 approach(std::clamp(bCenter.x, a.xMin, a.xMax), a.elevation,
                                std::clamp(bCenter.z, a.zMin, a.zMax));
      const glm::vec3 landing(std::clamp(approach.x, b.xMin, b.xMax), b.elevation,
                               std::clamp(approach.z, b.zMin, b.zMax));
      waypoints.push_back(approach);
      waypoints.push_back(landing);
    } else {
      const float zLo = std::max(a.zMin, b.zMin);
      const float zHi = std::min(a.zMax, b.zMax);
      const float boundaryX = std::fabs(a.xMax - b.xMin) < kEps ? a.xMax : a.xMin;
      waypoints.push_back(glm::vec3(boundaryX, a.elevation, (zLo + zHi) * 0.5f));
    }
  }
  waypoints.push_back(goal);

  *outPath = PullTaut(waypoints, paddedFootprints_);
  return true;
}

}  // namespace tactics
