#include "game/Visibility.h"

#include <cmath>

#include "game/Geometry.h"
#include "game/Raycast.h"

namespace tactics {

bool IsPointVisibleToTeam(Team team, const glm::vec3& point, const std::vector<Unit>& units,
                           const std::vector<AABB>& obstacles) {
  for (const auto& viewer : units) {
    if (!viewer.alive || viewer.team != team) continue;
    if (InFovCone(viewer.EyePosition(), viewer.FacingDirection(), point,
                  constants::kShootHalfFovDegrees, constants::kShootRange) &&
        LineOfSightClear(viewer.EyePosition(), point, obstacles)) {
      return true;
    }
  }
  return false;
}

bool IsPointVisibleToTeam(Team team, const glm::vec3& point, const std::vector<Unit>& units,
                          const std::vector<Obstacle>& obstacles) {
  for (const auto& viewer : units) {
    if (!viewer.alive || viewer.team != team) continue;
    if (InFovCone(viewer.EyePosition(), viewer.FacingDirection(), point,
                  constants::kShootHalfFovDegrees, constants::kShootRange) &&
        LineOfSightClear(viewer.EyePosition(), point, obstacles)) {
      return true;
    }
  }
  return false;
}

bool IsPointVisibleToTeam(Team team, const glm::vec3& point, const std::vector<Unit>& units,
                          const std::vector<Obstacle>& obstacles,
                          const std::vector<WalkSurface>& walkSurfaces) {
  for (const auto& viewer : units) {
    if (!viewer.alive || viewer.team != team) continue;
    if (InFovCone(viewer.EyePosition(), viewer.FacingDirection(), point,
                  constants::kShootHalfFovDegrees, constants::kShootRange) &&
        LineOfSightClear(viewer.EyePosition(), point, obstacles, walkSurfaces)) {
      return true;
    }
  }
  return false;
}

bool IsPointVisibleToTeam(Team team, const glm::vec3& point, const std::vector<Unit>& units,
                          const std::vector<Obstacle>& obstacles,
                          const std::vector<WalkSurface>& walkSurfaces,
                          const HeightField& terrain) {
  for (const auto& viewer : units) {
    if (!viewer.alive || viewer.team != team) continue;
    if (InFovCone(viewer.EyePosition(), viewer.FacingDirection(), point,
                  constants::kShootHalfFovDegrees, constants::kShootRange) &&
        LineOfSightClear(viewer.EyePosition(), point, obstacles, walkSurfaces, terrain)) {
      return true;
    }
  }
  return false;
}

namespace {

// The shadow-map overlay's default probe height is zero: it answers whether
// the surface under a figure is in FOV, not whether the figure's eye is.
// Ground figures are adjusted from the bilinear navigation height to the
// renderer's triangulated height so the CPU test samples the same surface.
glm::vec3 UnitFovProbe(const Unit& target, const std::vector<WalkSurface>& walkSurfaces,
                       const HeightField& terrain) {
  glm::vec3 probe = target.position;
  for (const WalkSurface& surface : walkSurfaces) {
    if (SurfaceContainsXZ(surface, probe.x, probe.z) &&
        std::fabs(SurfaceHeightAt(surface, probe.x, probe.z) - probe.y) < 0.1f) {
      return probe;
    }
  }
  if (std::fabs(terrain.HeightAt(probe.x, probe.z) - probe.y) < 0.25f) {
    probe.y = terrain.MeshHeightAt(probe.x, probe.z);
  }
  return probe;
}

}  // namespace

bool CanUnitSee(const Unit& viewer, const Unit& target, const std::vector<Obstacle>& obstacles,
                 const std::vector<WalkSurface>& walkSurfaces) {
  return viewer.alive &&
         InFovCone(viewer.EyePosition(), viewer.FacingDirection(), target.position,
                   constants::kShootHalfFovDegrees, constants::kShootRange) &&
         LineOfSightClear(viewer.EyePosition(), target.position, obstacles, walkSurfaces);
}

bool CanUnitSee(const Unit& viewer, const Unit& target, const std::vector<Obstacle>& obstacles,
                const std::vector<WalkSurface>& walkSurfaces, const HeightField& terrain) {
  const glm::vec3 probe = UnitFovProbe(target, walkSurfaces, terrain);
  return viewer.alive &&
         InFovCone(viewer.EyePosition(), viewer.FacingDirection(), probe,
                   constants::kShootHalfFovDegrees, constants::kShootRange) &&
         LineOfSightClear(viewer.EyePosition(), probe, obstacles, walkSurfaces, terrain);
}

namespace {

// Obstacles are boxes, not points. Approximate "some part of this obstacle
// is visible" by sampling its footprint center plus its four top corners
// against the FOV/LOS check above. Good enough for v1 fog-of-war dimming
// without a full box-vs-cone visibility test.
void CollectObstacleSamplePoints(const AABB& box, std::vector<glm::vec3>* outPoints) {
  const float y = box.max.y;
  outPoints->push_back(box.Center());
  outPoints->push_back(glm::vec3(box.min.x, y, box.min.z));
  outPoints->push_back(glm::vec3(box.min.x, y, box.max.z));
  outPoints->push_back(glm::vec3(box.max.x, y, box.min.z));
  outPoints->push_back(glm::vec3(box.max.x, y, box.max.z));
}

}  // namespace

TeamVisibility ComputeTeamVisibility(Team team, const std::vector<Unit>& units,
                                      const std::vector<AABB>& obstacles) {
  TeamVisibility result;
  result.visibleUnit.resize(units.size(), false);
  result.visibleObstacle.resize(obstacles.size(), false);

  for (const auto& target : units) {
    if (target.team == team) continue;  // Downed enemies stay visible while in FOV.
    if (IsPointVisibleToTeam(team, target.position, units, obstacles)) {
      result.visibleUnit[target.id] = true;
    }
  }

  std::vector<glm::vec3> samples;
  for (size_t i = 0; i < obstacles.size(); ++i) {
    samples.clear();
    CollectObstacleSamplePoints(obstacles[i], &samples);
    for (const auto& point : samples) {
      if (IsPointVisibleToTeam(team, point, units, obstacles)) {
        result.visibleObstacle[i] = true;
        break;
      }
    }
  }

  return result;
}

TeamVisibility ComputeTeamVisibility(Team team, const std::vector<Unit>& units,
                                     const std::vector<Obstacle>& obstacles) {
  TeamVisibility result;
  result.visibleUnit.resize(units.size(), false);
  result.visibleObstacle.resize(obstacles.size(), false);
  for (const Unit& target : units) {
    if (target.team == team) continue;
    if (IsPointVisibleToTeam(team, target.position, units, obstacles)) {
      result.visibleUnit[target.id] = true;
    }
  }
  std::vector<glm::vec3> samples;
  for (size_t i = 0; i < obstacles.size(); ++i) {
    samples.clear();
    const Obstacle& obstacle = obstacles[i];
    samples.push_back(obstacle.bounds.Center());
    for (const glm::vec2& p : ObstacleFootprint(obstacle)) {
      samples.emplace_back(p.x, obstacle.bounds.max.y, p.y);
    }
    for (const glm::vec3& point : samples) {
      if (IsPointVisibleToTeam(team, point, units, obstacles)) {
        result.visibleObstacle[i] = true;
        break;
      }
    }
  }
  return result;
}

TeamVisibility ComputeTeamVisibility(Team team, const std::vector<Unit>& units,
                                     const std::vector<Obstacle>& obstacles,
                                     const std::vector<WalkSurface>& walkSurfaces) {
  TeamVisibility result;
  result.visibleUnit.resize(units.size(), false);
  result.visibleObstacle.resize(obstacles.size(), false);
  for (const Unit& target : units) {
    if (target.team == team) continue;
    if (IsPointVisibleToTeam(team, target.position, units, obstacles, walkSurfaces)) {
      result.visibleUnit[target.id] = true;
    }
  }
  std::vector<glm::vec3> samples;
  for (size_t i = 0; i < obstacles.size(); ++i) {
    samples.clear();
    const Obstacle& obstacle = obstacles[i];
    samples.push_back(obstacle.bounds.Center());
    for (const glm::vec2& p : ObstacleFootprint(obstacle)) {
      samples.emplace_back(p.x, obstacle.bounds.max.y, p.y);
    }
    for (const glm::vec3& point : samples) {
      if (IsPointVisibleToTeam(team, point, units, obstacles, walkSurfaces)) {
        result.visibleObstacle[i] = true;
        break;
      }
    }
  }
  return result;
}

TeamVisibility ComputeTeamVisibility(Team team, const std::vector<Unit>& units,
                                     const std::vector<Obstacle>& obstacles,
                                     const std::vector<WalkSurface>& walkSurfaces,
                                     const HeightField& terrain) {
  TeamVisibility result;
  result.visibleUnit.resize(units.size(), false);
  result.visibleObstacle.resize(obstacles.size(), false);
  for (const Unit& target : units) {
    if (target.team == team) continue;
    const glm::vec3 probe = UnitFovProbe(target, walkSurfaces, terrain);
    if (IsPointVisibleToTeam(team, probe, units, obstacles, walkSurfaces, terrain)) {
      result.visibleUnit[target.id] = true;
    }
  }
  std::vector<glm::vec3> samples;
  for (size_t i = 0; i < obstacles.size(); ++i) {
    samples.clear();
    const Obstacle& obstacle = obstacles[i];
    samples.push_back(obstacle.bounds.Center());
    for (const glm::vec2& p : ObstacleFootprint(obstacle)) {
      samples.emplace_back(p.x, obstacle.bounds.max.y, p.y);
    }
    for (const glm::vec3& point : samples) {
      if (IsPointVisibleToTeam(team, point, units, obstacles, walkSurfaces, terrain)) {
        result.visibleObstacle[i] = true;
        break;
      }
    }
  }
  return result;
}

}  // namespace tactics
