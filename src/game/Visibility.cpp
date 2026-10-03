#include "game/Visibility.h"

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

bool CanUnitSee(const Unit& viewer, const Unit& target, const std::vector<Obstacle>& obstacles,
                 const std::vector<WalkSurface>& walkSurfaces) {
  return viewer.alive &&
         InFovCone(viewer.EyePosition(), viewer.FacingDirection(), target.EyePosition(),
                   constants::kShootHalfFovDegrees, constants::kShootRange) &&
         LineOfSightClear(viewer.EyePosition(), target.EyePosition(), obstacles, walkSurfaces);
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
    if (IsPointVisibleToTeam(team, target.EyePosition(), units, obstacles)) {
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
    if (IsPointVisibleToTeam(team, target.EyePosition(), units, obstacles)) {
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
    if (IsPointVisibleToTeam(team, target.EyePosition(), units, obstacles, walkSurfaces)) {
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

}  // namespace tactics
