#pragma once

#include <cstddef>
#include <vector>

#include <glm/glm.hpp>

#include "game/Types.h"
#include "game/Unit.h"

namespace tactics {

// Aggregated per-team fog-of-war result: which enemy figures and which
// obstacles are currently inside the combined field of view of at least one
// living figure on a given team. Indices/ids follow Scene::units and
// Scene::obstacles.
struct TeamVisibility {
  std::vector<bool> visibleUnit;      // Indexed by Unit::id.
  std::vector<bool> visibleObstacle;  // Indexed by position in Scene::obstacles.

  bool UnitVisible(int unitId) const {
    return unitId >= 0 && static_cast<size_t>(unitId) < visibleUnit.size() &&
           visibleUnit[unitId];
  }
  bool ObstacleVisible(size_t obstacleIndex) const {
    return obstacleIndex < visibleObstacle.size() && visibleObstacle[obstacleIndex];
  }
};

// True if `point` lies inside at least one living `team` figure's FOV cone
// with clear line of sight, reusing Stage-A's InFovCone/LineOfSightClear
// raycast helpers and the same cone angle as the shoot action (range is the
// longer kSightRange, so a target can be seen beyond the shot range cap). Enemy
// visibility calls this with the surface under the target's feet, matching
// the shadow-map overlay's zero-height probe; shots still aim at the eye.
bool IsPointVisibleToTeam(Team team, const glm::vec3& point, const std::vector<Unit>& units,
                           const std::vector<AABB>& obstacles);
bool IsPointVisibleToTeam(Team team, const glm::vec3& point, const std::vector<Unit>& units,
                          const std::vector<Obstacle>& obstacles);
bool IsPointVisibleToTeam(Team team, const glm::vec3& point, const std::vector<Unit>& units,
                          const std::vector<Obstacle>& obstacles,
                          const std::vector<WalkSurface>& walkSurfaces);
bool IsPointVisibleToTeam(Team team, const glm::vec3& point, const std::vector<Unit>& units,
                          const std::vector<Obstacle>& obstacles,
                          const std::vector<WalkSurface>& walkSurfaces,
                          const HeightField& terrain);

// True if `viewer` (alive) has the surface under `target` inside its FOV cone
// with clear line of sight, matching the shader's zero-height ground probe.
// Swap the arguments to ask "can the enemy see me back".
bool CanUnitSee(const Unit& viewer, const Unit& target, const std::vector<Obstacle>& obstacles,
                const std::vector<WalkSurface>& walkSurfaces);
bool CanUnitSee(const Unit& viewer, const Unit& target, const std::vector<Obstacle>& obstacles,
                const std::vector<WalkSurface>& walkSurfaces, const HeightField& terrain);

// Computes the full per-team visibility set: which enemy figures (downed included) and
// which obstacles are currently inside the combined FOV of `team`'s living
// figures.
TeamVisibility ComputeTeamVisibility(Team team, const std::vector<Unit>& units,
                                      const std::vector<AABB>& obstacles);
TeamVisibility ComputeTeamVisibility(Team team, const std::vector<Unit>& units,
                                     const std::vector<Obstacle>& obstacles);
TeamVisibility ComputeTeamVisibility(Team team, const std::vector<Unit>& units,
                                     const std::vector<Obstacle>& obstacles,
                                     const std::vector<WalkSurface>& walkSurfaces);
TeamVisibility ComputeTeamVisibility(Team team, const std::vector<Unit>& units,
                                     const std::vector<Obstacle>& obstacles,
                                     const std::vector<WalkSurface>& walkSurfaces,
                                     const HeightField& terrain);

}  // namespace tactics
