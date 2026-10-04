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
// longer kSightRange, so a target can be seen beyond the shot range cap).
// Enemy visibility calls this with each of a figure's two sighting probes
// (see CanUnitSee); shots still aim at the eye.
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

// True if `viewer` (alive) has `target` inside its FOV cone with clear line
// of sight to either of two probes: the surface under the target's feet (the
// shadow-map overlay's zero-height ground probe, so a figure standing on
// tinted ground is always seen, and terrain/deck occlusion of that ground
// counts) or the target's eye (so a figure whose body shows above a deck
// edge, low cover or a terrain crest is seen even though the surface under
// it is hidden -- from below, a deck top is hidden by the deck's own slab).
// Shots trace to the eye, so a figure a shot could land on is always seen.
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
