#pragma once

#include <vector>

#include <glm/glm.hpp>

namespace tactics {

enum class Team { Blue, Red };

inline Team OpposingTeam(Team team) {
  return team == Team::Blue ? Team::Red : Team::Blue;
}

// Axis-aligned bounding box in world space (Y is up).
struct AABB {
  glm::vec3 min{0.0f};
  glm::vec3 max{0.0f};

  glm::vec3 Center() const { return (min + max) * 0.5f; }
  glm::vec3 HalfExtents() const { return (max - min) * 0.5f; }
};

// A map obstacle. `climbable` obstacles get a navmesh connection from
// ground level up to their top surface (see NavMesh); non-climbable
// obstacles are simply impassable walls.
struct Obstacle {
  AABB bounds;
  bool climbable = false;
};

inline std::vector<AABB> ObstacleBounds(const std::vector<Obstacle>& obstacles) {
  std::vector<AABB> result;
  result.reserve(obstacles.size());
  for (const auto& obstacle : obstacles) result.push_back(obstacle.bounds);
  return result;
}

// Map/world tuning constants shared across gameplay systems.
namespace constants {
constexpr float kMapHalfExtent = 10.0f;  // ~20x20 playable area centered on origin.
constexpr float kAgentRadius = 0.4f;     // Padding used to inflate obstacles for the navmesh.
constexpr float kUnitHalfWidth = 0.35f;
constexpr float kUnitHeight = 1.8f;
constexpr float kEyeHeight = 1.5f;
constexpr float kShootRange = 30.0f;          // Effectively unlimited within the map.
constexpr float kShootHalfFovDegrees = 75.0f;  // 150 degree total FOV cone.
constexpr float kMoveSpeed = 4.0f;  // World units per second for move animation.
// Visual length of the rendered FOV cone overlay. Sized off kShootRange
// (already bigger than the map diagonal, 2*kMapHalfExtent*sqrt(2) ~= 28.3)
// so the cone visually reaches the map edge no matter where a unit stands
// or faces; it's fine for the cone to overshoot past the boundary.
constexpr float kFovConeVisualRange = kShootRange;
}  // namespace constants

}  // namespace tactics
