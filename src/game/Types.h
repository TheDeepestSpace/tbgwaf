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
constexpr float kMapHalfExtent = 15.0f;  // Default scene's half-size (~30x30); generated scenes set Scene::mapHalfExtent.
constexpr float kAgentRadius = 0.4f;     // Padding used to inflate obstacles for the navmesh.
constexpr float kUnitHalfWidth = 0.35f;
constexpr float kUnitHeight = 1.8f;
constexpr float kEyeHeight = 1.5f;
// Sighting range: effectively unlimited (exceeds any generated map's diagonal).
// Visibility stays on this so capping the shot range doesn't shrink what a team sees.
constexpr float kSightRange = 250.0f;
// Shot range cap: 3x the 20-unit per-round walking distance (4.0 speed * 5.0 s).
constexpr float kShootRange = 60.0f;
constexpr float kShootHalfFovDegrees = 75.0f;  // 150 degree total FOV cone.
constexpr float kKnockdownDuration = 0.4f;  // Seconds for a hit unit to fall over.
// Visual-only figure animation (procedural humanoid, see gfx/SceneRenderer):
// the walk cycle is driven by distance travelled -- one full stride cycle
// (both legs) per kWalkStrideLength world units, so limbs stay in step with
// the ground at any run speed -- and blends in/out with an exponential
// decay at kWalkBlendRate (same damping form as the camera zoom) so a
// figure stopping mid-stride settles instead of snapping to rest.
constexpr float kWalkStrideLength = 1.5f;
constexpr float kWalkBlendRate = 12.0f;  // Per second; ~90% settled after 0.2 s.
// Duration of the CC0 idle clip retargeted by the procedural figure rig.
constexpr float kIdleAnimDuration = 3.3333333f;
// Quick-draw pistol beat played by a shooter whose shot resolves: a short
// draw/aim raise, then a recoil kick that decays back down. Purely
// presentational; hit resolution itself stays instantaneous.
constexpr float kShootAnimDuration = 0.7f;
constexpr float kMoveSpeed = 4.0f;  // Default run speed, world units per second.
// WEGO rounds: both teams' committed plans execute together over one
// fixed-length window. A figure's plannable move distance is bounded by
// runSpeed * kRoundDuration, so every move animation fits in the window.
constexpr float kRoundDuration = 5.0f;  // Seconds of execution per round.
// Visual length of the rendered FOV cone overlay. Sized off kSightRange
// (bigger than any map's diagonal) so the cone reaches the map edge no matter
// where a unit stands or faces; the renderer clips each ray at the map
// boundary.
constexpr float kFovConeVisualRange = kSightRange;
// Enemy sighting memory: a figure continuously in FOV leaves one sample per
// interval (plus one on entry). Samples fade per completed round (not in
// real time) and are forgotten once fully faded.
constexpr float kSightingSampleInterval = 0.5f;
constexpr float kSightingFadePerRound = 0.1f;  // Fraction of opacity lost each round.
constexpr int kSightingMemoryRounds = 10;      // 1 / kSightingFadePerRound.
}  // namespace constants

}  // namespace tactics
