#pragma once

#include <algorithm>
#include <cmath>
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
  // Optional convex XZ footprint in counter-clockwise order. Empty keeps the
  // historical rectangular `bounds` footprint. `bounds` remains the broad
  // phase and vertical extent; collision, LOS, navigation and rendering use
  // these vertices when present.
  std::vector<glm::vec2> footprint;
};

// A convex, planar walkable patch above (or sloping away from) the ground.
// Patches form a separate navigation layer, so a deck never erases the
// usable ground beneath it. Neighbour indices are explicit: overlapping XZ
// alone must not connect vertically separated surfaces.
struct WalkSurface {
  std::vector<glm::vec3> vertices;  // Counter-clockwise when viewed from above.
  std::vector<int> neighbors;
  bool connectsToGround = false;    // The lowest edge is a legal ground transition.
};

// Visual road pavement. Elevated/ramp pavement is represented by
// WalkSurface instead so its rendered geometry and gameplay surface are one
// and the same.
struct RoadSurface {
  std::vector<glm::vec3> vertices;
};

inline std::vector<glm::vec2> ObstacleFootprint(const Obstacle& obstacle) {
  if (!obstacle.footprint.empty()) return obstacle.footprint;
  return {{obstacle.bounds.min.x, obstacle.bounds.min.z},
          {obstacle.bounds.max.x, obstacle.bounds.min.z},
          {obstacle.bounds.max.x, obstacle.bounds.max.z},
          {obstacle.bounds.min.x, obstacle.bounds.max.z}};
}

// Sampled ground elevation over a regular XZ grid: sample (ix, iz) sits at
// (minX + ix*step, minZ + iz*step). An empty field means flat ground at
// y = 0 everywhere (the hand-authored and urban scenes), so every consumer
// can sample unconditionally. Queries outside the grid clamp to the border.
struct HeightField {
  float minX = 0.0f, minZ = 0.0f;
  float step = 1.0f;
  int nx = 0, nz = 0;  // Samples (grid vertices) per axis.
  std::vector<float> heights;  // nz rows of nx samples.

  bool Empty() const { return heights.empty(); }

  float At(int ix, int iz) const {
    ix = ix < 0 ? 0 : (ix >= nx ? nx - 1 : ix);
    iz = iz < 0 ? 0 : (iz >= nz ? nz - 1 : iz);
    return heights[static_cast<size_t>(iz) * nx + ix];
  }

  // Bilinear ground height at an arbitrary XZ point; 0 when Empty().
  float HeightAt(float x, float z) const {
    if (Empty()) return 0.0f;
    const float fx = (x - minX) / step;
    const float fz = (z - minZ) / step;
    const int ix = static_cast<int>(std::floor(fx));
    const int iz = static_cast<int>(std::floor(fz));
    const float tx = fx - std::floor(fx);
    const float tz = fz - std::floor(fz);
    const float h00 = At(ix, iz), h10 = At(ix + 1, iz);
    const float h01 = At(ix, iz + 1), h11 = At(ix + 1, iz + 1);
    const float h0 = h00 + (h10 - h00) * tx;
    const float h1 = h01 + (h11 - h01) * tx;
    return h0 + (h1 - h0) * tz;
  }

  // Height of the actual rendered terrain triangles at (x,z). The renderer
  // splits every cell along its min/min -> max/max diagonal; this differs
  // from bilinear HeightAt inside a non-planar cell. Visibility uses this
  // form so its terrain occlusion agrees with the shadow-map FOV mask.
  float MeshHeightAt(float x, float z) const {
    if (Empty()) return 0.0f;
    if (nx < 2 || nz < 2 || step <= 0.0f) return At(0, 0);
    const float fx = std::clamp((x - minX) / step, 0.0f, static_cast<float>(nx - 1));
    const float fz = std::clamp((z - minZ) / step, 0.0f, static_cast<float>(nz - 1));
    const int ix = std::min(static_cast<int>(std::floor(fx)), nx - 2);
    const int iz = std::min(static_cast<int>(std::floor(fz)), nz - 2);
    const float tx = fx - ix;
    const float tz = fz - iz;
    const float a = At(ix, iz);
    if (tx >= tz) {
      const float b = At(ix + 1, iz);
      const float d = At(ix + 1, iz + 1);
      return a + (b - a) * tx + (d - b) * tz;
    }
    const float c = At(ix, iz + 1);
    const float d = At(ix + 1, iz + 1);
    return a + (d - c) * tx + (c - a) * tz;
  }
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
// Gun tip while aiming (figure-local: forward / up / right of the feet): the
// right arm and pistol extended level from the shoulder. Shots and the shot
// cone start here, not at the head.
constexpr float kMuzzleForward = 0.85f;
constexpr float kMuzzleHeight = 0.93f;
constexpr float kMuzzleSide = 0.30f;
constexpr float kConeStartAlpha = 0.5f;  // Shot-cone opacity at the gun tip; fades to 0 at kShootRange.
// Half-angle of the drawn shot-dispersion cone: a narrow wedge at the gun tip
// showing where a shot may stray from the aim line. Overlay only; the hard
// shot gate stays kShootHalfFovDegrees.
constexpr float kShotConeHalfAngleDegrees = 3.0f;
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
constexpr float kSightingFadePerRound = 1.0f / 3.0f;  // Fraction of opacity lost each round.
constexpr int kSightingMemoryRounds = 3;       // 1 / kSightingFadePerRound.
// Free-aim shooting (issue #129). Friendly fire is on by default; the single
// flag below (mirrored per GameLogic instance, settable from scenarios) is
// the one switch that disables it, making friendlies transparent to the
// ballistic trace.
constexpr bool kFriendlyFireDefault = true;
// A free-aim trace flies until it hits something, capped at the sight range
// (chance has long since fallen off by then; there is no hard shot range cap).
constexpr float kAimTraceRange = kSightRange;
}  // namespace constants

}  // namespace tactics
