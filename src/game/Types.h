#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
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

// Semantic tag used only to group static render geometry; collision, LOS,
// and navigation treat both kinds identically.
enum class ObstacleKind { Generic, StreetFurniture };

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
  // Small curb-side items share collision/LOS semantics with every other
  // obstacle but are tagged so the renderer can batch their static geometry.
  ObstacleKind kind = ObstacleKind::Generic;
  // Index into Scene::buildings when this prism is (part of) a generated
  // building, else -1 (hand-placed boxes, rocks, pier columns, crates). The
  // renderer reads the building's facade style through it.
  int building = -1;
  // Blocks movement and LOS but is never drawn: the unseen wall along a
  // terrain cliff edge, whose face the terrain mesh itself renders.
  bool navOnly = false;
};

// Building archetype (urban map issue #151, roadmap 3.2). Drawn per parcel
// from a street-class-weighted table with a per-block character bias;
// drives floor count/height (so the building's height) and the facade style.
enum class BuildingKind : uint8_t {
  Residential,  // Apartments: 3.0 floors, brick/plaster.
  MixedUse,     // Shops under apartments: 4.0 ground floor, awning.
  Office,       // 3.5 floors, glassy; upper storeys may step back.
  Commercial,   // Single-storey shop/warehouse hall, awning.
  Civic,        // Landmark: the towers.
  Garage,       // Parking decks: low 2.8 floors, raw concrete.
};
constexpr int kBuildingKindCount = 6;
const char* BuildingKindName(BuildingKind kind);

// One generated building: the gameplay geometry stays the solid Obstacle
// prism(s); this record is the typology beside it.
struct Building {
  BuildingKind kind = BuildingKind::Residential;
  int floorCount = 1;
  float groundFloorHeight = 3.0f;  // First floor; the rest are floorHeight.
  float floorHeight = 3.0f;
  // Facade palette index the renderer reads (kind * variants + variant).
  int style = 0;
  int obstacle = -1;       // Base prism (full parcel footprint).
  int upperObstacle = -1;  // Top floor stepped back as its own prism (offices), else -1.
  int parcel = -1;         // Index into UrbanParcels(seed, config).
  int block = -1;

  float Height() const {
    return groundFloorHeight + static_cast<float>(floorCount - 1) * floorHeight;
  }
};

// Visual-only massing detail (roadmap 3.3): a prism over a convex XZ
// footprint between y0 and y1. Blocks sight lines and FOV (see
// SightObstacles). Parapets and roof fixtures also block walking: they are
// copied into WalkSurface::blockers of their roof slab. Awnings are overhead.
struct MassingDetail {
  enum class Kind : uint8_t { Parapet, RoofFixture, Awning };
  Kind kind = Kind::Parapet;
  std::vector<glm::vec2> footprint;  // Counter-clockwise.
  float y0 = 0.0f, y1 = 0.0f;
  int building = -1;
};

// A window or door on a generated building's facade (issue #152, roadmap
// 3.4). Openings are recorded as data; the renderer builds pane extrusions
// from them (window frames extruded out of the wall, doors and factory
// windows recessed into it), so stage 5 can turn the same records into real
// holes without re-rolling where they are. The opening lies on footprint edge `wallEdge`
// (footprint[wallEdge] -> footprint[wallEdge + 1]) of
// Scene::obstacles[obstacle]: `u0..u1` world units along that edge from its
// first vertex, `y0..y1` world heights. A door whose y0 sits above the wall
// base opens onto a raised stoop or rear platform (a facade WalkSurface
// emitted alongside it).
struct FacadeOpening {
  enum class Kind : uint8_t { Window, Door };
  Kind kind = Kind::Window;
  int building = -1;  // Index into Scene::buildings.
  int obstacle = -1;  // Wall prism the opening sits on.
  int wallEdge = 0;
  float u0 = 0.0f, u1 = 0.0f;
  float y0 = 0.0f, y1 = 0.0f;
  int floor = 0;
};

// A convex, planar walkable patch above (or sloping away from) the ground.
// Patches form a separate navigation layer, so a deck never erases the
// usable ground beneath it. Neighbour indices are explicit: overlapping XZ
// alone must not connect vertically separated surfaces.
struct WalkSurface {
  std::vector<glm::vec3> vertices;  // Counter-clockwise when viewed from above.
  std::vector<int> neighbors;
  bool connectsToGround = false;    // The lowest edge is a legal ground transition.
  // Convex XZ footprints standing on the patch (roof parapets, furniture)
  // that a figure can't walk into. Honoured on single-patch routes only.
  std::vector<std::vector<glm::vec2>> blockers;
};

// A pre-built zipline: a two-way cable between a ground anchor post and a
// rooftop anchor post (ground-to-roof access to a building). `a`/`b` are the
// anchor foot positions (where a rider steps on/off: street level at one end,
// the roof surface at the other, in either order); the cable runs between
// the post tops (see constants::kZiplinePostHeight) and the rider glides
// straight from one foot position to the other (hanging from the cable, see
// constants::kZiplineHangLift). One rider at a time. Riding costs
// ZiplineRideCost() of a figure's move budget.
struct Zipline {
  glm::vec3 a{0.0f};
  glm::vec3 b{0.0f};

  float Length() const { return glm::distance(a, b); }
};

// Move-budget cost of riding `zipline` end to end: proportional to its
// length (constants::kZiplineCostFactor), in the same units as walking.
inline float ZiplineRideCost(const Zipline& zipline);

// Visual road pavement. Elevated/ramp pavement is represented by
// WalkSurface instead so its rendered geometry and gameplay surface are one
// and the same.
struct RoadSurface {
  std::vector<glm::vec3> vertices;
};

// A visual street fixture (issue #150): traffic lights at bigger junctions,
// stop/yield signs at smaller ones, street-name signs at block corners, and
// the curved arm/head of a lamp whose collision post remains an `Obstacle`.
// Fixtures are ignored by collision/navmesh/gameplay LOS, but their rendered
// geometry participates in both lighting shadows and the shadow-map FOV pass.
enum class StreetPropKind { TrafficLight, StopSign, YieldSign, StreetNameSign, LampPost };
struct StreetProp {
  StreetPropKind kind = StreetPropKind::TrafficLight;
  glm::vec3 position{0.0f};  // Foot of the post.
  float yaw = 0.0f;          // Facing/primary street heading about Y (0 = +X).
  // The other incident street's true heading for crossed street-name blades;
  // ignored by the other fixture kinds.
  float secondaryYaw = 0.0f;
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
// y = 0 everywhere (the hand-authored and terrain-disabled urban scenes), so every consumer
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
// Ground beneath a deck/ramp is only walkable with at least this much headroom.
constexpr float kMinDeckHeadroom = 2.2f;
// Steepest ground a figure can walk (rise over run). Stair flights (0.8) and
// road ramps (0.3) pass; the sheer sides of stair, ramp and level blocks do
// not, so an upper level is reachable only by walking up its stair or ramp.
// Equivalent to a 0.3 step over one 0.25 sample.
constexpr float kMaxWalkSlope = 1.2f;
// Where a locked-on shot aims on the target figure: torso center height.
// Shared by the ballistics (burst aim point), the staged-lock "+" marker and
// the planned-shot cone, so what the cone shader shows is what gets shot at.
constexpr float kTorsoAimHeight = 0.9f;
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
// Time into the shot beat at which the draw/raise ends and the muzzle
// fires (the recoil kick starts). Follow-up shots of a burst (issue #140)
// re-trigger the beat from here while it is still playing, so the weapon
// stays shouldered between shots instead of re-drawing every time.
constexpr float kShootRecoilStart = 0.12f;
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
// Enemy ghosts: a figure continuously in FOV leaves one sample per interval
// (plus one on entry) during a turn. Ghosts and bullet tracers show for that
// turn only: once it ends each fades out over kGhostFadeDuration, starting
// at the moment it appeared (same order and spacing). Nothing carries over
// to the next turn.
constexpr float kSightingSampleInterval = 0.5f;
constexpr float kGhostFadeDuration = kSightingSampleInterval;
// Timeline replay has no planning phase to fade in, so ghosts and tracers
// instead live on their own clock and carry over into the next turn: a ghost
// holds kReplayGhostHold after appearing, then fades over kGhostFadeDuration;
// a bullet line fades out over kReplayTracerLife after the shot.
constexpr float kReplayGhostHold = 1.0f;
constexpr float kReplayTracerLife = 1.0f;
// Ziplines: ride cost per world unit of cable, as a fraction of the cost of
// walking the same distance (0.25 = riding is 4x cheaper than running, so a
// 20-unit line costs 5 units of the 20-unit round budget). Longer line =
// more of the turn spent. Tune here; everything (planning, frontier,
// execution speed) derives from it.
constexpr float kZiplineCostFactor = 0.25f;
constexpr float kZiplinePostHeight = 1.7f;  // Short anchor post: cable-end height above its foot position.
// A rider hangs from a trolley on the cable with the feet this far above the
// foot-to-foot line mid-ride; the lift eases in over the first (mount) and
// out over the last (dismount) kZiplineMountDistance of the cable.
constexpr float kZiplineHangLift = 0.2f;
constexpr float kZiplineMountDistance = 1.4f;
// Free-aim shooting (issue #129). Friendly fire is on by default; the single
// flag below (mirrored per GameLogic instance, settable from scenarios) is
// the one switch that disables it, making friendlies transparent to the
// ballistic trace.
constexpr bool kFriendlyFireDefault = true;
// A free-aim trace flies until it hits something, capped at the sight range
// (chance has long since fallen off by then; there is no hard shot range cap).
constexpr float kAimTraceRange = kSightRange;
// Neutral flag objective (CTF part 1). A living figure whose path touches
// the flag -- within kFlagGrabRadius in XZ and kFlagGrabHeight vertically
// (so a deck above/below the flag does not grab it) -- picks it up for free.
constexpr float kFlagGrabRadius = 0.6f;
constexpr float kFlagGrabHeight = 1.2f;
constexpr float kGrabAnimDuration = 0.6f;  // Visual reach/pickup beat on the grabber.
constexpr float kFlagDropDuration = 0.8f;  // Visual drop/plant settle of a dropped flag.
}  // namespace constants

inline float ZiplineRideCost(const Zipline& zipline) {
  return zipline.Length() * constants::kZiplineCostFactor;
}

}  // namespace tactics
