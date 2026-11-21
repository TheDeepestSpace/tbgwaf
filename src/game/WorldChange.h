#pragma once

// World-change log (destructible world, part A1).
//
// Every edit to the map after generation is one ordered, timestamped
// WorldChange. The log is the single source of truth for "how does this match's
// geometry differ from the seeded map": the server appends to it, ships it
// (see net/Protocol.h), and every client applies the same changes in the same
// order to the same seeded map, so geometry == state on both sides.
//
// Indices stay stable. A removed obstacle / walk surface is marked `dead` and
// parked inert (see KillObstacle / KillWalkSurface), never erased from its
// vector, so every index holder keeps pointing at the right thing:
// Obstacle::building, Building::obstacle/floorSurfaces/
// stairSurfaces, FacadeOpening::obstacle, MassingDetail::building,
// WalkSurface::neighbors, TerrainTunnel::roofSurface and
// TeamVisibility::visibleObstacle (indexed by obstacle). Replacement pieces
// are appended, so their indices are deterministic given the log order.

#include <iosfwd>
#include <vector>

#include <glm/glm.hpp>

#include "game/Scene.h"
#include "game/Types.h"

namespace tactics {

enum class WorldChangeKind : int {
  ObstacleRemoved = 0,   // target = obstacle index.
  ObstacleReplaced = 1,  // target removed; `pieces` appended (e.g. a rubble stump).
  SurfaceRemoved = 2,    // target = walk-surface index.
  SurfaceCut = 3,        // target's polygon replaced by `polygon` (a smaller convex patch).
  HeightPatch = 4,       // ground samples inside `rect` (x0,z0,x1,z1) raised by `delta`.
  WallHole = 5,          // opening in obstacle `target`'s wall edge `wallEdge`: u0..u1, y0..y1.
};
constexpr int kWorldChangeKindCount = 6;

struct WorldChange {
  int id = 0;           // Position in the log (0-based, gapless).
  int round = 1;        // Round it happened in.
  float at = 0.0f;      // Seconds into that round's execution (0 between rounds).
  WorldChangeKind kind = WorldChangeKind::ObstacleRemoved;
  int target = -1;
  std::vector<Obstacle> pieces;   // ObstacleReplaced.
  std::vector<glm::vec3> polygon; // SurfaceCut.
  glm::vec4 rect{0.0f};           // HeightPatch: x0, z0, x1, z1.
  float delta = 0.0f;             // HeightPatch.
  int wallEdge = 0;               // WallHole.
  float u0 = 0.0f, u1 = 0.0f, y0 = 0.0f, y1 = 0.0f;  // WallHole.

  bool operator==(const WorldChange& o) const;
};

// Marks an obstacle dead and inert (idempotent), and propagates to what hangs
// off it: its facade openings, and -- once a building has no live prism left
// -- the building, its massing and its floor/stair surfaces.
void KillObstacle(Scene* scene, int index);
// Marks a walk surface dead: no vertices, neighbours or ground links, and
// removed from every neighbour list. Idempotent.
void KillWalkSurface(Scene* scene, int index);

// Validates `change` against the scene and applies it. Returns false (scene
// untouched) for a change that does not make sense there: bad/dead target,
// degenerate polygon, a height patch on flat ground, ...
bool ApplyWorldChange(Scene* scene, const WorldChange& change);

// Text codec (one change = whitespace-separated tokens), used by the
// snapshot text. Read returns false on malformed input.
void WriteWorldChange(std::ostream& out, const WorldChange& change);
bool ReadWorldChange(std::istream& in, WorldChange* change);

}  // namespace tactics
