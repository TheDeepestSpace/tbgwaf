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
// Obstacle::building, Building::obstacle/upperObstacle/floorSurfaces/
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
  // A hole punched through wall prism `target`, on its footprint edge
  // `wallEdge`: u0..u1 world units along the edge, y0..y1 world heights, and
  // -- when `polygon` is set -- an irregular fracture outline inside that box
  // (vertices (u, y, 0), one y run per u). Applying it really cuts the
  // geometry: the target is killed and the convex pieces of its silhouette
  // around the outline are appended -- jamb prisms beside it plus pieces whose
  // `profile` carries the slanted torn edges -- so LOS, shots and (when the
  // hole reaches the floor with headroom) walking pass through exactly the
  // drawn hole. A Breach FacadeOpening records the outline's bounding box.
  WallHole = 5,
  // A rectangular hole cut out of walk surface `target` (a building floor):
  // `rect` = (x0,z0,x1,z1) in XZ. The target is killed and up to four convex
  // patches around the hole are appended, linked to each other and to the
  // target's old neighbours where they still share a boundary. The ceiling
  // slab below is cut by a separate ObstacleReplaced change (see
  // PlanExplosion in game/Destruction.h).
  FloorHole = 6,
  // Scene::massing[target] (a parapet band, roof fixture or awning) cut
  // around `rect` = (x0,z0,x1,z1) in XZ: the target detail dies and the
  // convex pieces of its footprint outside the rect are appended, so the
  // roof-edge wrap-around breaks with the wall below it. A detail entirely
  // inside the rect is simply removed (no pieces). Details with a sloped
  // `top` (deck rails) are only removed whole, never partially cut.
  MassingCut = 7,
  // Rubble knocked loose by a blast (destruction: chunks detach, fall and
  // become obstacles). `pieces` are the chunks at rest -- plain prisms
  // (ObstacleKind::Rubble) appended to the scene where the server's ballistic
  // simulation landed them, where they block movement and sight like any
  // obstacle. `flights`, one per piece, replays the fall on clients: launch
  // point/yaw, flight time and tumble rate; the arc between launch and the
  // rest pose is the closed-form ballistic the server integrated, so every
  // client animates the same fall and ends exactly on the solid rest pose.
  Debris = 8,
};
constexpr int kWorldChangeKindCount = 9;

// How one debris chunk flew: enough for a client to replay the server's
// ballistic fall of the matching `pieces` entry (which holds the rest pose).
// The replayed tilt about the chunk's long axis is spin * (t/duration - 1):
// it starts mid-tumble and lands exactly flush with the rest pose.
struct DebrisFlight {
  glm::vec3 from{0.0f};  // Chunk center at launch.
  float duration = 0.0f; // Seconds from launch (the change's `at`) to rest.
  float spin = 0.0f;     // Total tumble over the flight, radians (signed).
  float fromYaw = 0.0f;  // Yaw at launch; the rest yaw comes from the piece footprint.

  bool operator==(const DebrisFlight& o) const {
    return from == o.from && duration == o.duration && spin == o.spin && fromYaw == o.fromYaw;
  }
};

struct WorldChange {
  int id = 0;           // Position in the log (0-based, gapless).
  int round = 1;        // Round it happened in.
  float at = 0.0f;      // Seconds into that round's execution (0 between rounds).
  WorldChangeKind kind = WorldChangeKind::ObstacleRemoved;
  int target = -1;
  std::vector<Obstacle> pieces;   // ObstacleReplaced / Debris.
  // SurfaceCut: the replacement patch. WallHole: optional fracture outline in
  // the wall plane, (u, y, 0) tuples forming a u-monotone polygon (one y run
  // per u) in the frame of `wallEdge`; empty keeps the plain u0/u1/y0/y1
  // rect, so pre-fracture logs replay unchanged.
  std::vector<glm::vec3> polygon;
  glm::vec4 rect{0.0f};           // HeightPatch / FloorHole / MassingCut: x0, z0, x1, z1.
  float delta = 0.0f;             // HeightPatch.
  int wallEdge = 0;               // WallHole.
  float u0 = 0.0f, u1 = 0.0f, y0 = 0.0f, y1 = 0.0f;  // WallHole (outline bounding box).
  std::vector<DebrisFlight> flights;  // Debris: one per piece.

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
