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
#include <glm/gtc/quaternion.hpp>

#include "game/Scene.h"
#include "game/Types.h"

namespace tactics {

enum class WorldChangeKind : int {
  ObstacleRemoved = 0,   // target = obstacle index.
  // target removed; `pieces` appended (the fracture cells left standing).
  // Facade openings laid out on the target stay, detached onto their own
  // world-space edge (FacadeOpening::detached).
  ObstacleReplaced = 1,
  SurfaceRemoved = 2,    // target = walk-surface index (a stair flight brought down whole).
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
  // A hole torn out of walk surface `target` (a building floor). With
  // `pieces`, their footprints are the convex patches left standing (the
  // fracture cells that survived; the target's plane supplies the heights);
  // without, the hole is `rect` = (x0,z0,x1,z1) in XZ and up to four convex
  // patches remain around it. The target is killed and the patches are
  // appended, linked to each other and to the target's old neighbours where
  // they still share a boundary. `rect` always bounds the hole (figures
  // inside it that lost their footing fall). The ceiling slab below is cut
  // by a separate ObstacleReplaced change (see PlanExplosion). The rect form
  // only cuts flat patches; cell pieces may also cut a sloped planar one (a
  // ramp), each piece keeping the plane's heights.
  FloorHole = 6,
  // Scene::massing[target] (a parapet band, roof fixture or awning) broken
  // up: the target detail dies and what is left of it is appended, so the
  // roof-edge wrap-around breaks with the wall below it. With `pieces`,
  // their footprints are the convex parts left standing (fracture cells);
  // without, the convex parts of the footprint outside `rect` =
  // (x0,z0,x1,z1) remain -- none when it lies entirely inside. Details with
  // a sloped `top` (deck rails) keep it on cell pieces (the top polygon
  // clipped to each footprint); the rect form only removes them whole.
  MassingCut = 7,
  // Chunks knocked loose by a blast (destruction: the knocked-out fracture
  // cells detach, tumble and become obstacles). `pieces` are the chunks as
  // they left the wall / slab -- each cell in place, the prism it was cut
  // from. `flights`, one per piece, is the server's rigid-body simulation of
  // that chunk (see DebrisPhysics.h): a pose track clients replay verbatim
  // (DebrisPose in game/Destruction.h), so every client animates the same
  // collisions and ends on exactly the same rest pose, however it came to
  // rest -- tilted, leaning, on top of other chunks. Pieces whose flight is
  // `solid` are appended to the scene as ObstacleKind::Rubble (their vertical
  // envelope at rest) and block movement and sight like any obstacle; the
  // rest are litter, drawn but never simulated.
  Debris = 8,
  // Scene::facadeOpenings[target] (a window frame / door leaf) blown off its
  // wall: the opening dies. Its bars leave as debris chunks.
  OpeningRemoved = 9,
};
constexpr int kWorldChangeKindCount = 10;

// Seconds between a debris chunk's pose samples (20 Hz).
constexpr float kDebrisKeyDt = 1.0f / 20.0f;

// One sample of a chunk's body: where its centre of mass is and how it is
// oriented (unit quaternion, world frame), kDebrisKeyDt apart.
struct DebrisKey {
  glm::vec3 position{0.0f};
  glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};

  bool operator==(const DebrisKey& o) const {
    return position == o.position && rotation == o.rotation;
  }
};

// How one debris chunk moved: the matching `pieces` entry's body sampled at
// a fixed rate from launch (keys[0], its pose in the wall) until it came to
// rest (keys.back(), held from then on). Computed once on the authority, so
// who a chunk hit and where it ended is never up to a client's arithmetic.
struct DebrisFlight {
  std::vector<DebrisKey> keys;
  bool solid = true;     // Becomes a rubble obstacle (else cosmetic litter).

  float Duration() const;
  bool operator==(const DebrisFlight& o) const { return solid == o.solid && keys == o.keys; }
};

struct WorldChange {
  int id = 0;           // Position in the log (0-based, gapless).
  int round = 1;        // Round it happened in.
  float at = 0.0f;      // Seconds into that round's execution (0 between rounds).
  WorldChangeKind kind = WorldChangeKind::ObstacleRemoved;
  int target = -1;
  std::vector<Obstacle> pieces;   // ObstacleReplaced / Debris; FloorHole / MassingCut footprints.
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

// World-space wall edge a facade opening's `u` runs along (a -> b): its own
// record once detached, else footprint edge `wallEdge` of its prism. False
// when the opening has no usable edge (dead prism, bad indices).
bool FacadeOpeningEdge(const Scene& scene, const FacadeOpening& opening, glm::vec2* a,
                       glm::vec2* b);

// Validates `change` against the scene and applies it. Returns false (scene
// untouched) for a change that does not make sense there: bad/dead target,
// degenerate polygon, a height patch on flat ground, ...
bool ApplyWorldChange(Scene* scene, const WorldChange& change);

// Text codec (one change = whitespace-separated tokens), used by the
// snapshot text. Read returns false on malformed input.
void WriteWorldChange(std::ostream& out, const WorldChange& change);
bool ReadWorldChange(std::istream& in, WorldChange* change);

}  // namespace tactics
