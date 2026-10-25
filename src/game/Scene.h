#pragma once

#include <optional>
#include <vector>

#include "game/Types.h"
#include "game/Unit.h"

namespace tactics {

// A stair up one terrain level wall (issue #155). `bottom` and `top` are
// ground points on the low/high side. A through-wall stair cuts a walled
// channel whose ground carries the flight; an along-wall stair hangs off
// the wall face as explicit WalkSurfaces, climbing parallel to it, and
// turns onto the level over the wall top.
struct TerrainAccess {
  glm::vec3 bottom, top;
  float width = 4.0f;
  bool alongWall = false;
};

// One map-spanning terrain step: ground on the `raisedNormal` side of the
// step line through `origin` along `dir` sits `rise` higher, behind a
// retaining wall that runs parallel to its boundary street. Steps overlap
// into nested tiers; highway ramps and the accesses climb them.
struct TerrainWall {
  glm::vec2 origin{0.0f}, dir{1.0f, 0.0f};
  glm::vec2 raisedNormal{0.0f, 1.0f};
  float rise = 0.0f;
  std::vector<TerrainAccess> accesses;
};

// Hardcoded Stage-A scene: a ~30x30 unit field with box-shaped obstacles and
// two 3-figure squads facing each other across the map.
// Neutral flag objective (CTF part 1). Off by default so existing scenes are
// unaffected. When enabled the flag starts at `position` (the nearest free
// spot to the map center when unset).
struct FlagConfig {
  bool enabled = false;
  std::optional<glm::vec3> position;  // y == 0 means "on the ground there".
  // true: the figure that grabs the flag wins at once (part 1). false: the
  // flag is only carried/dropped and play continues (scaffolding for the
  // bring-it-home mode of part 2).
  bool winOnGrab = true;
};

struct Scene {
  std::vector<Obstacle> obstacles;
  // Flat, walkable, visual-only slabs (e.g. legacy rectangular sidewalks).
  std::vector<AABB> sidewalks;
  // Asphalt polygons at ground level, plus explicit ramp/deck layers. The
  // latter are navigable and may overlap ground in XZ without connecting to
  // it except where connectsToGround is set.
  std::vector<RoadSurface> roads;
  std::vector<RoadSurface> sidewalkSurfaces;
  std::vector<WalkSurface> walkSurfaces;
  // Pre-built ziplines (extra nav-graph edges; see Zipline). Anchors are
  // clear ground spots; the straight line between them avoids obstacles.
  std::vector<Zipline> ziplines;
  std::vector<Unit> units;  // 3 Blue + 3 Red, in this order.
  // Playable area is [-mapHalfExtent, mapHalfExtent]^2 in XZ. Defaults to the
  // hand-authored scene's size; MapGenerator sets a much larger value.
  float mapHalfExtent = constants::kMapHalfExtent;
  // Sampled ground elevation. Empty (the default) means flat ground at
  // y = 0; hilly and terrain-enabled urban maps fill it in. Navigation, placement and
  // rendering all sample it (see HeightField).
  HeightField ground;
  std::vector<TerrainWall> terrainWalls;
  FlagConfig flag;
  // Round limit: when > 0 the match ends in a draw once this many rounds
  // have been played without a winner. 0 (the default) means unlimited.
  int roundLimit = 0;
};

Scene BuildDefaultScene();

}  // namespace tactics
