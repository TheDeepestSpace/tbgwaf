#pragma once

#include <optional>
#include <vector>

#include "game/Types.h"
#include "game/Unit.h"

namespace tactics {

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
  // y = 0; the hilly generator fills it in, and NavMesh / unit placement /
  // rendering all sample it (see HeightField).
  HeightField ground;
  FlagConfig flag;
  // Round limit: when > 0 the match ends in a draw once this many rounds
  // have been played without a winner. 0 (the default) means unlimited.
  int roundLimit = 0;
};

Scene BuildDefaultScene();

}  // namespace tactics
