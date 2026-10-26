#pragma once

#include <optional>
#include <vector>

#include "game/Types.h"
#include "game/Unit.h"

namespace tactics {

// A stair up one terrain level (issue #155): a flight attached along the
// cliff face on its low side, climbing parallel to it, then a flat landing
// at the level's height that lets onto the level over the cliff top.
// `bottom` is the ground point at the flight's foot, `top` the ground point
// on the level beside the landing.
struct TerrainAccess {
  glm::vec3 bottom, top;
  float width = 2.4f;
};

// One map-spanning terrain step: ground on the `raisedNormal` side of the
// step line through `origin` along `dir` sits `rise` higher, behind a sheer
// cliff. The step line is a block boundary like a street: a 6.4-unit band
// centered on it is carved from the blocks on both sides, its low half
// holding the stairs. Steps overlap into nested tiers; highway ramps and
// the stairs climb them.
struct TerrainWall {
  glm::vec2 origin{0.0f}, dir{1.0f, 0.0f};
  glm::vec2 raisedNormal{0.0f, 1.0f};
  glm::vec2 lineOrigin{0.0f};  // A point on the step line (== origin).
  float rise = 0.0f;
  std::vector<TerrainAccess> accesses;
};

// One stair up a level, as drawn: a sloped flight quad hugging the cliff on
// its low side and a flat landing quad at the level's height past its head,
// both extruded down to `base` as one concrete wedge. Movement follows the
// same grades in `ground`.
struct TerrainStair {
  std::vector<glm::vec3> flight, landing;  // Four vertices each.
  float base = 0.0f;                       // Ground height at the foot.
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

// Generator-suggested placement (XZ). Unset hints keep a scene's own units and
// the flag's default; see SuggestSpawnHints / ApplySpawnHints.
struct SpawnHints {
  std::optional<glm::vec2> blue;  // Center of Blue's spawn area.
  std::optional<glm::vec2> red;   // Center of Red's spawn area (opposite corner).
  std::optional<glm::vec2> flag;  // Preferred flag spot (map center).
};

struct Scene {
  std::vector<Obstacle> obstacles;
  // Typology of the generated buildings (issue #151): one record per
  // building, indexed from Obstacle::building. Empty for hand-authored
  // scenes. `massing` holds the visual-only parapets, roof furniture and
  // awnings the renderer draws on/over them.
  std::vector<Building> buildings;
  std::vector<MassingDetail> massing;
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
  // Urban levels as drawn: each tier is an extruded polygon (the map split
  // by the walls' step lines, minus the ramp corridors in `terrainCutouts`,
  // convex rectangles exactly the width of their pavement), and each stair
  // one extruded wedge. The heightfield above stays the movement model.
  std::vector<TerrainWall> terrainWalls;
  std::vector<std::vector<glm::vec2>> terrainCutouts;
  std::vector<TerrainStair> terrainStairs;
  FlagConfig flag;
  SpawnHints spawnHints;
  // Round limit: when > 0 the match ends in a draw once this many rounds
  // have been played without a winner. 0 (the default) means unlimited.
  int roundLimit = 0;
};

// One flat piece of a city level, as drawn: a convex polygon of the map
// square split by every wall's step line and cut by every ramp corridor,
// standing `height` above the base plane. The renderer extrudes each into a
// prism (nothing else of the levels is drawn), so a pavement patch that
// lies under a tier's top is buried behind the tier's face.
struct TerrainTier {
  std::vector<glm::vec2> polygon;
  float height = 0.0f;
};
std::vector<TerrainTier> TerrainTiers(const std::vector<TerrainWall>& walls,
                                      const std::vector<std::vector<glm::vec2>>& cutouts,
                                      const std::vector<glm::vec2>& square);

Scene BuildDefaultScene();

// True when a figure can stand at (x, z): on the map and clear of obstacles.
bool IsFreeSpot(const Scene& scene, float x, float z);

// Nearest free spot to `target` (expanding square rings, deterministic), or
// `target` itself if nothing within the map is free.
glm::vec2 NearestFreeSpot(const Scene& scene, glm::vec2 target);

// Fills `scene->spawnHints`: Blue and Red at opposite (south-west / north-east)
// corners, flag at the map center.
void SuggestSpawnHints(Scene* scene);

// Moves each team's units to its hinted spawn area (nearest free spots, facing
// the opposite corner). No-op for a team without a hint.
void ApplySpawnHints(Scene* scene);

// Obstacles plus the building massing (parapets, roof furniture, awnings) as
// prisms, appended after the real obstacles so obstacle indices are
// unchanged. For sight lines and FOV only: massing never collides.
std::vector<Obstacle> SightObstacles(const Scene& scene);

}  // namespace tactics
