#pragma once

#include <vector>

#include "game/Types.h"
#include "game/Unit.h"

namespace tactics {

// Hardcoded Stage-A scene: a ~30x30 unit field with box-shaped obstacles and
// two 3-figure squads facing each other across the map.
struct Scene {
  std::vector<Obstacle> obstacles;
  // Flat, walkable, visual-only slabs (e.g. sidewalks); no gameplay effect.
  std::vector<AABB> sidewalks;
  std::vector<Unit> units;  // 3 Blue + 3 Red, in this order.
  // Playable area is [-mapHalfExtent, mapHalfExtent]^2 in XZ. Defaults to the
  // hand-authored scene's size; MapGenerator sets a much larger value.
  float mapHalfExtent = constants::kMapHalfExtent;
  // Sampled ground elevation. Empty (the default) means flat ground at
  // y = 0; the hilly generator fills it in, and NavMesh / unit placement /
  // rendering all sample it (see HeightField).
  HeightField ground;
};

Scene BuildDefaultScene();

}  // namespace tactics
