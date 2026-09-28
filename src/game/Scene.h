#pragma once

#include <vector>

#include "game/Types.h"
#include "game/Unit.h"

namespace tactics {

// Hardcoded Stage-A scene: a ~20x20 unit field with box-shaped obstacles and
// two 3-figure squads facing each other across the map.
struct Scene {
  std::vector<AABB> obstacles;
  std::vector<Unit> units;  // 3 Blue + 3 Red, in this order.
};

Scene BuildDefaultScene();

}  // namespace tactics
