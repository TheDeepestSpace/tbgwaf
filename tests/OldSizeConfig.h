#pragma once

#include "game/MapGenerator.h"

namespace tactics {

// The pre-#224 default city (4x4 blocks, ~144 units across). Fingerprint and
// golden tests pin to it; the generator default is now 8x8.
inline MapGeneratorConfig OldSizeConfig() {
  MapGeneratorConfig c;
  c.blocksX = 4;
  c.blocksZ = 4;
  return c;
}

}  // namespace tactics
