#pragma once

#include <cstdint>
#include <string>

#include "game/Scene.h"

namespace gallery {

// Compact JSON description of a generated map's geometry for the gallery's
// three.js seed viewer (web/gallery.html): ground heightfield, obstacle
// prisms (kind 0 = generic, 1 + BuildingKind for buildings) and flat
// polygons (kind 0 road, 1 sidewalk, 2 walk surface). Units are world units.
std::string MapToJson(const tactics::Scene& scene);

// GenerateUrbanMap(seed, config) for the "Urban" (4x4 blocks) or "Urban x4"
// (default 8x8) Map Select entry, as MapToJson.
std::string UrbanMapJson(uint32_t seed, bool x4);

}  // namespace gallery
