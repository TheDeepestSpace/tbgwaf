#pragma once

#include <cstdint>
#include <vector>

#include "game/Scene.h"

namespace tactics {

// Tuning for GenerateUrbanMap. Defaults give a 4x4 city of ~26-unit average
// blocks (~144 units across, ~4.8x the default scene's 30). The overall
// extent is fixed by (blocks, blockSize, streetWidth); per-seed variation
// redistributes it between blocks and streets.
struct MapGeneratorConfig {
  int blocksX = 4;
  int blocksZ = 4;
  float blockSize = 26.0f;      // Average block footprint; real sizes vary per column/row.
  float streetWidth = 8.0f;     // Average corridor between blocks; rim streets use exactly this.
  float streetWidthMin = 0.75f; // Interior street width range, as a multiple of streetWidth.
  float streetWidthMax = 1.4f;
  float minBlockSize = 22.0f;   // Floor so every block fits a sidewalk, a building ring and gaps.
  float sidewalkWidth = 1.5f;   // Obstacle-free strip inside each block edge, along the street.
  float buildingDepth = 5.0f;   // Thickness of the building ring inside the sidewalk setback.
  float minBuildingHeight = 3.5f;  // Taller than a figure can climb: walls block movement and LOS.
  float maxBuildingHeight = 9.0f;  // Cap for ordinary (non-tower) buildings.
  int towerCount = 2;           // Tall landmark buildings scattered across the city.
  float towerMinHeight = 14.0f;
  float towerMaxHeight = 20.0f;
  float gapWidth = 3.0f;        // Passable alley between two buildings (> 2 * kAgentRadius).
  float gapProbability = 0.45f; // Chance a junction between adjacent buildings is a gap.
  // Hierarchical street layout. Set arteryCount=0 for the original orthogonal
  // grid generator; the default lays one oblique boundary-to-boundary artery.
  int arteryCount = 1;           // 1 oblique through-road; >=2 adds a wide branching avenue.
  float arteryWidth = 14.0f;
  float localStreetWidth = 6.0f;
  float localStreetSkew = 0.45f; // Max radians a cross street deviates from square to the artery.
  bool elevatedHighway = false;  // Artery becomes an overpass: ramps up, crosses on piers, ramps down.
  float highwayElevation = 5.0f;
  float highwayThickness = 0.45f;
  float supportSpacing = 18.0f;  // Arc-length between pier bents under the deck.
  float minPolygonArea = 22.0f;  // Smaller inset/sliver regions remain open.
};

// A block's footprint, street edge to street edge. The bounds are retained
// for compatibility; `vertices` is authoritative for angled layouts.
struct UrbanBlock {
  float x0, x1, z0, z1;
  std::vector<glm::vec2> vertices;
};

struct UrbanRoad {
  std::vector<glm::vec3> centerline;
  float width = 0.0f;
  bool artery = false;
  bool elevated = false;
};

// Deterministic hierarchical city. The default cuts the map with one wide
// oblique boundary-to-boundary artery, then angled avenues and oblique cross
// streets on each side; the resulting convex polygon blocks all differ in
// shape and their frontage-following building rows adjust to the angles
// (acute corner wedges become real angled buildings; collapsed slivers stay
// open). arteryCount>=2 adds a wide branching avenue that merges into the
// artery. elevatedHighway turns the artery into a true overpass: it ramps up
// from grade, crosses the city center as a bridge deck on paired pier
// columns with usable ground beneath, and ramps back down to grade at the
// far side, while the branch becomes an on-ramp that climbs and merges onto
// the deck mid-span. arteryCount=0 retains the original orthogonal generator
// for compatibility. The same (seed, config) always yields an identical Scene.
Scene GenerateUrbanMap(uint32_t seed, const MapGeneratorConfig& config = {});

// Block footprints GenerateUrbanMap(seed, config) uses. The hierarchical
// layout returns one convex polygon per block (artery-left side first);
// arteryCount=0 returns the legacy grid rectangles in x-major order.
std::vector<UrbanBlock> UrbanBlocks(uint32_t seed, const MapGeneratorConfig& config = {});

// Centerlines used by the generated scene. The oblique artery is returned
// first (sampled boundary-to-boundary, with its elevation profile when
// elevatedHighway is set); the branching avenue follows, sampled from the
// map boundary to its merge point on the artery.
std::vector<UrbanRoad> UrbanRoads(uint32_t seed, const MapGeneratorConfig& config = {});

// A city block as built: one or more grid cells joined with the streets
// between them (`x0..x1` / `z0..z1` is the bounding box). `notch` marks an
// L of three cells (the box minus one corner cell); `empty` an open area
// with a sidewalk ring but no buildings.
struct UrbanLot { float x0, x1, z0, z1; bool empty, notch; };

// Lot bounds for compatibility/inspection. Hierarchical layouts return one
// bound per polygon block; arteryCount=0 returns the original merged/L lots.
std::vector<UrbanLot> UrbanLots(uint32_t seed, const MapGeneratorConfig& config = {});

// Half-extent of the generated map for `config` (independent of seed).
float UrbanMapHalfExtent(const MapGeneratorConfig& config);

// Tuning for GenerateHillyMap. Defaults give an 80x80 field of rolling
// hills (~10 units of relief over ~16-unit wavelengths -- steep, but
// every slope is still walkable) with a scattering of impassable rocks.
struct HillyMapConfig {
  float halfExtent = 40.0f;
  float cellSize = 1.0f;         // Heightfield sample spacing.
  float hillAmplitude = 12.0f;   // Height scale of the base noise octave.
  float hillWavelength = 16.0f;  // Size of the dominant hills.
  int rockCount = 12;            // Impassable boulders scattered mid-field.
  float rockMinExtent = 1.2f;    // Half-extent range of a rock's footprint.
  float rockMaxExtent = 3.0f;
  float rockHeight = 2.8f;       // Rock top above the local terrain: blocks LOS.
  float spawnMargin = 8.0f;      // Rock-free strip at the west/east spawn edges.
};

// Deterministic rolling-hills map: continuous ground-height variation
// (Scene::ground is filled in) from seeded value noise, a handful of
// impassable rocks embedded in the slopes, and the usual 3v3 spawn rows on
// the west (Blue) and east (Red) edges, each figure standing on the terrain.
// The same (seed, config) always yields an identical Scene.
Scene GenerateHillyMap(uint32_t seed, const HillyMapConfig& config = {});

}  // namespace tactics
