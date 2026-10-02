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
};

// A block's footprint, street edge to street edge.
struct UrbanBlock { float x0, x1, z0, z1; };

// Deterministic procedural city: a straight grid of streets of varying
// width separating blocks of varying size. Some blocks merge two or three
// cells (roads included) or three cells at an angle (L), and some are empty
// open areas. Each block is ringed by a sidewalk
// and, behind it, buildings. Building heights cluster by block (a smooth
// low-to-high field with mostly medium heights) plus a couple of towers.
// Junctions between adjacent buildings are either wall-to-wall (touching,
// impassable) or a gap (passable alley into the block's courtyard); every
// block gets at least one of each. Blue spawns on the west rim street, Red
// on the east, on the row centered on the middle east-west street.
// The same (seed, config) always yields an identical Scene.
Scene GenerateUrbanMap(uint32_t seed, const MapGeneratorConfig& config = {});

// Block footprints GenerateUrbanMap(seed, config) uses (x-major order).
std::vector<UrbanBlock> UrbanBlocks(uint32_t seed, const MapGeneratorConfig& config = {});

// A city block as built: one or more grid cells joined with the streets
// between them (`x0..x1` / `z0..z1` is the bounding box). `notch` marks an
// L of three cells (the box minus one corner cell); `empty` an open area
// with a sidewalk ring but no buildings.
struct UrbanLot { float x0, x1, z0, z1; bool empty, notch; };

// Lots GenerateUrbanMap(seed, config) builds.
std::vector<UrbanLot> UrbanLots(uint32_t seed, const MapGeneratorConfig& config = {});

// Half-extent of the generated map for `config` (independent of seed).
float UrbanMapHalfExtent(const MapGeneratorConfig& config);

// Tuning for GenerateHillyMap. Defaults give an 80x80 field of rolling
// hills (a few units of relief over ~24-unit wavelengths -- gentle enough
// that every slope is walkable) with a scattering of impassable rocks.
struct HillyMapConfig {
  float halfExtent = 40.0f;
  float cellSize = 1.0f;         // Heightfield sample spacing.
  float hillAmplitude = 6.0f;    // Height scale of the base noise octave.
  float hillWavelength = 20.0f;  // Size of the dominant hills.
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
