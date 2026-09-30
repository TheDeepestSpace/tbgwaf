#pragma once

#include <cstdint>

#include "game/Scene.h"

namespace tactics {

// Tuning for GenerateUrbanMap. Defaults give a 4x4 city of 24-unit blocks
// (~136 units across, ~4.5x the default scene's 30).
struct MapGeneratorConfig {
  int blocksX = 4;
  int blocksZ = 4;
  float blockSize = 24.0f;      // Square block footprint, street edge to street edge.
  float streetWidth = 8.0f;     // Corridor between blocks (and around the city's rim).
  float sidewalkWidth = 1.5f;   // Obstacle-free strip inside each block edge, along the street.
  float buildingDepth = 5.0f;   // Thickness of the building ring inside the sidewalk setback.
  float buildingHeight = 3.5f;  // Taller than a figure can climb: walls block movement and LOS.
  float gapWidth = 3.0f;        // Passable alley between two buildings (> 2 * kAgentRadius).
  float gapProbability = 0.45f; // Chance a junction between adjacent buildings is a gap.
};

// Deterministic procedural city: a grid of blocks separated by streets, each
// block ringed by buildings behind a sidewalk setback. Junctions between
// adjacent buildings are either wall-to-wall (touching, impassable) or a
// gap (passable alley into the block's courtyard); every block gets at
// least one of each. Blue spawns on the west rim street, Red on the east.
// The same (seed, config) always yields an identical Scene.
Scene GenerateUrbanMap(uint32_t seed, const MapGeneratorConfig& config = {});

// Half-extent of the generated map for `config`.
float UrbanMapHalfExtent(const MapGeneratorConfig& config);

}  // namespace tactics
