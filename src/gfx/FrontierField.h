#pragma once

#include <vector>

#include <glm/glm.hpp>

#include "game/Cover.h"
#include "game/NavMesh.h"

namespace gfx {

// Smooth signed distance used to contour a movement ReachField. Positive
// samples are reachable and negative samples are not; smoothing never
// changes that classification, so a narrow reachable band cannot disappear.
// Where the boundary is an obstacle rather than the budget running out
// (per field.walkable), the contour follows the obstacle exactly instead
// of being rounded.
std::vector<float> SmoothFrontierSignedDepth(const tactics::ReachField& field);

// Per-node style weights for the frontier glow (issue #292): .x is the
// tactical-cover weight (full gradient) and .y the flat-cover weight
// (striped gradient); both zero means open ground (no gradient). Each node
// samples CoverZoneWeights against the obstacles directly (band `reach`
// deep from the wall faces), so the painted zone runs flat along a wall
// and tapers into the 45-degree wedge at an open end, with nothing
// diagonally past a corner. Unreached nodes beside a wall are sampled too
// (at a reached neighbour's height), since they corner the boundary cells.
std::vector<glm::vec2> FrontierCoverStyle(const tactics::ReachField& field,
                                          const std::vector<tactics::Obstacle>& obstacles,
                                          float reach);

}  // namespace gfx
