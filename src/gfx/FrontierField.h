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
// takes the kind of its nearest cover tile within `reach` world units on
// its own layer, tactical winning ties. Empty when the maps don't match.
std::vector<glm::vec2> FrontierCoverStyle(const tactics::ReachField& field,
                                          const tactics::CoverMap& cover, float reach);

}  // namespace gfx
