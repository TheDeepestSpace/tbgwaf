#pragma once

#include <vector>

#include "game/NavMesh.h"

namespace gfx {

// Smooth signed distance used to contour a movement ReachField. Positive
// samples are reachable and negative samples are not; smoothing never
// changes that classification, so a narrow reachable band cannot disappear.
std::vector<float> SmoothFrontierSignedDepth(const tactics::ReachField& field);

}  // namespace gfx
