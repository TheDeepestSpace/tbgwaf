#pragma once

#include <vector>

#include <glm/glm.hpp>

#include "game/Cover.h"
#include "game/NavMesh.h"

namespace gfx {

// How far inside a level-edge line (cliff top/base, stair flank, deck
// outline; see ReachField::edgeDist) the drawn boundary sits, so the line
// reads as hugging the edge without z-fighting the face below/above it.
// Shared by the depth smoothing and the renderer's layer-step cells so the
// contour is continuous across cell kinds.
constexpr float kLevelEdgeInset = 0.06f;

// Smooth signed distance used to contour a movement ReachField. Positive
// samples are reachable and negative samples are not; smoothing never
// changes that classification, so a narrow reachable band cannot disappear.
// Where the boundary is an obstacle rather than the budget running out
// (per field.walkable), the contour follows the obstacle exactly instead
// of being rounded.
// Only the frontier's fill (budget glow, cover marks) is clipped to this
// field's zero contour; the boundary line is TraceFrontierOutline below.
std::vector<float> SmoothFrontierSignedDepth(const tactics::ReachField& field);

// The movement boundary as vector polylines (issue #292). The reach mask's
// outline is traced into loops, and every outline vertex is then placed by
// what actually bounds the reach there:
//  - an obstacle: projected onto that padded outline edge's line (drawn
//    `inset` past it, just off the real face);
//  - a level edge (cliff, stair flank, deck outline): projected onto that
//    edge's line;
//  - the move budget: the interpolated budget crossing, lightly smoothed.
// Every vertex bounded by the same segment lies exactly on one line, so a
// wall or cliff side is straight by construction whatever its angle to the
// sampling grid, and neighbouring sides meet at their lines' intersection.
// y is the reached side's surface height.
struct FrontierOutline {
  std::vector<glm::vec3> points;
  bool closed = false;
};
std::vector<FrontierOutline> TraceFrontierOutline(const tactics::ReachField& field);

}  // namespace gfx
