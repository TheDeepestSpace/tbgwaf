#pragma once

#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "game/Types.h"

namespace tactics {

float PolygonSignedArea(const std::vector<glm::vec2>& polygon);
float PolygonArea(const std::vector<glm::vec2>& polygon);
bool PointInConvexPolygon(const glm::vec2& point, const std::vector<glm::vec2>& polygon,
                          float epsilon = 1e-4f);
bool PointStrictlyInConvexPolygon(const glm::vec2& point,
                                  const std::vector<glm::vec2>& polygon,
                                  float epsilon = 1e-4f);

// Agent-radius offset of a convex polygon. Empty means the polygon was
// degenerate. Input may be clockwise; output is counter-clockwise.
std::vector<glm::vec2> ExpandConvexPolygon(const std::vector<glm::vec2>& polygon, float amount);
std::vector<glm::vec2> InsetConvexPolygon(const std::vector<glm::vec2>& polygon, float amount);

// True only when a non-zero portion of the segment enters the polygon's
// strict interior. Touching a vertex or travelling along an edge is clear.
bool SegmentEntersConvexPolygon(glm::vec2 from, glm::vec2 to,
                                const std::vector<glm::vec2>& polygon);

// Exact ray/convex-prism intersection. The Obstacle AABB supplies the Y
// interval; its optional polygon supplies the XZ interval.
bool RayIntersectsObstacle(const glm::vec3& origin, const glm::vec3& direction,
                           const Obstacle& obstacle, float* outT = nullptr);
bool RayIntersectsWalkSurface(const glm::vec3& origin, const glm::vec3& direction,
                              const WalkSurface& surface, float thickness,
                              float* outT = nullptr);

// Convex decomposition of a simple polygon (roadmap 5.4). Non-convex floor
// plans (an L, a floor with a stairwell notch) become convex `pieces`, each
// counter-clockwise and made only of the input's vertices, plus `links`:
// index pairs of pieces sharing an edge (the seams a figure crosses between
// them). A convex input yields one piece and no links; fewer than three
// non-degenerate vertices yield nothing. Input may be clockwise.
struct ConvexDecomposition {
  std::vector<std::vector<glm::vec2>> pieces;
  std::vector<std::pair<int, int>> links;
};
ConvexDecomposition DecomposeToConvex(const std::vector<glm::vec2>& polygon);

bool SurfaceContainsXZ(const WalkSurface& surface, float x, float z);
float SurfaceHeightAt(const WalkSurface& surface, float x, float z);
glm::vec3 SurfaceCenter(const WalkSurface& surface);

}  // namespace tactics
