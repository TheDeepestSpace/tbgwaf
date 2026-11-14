#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include <glm/glm.hpp>

#include "gfx/FrontierField.h"

namespace {

int gFailures = 0;

#define CHECK(expr) \
  do { \
    if (!(expr)) { \
      std::fprintf(stderr, "CHECK FAILED at %s:%d: %s\n", __FILE__, __LINE__, #expr); \
      ++gFailures; \
    } \
  } while (0)

tactics::ReachField EmptyField(int n) {
  tactics::ReachField field;
  field.step = 0.25f;
  field.budget = 1.0f;
  field.nx = field.nz = n;
  field.dist.assign(static_cast<size_t>(n) * n, std::numeric_limits<float>::infinity());
  field.surfaceY.assign(static_cast<size_t>(n) * n, 0.0f);
  return field;
}

// Every reached sample ends positive, and every unreached sample more than
// one sample away from the reach ends negative. The first ring of outside
// samples may float either way: the smoothed contour is allowed to pass
// just outside the stair-corner samples of an oblique edge, within the
// half-step outward bias the contour carries by construction.
void CheckSignsPreserved(const tactics::ReachField& field, const std::vector<float>& depth) {
  CHECK(depth.size() == field.dist.size());
  if (depth.size() != field.dist.size()) return;
  for (int iz = 0; iz < field.nz; ++iz) {
    for (int ix = 0; ix < field.nx; ++ix) {
      if (field.Reached(ix, iz)) {
        CHECK(depth[iz * field.nx + ix] > 0.0f);
        continue;
      }
      bool nearReach = false;
      for (int dz = -1; dz <= 1 && !nearReach; ++dz) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (field.Reached(ix + dx, iz + dz)) {
            nearReach = true;
            break;
          }
        }
      }
      if (!nearReach) CHECK(depth[iz * field.nx + ix] < 0.0f);
    }
  }
}

void TestNarrowReachableBandSurvivesSmoothing() {
  // Bands two to four samples wide. Two radius-2 blur passes used to
  // average these entirely negative, so no fill or outline was emitted.
  for (int width = 2; width <= 4; ++width) {
    tactics::ReachField field = EmptyField(15);
    for (int iz = 5; iz < 5 + width; ++iz) {
      for (int ix = 1; ix < field.nx - 1; ++ix) field.dist[iz * field.nx + ix] = 0.0f;
    }
    CheckSignsPreserved(field, gfx::SmoothFrontierSignedDepth(field));
  }
}

void TestOneSampleHairlineIsDropped() {
  // A route grazing an obstacle's padded edge reaches a line of nodes one
  // sample wide. It is not an area, and drawn it is a stray strip across the
  // ground, so it must not produce any positive depth.
  tactics::ReachField field = EmptyField(15);
  for (int ix = 1; ix < field.nx - 1; ++ix) field.dist[7 * field.nx + ix] = 0.0f;
  for (float d : gfx::SmoothFrontierSignedDepth(field)) CHECK(d < 0.0f);
}

void TestWideRegionStaysRounded() {
  // A 15x15 block in a 31x31 field: wide enough that the blur is left
  // alone. Its straight edges keep the contour half a step outside the
  // edge samples, and its convex corners are rounded off (the corner
  // sample itself falls outside the contour) rather than snapped back.
  tactics::ReachField field = EmptyField(31);
  for (int iz = 8; iz <= 22; ++iz) {
    for (int ix = 8; ix <= 22; ++ix) field.dist[iz * field.nx + ix] = 0.0f;
  }
  const std::vector<float> depth = gfx::SmoothFrontierSignedDepth(field);
  CHECK(depth.size() == field.dist.size());
  const auto at = [&](int ix, int iz) { return depth[iz * field.nx + ix]; };
  const float half = 0.5f * field.step;
  CHECK(std::abs(at(15, 8) - half) < 0.01f);   // Edge sample, mid-edge.
  CHECK(std::abs(at(15, 7) + half) < 0.01f);   // First outside sample, mid-edge.
  CHECK(at(8, 8) < 0.0f);                      // Corner sample: rounded off.
  CHECK(at(15, 15) > 1.0f);                    // Deep interior.
}

void TestLedgeBesideDeckSurvivesSmoothing() {
  // A three-sample ledge at ground level runs along a wide deck three units
  // up. The deck is on another layer, so it must not make the ledge count
  // as thick (and then let the blur erode it).
  tactics::ReachField field = EmptyField(21);
  for (int iz = 0; iz < field.nz; ++iz) {
    for (int ix = 0; ix < field.nx; ++ix) {
      const int i = iz * field.nx + ix;
      if (ix >= 9) {
        field.surfaceY[i] = 3.0f;
        field.dist[i] = 0.0f;
      } else if (ix >= 6) {
        field.dist[i] = 0.0f;
      }
    }
  }
  CheckSignsPreserved(field, gfx::SmoothFrontierSignedDepth(field));
}

// Max deviation (in samples) of the interpolated zero crossings from their
// least-squares line, over columns [x0, x1], scanning +z. `falling` picks
// the positive-to-negative crossing instead of the first negative-to-
// positive one. Mirrors the renderer at layer edges: the corner on the
// other layer than the dominant one counts as -0.5*step.
float CrossResidual(const tactics::ReachField& f, const std::vector<float>& d,
                    int x0, int x1, bool falling) {
  std::vector<float> xs, zs;
  for (int ix = x0; ix <= x1; ++ix) {
    float crossing = -1.0f;
    bool wasPos = d[0 * f.nx + ix] > 0.0f;
    for (int iz = 1; iz < f.nz; ++iz) {
      float a = d[(iz - 1) * f.nx + ix], b = d[iz * f.nx + ix];
      if (std::fabs(f.surfaceY[(iz - 1) * f.nx + ix] - f.surfaceY[iz * f.nx + ix]) > 0.6f) {
        if (a > b) b = -0.5f * f.step; else a = -0.5f * f.step;
      }
      const bool isPos = b > 0.0f;
      if (isPos != wasPos && isPos != falling) {
        crossing = (iz - 1) + a / (a - b);
        break;
      }
      wasPos = isPos;
    }
    if (crossing < 0.0f) return std::numeric_limits<float>::infinity();
    xs.push_back(static_cast<float>(ix));
    zs.push_back(crossing);
  }
  const int n = static_cast<int>(xs.size());
  float sx = 0, sz = 0, sxx = 0, sxz = 0;
  for (int i = 0; i < n; ++i) {
    sx += xs[i]; sz += zs[i]; sxx += xs[i] * xs[i]; sxz += xs[i] * zs[i];
  }
  const float m = (n * sxz - sx * sz) / (n * sxx - sx * sx);
  const float q = (sz - m * sx) / n;
  float worst = 0.0f;
  for (int i = 0; i < n; ++i) worst = std::max(worst, std::fabs(zs[i] - (m * xs[i] + q)));
  return worst;
}

void TestObliqueThinBandOutlineIsStraight() {
  // A straight thin band crossing the grid at a shallow angle rasterises
  // to a staircase with treads several samples long. The smoothed outline
  // must recover the straight line, not trace the stairs as a wave
  // (issue #244: wavy movement range along oblique roads and ledges).
  for (int slope = 3; slope <= 5; ++slope) {
    tactics::ReachField field = EmptyField(61);
    for (int iz = 0; iz < field.nz; ++iz) {
      for (int ix = 0; ix < field.nx; ++ix) {
        const int v = ix + slope * iz;
        const int v0 = 50 + 7 * slope;  // Keeps the band well inside the grid.
        if (v >= v0 && v <= v0 + 3 * slope) field.dist[iz * field.nx + ix] = 0.0f;
      }
    }
    const std::vector<float> depth = gfx::SmoothFrontierSignedDepth(field);
    CHECK(CrossResidual(field, depth, 10, 50, false) < 0.15f);
    CHECK(CrossResidual(field, depth, 10, 50, true) < 0.15f);
  }
}

void TestLedgeAlongObliqueCliffOutlineIsStraight() {
  // A three-sample ledge under a wall three units up, both cut obliquely
  // across the grid. The outline along the wall is bound to the quad row
  // the layer edge passes through, so it cannot be perfectly straight,
  // but it must stay well under the old half-sample stair amplitude.
  tactics::ReachField field = EmptyField(61);
  for (int iz = 0; iz < field.nz; ++iz) {
    for (int ix = 0; ix < field.nx; ++ix) {
      const int v = ix + 4 * iz;
      const int i = iz * field.nx + ix;
      if (v < 60) {
        field.surfaceY[i] = 3.0f;  // High side: another layer, unreached.
      } else if (v <= 60 + 12) {
        field.dist[i] = 0.0f;  // The ledge.
      }
    }
  }
  const std::vector<float> depth = gfx::SmoothFrontierSignedDepth(field);
  CHECK(CrossResidual(field, depth, 10, 50, false) < 0.3f);  // Against the wall.
  CHECK(CrossResidual(field, depth, 10, 50, true) < 0.15f);  // Open side.
}

// Number of 4-connected positive-depth pieces (same layer).
int PositivePieces(const tactics::ReachField& f, const std::vector<float>& d) {
  std::vector<char> seen(d.size(), 0);
  int pieces = 0;
  for (size_t s = 0; s < d.size(); ++s) {
    if (d[s] <= 0.0f || seen[s]) continue;
    ++pieces;
    std::vector<int> stack{static_cast<int>(s)};
    seen[s] = 1;
    while (!stack.empty()) {
      const int c = stack.back();
      stack.pop_back();
      const int dx[4] = {-1, 1, 0, 0}, dz[4] = {0, 0, -1, 1};
      for (int k = 0; k < 4; ++k) {
        const int x = c % f.nx + dx[k], z = c / f.nx + dz[k];
        if (x < 0 || z < 0 || x >= f.nx || z >= f.nz) continue;
        const int j = z * f.nx + x;
        if (d[j] > 0.0f && !seen[j]) {
          seen[j] = 1;
          stack.push_back(j);
        }
      }
    }
  }
  return pieces;
}

void TestThinNeckStaysConnectedToWideRegion() {
  // A two-sample corridor leaving a wide block, and a second block touching
  // the first only at a corner. Smoothing must keep each region whole.
  for (int width = 2; width <= 3; ++width) {
    tactics::ReachField field = EmptyField(41);
    for (int iz = 5; iz <= 19; ++iz) {
      for (int ix = 5; ix <= 19; ++ix) field.dist[iz * field.nx + ix] = 0.0f;
    }
    for (int iz = 12; iz < 12 + width; ++iz) {
      for (int ix = 20; ix <= 36; ++ix) field.dist[iz * field.nx + ix] = 0.0f;
    }
    CHECK(PositivePieces(field, gfx::SmoothFrontierSignedDepth(field)) == 1);
  }
  tactics::ReachField field = EmptyField(41);
  for (int iz = 5; iz <= 14; ++iz) {
    for (int ix = 5; ix <= 14; ++ix) field.dist[iz * field.nx + ix] = 0.0f;
  }
  for (int iz = 15; iz <= 24; ++iz) {
    for (int ix = 15; ix <= 24; ++ix) field.dist[iz * field.nx + ix] = 0.0f;
  }
  CHECK(PositivePieces(field, gfx::SmoothFrontierSignedDepth(field)) == 1);
}

void TestWallBoundaryIsExact() {
  // The reach ends at a wall on +x and at the budget elsewhere (issue
  // #292). Where an obstacle causes the boundary the contour must hug it:
  // the last reached column sits at +step/2 and the wall column at -step/2
  // along the whole face, instead of the blur's rounded pull-back.
  tactics::ReachField field = EmptyField(31);
  field.walkable.assign(field.dist.size(), 1);
  for (int iz = 5; iz <= 25; ++iz) {
    for (int ix = 5; ix <= 15; ++ix) field.dist[iz * field.nx + ix] = 0.0f;
    for (int ix = 16; ix <= 20; ++ix) field.walkable[iz * field.nx + ix] = 0;
  }
  const std::vector<float> depth = gfx::SmoothFrontierSignedDepth(field);
  const float half = 0.5f * field.step;
  for (int iz = 10; iz <= 20; ++iz) {
    CHECK(std::abs(depth[iz * field.nx + 15] - half) < 0.02f);
    CHECK(std::abs(depth[iz * field.nx + 16] + half) < 0.02f);
  }
  // Away from the wall the budget edge keeps its smoothing: the block's
  // convex corner sample is still rounded off.
  CHECK(depth[5 * field.nx + 5] < 0.0f);
}

void TestWallCornerStaysSquare() {
  // A rectangular obstacle corner poking into a wide reached region
  // (issue #292): the contour must hug both faces squarely all the way
  // into the corner instead of rounding across it, and the wall itself
  // must stay negative (no positive depth leaking over the obstacle).
  tactics::ReachField field = EmptyField(41);
  field.walkable.assign(field.dist.size(), 1);
  for (int iz = 0; iz < field.nz; ++iz) {
    for (int ix = 0; ix < field.nx; ++ix) {
      const int i = iz * field.nx + ix;
      if (ix >= 20 && iz <= 20) field.walkable[i] = 0;  // Obstacle quadrant.
      else field.dist[i] = 0.0f;
    }
  }
  const std::vector<float> depth = gfx::SmoothFrontierSignedDepth(field);
  const float half = 0.5f * field.step;
  const auto at = [&](int ix, int iz) { return depth[iz * field.nx + ix]; };
  // Vertical face, right up to the corner at (20, 20).
  for (int iz = 2; iz <= 20; ++iz) {
    CHECK(std::abs(at(19, iz) - half) < 0.02f);
    CHECK(std::abs(at(20, iz) + half) < 0.02f);
  }
  // Horizontal face, right up to the corner.
  for (int ix = 22; ix <= 38; ++ix) {
    CHECK(std::abs(at(ix, 21) - half) < 0.02f);
    CHECK(std::abs(at(ix, 20) + half) < 0.02f);
  }
}

void TestWallContourUsesExactOutline() {
  // With wallDist filled, the wall part of the contour must come from the
  // exact signed outline distance, not from the grid's half-step line.
  // wallDist is signed against the padded outline at x = 19.6 * step
  // (positive on the walkable side, negative past it), and the drawn
  // contour sits kObstacleInset = kAgentRadius - 0.05 (1.4 steps here)
  // beyond the padded outline, i.e. 0.05 off the obstacle's real face: the
  // depth must be the signed outline distance plus that inset, putting the
  // zero crossing at x = 21.0 * step exactly.
  tactics::ReachField field = EmptyField(41);
  field.walkable.assign(field.dist.size(), 1);
  field.wallDist.assign(field.dist.size(), 0.0f);
  const float wallX = 19.6f * field.step;
  for (int iz = 0; iz < field.nz; ++iz) {
    for (int ix = 0; ix < field.nx; ++ix) {
      const int i = iz * field.nx + ix;
      if (ix >= 20) field.walkable[i] = 0;
      else field.dist[i] = 0.0f;
      field.wallDist[i] = wallX - ix * field.step;
    }
  }
  const std::vector<float> depth = gfx::SmoothFrontierSignedDepth(field);
  for (int iz = 10; iz <= 30; ++iz) {
    const float a = depth[iz * field.nx + 20];  // +1.0 * step exactly.
    const float b = depth[iz * field.nx + 22];  // -1.0 * step exactly.
    CHECK(std::abs(a - 1.0f * field.step) < 0.005f);
    CHECK(std::abs(b + 1.0f * field.step) < 0.005f);
    CHECK(std::abs(depth[iz * field.nx + 21]) < 0.005f);
  }
}

void TestLevelEdgeContourUsesExactLine() {
  // With edgeDist filled (FillLevelEdgeDistances), the part of the contour
  // that runs along a level edge -- a terrain cliff here: the right half is
  // another walk layer -- must come from the exact distance to the edge
  // line, zero crossing kLevelEdgeInset before it, not from the grid's
  // half-step line.
  tactics::ReachField field = EmptyField(41);
  field.edgeDist.assign(field.dist.size(), std::numeric_limits<float>::infinity());
  const float edgeX = 19.6f * field.step;
  for (int iz = 0; iz < field.nz; ++iz) {
    for (int ix = 0; ix < field.nx; ++ix) {
      const int i = iz * field.nx + ix;
      if (ix >= 20) field.surfaceY[i] = 3.0f;  // The level: unreached layer.
      else field.dist[i] = 0.0f;
      field.edgeDist[i] = std::fabs(edgeX - ix * field.step);
    }
  }
  const std::vector<float> depth = gfx::SmoothFrontierSignedDepth(field);
  for (int iz = 10; iz <= 30; ++iz) {
    for (int ix = 15; ix <= 19; ++ix) {
      const float expected = (edgeX - ix * field.step) - gfx::kLevelEdgeInset;
      CHECK(std::abs(depth[iz * field.nx + ix] - expected) < 0.005f);
    }
    // The far side stays negative: its own layer has no reach at all.
    CHECK(depth[iz * field.nx + 21] < 0.0f);
  }
}

void TestCoverStyleWeights() {
  // A wall obstacle: hatched (flat) along its face, solid (tactical) on the
  // full band-deep stretch within kCoverEdgeReach of each open end, and
  // nothing diagonally past a corner or on open ground (no cover mark on
  // the budget boundary). Nodes sit at (ix * 0.25, iz * 0.25).
  tactics::ReachField field = EmptyField(21);
  for (float& d : field.dist) d = 0.0f;
  tactics::Obstacle wall;
  wall.bounds.min = glm::vec3(3.75f, 0.0f, 1.25f);
  wall.bounds.max = glm::vec3(4.5f, 3.0f, 3.75f);
  const std::vector<glm::vec2> style =
      gfx::FrontierCoverStyle(field, {wall}, 0.8f);
  CHECK(style.size() == field.dist.size());
  if (style.size() != field.dist.size()) return;
  const auto at = [&](int ix, int iz) { return style[iz * field.nx + ix]; };
  CHECK(at(14, 10).y == 1.0f && at(14, 10).x == 0.0f);  // Mid-wall: striped (flat).
  CHECK(at(14, 6).x == 1.0f && at(14, 6).y == 0.0f);    // Wedge at the wall's end.
  CHECK(at(12, 6).x == 1.0f && at(12, 6).y == 0.0f);    // Same height, farther out: still the solid patch.
  CHECK(at(14, 4).x == 0.0f && at(14, 4).y == 0.0f);    // Diagonally past the corner: nothing.
  CHECK(at(10, 10).x == 0.0f && at(10, 10).y == 0.0f);  // Open ground: none.
}

}  // namespace

void TestThinStraightBandHasSmoothOutline() {
  // Away from its ends, a straight thin band must have the same depth
  // along its whole length: no bumps from mixing smoothing levels.
  for (int width = 1; width <= 6; ++width) {
    tactics::ReachField field = EmptyField(31);
    for (int iz = 12; iz < 12 + width; ++iz) {
      for (int ix = 0; ix < field.nx; ++ix) field.dist[iz * field.nx + ix] = 0.0f;
    }
    const std::vector<float> depth = gfx::SmoothFrontierSignedDepth(field);
    for (int iz = 8; iz < 20; ++iz) {
      for (int ix = 5; ix < 25; ++ix) {
        CHECK(std::abs(depth[iz * field.nx + ix] - depth[iz * field.nx + 15]) < 1e-4f);
      }
    }
  }
}

// Appends a convex counter-clockwise padded outline as wall segments.
void AddWallOutline(tactics::ReachField* field, const std::vector<glm::vec2>& polygon,
                    float inset, int outline) {
  for (size_t i = 0; i < polygon.size(); ++i) {
    tactics::BoundarySeg seg;
    seg.a = polygon[i];
    seg.b = polygon[(i + 1) % polygon.size()];
    seg.inset = inset;
    seg.outline = outline;
    field->wallSegs.push_back(seg);
  }
}

bool InsideCcw(const std::vector<glm::vec2>& polygon, glm::vec2 p) {
  for (size_t i = 0; i < polygon.size(); ++i) {
    const glm::vec2 ab = polygon[(i + 1) % polygon.size()] - polygon[i], ap = p - polygon[i];
    if (ab.x * ap.y - ab.y * ap.x < 0.0f) return false;
  }
  return true;
}

void TestOutlineAlongObliqueWallIsOneStraightLine() {
  // Issue #292: everything is reached except an obstacle rotated 27 degrees
  // against the grid. The traced outline around it must be exactly the four
  // inset lines meeting at four sharp corners -- no grid staircase, whatever
  // side of the obstacle -- i.e. four points.
  tactics::ReachField field = EmptyField(81);
  field.walkable.assign(field.dist.size(), 1);
  const glm::vec2 centre(10.0f, 10.0f);
  const float angle = 0.47f, c = std::cos(angle), s = std::sin(angle);
  const auto corner = [&](float x, float z) {
    return centre + glm::vec2(c * x - s * z, s * x + c * z);
  };
  const std::vector<glm::vec2> padded = {corner(-4, -2.5f), corner(4, -2.5f), corner(4, 2.5f),
                                         corner(-4, 2.5f)};
  const float inset = 0.35f;
  AddWallOutline(&field, padded, inset, 0);
  for (int iz = 0; iz < field.nz; ++iz) {
    for (int ix = 0; ix < field.nx; ++ix) {
      const int i = iz * field.nx + ix;
      const glm::vec2 p(ix * field.step, iz * field.step);
      if (InsideCcw(padded, p)) field.walkable[i] = 0;
      else if (ix > 0 && iz > 0 && ix < field.nx - 1 && iz < field.nz - 1) field.dist[i] = 0.0f;
    }
  }
  const std::vector<gfx::FrontierOutline> outlines = gfx::TraceFrontierOutline(field);
  const std::vector<glm::vec2> face = {corner(-4 + inset, -2.5f + inset),
                                       corner(4 - inset, -2.5f + inset),
                                       corner(4 - inset, 2.5f - inset),
                                       corner(-4 + inset, 2.5f - inset)};
  int found = 0;
  for (const gfx::FrontierOutline& outline : outlines) {
    if (outline.points.empty()) continue;
    const glm::vec2 first(outline.points[0].x, outline.points[0].z);
    if (glm::length(first - centre) > 6.0f) continue;  // The window's own rim.
    ++found;
    CHECK(outline.closed);
    // Every point lies on one of the four inset lines, and the four corners
    // are all present exactly.
    for (const glm::vec3& q : outline.points) {
      const glm::vec2 p(q.x, q.z);
      float best = 1e9f;
      for (size_t i = 0; i < 4; ++i) {
        const glm::vec2 a = face[i], d = glm::normalize(face[(i + 1) % 4] - a);
        best = std::min(best, std::fabs((p.x - a.x) * d.y - (p.y - a.y) * d.x));
      }
      CHECK(best < 1e-3f);
    }
    for (const glm::vec2& f : face) {
      float best = 1e9f;
      for (const glm::vec3& q : outline.points) {
        best = std::min(best, glm::length(glm::vec2(q.x, q.z) - f));
      }
      CHECK(best < 1e-3f);
    }
  }
  CHECK(found == 1);
}

void TestOutlineIgnoresEdgesBuriedInAbuttingOutline() {
  // A long wall built from two abutting pieces: their shared end caps are
  // inside the union and must not pull the outline into a notch.
  tactics::ReachField field = EmptyField(81);
  field.walkable.assign(field.dist.size(), 1);
  const std::vector<glm::vec2> left = {{4.0f, 9.0f}, {10.2f, 9.0f}, {10.2f, 11.0f}, {4.0f, 11.0f}};
  const std::vector<glm::vec2> right = {{9.8f, 9.0f}, {16.0f, 9.0f}, {16.0f, 11.0f},
                                        {9.8f, 11.0f}};
  AddWallOutline(&field, left, 0.05f, 0);
  AddWallOutline(&field, right, 0.05f, 1);
  for (int iz = 0; iz < field.nz; ++iz) {
    for (int ix = 0; ix < field.nx; ++ix) {
      const int i = iz * field.nx + ix;
      const glm::vec2 p(ix * field.step, iz * field.step);
      if (InsideCcw(left, p) || InsideCcw(right, p)) field.walkable[i] = 0;
      else if (ix > 0 && iz > 0 && ix < field.nx - 1 && iz < field.nz - 1) field.dist[i] = 0.0f;
    }
  }
  for (const gfx::FrontierOutline& outline : gfx::TraceFrontierOutline(field)) {
    for (const glm::vec3& q : outline.points) {
      if (q.x < 5.0f || q.x > 15.0f || q.z < 8.0f || q.z > 12.0f) continue;
      // Along the long sides: exactly on z = 9.05 or z = 10.95.
      CHECK(std::fabs(q.z - 9.05f) < 1e-3f || std::fabs(q.z - 10.95f) < 1e-3f);
    }
  }
}

void TestOutlineAlongLevelEdgeIsOneStraightLine() {
  // The reach ends at an oblique cliff line (the far side is another walk
  // layer): the outline along it is the edge line, kLevelEdgeInset inside.
  tactics::ReachField field = EmptyField(81);
  const glm::vec2 a(2.0f, 9.3f), b(18.0f, 12.1f);
  const glm::vec2 d = glm::normalize(b - a);
  tactics::BoundarySeg seg;
  seg.a = a;
  seg.b = b;
  seg.inset = gfx::kLevelEdgeInset;
  seg.yMin = 0.0f;
  seg.yMax = 3.0f;
  field.edgeSegs.push_back(seg);
  for (int iz = 0; iz < field.nz; ++iz) {
    for (int ix = 0; ix < field.nx; ++ix) {
      const int i = iz * field.nx + ix;
      const glm::vec2 p(ix * field.step, iz * field.step);
      const float side = (p.x - a.x) * d.y - (p.y - a.y) * d.x;  // > 0: low side.
      if (side < 0.0f) field.surfaceY[i] = 3.0f;
      else if (side > 0.3f && ix > 0 && iz > 0 && ix < field.nx - 1 && iz < field.nz - 1) {
        field.dist[i] = 0.0f;
      }
    }
  }
  int onLine = 0;
  for (const gfx::FrontierOutline& outline : gfx::TraceFrontierOutline(field)) {
    for (const glm::vec3& q : outline.points) {
      const float side = (q.x - a.x) * d.y - (q.z - a.y) * d.x;
      if (q.x < 4.0f || q.x > 16.0f || std::fabs(side) > 1.0f) continue;
      CHECK(std::fabs(side - gfx::kLevelEdgeInset) < 1e-3f);
      ++onLine;
    }
  }
  CHECK(onLine > 20);
}

void TestBudgetArcOutlineIsSmooth() {
  // Open ground, the reach a disc: the outline is the budget circle, every
  // point within a small fraction of a sample of it (no grid staircase).
  tactics::ReachField field = EmptyField(81);
  field.walkable.assign(field.dist.size(), 1);
  field.budget = 6.0f;
  const glm::vec2 centre(10.0f, 10.0f);
  for (int iz = 0; iz < field.nz; ++iz) {
    for (int ix = 0; ix < field.nx; ++ix) {
      const float r = glm::length(glm::vec2(ix * field.step, iz * field.step) - centre);
      if (r <= field.budget) field.dist[iz * field.nx + ix] = r;
    }
  }
  const std::vector<gfx::FrontierOutline> outlines = gfx::TraceFrontierOutline(field);
  CHECK(outlines.size() == 1);
  for (const gfx::FrontierOutline& outline : outlines) {
    CHECK(outline.closed);
    for (const glm::vec3& q : outline.points) {
      CHECK(std::fabs(glm::length(glm::vec2(q.x, q.z) - centre) - field.budget) < 0.04f);
    }
  }
}

int main() {
  TestOutlineAlongObliqueWallIsOneStraightLine();
  TestOutlineIgnoresEdgesBuriedInAbuttingOutline();
  TestOutlineAlongLevelEdgeIsOneStraightLine();
  TestBudgetArcOutlineIsSmooth();
  TestThinStraightBandHasSmoothOutline();
  TestThinNeckStaysConnectedToWideRegion();
  TestNarrowReachableBandSurvivesSmoothing();
  TestOneSampleHairlineIsDropped();
  TestWideRegionStaysRounded();
  TestLedgeBesideDeckSurvivesSmoothing();
  TestObliqueThinBandOutlineIsStraight();
  TestLedgeAlongObliqueCliffOutlineIsStraight();
  TestWallBoundaryIsExact();
  TestWallCornerStaysSquare();
  TestWallContourUsesExactOutline();
  TestLevelEdgeContourUsesExactLine();
  TestCoverStyleWeights();
  if (gFailures == 0) {
    std::printf("All frontier field tests passed.\n");
    return 0;
  }
  std::fprintf(stderr, "%d check(s) failed.\n", gFailures);
  return 1;
}
