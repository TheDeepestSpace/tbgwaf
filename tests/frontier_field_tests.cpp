#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

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

int main() {
  TestThinStraightBandHasSmoothOutline();
  TestNarrowReachableBandSurvivesSmoothing();
  TestOneSampleHairlineIsDropped();
  TestWideRegionStaysRounded();
  TestLedgeBesideDeckSurvivesSmoothing();
  TestObliqueThinBandOutlineIsStraight();
  TestLedgeAlongObliqueCliffOutlineIsStraight();
  if (gFailures == 0) {
    std::printf("All frontier field tests passed.\n");
    return 0;
  }
  std::fprintf(stderr, "%d check(s) failed.\n", gFailures);
  return 1;
}
