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

// Every reached sample ends positive and every unreached one negative, so
// smoothing never changes what the overlay says is reachable.
void CheckSignsPreserved(const tactics::ReachField& field, const std::vector<float>& depth) {
  CHECK(depth.size() == field.dist.size());
  if (depth.size() != field.dist.size()) return;
  for (int iz = 0; iz < field.nz; ++iz) {
    for (int ix = 0; ix < field.nx; ++ix) {
      const bool reached = field.Reached(ix, iz);
      CHECK(reached ? depth[iz * field.nx + ix] > 0.0f : depth[iz * field.nx + ix] < 0.0f);
    }
  }
}

void TestNarrowReachableBandSurvivesSmoothing() {
  // Bands one to four samples wide. Two radius-2 blur passes used to
  // average these entirely negative, so no fill or outline was emitted.
  for (int width = 1; width <= 4; ++width) {
    tactics::ReachField field = EmptyField(15);
    for (int iz = 5; iz < 5 + width; ++iz) {
      for (int ix = 1; ix < field.nx - 1; ++ix) field.dist[iz * field.nx + ix] = 0.0f;
    }
    CheckSignsPreserved(field, gfx::SmoothFrontierSignedDepth(field));
  }
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

}  // namespace

int main() {
  TestNarrowReachableBandSurvivesSmoothing();
  TestWideRegionStaysRounded();
  TestLedgeBesideDeckSurvivesSmoothing();
  if (gFailures == 0) {
    std::printf("All frontier field tests passed.\n");
    return 0;
  }
  std::fprintf(stderr, "%d check(s) failed.\n", gFailures);
  return 1;
}
