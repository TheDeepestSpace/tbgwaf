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

void TestNarrowReachableBandSurvivesSmoothing() {
  tactics::ReachField field;
  field.step = 0.25f;
  field.budget = 1.0f;
  field.nx = field.nz = 9;
  field.dist.assign(81, std::numeric_limits<float>::infinity());
  field.surfaceY.assign(81, 0.0f);

  // A one-sample-wide reachable alley. Two radius-2 blur passes used to
  // average this entire band negative, so no fill or outline was emitted.
  for (int ix = 1; ix < field.nx - 1; ++ix) field.dist[4 * field.nx + ix] = 0.0f;

  const std::vector<float> depth = gfx::SmoothFrontierSignedDepth(field);
  CHECK(depth.size() == field.dist.size());
  for (int iz = 0; iz < field.nz; ++iz) {
    for (int ix = 0; ix < field.nx; ++ix) {
      const bool reached = field.Reached(ix, iz);
      CHECK(reached ? depth[iz * field.nx + ix] > 0.0f
                    : depth[iz * field.nx + ix] < 0.0f);
    }
  }
}

}  // namespace

int main() {
  TestNarrowReachableBandSurvivesSmoothing();
  if (gFailures == 0) {
    std::printf("All frontier field tests passed.\n");
    return 0;
  }
  std::fprintf(stderr, "%d check(s) failed.\n", gFailures);
  return 1;
}
