#include "gfx/FrontierField.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace gfx {

std::vector<float> SmoothFrontierSignedDepth(const tactics::ReachField& field) {
  const int nx = field.nx, nz = field.nz;
  if (nx <= 0 || nz <= 0 || field.step <= 0.0f ||
      field.dist.size() != static_cast<size_t>(nx) * nz) {
    return {};
  }

  // Nodes whose surface heights differ by more than a step belong to
  // different walk layers. Distances and blur must not cross between them.
  constexpr float kLayerStep = 0.6f;
  const auto sameLayer = [&](int ax, int az, int bx, int bz) {
    if (field.surfaceY.size() != static_cast<size_t>(nx) * nz) return true;
    return std::abs(field.surfaceY[az * nx + ax] - field.surfaceY[bz * nx + bx]) <=
           kLayerStep;
  };
  // A node on its layer's edge counts as touching the unreached class one
  // step away, just like a reached ground node beside an obstacle.
  const auto onLayerEdge = [&](int ix, int iz) {
    constexpr int kDx[4] = {-1, 1, 0, 0}, kDz[4] = {0, 0, -1, 1};
    for (int k = 0; k < 4; ++k) {
      const int jx = ix + kDx[k], jz = iz + kDz[k];
      if (jx < 0 || jz < 0 || jx >= nx || jz >= nz) continue;
      if (!sameLayer(ix, iz, jx, jz)) return true;
    }
    return false;
  };

  // Chamfer distance (in world units) from each node to the nearest node
  // of the opposite reached/unreached class, within its own layer.
  const auto chamfer = [&](bool target) {
    const float inf = std::numeric_limits<float>::infinity();
    std::vector<float> d(static_cast<size_t>(nx) * nz, inf);
    for (int iz = 0; iz < nz; ++iz) {
      for (int ix = 0; ix < nx; ++ix) {
        if (field.Reached(ix, iz) == target ||
            (!target && field.Reached(ix, iz) && onLayerEdge(ix, iz))) {
          d[iz * nx + ix] = field.Reached(ix, iz) && !target ? field.step : 0.0f;
        }
      }
    }
    const float s = field.step, sd = field.step * 1.41421356f;
    const auto relax = [&](int ix, int iz, int dx, int dz, float weight) {
      const int jx = ix + dx, jz = iz + dz;
      if (jx < 0 || jz < 0 || jx >= nx || jz >= nz) return;
      if (!sameLayer(ix, iz, jx, jz)) return;
      float& value = d[iz * nx + ix];
      value = std::min(value, d[jz * nx + jx] + weight);
    };
    for (int iz = 0; iz < nz; ++iz) {
      for (int ix = 0; ix < nx; ++ix) {
        relax(ix, iz, -1, 0, s);
        relax(ix, iz, 0, -1, s);
        relax(ix, iz, -1, -1, sd);
        relax(ix, iz, 1, -1, sd);
      }
    }
    for (int iz = nz - 1; iz >= 0; --iz) {
      for (int ix = nx - 1; ix >= 0; --ix) {
        relax(ix, iz, 1, 0, s);
        relax(ix, iz, 0, 1, s);
        relax(ix, iz, 1, 1, sd);
        relax(ix, iz, -1, 1, sd);
      }
    }
    return d;
  };

  constexpr float kMaxDepth = 4.0f;
  const std::vector<float> dIn = chamfer(false);
  const std::vector<float> dOut = chamfer(true);
  std::vector<float> raw(dIn.size());
  for (int iz = 0; iz < nz; ++iz) {
    for (int ix = 0; ix < nx; ++ix) {
      const int i = iz * nx + ix;
      // Signed depth: positive inside, zero contour half a step out.
      raw[i] = field.Reached(ix, iz)
                   ? std::min(dIn[i], kMaxDepth) - 0.5f * field.step
                   : -(std::min(dOut[i], kMaxDepth) - 0.5f * field.step);
    }
  }

  // Separable box blur rounds off grid steps. Out-of-layer taps count as
  // just outside the contour.
  const auto blurField = [&](int radius, int passes) {
    std::vector<float> out = raw;
    std::vector<float> tmp(out.size());
    for (int pass = 0; pass < passes; ++pass) {
      for (int iz = 0; iz < nz; ++iz) {
        for (int ix = 0; ix < nx; ++ix) {
          float sum = 0.0f;
          for (int k = -radius; k <= radius; ++k) {
            const int jx = std::clamp(ix + k, 0, nx - 1);
            sum += sameLayer(ix, iz, jx, iz) ? out[iz * nx + jx] : -0.5f * field.step;
          }
          tmp[iz * nx + ix] = sum / (2 * radius + 1);
        }
      }
      for (int iz = 0; iz < nz; ++iz) {
        for (int ix = 0; ix < nx; ++ix) {
          float sum = 0.0f;
          for (int k = -radius; k <= radius; ++k) {
            const int jz = std::clamp(iz + k, 0, nz - 1);
            sum += sameLayer(ix, iz, ix, jz) ? tmp[jz * nx + ix] : -0.5f * field.step;
          }
          out[iz * nx + ix] = sum / (2 * radius + 1);
        }
      }
    }
    return out;
  };
  constexpr int kBlurRadius = 2;
  constexpr int kBlurPasses = 2;
  // Increasingly strong smoothing; level 0 is the grid-exact field.
  const std::vector<float> levels[4] = {raw, blurField(1, 1), blurField(1, 2),
                                        blurField(kBlurRadius, kBlurPasses)};
  std::vector<float> depth = levels[3];
  std::vector<float> scratch(depth.size());

  // The full blur erodes anything under about five samples across -- a
  // ledge along a cliff wall, a sidewalk strip between a building and a
  // drop -- until nothing is drawn. Measure each node's local thickness as
  // the deepest inside-distance within the blur's support (within its own
  // layer) and use weaker smoothing where that is small.
  constexpr int kSupport = kBlurRadius * kBlurPasses;
  std::vector<float> thickness(dIn.size(), 0.0f);
  for (int iz = 0; iz < nz; ++iz) {
    for (int ix = 0; ix < nx; ++ix) {
      float best = 0.0f;
      for (int k = -kSupport; k <= kSupport; ++k) {
        const int jx = ix + k;
        if (jx < 0 || jx >= nx || !sameLayer(ix, iz, jx, iz)) continue;
        if (field.Reached(jx, iz)) best = std::max(best, dIn[iz * nx + jx]);
      }
      scratch[iz * nx + ix] = best;
    }
  }
  for (int iz = 0; iz < nz; ++iz) {
    for (int ix = 0; ix < nx; ++ix) {
      float best = 0.0f;
      for (int k = -kSupport; k <= kSupport; ++k) {
        const int jz = iz + k;
        if (jz < 0 || jz >= nz || !sameLayer(ix, iz, ix, jz)) continue;
        best = std::max(best, scratch[jz * nx + ix]);
      }
      thickness[iz * nx + ix] = best;
    }
  }
  // Pick the smoothing strength from the local thickness (in samples),
  // interpolating between neighbouring levels so the outline never jumps
  // between a grid-exact and a rounded stretch. Thin bands get the
  // strongest blur they can take, rather than none, so their edges are
  // not stair-stepped either.
  const float sampleDepth = 0.1f * field.step;
  for (int iz = 0; iz < nz; ++iz) {
    for (int ix = 0; ix < nx; ++ix) {
      const int i = iz * nx + ix;
      const float level = std::clamp(thickness[i] / field.step - 1.0f, 0.0f, 3.0f);
      const int lo = std::min(static_cast<int>(level), 2);
      const float f = level - lo;
      float value = levels[lo][i] + (levels[lo + 1][i] - levels[lo][i]) * f;
      // Smoothing is visual only and must not change what is reachable;
      // outside the fully blurred zone a small constant margin keeps the outline smooth where a thin band
      // would otherwise be eroded across zero.
      if (level < 3.0f) {
        if (field.Reached(ix, iz)) value = std::max(value, sampleDepth);
        else value = std::min(value, -sampleDepth);
      }
      depth[i] = value;
    }
  }
  return depth;
}

}  // namespace gfx
