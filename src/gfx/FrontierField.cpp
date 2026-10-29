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
  std::vector<float> depth(dIn.size());
  for (int iz = 0; iz < nz; ++iz) {
    for (int ix = 0; ix < nx; ++ix) {
      const int i = iz * nx + ix;
      // Signed depth: positive inside, zero contour half a step out.
      depth[i] = field.Reached(ix, iz)
                     ? std::min(dIn[i], kMaxDepth) - 0.5f * field.step
                     : -(std::min(dOut[i], kMaxDepth) - 0.5f * field.step);
    }
  }

  // Separable box blur (radius 2, two passes) rounds off grid steps.
  std::vector<float> scratch(depth.size());
  for (int pass = 0; pass < 2; ++pass) {
    for (int iz = 0; iz < nz; ++iz) {
      for (int ix = 0; ix < nx; ++ix) {
        float sum = 0.0f;
        for (int k = -2; k <= 2; ++k) {
          const int jx = std::clamp(ix + k, 0, nx - 1);
          sum += sameLayer(ix, iz, jx, iz) ? depth[iz * nx + jx]
                                           : -0.5f * field.step;
        }
        scratch[iz * nx + ix] = sum / 5.0f;
      }
    }
    for (int iz = 0; iz < nz; ++iz) {
      for (int ix = 0; ix < nx; ++ix) {
        float sum = 0.0f;
        for (int k = -2; k <= 2; ++k) {
          const int jz = std::clamp(iz + k, 0, nz - 1);
          sum += sameLayer(ix, iz, ix, jz) ? scratch[jz * nx + ix]
                                           : -0.5f * field.step;
        }
        depth[iz * nx + ix] = sum / 5.0f;
      }
    }
  }

  // Blur is visual only: it must not change reachable topology. Without
  // restoring each sample's sign, a one-node-wide alley is averaged with
  // its surroundings until every value is negative and renders nothing.
  const float sampleDepth = 0.5f * field.step;
  for (int iz = 0; iz < nz; ++iz) {
    for (int ix = 0; ix < nx; ++ix) {
      float& value = depth[iz * nx + ix];
      if (field.Reached(ix, iz) && value <= 0.0f) value = sampleDepth;
      if (!field.Reached(ix, iz) && value >= 0.0f) value = -sampleDepth;
    }
  }
  return depth;
}

}  // namespace gfx
