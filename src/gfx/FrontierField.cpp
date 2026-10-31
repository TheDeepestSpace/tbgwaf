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
  // The grid-exact reach can contain lines one node wide: a route that grazes
  // an obstacle's padded edge reaches the nodes exactly along it and nothing
  // beside them. Drawn, that is a hairline strip across the ground, so only
  // nodes inside a fully reached 2x2 block of one layer count as reached.
  std::vector<char> solid(static_cast<size_t>(nx) * nz, 0);
  for (int iz = 0; iz + 1 < nz; ++iz) {
    for (int ix = 0; ix + 1 < nx; ++ix) {
      if (!field.Reached(ix, iz) || !field.Reached(ix + 1, iz) ||
          !field.Reached(ix, iz + 1) || !field.Reached(ix + 1, iz + 1) ||
          !sameLayer(ix, iz, ix + 1, iz) || !sameLayer(ix, iz, ix, iz + 1) ||
          !sameLayer(ix, iz, ix + 1, iz + 1)) {
        continue;
      }
      solid[iz * nx + ix] = solid[iz * nx + ix + 1] = 1;
      solid[(iz + 1) * nx + ix] = solid[(iz + 1) * nx + ix + 1] = 1;
    }
  }
  const auto reached = [&](int ix, int iz) { return solid[iz * nx + ix] != 0; };
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
        if (reached(ix, iz) == target ||
            (!target && reached(ix, iz) && onLayerEdge(ix, iz))) {
          d[iz * nx + ix] = reached(ix, iz) && !target ? field.step : 0.0f;
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
      raw[i] = reached(ix, iz)
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
  const std::vector<float> blurred = blurField(kBlurRadius, kBlurPasses);
  std::vector<float> depth = blurred;
  std::vector<float> scratch(depth.size());

  // The full blur erodes anything under about five samples across -- a
  // ledge along a cliff wall, a sidewalk strip between a building and a
  // drop -- until nothing is drawn. Measure each node's local thickness as
  // the deepest inside-distance within the blur's support (within its own
  // layer); where that is small the result below leans on the anisotropic
  // average instead, and the final sign guarantees apply.
  constexpr int kSupport = kBlurRadius * kBlurPasses;
  std::vector<float> thickness(dIn.size(), 0.0f);
  for (int iz = 0; iz < nz; ++iz) {
    for (int ix = 0; ix < nx; ++ix) {
      float best = 0.0f;
      for (int k = -kSupport; k <= kSupport; ++k) {
        const int jx = ix + k;
        if (jx < 0 || jx >= nx || !sameLayer(ix, iz, jx, iz)) continue;
        if (reached(jx, iz)) best = std::max(best, dIn[iz * nx + jx]);
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
  // An oblique straight edge -- a building's padded side, a cliff line, a
  // ramp's flank -- rasterises to a staircase in the node classification.
  // The isotropic blur straightens it in wide areas but erodes thin bands,
  // and any weaker fallback inherits the staircase as a wavy outline. So
  // smooth anisotropically instead: average `raw` only along the
  // boundary's local tangent, which straightens the staircase (bilinear
  // samples taken along the true edge direction recover its sub-sample
  // position) while never mixing across a thin band's profile.
  //
  // The tangent comes from the structure tensor of the blurred field over
  // a small window: unlike the plain gradient it stays well-defined on a
  // band's crest, where the gradient itself vanishes.
  const auto tensorBlur = [&](std::vector<float>& v) {
    constexpr int kR = 2;
    for (int iz = 0; iz < nz; ++iz) {
      for (int ix = 0; ix < nx; ++ix) {
        float sum = 0.0f;
        int count = 0;
        for (int k = -kR; k <= kR; ++k) {
          const int jx = std::clamp(ix + k, 0, nx - 1);
          if (!sameLayer(ix, iz, jx, iz)) continue;
          sum += v[iz * nx + jx];
          ++count;
        }
        scratch[iz * nx + ix] = sum / count;
      }
    }
    for (int iz = 0; iz < nz; ++iz) {
      for (int ix = 0; ix < nx; ++ix) {
        float sum = 0.0f;
        int count = 0;
        for (int k = -kR; k <= kR; ++k) {
          const int jz = std::clamp(iz + k, 0, nz - 1);
          if (!sameLayer(ix, iz, ix, jz)) continue;
          sum += scratch[jz * nx + ix];
          ++count;
        }
        v[iz * nx + ix] = sum / count;
      }
    }
  };
  std::vector<float> txx(depth.size()), txz(depth.size()), tzz(depth.size());
  for (int iz = 0; iz < nz; ++iz) {
    for (int ix = 0; ix < nx; ++ix) {
      const int i = iz * nx + ix;
      const auto at = [&](int jx, int jz) {
        jx = std::clamp(jx, 0, nx - 1);
        jz = std::clamp(jz, 0, nz - 1);
        return sameLayer(ix, iz, jx, jz) ? blurred[jz * nx + jx] : blurred[i];
      };
      const float gx = at(ix + 1, iz) - at(ix - 1, iz);
      const float gz = at(ix, iz + 1) - at(ix, iz - 1);
      txx[i] = gx * gx;
      txz[i] = gx * gz;
      tzz[i] = gz * gz;
    }
  }
  tensorBlur(txx);
  tensorBlur(txz);
  tensorBlur(tzz);

  // Average of `raw` along the tangent. Bilinear corners on another layer
  // count as just outside the contour, like the blur's taps: along a cliff
  // edge the average then grades with how much of the staircase each
  // sample row covers, which is exactly the edge's sub-sample position.
  constexpr int kTangentRadius = 6;
  const auto rawAlong = [&](int cx, int cz, float tx, float tz) {
    const int c = cz * nx + cx;
    const auto sampleAt = [&](float fx, float fz) {
      const int ix = std::clamp(static_cast<int>(std::floor(fx)), 0, nx - 2);
      const int iz = std::clamp(static_cast<int>(std::floor(fz)), 0, nz - 2);
      const float ux = std::clamp(fx - ix, 0.0f, 1.0f);
      const float uz = std::clamp(fz - iz, 0.0f, 1.0f);
      const auto corner = [&](int jx, int jz) {
        return sameLayer(cx, cz, jx, jz) ? raw[jz * nx + jx] : -0.5f * field.step;
      };
      return (corner(ix, iz) * (1 - ux) + corner(ix + 1, iz) * ux) * (1 - uz) +
             (corner(ix, iz + 1) * (1 - ux) + corner(ix + 1, iz + 1) * ux) * uz;
    };
    float sum = raw[c] * (kTangentRadius + 1);
    float weight = static_cast<float>(kTangentRadius + 1);
    for (int k = 1; k <= kTangentRadius; ++k) {
      const float w = static_cast<float>(kTangentRadius + 1 - k);
      sum += w * sampleAt(cx + tx * k, cz + tz * k);
      sum += w * sampleAt(cx - tx * k, cz - tz * k);
      weight += 2.0f * w;
    }
    return sum / weight;
  };
  const float sampleDepth = 0.1f * field.step;
  for (int iz = 0; iz < nz; ++iz) {
    for (int ix = 0; ix < nx; ++ix) {
      const int i = iz * nx + ix;
      const float a = txx[i], b = txz[i], c = tzz[i];
      const float trace = a + c;
      float value = depth[i];
      // Only near-contour values are visible (the fill fades out well
      // inside kMaxDepth); skip the sampling work elsewhere.
      if (std::fabs(value) < 1.5f && trace > 1e-12f) {
        const float disc = std::sqrt(0.25f * (a - c) * (a - c) + b * b);
        // Dominant eigenvector of the tensor is the boundary normal.
        float nxv = b, nzv = 0.5f * trace + disc - a;
        if (nxv * nxv + nzv * nzv < 1e-20f) {
          nxv = 0.5f * trace + disc - c;
          nzv = b;
        }
        const float len = std::sqrt(nxv * nxv + nzv * nzv);
        if (len > 1e-10f) {
          // Anisotropy in [0, 1]: 1 on a straight edge or band, 0 at a
          // corner. Corners keep the isotropic blur (rounded), straight
          // stretches take the tangential average (straightened).
          const float aniso = 2.0f * disc / trace;
          const float avg = rawAlong(ix, iz, -nzv / len, nxv / len);
          value = aniso * avg + (1.0f - aniso) * value;
        }
      }
      depth[i] = value;
    }
  }

  // Smoothing is visual only and must not drift from what is reachable.
  // Outside the fully blurred wide zone (where convex corner samples are
  // deliberately rounded off), every reached node stays positive -- a thin
  // band can never be smoothed away -- and every node more than one sample
  // outside the reach stays negative. The first ring of outside nodes
  // floats free: a straight contour through an oblique staircase has to be
  // able to pass just outside the stair-corner nodes, and that ring is
  // within the half-step bias the contour carries by construction.
  const float kThin = 4.0f * field.step;
  for (int iz = 0; iz < nz; ++iz) {
    for (int ix = 0; ix < nx; ++ix) {
      const int i = iz * nx + ix;
      if (thickness[i] >= kThin) continue;
      if (reached(ix, iz)) depth[i] = std::max(depth[i], sampleDepth);
      else if (dOut[i] > 1.5f * field.step) depth[i] = std::min(depth[i], -sampleDepth);
    }
  }
  return depth;
}

}  // namespace gfx
