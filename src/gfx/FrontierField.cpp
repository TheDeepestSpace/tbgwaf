#include "gfx/FrontierField.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <unordered_map>

#include "game/Types.h"

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

  // Two-sweep chamfer relaxation of seeded distances, within each layer.
  const auto chamferSweep = [&](std::vector<float>& d) {
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
  };
  // Chamfer distance (in world units) from each node to the nearest node
  // of the opposite reached/unreached class, within its own layer.
  const float inf = std::numeric_limits<float>::infinity();
  const auto chamfer = [&](bool target) {
    std::vector<float> d(static_cast<size_t>(nx) * nz, inf);
    for (int iz = 0; iz < nz; ++iz) {
      for (int ix = 0; ix < nx; ++ix) {
        if (reached(ix, iz) == target ||
            (!target && reached(ix, iz) && onLayerEdge(ix, iz))) {
          d[iz * nx + ix] = reached(ix, iz) && !target ? field.step : 0.0f;
        }
      }
    }
    chamferSweep(d);
    return d;
  };

  constexpr float kMaxDepth = 4.0f;
  constexpr float kObstacleInset = tactics::constants::kAgentRadius - 0.05f;
  const std::vector<float> dIn = chamfer(false);
  const std::vector<float> dOut = chamfer(true);

  // Distance to the nearest obstacle-blocked node (the reach computation
  // caches walkability around the frontier). Where it ties with dIn the
  // boundary is a wall rather than the budget running out, and the drawn
  // contour must follow the obstacle exactly instead of being rounded.
  const bool haveWalkable = field.walkable.size() == dIn.size();
  std::vector<float> dWall;
  if (haveWalkable) {
    dWall.assign(dIn.size(), inf);
    for (size_t i = 0; i < dWall.size(); ++i) {
      if (field.walkable[i] == 0) dWall[i] = 0.0f;
    }
    chamferSweep(dWall);
  }
  // 1 where the local boundary is an obstacle, 0 where it is the move
  // budget, blending over two steps where a wall meets the open arc.
  const auto wallWeight = [&](int i) {
    if (!haveWalkable || !std::isfinite(dWall[i])) return 0.0f;
    return std::clamp(1.0f - (dWall[i] - dIn[i]) / (2.0f * field.step), 0.0f, 1.0f);
  };
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
  const bool haveWallDist = field.wallDist.size() == dIn.size();
  const bool haveEdgeDist = field.edgeDist.size() == dIn.size();
  // Nodes whose trusted wall-exact depth is non-negative: the drawn line is
  // inset past the padded outline (just off the obstacle's real face), so
  // these unreached nodes legitimately sit on the inside of the contour.
  // The sign guarantee below must not push them back out.
  std::vector<char> insetPositive(dIn.size(), 0);
  for (int iz = 0; iz < nz; ++iz) {
    for (int ix = 0; ix < nx; ++ix) {
      const int i = iz * nx + ix;
      const float a = txx[i], b = txz[i], c = tzz[i];
      const float trace = a + c;
      const float wall = wallWeight(i);
      float value = depth[i];
      // Only near-contour values are visible (the fill fades out well
      // inside kMaxDepth); skip the sampling work there and where the wall
      // takes the whole blend below.
      if (wall < 1.0f && std::fabs(value) < 1.5f && trace > 1e-12f) {
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
          // corner. Straight stretches take the tangential average
          // (straightened); corners fall back to the unblurred grid field
          // instead of the blur's round-off.
          const float aniso = 2.0f * disc / trace;
          const float avg = rawAlong(ix, iz, -nzv / len, nxv / len);
          value = aniso * avg + (1.0f - aniso) * raw[i];
        }
      }
      // Level-edge-caused boundary (issue #292): where a node sits near a
      // measured level-edge line (cliff top/base, stair flank, deck
      // outline; see FillLevelEdgeDistances) and that exact distance agrees
      // with the grid depth, the contour follows the real edge geometry --
      // straight cliff lines -- instead of the grid rasterisation of the
      // layer step. Disagreement means the local boundary is something else
      // (the budget arc, or reach flowing through a stair/ramp opening in
      // the edge), and the grid depth stands. Unreached nodes take part
      // too: the rim nodes on a cliff's slope band that still count as the
      // reached strip's layer carry the sub-step tail of the exact depth,
      // and leaving them at grid values saw-toothed the line.
      float edgeExact = 0.0f;
      float edgeW = 0.0f;
      if (haveEdgeDist && std::isfinite(field.edgeDist[i])) {
        const float e = field.edgeDist[i] - kLevelEdgeInset;
        // The grid boundary at a level edge sits systematically inside the
        // geometric line (the heightfield rasterises the cliff as a one-cell
        // slope band the walk gate rejects), so the agreement window is
        // wider than the wall one and one-sided-friendly: full weight while
        // the grid depth is within two steps of the exact line, fading out
        // by 2.75 steps so reach that genuinely flows past the line (a
        // stair mouth, a ramp at grade) never snaps to it.
        // Where the boundary is wall-caused (terrain cliffs carry thin
        // nav-only wall obstacles whose padding keeps the grid reach up to
        // ~0.85 units from the step line) the window widens further: blocked
        // nodes right there mean it cannot be the budget arc snapping.
        const float tolerance = (2.75f + 1.5f * wall) * field.step;
        const float diff = std::fabs(e - raw[i]);
        if (diff <= tolerance) {
          edgeExact = e;
          edgeW = std::min(1.0f, (tolerance - diff) / (0.75f * field.step));
        }
      }
      // Wall-caused boundary, applied after every smoothing pass so none of
      // them can round it off: the exact node-to-outline distance where the
      // reach field recorded one, signed by reachability. Its zero contour
      // lies on the padded obstacle outline itself -- straight faces,
      // square corners, oblique walls exact. The grid chamfer depth (zero
      // contour half a step outside the last reached node, corners square
      // at grid resolution) stands in when the exact distance is missing or
      // disagrees with the grid, i.e. the local blocker is not a measured
      // outline (the window edge, or an obstacle on the far side of this
      // one) -- unless a level edge measured it above (a deck edge over
      // blocked ground reads as wall-bounded, but the deck outline is the
      // boundary that is actually drawn).
      if (edgeW >= 1.0f) {
        // A fully agreeing level edge outranks the wall outline: terrain
        // cliffs carry thin nav-only wall obstacles ON the step line, and
        // the padded-outline metric would put the line a fifth of a unit
        // past the visible cliff face instead of hugging it. The renderer's
        // layer-step cells interpolate against edgeDist, so this keeps the
        // two sides of every crossing on one metric.
        value = edgeExact;
        // The snapped band's unreached nodes (between the grid reach and
        // the drawn line) must survive the sign guarantee below, exactly
        // like the wall-inset band.
        if (edgeExact >= -1e-4f) insetPositive[i] = 1;
      } else if (wall > 0.0f) {
        float exact = edgeW > 0.0f ? edgeW * edgeExact + (1.0f - edgeW) * raw[i] : raw[i];
        if (haveWallDist && std::isfinite(field.wallDist[i])) {
          // wallDist is signed against the union of padded outlines, so a
          // nested inner outline (a pier shaft inside its footing) cannot
          // flip or shrink it; the agreement check below still drops nodes
          // whose nearest outline is not the local blocker.
          const float signedDist =
              std::clamp(field.wallDist[i], -kMaxDepth, kMaxDepth);
          if (std::fabs(signedDist - raw[i]) <= 1.5f * field.step) {
            // The measured outline is the navmesh's agent-padded one; draw the
            // line just off the obstacle's real face instead of a body radius
            // away from it.
            exact = signedDist + kObstacleInset;
            if (exact >= -1e-4f) insetPositive[i] = 1;
          }
        }
        value = wall * exact + (1.0f - wall) * value;
      } else if (edgeW > 0.0f) {
        value = edgeW * edgeExact + (1.0f - edgeW) * value;
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
  // within the half-step bias the contour carries by construction. Nodes in
  // the wall-inset band (trusted exact depth >= 0, and close enough to the
  // reach that they cannot be the middle of an unwalkable sliver between
  // two paddings) also float: the line is drawn kObstacleInset past the
  // padded outline there on purpose, and clamping them negative at some
  // grid phases but not others saw-toothed an otherwise straight wall line.
  const float kThin = 4.0f * field.step;
  for (int iz = 0; iz < nz; ++iz) {
    for (int ix = 0; ix < nx; ++ix) {
      const int i = iz * nx + ix;
      if (thickness[i] >= kThin) continue;
      if (reached(ix, iz)) {
        depth[i] = std::max(depth[i], sampleDepth);
      } else if (dOut[i] > 1.5f * field.step &&
                 !(insetPositive[i] &&
                   dOut[i] <= kObstacleInset + 1.5f * field.step)) {
        depth[i] = std::min(depth[i], -sampleDepth);
      }
    }
  }

  // Connectivity guarantee. Smoothing is free to round and straighten, but
  // it must not cut one reached region into pieces: a thin neck beside a
  // wide area is eroded by the wide-area blur, and two reached blocks that
  // meet only at a corner pinch off. Wherever the solid reach (8-connected,
  // per layer) is split into several positive pieces, raise the cheapest
  // chain of nodes (solid nodes first) joining them to just above zero.
  const size_t total = static_cast<size_t>(nx) * nz;
  constexpr int kDx4[4] = {-1, 1, 0, 0}, kDz4[4] = {0, 0, -1, 1};
  std::vector<int> solidComp(total, -1);
  int solidCount = 0;
  {
    std::vector<int> stack;
    for (size_t s = 0; s < total; ++s) {
      if (!solid[s] || solidComp[s] >= 0) continue;
      solidComp[s] = solidCount;
      stack.push_back(static_cast<int>(s));
      while (!stack.empty()) {
        const int cur = stack.back();
        stack.pop_back();
        const int cx = cur % nx, cz = cur / nx;
        for (int dz = -1; dz <= 1; ++dz) {
          for (int dx = -1; dx <= 1; ++dx) {
            const int jx = cx + dx, jz = cz + dz;
            if ((dx == 0 && dz == 0) || jx < 0 || jz < 0 || jx >= nx || jz >= nz) continue;
            const int j = jz * nx + jx;
            if (!solid[j] || solidComp[j] >= 0 || !sameLayer(cx, cz, jx, jz)) continue;
            solidComp[j] = solidCount;
            stack.push_back(j);
          }
        }
      }
      ++solidCount;
    }
  }
  const auto labelPositive = [&](std::vector<int>& label) {
    label.assign(total, -1);
    int count = 0;
    std::vector<int> stack;
    for (size_t s = 0; s < total; ++s) {
      if (depth[s] <= 0.0f || label[s] >= 0) continue;
      label[s] = count;
      stack.push_back(static_cast<int>(s));
      while (!stack.empty()) {
        const int cur = stack.back();
        stack.pop_back();
        const int cx = cur % nx, cz = cur / nx;
        for (int k = 0; k < 4; ++k) {
          const int jx = cx + kDx4[k], jz = cz + kDz4[k];
          if (jx < 0 || jz < 0 || jx >= nx || jz >= nz) continue;
          const int j = jz * nx + jx;
          if (depth[j] <= 0.0f || label[j] >= 0 || !sameLayer(cx, cz, jx, jz)) continue;
          label[j] = count;
          stack.push_back(j);
        }
      }
      ++count;
    }
    return count;
  };
  std::vector<int> label;
  for (int guard = 0; guard < 4 * static_cast<int>(total); ++guard) {
    labelPositive(label);
    // Find a solid component with two positive pieces: a node in the first
    // piece and the label of another piece in the same component.
    std::vector<int> firstLabel(solidCount, -1);
    int seedLabel = -1, seedComp = -1;
    for (size_t s = 0; s < total && seedLabel < 0; ++s) {
      if (!solid[s] || label[s] < 0 || solidComp[s] < 0) continue;
      int& f = firstLabel[solidComp[s]];
      if (f < 0) f = label[s];
      else if (f != label[s]) {
        seedLabel = f;
        seedComp = solidComp[s];
      }
    }
    if (seedLabel < 0) break;
    // 0-1 BFS from piece `seedLabel` to any node of another piece of this
    // component. Entering a solid node is free, any other node costs one.
    std::vector<int> cost(total, std::numeric_limits<int>::max()), prev(total, -1);
    std::deque<int> queue;
    for (size_t s = 0; s < total; ++s) {
      if (label[s] == seedLabel) {
        cost[s] = 0;
        queue.push_back(static_cast<int>(s));
      }
    }
    int hit = -1;
    while (!queue.empty()) {
      const int cur = queue.front();
      queue.pop_front();
      if (label[cur] >= 0 && label[cur] != seedLabel && solid[cur] &&
          solidComp[cur] == seedComp) {
        hit = cur;
        break;
      }
      const int cx = cur % nx, cz = cur / nx;
      for (int k = 0; k < 4; ++k) {
        const int jx = cx + kDx4[k], jz = cz + kDz4[k];
        if (jx < 0 || jz < 0 || jx >= nx || jz >= nz) continue;
        const int j = jz * nx + jx;
        if (!sameLayer(cx, cz, jx, jz)) continue;
        const int w = solid[j] ? 0 : 1;
        if (cost[cur] + w >= cost[j]) continue;
        cost[j] = cost[cur] + w;
        prev[j] = cur;
        if (w == 0) queue.push_front(j);
        else queue.push_back(j);
      }
    }
    if (hit < 0) {
      // Unreachable on this layer: give up on this component.
      for (size_t s = 0; s < total; ++s) {
        if (solidComp[s] == seedComp) solidComp[s] = -1;
      }
      continue;
    }
    for (int cur = hit; cur >= 0; cur = prev[cur]) {
      depth[cur] = std::max(depth[cur], sampleDepth);
    }
  }
  return depth;
}

std::vector<FrontierOutline> TraceFrontierOutline(const tactics::ReachField& field) {
  const int nx = field.nx, nz = field.nz;
  const size_t total = static_cast<size_t>(nx) * nz;
  if (nx <= 1 || nz <= 1 || field.step <= 0.0f || field.dist.size() != total) return {};
  const float step = field.step;
  constexpr float kLayerStep = 0.6f;
  const bool haveY = field.surfaceY.size() == total;
  const auto sameLayer = [&](int a, int b) {
    return !haveY || std::abs(field.surfaceY[a] - field.surfaceY[b]) <= kLayerStep;
  };
  // Same hairline rule as SmoothFrontierSignedDepth: only nodes inside a
  // fully reached 2x2 block of one layer count.
  std::vector<char> solid(total, 0);
  for (int iz = 0; iz + 1 < nz; ++iz) {
    for (int ix = 0; ix + 1 < nx; ++ix) {
      const int a = iz * nx + ix, b = a + 1, c = a + nx, d = c + 1;
      if (!field.Reached(ix, iz) || !field.Reached(ix + 1, iz) || !field.Reached(ix, iz + 1) ||
          !field.Reached(ix + 1, iz + 1) || !sameLayer(a, b) || !sameLayer(a, c) ||
          !sameLayer(a, d)) {
        continue;
      }
      solid[a] = solid[b] = solid[c] = solid[d] = 1;
    }
  }
  const auto nodeXZ = [&](int i) {
    return glm::vec2(field.minX + (i % nx) * step, field.minZ + (i / nx) * step);
  };

  // --- 1. Trace the mask outline: one vertex per (inside, outside) node
  // pair on a grid edge, linked cell by cell.
  struct Vertex {
    int in = -1, out = -1;
    int link[2] = {-1, -1};
    glm::vec2 p{0.0f};
    float y = 0.0f;
    int kind = 0;  // 0 free, 1 wall segment, 2 level-edge segment.
    int seg = -1;
    bool budget = false;
  };
  std::vector<Vertex> verts;
  std::unordered_map<long long, int> vertexOf;
  const auto vertexFor = [&](int in, int out) {
    const long long key = static_cast<long long>(in) * static_cast<long long>(total) + out;
    const auto it = vertexOf.find(key);
    if (it != vertexOf.end()) return it->second;
    Vertex v;
    v.in = in;
    v.out = out;
    verts.push_back(v);
    vertexOf.emplace(key, static_cast<int>(verts.size()) - 1);
    return static_cast<int>(verts.size()) - 1;
  };
  const auto link = [&](int a, int b) {
    if (a == b) return;
    for (int k = 0; k < 2; ++k) {
      if (verts[a].link[k] == b) return;
    }
    for (int k = 0; k < 2; ++k) {
      if (verts[a].link[k] < 0) {
        verts[a].link[k] = b;
        break;
      }
    }
    for (int k = 0; k < 2; ++k) {
      if (verts[b].link[k] < 0) {
        verts[b].link[k] = a;
        break;
      }
    }
  };
  for (int iz = 0; iz + 1 < nz; ++iz) {
    for (int ix = 0; ix + 1 < nx; ++ix) {
      const int corner[4] = {iz * nx + ix, iz * nx + ix + 1, (iz + 1) * nx + ix + 1,
                             (iz + 1) * nx + ix};
      bool used[4] = {false, false, false, false};
      for (int g0 = 0; g0 < 4; ++g0) {
        if (used[g0] || !solid[corner[g0]]) continue;
        bool in[4];
        int count = 0;
        for (int k = 0; k < 4; ++k) {
          in[k] = !used[k] && solid[corner[k]] && sameLayer(corner[g0], corner[k]);
          if (in[k]) {
            used[k] = true;
            ++count;
          }
        }
        if (count == 4) continue;
        // Crossings in polygon order; an inside run goes from an entering
        // crossing to the next exiting one (saddles have two runs).
        int cross[4];
        bool enters[4];
        int n = 0;
        for (int k = 0; k < 4; ++k) {
          const int k1 = (k + 1) % 4;
          if (in[k] == in[k1]) continue;
          cross[n] = in[k] ? vertexFor(corner[k], corner[k1]) : vertexFor(corner[k1], corner[k]);
          enters[n] = in[k1];
          ++n;
        }
        for (int k = 0; k < n; ++k) {
          if (enters[k]) link(cross[k], cross[(k + 1) % n]);
        }
      }
    }
  }
  if (verts.empty()) return {};

  // --- 2. Place each vertex.
  // Segment broadphase: buckets over the field window.
  const float cell = 1.0f;
  const float gx0 = field.minX - 1.0f, gz0 = field.minZ - 1.0f;
  const int gn = std::max(1, static_cast<int>(std::ceil(((nx - 1) * step + 2.0f) / cell)));
  const int gm = std::max(1, static_cast<int>(std::ceil(((nz - 1) * step + 2.0f) / cell)));
  struct Buckets {
    std::vector<std::vector<int>> cells;
  };
  const auto bucketize = [&](const std::vector<tactics::BoundarySeg>& segs, float reach) {
    Buckets b;
    b.cells.resize(static_cast<size_t>(gn) * gm);
    for (size_t s = 0; s < segs.size(); ++s) {
      const glm::vec2 lo = glm::min(segs[s].a, segs[s].b) - reach;
      const glm::vec2 hi = glm::max(segs[s].a, segs[s].b) + reach;
      const int x0 = static_cast<int>(std::floor((lo.x - gx0) / cell));
      const int x1 = static_cast<int>(std::floor((hi.x - gx0) / cell));
      const int z0 = static_cast<int>(std::floor((lo.y - gz0) / cell));
      const int z1 = static_cast<int>(std::floor((hi.y - gz0) / cell));
      for (int z = std::max(0, z0); z <= std::min(gm - 1, z1); ++z) {
        for (int x = std::max(0, x0); x <= std::min(gn - 1, x1); ++x) {
          b.cells[static_cast<size_t>(z) * gn + x].push_back(static_cast<int>(s));
        }
      }
    }
    return b;
  };
  // A vertex snaps to a wall when the padded outline passes within kWallTol
  // of it (the grid boundary straddles the outline, give or take the
  // hairline erosion), and to a level edge within kEdgeTol (the heightfield
  // rasterises a cliff as a slope band the walk gate rejects, so the grid
  // boundary sits up to a few samples off the geometric line).
  constexpr float kWallTol = 0.45f;
  constexpr float kEdgeTol = 0.9f;
  constexpr float kStoreyTolerance = 1.0f;
  const Buckets wallBuckets = bucketize(field.wallSegs, kWallTol);
  const Buckets edgeBuckets = bucketize(field.edgeSegs, kEdgeTol);
  const auto segDistance = [](const tactics::BoundarySeg& s, glm::vec2 p) {
    const glm::vec2 ab = s.b - s.a;
    const float t = std::clamp(glm::dot(p - s.a, ab) / std::max(glm::dot(ab, ab), 1e-12f),
                               0.0f, 1.0f);
    return glm::length(p - s.a - ab * t);
  };
  const bool haveElevated = field.elevated.size() == total;
  const bool haveWalkable = field.walkable.size() == total;
  // First segment index of each wall outline (segments of one outline are
  // consecutive), for the "is this edge buried inside another outline" test.
  std::vector<int> outlineBegin, outlineEnd;
  for (size_t k = 0; k < field.wallSegs.size(); ++k) {
    const int o = field.wallSegs[k].outline;
    if (o < 0) continue;
    if (static_cast<size_t>(o) >= outlineBegin.size()) {
      outlineBegin.resize(o + 1, -1);
      outlineEnd.resize(o + 1, -1);
    }
    if (outlineBegin[o] < 0) outlineBegin[o] = static_cast<int>(k);
    outlineEnd[o] = static_cast<int>(k) + 1;
  }
  const auto insideOutline = [&](int o, glm::vec2 p) {
    for (int k = outlineBegin[o]; k < outlineEnd[o]; ++k) {
      const tactics::BoundarySeg& e = field.wallSegs[k];
      const glm::vec2 ab = e.b - e.a, ap = p - e.a;
      if (ab.x * ap.y - ab.y * ap.x < 0.01f * glm::length(ab)) return false;
    }
    return true;
  };
  const auto nearest = [&](const std::vector<tactics::BoundarySeg>& segs, const Buckets& buckets,
                           glm::vec2 p, float y, bool elevated, bool wall, float tol) {
    const int x = static_cast<int>(std::floor((p.x - gx0) / cell));
    const int z = static_cast<int>(std::floor((p.y - gz0) / cell));
    int best = -1;
    if (x < 0 || z < 0 || x >= gn || z >= gm) return best;
    const std::vector<int>& bucket = buckets.cells[static_cast<size_t>(z) * gn + x];
    const auto applies = [&](const tactics::BoundarySeg& seg) {
      if (wall && seg.ground == elevated) return false;
      if (wall && seg.ground) return true;
      return y >= seg.yMin - kStoreyTolerance && y <= seg.yMax + kStoreyTolerance;
    };
    float bestD = tol;
    for (int s : bucket) {
      const tactics::BoundarySeg& seg = segs[s];
      if (!applies(seg)) continue;
      const float d = segDistance(seg, p);
      if (d >= bestD) continue;
      if (wall && seg.outline >= 0) {
        // Abutting / nested outlines (a cliff wall built from pieces, a pier
        // shaft in its footing): an edge buried inside another outline is
        // not part of the boundary of their union.
        const glm::vec2 ab = seg.b - seg.a;
        const float t = std::clamp(glm::dot(p - seg.a, ab) / std::max(glm::dot(ab, ab), 1e-12f),
                                   0.0f, 1.0f);
        const glm::vec2 out = glm::normalize(glm::vec2(ab.y, -ab.x));
        const glm::vec2 probe = seg.a + ab * t + out * 0.03f;
        bool buried = false;
        int last = -1;
        for (int other : bucket) {
          const tactics::BoundarySeg& o = segs[other];
          if (o.outline < 0 || o.outline == seg.outline || o.outline == last || !applies(o)) {
            continue;
          }
          last = o.outline;
          if (insideOutline(o.outline, probe)) {
            buried = true;
            break;
          }
        }
        if (buried) continue;
      }
      bestD = d;
      best = s;
    }
    return best;
  };
  // The line a snapped vertex is drawn on: point + unit direction.
  const auto drawLine = [&](const Vertex& v, glm::vec2* origin, glm::vec2* dir) {
    const tactics::BoundarySeg& seg = v.kind == 1 ? field.wallSegs[v.seg] : field.edgeSegs[v.seg];
    const glm::vec2 d = glm::normalize(seg.b - seg.a);
    glm::vec2 n(-d.y, d.x);  // Left of a -> b: into a padded outline.
    if (v.kind == 2 && glm::dot(nodeXZ(v.in) - seg.a, n) < 0.0f) n = -n;
    // Level edges are inset toward the reached side, walls past the outline.
    *origin = seg.a + n * seg.inset;
    *dir = d;
  };
  static const int kMoves[16][2] = {{1, 0},  {-1, 0}, {0, 1},  {0, -1}, {1, 1},  {1, -1},
                                    {-1, 1}, {-1, -1}, {2, 1},  {2, -1}, {-2, 1}, {-2, -1},
                                    {1, 2},  {1, -2}, {-1, 2}, {-1, -2}};
  for (Vertex& v : verts) {
    const glm::vec2 pin = nodeXZ(v.in), pout = nodeXZ(v.out);
    const glm::vec2 mid = 0.5f * (pin + pout);
    v.y = haveY ? field.surfaceY[v.in] : 0.0f;
    v.p = mid;
    const bool elevated = haveElevated && field.elevated[v.in];
    const bool layerStep = !sameLayer(v.in, v.out);
    const bool blocked = haveWalkable && field.walkable[v.out] == 0 && !elevated;
    // The budget ran out here if the outside node could have been walked to
    // from a reached neighbour but for the cost.
    float crossing = -1.0f;
    if (!layerStep && !blocked && !solid[v.out] && field.dist[v.in] <= field.budget) {
      float outDist = std::numeric_limits<float>::infinity();
      const int ox = v.out % nx, oz = v.out / nx;
      for (const auto& m : kMoves) {
        const int jx = ox + m[0], jz = oz + m[1];
        if (!field.Reached(jx, jz)) continue;
        const int j = jz * nx + jx;
        if (!sameLayer(j, v.out) && !sameLayer(j, v.in)) continue;
        outDist = std::min(outDist, field.dist[j] + step * std::hypot(static_cast<float>(m[0]),
                                                                    static_cast<float>(m[1])));
      }
      const int outX = v.out % nx, outZ = v.out / nx;
      if (field.Reached(outX, outZ)) {
        // Reached but dropped by the hairline rule: the arc passes just
        // beyond it.
        outDist = std::max(field.dist[v.out], field.dist[v.in]);
        crossing = 1.0f;
      } else if (outDist > field.budget && outDist - field.dist[v.in] <= 1.5f * step) {
        crossing = std::clamp((field.budget - field.dist[v.in]) /
                                  std::max(outDist - field.dist[v.in], 1e-4f),
                              0.0f, 1.0f);
      }
    }
    if (crossing >= 0.0f) {
      v.budget = true;
      v.p = pin + (pout - pin) * crossing;
      continue;
    }
    int wall = -1, edge = -1;
    if (!field.wallSegs.empty()) {
      wall = nearest(field.wallSegs, wallBuckets, mid, v.y, elevated, true, kWallTol);
    }
    if (!field.edgeSegs.empty()) {
      edge = nearest(field.edgeSegs, edgeBuckets, mid, v.y, elevated, false, kEdgeTol);
    }
    if (wall >= 0 && edge >= 0) {
      // A layer step is a level edge first; an obstacle-blocked node a wall
      // first; otherwise whichever line is (relatively) nearer.
      if (layerStep) wall = -1;
      else if (blocked) edge = -1;
      else if (segDistance(field.wallSegs[wall], mid) / kWallTol <=
               segDistance(field.edgeSegs[edge], mid) / kEdgeTol) edge = -1;
      else wall = -1;
    }
    if (wall >= 0) {
      v.kind = 1;
      v.seg = wall;
    } else if (edge >= 0) {
      v.kind = 2;
      v.seg = edge;
    }
  }

  // --- 3. Chain into loops and straighten.
  std::vector<FrontierOutline> outlines;
  std::vector<char> visited(verts.size(), 0);
  const auto walk = [&](int start, std::vector<int>* chain) {
    // Returns true when the chain closes on itself.
    int prev = -1, cur = start;
    while (cur >= 0 && !visited[cur]) {
      visited[cur] = 1;
      chain->push_back(cur);
      int next = verts[cur].link[0] != prev ? verts[cur].link[0] : verts[cur].link[1];
      if (next == prev && verts[cur].link[0] == verts[cur].link[1]) next = -1;
      prev = cur;
      cur = next;
    }
    return cur == start && chain->size() > 2;
  };
  const auto sameLine = [&](const Vertex& a, const Vertex& b) {
    return a.kind == b.kind && a.seg == b.seg;
  };
  for (int pass = 0; pass < 2; ++pass) {
    for (int s = 0; s < static_cast<int>(verts.size()); ++s) {
      if (visited[s]) continue;
      // Open chains first (start from their ends), then closed loops.
      const bool isEnd = verts[s].link[0] < 0 || verts[s].link[1] < 0;
      if (pass == 0 && !isEnd) continue;
      std::vector<int> chain;
      const bool closed = walk(s, &chain);
      const int n = static_cast<int>(chain.size());
      if (n < 2) continue;
      std::vector<Vertex> loop(n);
      for (int k = 0; k < n; ++k) loop[k] = verts[chain[k]];
      const auto at = [&](int k) -> Vertex& { return loop[((k % n) + n) % n]; };
      const auto valid = [&](int k) { return closed || (k >= 0 && k < n); };

      // 3a. Flicker repair: a short stretch (up to three vertices) of
      // anything else between two stretches on one line joins that line if
      // it is close to it -- one wall must not alternate snapped/unsnapped.
      for (int k = 0; k < n; ++k) {
        if (loop[k].kind == 0) continue;
        for (int gap = 1; gap <= 3; ++gap) {
          const int far = k + gap + 1;
          if (!valid(far) || !sameLine(loop[k], at(far))) continue;
          bool any = false, ok = true;
          glm::vec2 origin, dir;
          drawLine(loop[k], &origin, &dir);
          const tactics::BoundarySeg& seg =
              loop[k].kind == 1 ? field.wallSegs[loop[k].seg] : field.edgeSegs[loop[k].seg];
          for (int j = k + 1; j < far; ++j) {
            if (sameLine(at(j), loop[k])) continue;
            any = true;
            const glm::vec2 mid = 0.5f * (nodeXZ(at(j).in) + nodeXZ(at(j).out));
            const glm::vec2 rel = mid - seg.a;
            ok &= std::fabs(rel.x * dir.y - rel.y * dir.x) <=
                  1.5f * (loop[k].kind == 1 ? kWallTol : kEdgeTol);
          }
          if (!any || !ok) continue;
          for (int j = k + 1; j < far; ++j) {
            at(j).kind = loop[k].kind;
            at(j).seg = loop[k].seg;
            at(j).budget = false;
          }
          break;
        }
      }
      // 3b. Project snapped vertices onto their line.
      for (Vertex& v : loop) {
        if (v.kind == 0) continue;
        glm::vec2 origin, dir;
        drawLine(v, &origin, &dir);
        const glm::vec2 mid = 0.5f * (nodeXZ(v.in) + nodeXZ(v.out));
        v.p = origin + dir * glm::dot(mid - origin, dir);
      }
      // 3c. Smooth the free stretches (budget arc, anything unmeasured),
      // with snapped vertices and open ends pinned.
      {
        std::vector<glm::vec2> next(n);
        for (int iter = 0; iter < 10; ++iter) {
          for (int k = 0; k < n; ++k) {
            next[k] = loop[k].p;
            if (loop[k].kind != 0 || !valid(k - 1) || !valid(k + 1)) continue;
            next[k] = 0.25f * at(k - 1).p + 0.5f * loop[k].p + 0.25f * at(k + 1).p;
          }
          for (int k = 0; k < n; ++k) loop[k].p = next[k];
        }
      }
      // 3d. Emit, replacing the joint between two different lines with
      // their intersection (a sharp corner) and trimming any overshoot.
      std::vector<char> drop(n, 0), hasCorner(n, 0);
      std::vector<glm::vec2> corner(n);
      const auto lineDir = [&](const Vertex& v) {
        const tactics::BoundarySeg& seg =
            v.kind == 1 ? field.wallSegs[v.seg] : field.edgeSegs[v.seg];
        return glm::normalize(seg.b - seg.a);
      };
      for (int k = 0; k < n; ++k) {
        if (!valid(k + 1)) break;
        const Vertex& a = loop[k];
        const Vertex& b = at(k + 1);
        if (a.kind == 0 || b.kind == 0 || sameLine(a, b)) continue;
        // Snapped points already sit on their (inset) lines.
        const glm::vec2 da = lineDir(a), db = lineDir(b);
        const float det = da.x * db.y - da.y * db.x;
        if (std::fabs(det) < 0.1f) continue;  // Near-parallel: plain join.
        const glm::vec2 r = b.p - a.p;
        const glm::vec2 x = a.p + da * ((r.x * db.y - r.y * db.x) / det);
        if (glm::length(x - a.p) > 1.0f || glm::length(x - b.p) > 1.0f) continue;
        hasCorner[k] = 1;
        corner[k] = x;
        // Walking away from the joint on each side, drop the vertices of
        // that side's line lying beyond the corner (overshoot). A stretch's
        // only vertex is always replaced by the corner.
        for (int side = 0; side < 2; ++side) {
          const int origin = side == 0 ? k : k + 1;
          const int dir = side == 0 ? -1 : 1;
          const Vertex& first = at(origin);
          for (int j = 0; j < 3; ++j) {
            const int idx = origin + dir * j;
            if (!valid(idx) || !sameLine(at(idx), first)) break;
            const int beyond = idx + dir;
            if (!valid(beyond) || !sameLine(at(beyond), first)) {
              if (j == 0) drop[((idx % n) + n) % n] = 1;
              break;
            }
            // Travelling away from the joint, at(idx) -> at(beyond): the
            // vertex overshoots if the corner lies further that way.
            if (glm::dot(x - at(idx).p, at(beyond).p - at(idx).p) <= 1e-5f) break;
            drop[((idx % n) + n) % n] = 1;
          }
        }
      }
      struct Out {
        glm::vec2 p;
        float y;
      };
      std::vector<Out> out;
      out.reserve(n + 8);
      for (int k = 0; k < n; ++k) {
        if (!drop[k]) out.push_back({loop[k].p, loop[k].y});
        if (hasCorner[k]) out.push_back({corner[k], loop[k].y});
      }
      FrontierOutline outline;
      outline.closed = closed;
      outline.points.reserve(out.size());
      for (const Out& o : out) outline.points.emplace_back(o.p.x, o.y, o.p.y);
      outlines.push_back(std::move(outline));
    }
  }
  return outlines;
}

std::vector<glm::vec2> FrontierCoverStyle(const tactics::ReachField& field,
                                          const std::vector<tactics::Obstacle>& obstacles,
                                          float reach) {
  const int nx = field.nx, nz = field.nz;
  if (nx <= 0 || nz <= 0 || field.step <= 0.0f) return {};
  // Only blockers near the field can matter; the full list can be huge.
  const float margin = reach + tactics::kCoverEdgeReach + 1.0f;
  const float x0 = field.minX - margin, x1 = field.minX + nx * field.step + margin;
  const float z0 = field.minZ - margin, z1 = field.minZ + nz * field.step + margin;
  std::vector<tactics::Obstacle> nearby;
  for (const tactics::Obstacle& o : obstacles) {
    const tactics::AABB& b = o.bounds;
    if (b.max.x >= x0 && b.min.x <= x1 && b.max.z >= z0 && b.min.z <= z1) nearby.push_back(o);
  }
  std::vector<glm::vec2> style(static_cast<size_t>(nx) * nz, glm::vec2(0.0f));
  for (int iz = 0; iz < nz; ++iz) {
    for (int ix = 0; ix < nx; ++ix) {
      glm::vec3 p = field.Node(ix, iz);
      if (!field.Reached(ix, iz)) {
        // An unreached node (a wall and its navmesh padding) still corners
        // the boundary cells around it, so it needs a style too; sample it
        // at a reached neighbour's height so a node beside a deck never
        // reads the floor below. No reached neighbour at all means no
        // drawn cell touches the node: leave it unstyled.
        bool found = false;
        for (int dz = -1; dz <= 1 && !found; ++dz) {
          for (int dx = -1; dx <= 1 && !found; ++dx) {
            if (field.Reached(ix + dx, iz + dz)) {
              p.y = field.Node(ix + dx, iz + dz).y;
              found = true;
            }
          }
        }
        if (!found) continue;
      }
      style[iz * nx + ix] = tactics::CoverZoneWeights(nearby, p, reach);
    }
  }
  return style;
}

}  // namespace gfx
