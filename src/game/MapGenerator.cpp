#include "game/MapGenerator.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

namespace tactics {
namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kSidewalkHeight = 0.12f;  // Curb: purely visual, well under a step.

// std::mt19937 output is fully specified by the standard, but the
// <random> distributions are not, so draw values by hand to keep a seed's
// output identical across standard libraries.
class Rng {
 public:
  explicit Rng(uint32_t seed) : engine_(seed) {}
  int Int(int lo, int hi) { return lo + static_cast<int>(engine_() % static_cast<uint32_t>(hi - lo + 1)); }
  float Unit01() { return static_cast<float>(engine_() >> 8) / 16777216.0f; }
  float Float(float lo, float hi) { return lo + (hi - lo) * Unit01(); }
  bool Chance(float p) { return Unit01() < p; }

 private:
  std::mt19937 engine_;
};

void AddBuilding(Scene* scene, float x0, float z0, float x1, float z1, float height) {
  scene->obstacles.push_back(
      Obstacle{AABB{glm::vec3(x0, 0.0f, z0), glm::vec3(x1, height, z1)}, /*climbable=*/false});
}

// Splits [a, b] into `count` buildings. Junction i (between building i and
// i+1) is a gap of `gapWidth` if gaps[i], else the footprints touch.
// `startGap`/`endGap` shorten the run to leave an opening at that end.
// Calls emit(lo, hi) per building.
template <typename Emit>
void SplitRun(float a, float b, const std::vector<bool>& gaps, float gapWidth, bool startGap,
              bool endGap, Emit emit) {
  if (startGap) a += gapWidth;
  if (endGap) b -= gapWidth;
  const int count = static_cast<int>(gaps.size()) + 1;
  int gapCount = 0;
  for (bool g : gaps) gapCount += g ? 1 : 0;
  const float buildable = (b - a) - gapCount * gapWidth;
  const float each = buildable / static_cast<float>(count);
  float cursor = a;
  for (int i = 0; i < count; ++i) {
    emit(cursor, cursor + each);
    cursor += each;
    if (i < count - 1 && gaps[i]) cursor += gapWidth;
  }
}

// One axis of the street grid: `n` blocks separated by n+1 straight streets
// that fill exactly `total`. Rim streets keep the nominal width; interior
// streets vary. Block sizes vary too, but never drop below the floor.
struct Span { float start, size; };
std::vector<Span> LayoutAxis(int n, float total, const MapGeneratorConfig& c, Rng* rng) {
  std::vector<float> streets(n + 1, c.streetWidth);
  float streetSum = 2.0f * c.streetWidth;
  for (int i = 1; i < n; ++i) {
    streets[i] = c.streetWidth * rng->Float(c.streetWidthMin, c.streetWidthMax);
    streetSum += streets[i];
  }
  const float blockSum = total - streetSum;
  const float extra = std::max(0.0f, blockSum - n * c.minBlockSize);
  std::vector<float> weights(n);
  float weightSum = 0.0f;
  for (int i = 0; i < n; ++i) weightSum += weights[i] = rng->Float(0.5f, 1.5f);
  std::vector<Span> spans;
  float cursor = -total * 0.5f;
  for (int i = 0; i < n; ++i) {
    cursor += streets[i];
    const float size = extra > 0.0f ? c.minBlockSize + extra * weights[i] / weightSum
                                    : blockSum / static_cast<float>(n);
    spans.push_back({cursor, size});
    cursor += size;
  }
  return spans;
}

struct Layout {
  std::vector<Span> xs, zs;
};

Layout LayoutCity(const MapGeneratorConfig& c, Rng* rng) {
  const float total = 2.0f * UrbanMapHalfExtent(c);
  Layout layout;
  layout.xs = LayoutAxis(c.blocksX, total, c, rng);
  layout.zs = LayoutAxis(c.blocksZ, total, c, rng);
  return layout;
}

// A lot is one or more grid cells joined into a single city block (the
// streets between them are built over): a rectangle of 1-3 cells, an
// L of three cells around a 2x2 corner, or an empty open area.
struct Lot {
  int cx0, cx1, cz0, cz1;      // Inclusive cell ranges (the bounding box).
  bool empty = false;          // Open area: sidewalks only, no buildings.
  bool notch = false;          // L-shape: the 2x2 box minus one corner cell.
  bool missingEast = false, missingNorth = false;  // Which corner cell is missing.
};

std::vector<Lot> AssignLots(const MapGeneratorConfig& c, Rng* rng) {
  const int nx = c.blocksX, nz = c.blocksZ;
  std::vector<int> owner(static_cast<size_t>(nx) * nz, -1);
  auto at = [&](int x, int z) -> int& { return owner[x * nz + z]; };
  // The middle east-west street stays open so the spawn rows keep a clear run.
  const int midRow = nz >= 2 ? nz / 2 : -1;
  auto crossesMid = [&](int z0, int z1) { return midRow >= 0 && z0 < midRow && z1 >= midRow; };
  std::vector<Lot> lots;
  auto free = [&](int x0, int x1, int z0, int z1) {
    if (x0 < 0 || z0 < 0 || x1 >= nx || z1 >= nz || crossesMid(z0, z1)) return false;
    for (int x = x0; x <= x1; ++x)
      for (int z = z0; z <= z1; ++z)
        if (at(x, z) >= 0) return false;
    return true;
  };
  auto claim = [&](Lot lot) {
    const int id = static_cast<int>(lots.size());
    for (int x = lot.cx0; x <= lot.cx1; ++x)
      for (int z = lot.cz0; z <= lot.cz1; ++z) {
        const bool missing = lot.notch && x == (lot.missingEast ? lot.cx1 : lot.cx0) &&
                             z == (lot.missingNorth ? lot.cz0 : lot.cz1);
        if (!missing) at(x, z) = id;
      }
    lots.push_back(lot);
  };
  for (int x = 0; x < nx; ++x) {
    for (int z = 0; z < nz; ++z) {
      if (at(x, z) >= 0) continue;
      const float roll = rng->Unit01();
      const bool alongX = rng->Chance(0.5f);
      const int len = rng->Chance(0.3f) ? 3 : 2;
      const int corner = rng->Int(0, 3);
      Lot lot{x, x, z, z};
      if (roll < 0.10f) {
        lot.empty = true;
      } else if (roll < 0.30f) {  // Two or three cells in a row, plus the roads between.
        const int ex = alongX ? x + len - 1 : x, ez = alongX ? z : z + len - 1;
        if (free(x, ex, z, ez)) { lot.cx1 = ex; lot.cz1 = ez; }
      } else if (roll < 0.42f) {  // Three cells at an angle.
        // The 2x2 box anchored at (x, z) with one corner cell dropped; only
        // corners other than the anchor cell itself, which we know is free.
        if (free(x, x + 1, z, z + 1)) {
          lot.cx1 = x + 1; lot.cz1 = z + 1; lot.notch = true;
          lot.missingEast = corner & 1;
          lot.missingNorth = corner & 2;
          if (!lot.missingEast && lot.missingNorth) lot.missingEast = true;  // Keep the anchor.
        }
      }
      claim(lot);
    }
  }
  return lots;
}

}  // namespace

std::vector<UrbanBlock> UrbanBlocks(uint32_t seed, const MapGeneratorConfig& c) {
  Rng rng(seed);
  const Layout layout = LayoutCity(c, &rng);
  std::vector<UrbanBlock> blocks;
  for (const Span& x : layout.xs) {
    for (const Span& z : layout.zs) {
      blocks.push_back({x.start, x.start + x.size, z.start, z.start + z.size});
    }
  }
  return blocks;
}

std::vector<UrbanLot> UrbanLots(uint32_t seed, const MapGeneratorConfig& c) {
  Rng rng(seed);
  const Layout layout = LayoutCity(c, &rng);
  // Mirror GenerateUrbanMap's draws: one height per cell before the lots.
  for (int i = 0; i < c.blocksX * c.blocksZ; ++i) rng.Float(-1.6f, 1.6f);
  std::vector<UrbanLot> out;
  for (const Lot& lot : AssignLots(c, &rng)) {
    out.push_back({layout.xs[lot.cx0].start, layout.xs[lot.cx1].start + layout.xs[lot.cx1].size,
                   layout.zs[lot.cz0].start, layout.zs[lot.cz1].start + layout.zs[lot.cz1].size,
                   lot.empty, lot.notch});
  }
  return out;
}

float UrbanMapHalfExtent(const MapGeneratorConfig& c) {
  const float width = c.blocksX * c.blockSize + (c.blocksX + 1) * c.streetWidth;
  const float depth = c.blocksZ * c.blockSize + (c.blocksZ + 1) * c.streetWidth;
  return std::max(width, depth) * 0.5f;
}

Scene GenerateUrbanMap(uint32_t seed, const MapGeneratorConfig& c) {
  Rng rng(seed);
  Scene scene;
  scene.mapHalfExtent = UrbanMapHalfExtent(c);
  const Layout layout = LayoutCity(c, &rng);

  // Per-block base height: a random walk over the grid that blends the
  // already-assigned west/south neighbors with a medium default, so nearby
  // blocks end up similar and most of the city sits at medium heights.
  const float mediumHeight = 0.5f * (c.minBuildingHeight + c.maxBuildingHeight) - 0.5f;
  std::vector<float> baseHeight(static_cast<size_t>(c.blocksX) * c.blocksZ);
  auto baseAt = [&](int bx, int bz) -> float& { return baseHeight[bx * c.blocksZ + bz]; };
  for (int bx = 0; bx < c.blocksX; ++bx) {
    for (int bz = 0; bz < c.blocksZ; ++bz) {
      float sum = 0.0f;
      int n = 0;
      if (bx > 0) { sum += baseAt(bx - 1, bz); ++n; }
      if (bz > 0) { sum += baseAt(bx, bz - 1); ++n; }
      const float neighbors = n ? sum / static_cast<float>(n) : mediumHeight;
      baseAt(bx, bz) = std::clamp(0.65f * neighbors + 0.35f * mediumHeight + rng.Float(-1.6f, 1.6f),
                                  c.minBuildingHeight, c.maxBuildingHeight);
    }
  }

  for (const Lot& lot : AssignLots(c, &rng)) {
    const Span& sx0 = layout.xs[lot.cx0];
    const Span& sx1 = layout.xs[lot.cx1];
    const Span& sz0 = layout.zs[lot.cz0];
    const Span& sz1 = layout.zs[lot.cz1];
    const float sw = c.sidewalkWidth, d = c.buildingDepth;
    // Everything below is built in a canonical frame (u, v) where an L's
    // notch is the north-east corner, then mirrored back into world (x, z).
    const bool flipU = lot.notch && !lot.missingEast;
    const bool flipV = lot.notch && !lot.missingNorth;
    auto toU = [&](float x) { return flipU ? -x : x; };
    auto toV = [&](float z) { return flipV ? -z : z; };
    const float xa = toU(sx0.start), xb = toU(sx1.start + sx1.size);
    const float za = toV(sz0.start), zb = toV(sz1.start + sz1.size);
    const float U0 = std::min(xa, xb), U1 = std::max(xa, xb);
    const float V0 = std::min(za, zb), V1 = std::max(za, zb);
    float nu = U1, nv = V0;  // Notch edges (a rectangle has none).
    if (lot.notch) {
      const float worldX = lot.missingEast ? sx0.start + sx0.size : layout.xs[lot.cx0 + 1].start;
      const float worldZ = lot.missingNorth ? layout.zs[lot.cz0 + 1].start : sz0.start + sz0.size;
      nu = toU(worldX);
      nv = toV(worldZ);
    }
    auto box = [&](float u0, float v0, float u1, float v1, float h, bool building) {
      const float xl = flipU ? -u1 : u0, xr = flipU ? -u0 : u1;
      const float zl = flipV ? -v1 : v0, zr = flipV ? -v0 : v1;
      if (building) AddBuilding(&scene, xl, zl, xr, zr, h);
      else scene.sidewalks.push_back(AABB{glm::vec3(xl, 0.0f, zl), glm::vec3(xr, kSidewalkHeight, zr)});
    };
    auto walk = [&](float u0, float v0, float u1, float v1) { box(u0, v0, u1, v1, 0.0f, false); };

    // Outer edge of the building ring (behind the sidewalk setback).
    const float iu0 = U0 + sw, iu1 = U1 - sw, iv0 = V0 + sw, iv1 = V1 - sw;
    const float inu = lot.notch ? nu - sw : iu1, inv = lot.notch ? nv + sw : iv0;

    // The whole block sits at curb height, buildings included.
    if (!lot.notch) {
      walk(U0, V0, U1, V1);
    } else {
      walk(U0, V0, nu, V1);
      walk(nu, nv, U1, V1);
    }
    if (lot.empty) continue;

    // Building runs in ring order. Horizontal runs own the corners; vertical
    // runs sit between and may open a gap at either end.
    struct Run { bool alongU; float a, b, c0, c1; };
    std::vector<Run> runs;
    if (!lot.notch) {
      runs = {{true, iu0, iu1, iv0, iv0 + d}, {true, iu0, iu1, iv1 - d, iv1},
              {false, iv0 + d, iv1 - d, iu0, iu0 + d}, {false, iv0 + d, iv1 - d, iu1 - d, iu1}};
    } else {
      runs = {{true, iu0, inu, iv0, iv0 + d}, {true, inu - d, iu1, inv, inv + d},
              {true, iu0, iu1, iv1 - d, iv1},
              {false, iv0 + d, inv, inu - d, inu}, {false, inv + d, iv1 - d, iu1 - d, iu1},
              {false, iv0 + d, iv1 - d, iu0, iu0 + d}};
    }
    const float base = baseAt(lot.cx0, lot.cz0);
    auto heightHere = [&] {
      return std::clamp(base + rng.Float(-0.6f, 0.6f), c.minBuildingHeight, c.maxBuildingHeight);
    };

    // Decide every junction up front so the lot is guaranteed a mix of
    // wall-to-wall and gapped runs.
    struct Side {
      std::vector<bool> inner;  // Junctions between this run's buildings.
      bool startGap = false, endGap = false;
    };
    std::vector<Side> sides(runs.size());
    struct Ref { size_t side; int index; };  // index -1 = start, -2 = end, >=0 inner.
    std::vector<Ref> refs;
    for (size_t s = 0; s < runs.size(); ++s) {
      const float len = runs[s].b - runs[s].a;
      const int lo = std::max(1, static_cast<int>(std::lround(len / 10.0f)));
      const int hi = std::max(lo, static_cast<int>(std::lround(len / 7.0f)));
      const int count = rng.Int(lo, hi);
      sides[s].inner.assign(count - 1, false);
      for (int i = 0; i < count - 1; ++i) refs.push_back({s, i});
      if (!runs[s].alongU) {
        refs.push_back({s, -1});
        refs.push_back({s, -2});
      }
    }
    auto set = [&](const Ref& r, bool v) {
      if (r.index >= 0) sides[r.side].inner[r.index] = v;
      else if (r.index == -1) sides[r.side].startGap = v;
      else sides[r.side].endGap = v;
    };
    std::vector<bool> isGap(refs.size());
    for (size_t i = 0; i < refs.size(); ++i) isGap[i] = rng.Chance(c.gapProbability);
    if (std::find(isGap.begin(), isGap.end(), true) == isGap.end()) {
      isGap[static_cast<size_t>(rng.Int(0, static_cast<int>(refs.size()) - 1))] = true;
    }
    if (std::find(isGap.begin(), isGap.end(), false) == isGap.end()) {
      isGap[static_cast<size_t>(rng.Int(0, static_cast<int>(refs.size()) - 1))] = false;
    }
    for (size_t i = 0; i < refs.size(); ++i) set(refs[i], isGap[i]);

    for (size_t s = 0; s < runs.size(); ++s) {
      const Run& r = runs[s];
      SplitRun(r.a, r.b, sides[s].inner, c.gapWidth, sides[s].startGap, sides[s].endGap,
               [&](float lo, float hi) {
                 if (r.alongU) box(lo, r.c0, hi, r.c1, heightHere(), true);
                 else box(r.c0, lo, r.c1, hi, heightHere(), true);
               });
    }
  }

  // A couple of towers: pick distinct buildings and stretch them well above
  // the medium-height fabric.
  const int towers = std::min<int>(c.towerCount, static_cast<int>(scene.obstacles.size()));
  std::vector<size_t> picked;
  while (static_cast<int>(picked.size()) < towers) {
    const size_t i = static_cast<size_t>(rng.Int(0, static_cast<int>(scene.obstacles.size()) - 1));
    if (std::find(picked.begin(), picked.end(), i) != picked.end()) continue;
    picked.push_back(i);
    scene.obstacles[i].bounds.max.y = rng.Float(c.towerMinHeight, c.towerMaxHeight);
  }

  // Squads line up on the west (Blue) and east (Red) rim streets, centered
  // on the middle east-west street so the z == midZ row has clear sightlines.
  const float spawnX = scene.mapHalfExtent - c.streetWidth * 0.5f;
  float midZ = 0.0f;
  if (c.blocksZ >= 2) {
    const Span& before = layout.zs[c.blocksZ / 2 - 1];
    midZ = 0.5f * (before.start + before.size + layout.zs[c.blocksZ / 2].start);
  }
  const float rows[3] = {midZ - 6.0f, midZ, midZ + 6.0f};
  for (int i = 0; i < 3; ++i) {
    Unit blue;
    blue.id = i;
    blue.team = Team::Blue;
    blue.position = glm::vec3(-spawnX, 0.0f, rows[i]);
    blue.facingYaw = 0.0f;
    scene.units.push_back(blue);
  }
  for (int i = 0; i < 3; ++i) {
    Unit red;
    red.id = 3 + i;
    red.team = Team::Red;
    red.position = glm::vec3(spawnX, 0.0f, rows[i]);
    red.facingYaw = kPi;
    scene.units.push_back(red);
  }
  return scene;
}

}  // namespace tactics
