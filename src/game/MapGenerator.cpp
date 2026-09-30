#include "game/MapGenerator.h"

#include <algorithm>
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

  auto addSidewalk = [&](float ax, float az, float bx, float bz) {
    scene.sidewalks.push_back(AABB{glm::vec3(ax, 0.0f, az), glm::vec3(bx, kSidewalkHeight, bz)});
  };

  for (int bx = 0; bx < c.blocksX; ++bx) {
    for (int bz = 0; bz < c.blocksZ; ++bz) {
      const Span& sx = layout.xs[bx];
      const Span& sz = layout.zs[bz];
      const float blockX0 = sx.start, blockX1 = sx.start + sx.size;
      const float blockZ0 = sz.start, blockZ1 = sz.start + sz.size;
      // Outer edge of the building ring, behind the sidewalk setback.
      const float x0 = blockX0 + c.sidewalkWidth, x1 = blockX1 - c.sidewalkWidth;
      const float z0 = blockZ0 + c.sidewalkWidth, z1 = blockZ1 - c.sidewalkWidth;
      const float d = c.buildingDepth;
      const float base = baseAt(bx, bz);
      auto heightHere = [&] {
        return std::clamp(base + rng.Float(-0.6f, 0.6f), c.minBuildingHeight, c.maxBuildingHeight);
      };

      // Sidewalk ring along the block's street edges.
      addSidewalk(blockX0, blockZ0, blockX1, z0);
      addSidewalk(blockX0, z1, blockX1, blockZ1);
      addSidewalk(blockX0, z0, x0, z1);
      addSidewalk(x1, z0, blockX1, z1);

      // Sides in ring order: north (z0) and south (z1) own the corners and
      // span the full width; west (x0) and east (x1) sit between them and
      // additionally have a junction with the adjacent corner building at
      // each end. Decide every junction up front so the block can be
      // guaranteed a mix of wall-to-wall and gapped runs.
      struct Side {
        std::vector<bool> inner;  // Junctions between this side's buildings.
        bool startGap = false, endGap = false;  // Corner junctions (west/east only).
      };
      Side sides[4];  // 0 north, 1 south, 2 west, 3 east.
      struct Ref { int side; int index; };  // index -1 = start, -2 = end, >=0 inner.
      std::vector<Ref> refs;
      for (int s = 0; s < 4; ++s) {
        const int count = s >= 2 ? rng.Int(1, 2) : rng.Int(2, 3);
        sides[s].inner.assign(count - 1, false);
        for (int i = 0; i < count - 1; ++i) refs.push_back({s, i});
        if (s >= 2) {
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

      SplitRun(x0, x1, sides[0].inner, c.gapWidth, false, false, [&](float a, float b) {
        AddBuilding(&scene, a, z0, b, z0 + d, heightHere());
      });
      SplitRun(x0, x1, sides[1].inner, c.gapWidth, false, false, [&](float a, float b) {
        AddBuilding(&scene, a, z1 - d, b, z1, heightHere());
      });
      SplitRun(z0 + d, z1 - d, sides[2].inner, c.gapWidth, sides[2].startGap, sides[2].endGap,
               [&](float a, float b) { AddBuilding(&scene, x0, a, x0 + d, b, heightHere()); });
      SplitRun(z0 + d, z1 - d, sides[3].inner, c.gapWidth, sides[3].startGap, sides[3].endGap,
               [&](float a, float b) { AddBuilding(&scene, x1 - d, a, x1, b, heightHere()); });
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
