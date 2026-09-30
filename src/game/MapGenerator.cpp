#include "game/MapGenerator.h"

#include <algorithm>
#include <random>
#include <vector>

namespace tactics {
namespace {

constexpr float kPi = 3.14159265358979323846f;

// std::mt19937 output is fully specified by the standard, but the
// <random> distributions are not, so draw values by hand to keep a seed's
// output identical across standard libraries.
class Rng {
 public:
  explicit Rng(uint32_t seed) : engine_(seed) {}
  int Int(int lo, int hi) { return lo + static_cast<int>(engine_() % static_cast<uint32_t>(hi - lo + 1)); }
  bool Chance(float p) { return static_cast<float>(engine_() >> 8) / 16777216.0f < p; }

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

}  // namespace

float UrbanMapHalfExtent(const MapGeneratorConfig& c) {
  const float width = c.blocksX * c.blockSize + (c.blocksX + 1) * c.streetWidth;
  const float depth = c.blocksZ * c.blockSize + (c.blocksZ + 1) * c.streetWidth;
  return std::max(width, depth) * 0.5f;
}

Scene GenerateUrbanMap(uint32_t seed, const MapGeneratorConfig& c) {
  Rng rng(seed);
  Scene scene;
  scene.mapHalfExtent = UrbanMapHalfExtent(c);
  const float totalX = c.blocksX * c.blockSize + (c.blocksX + 1) * c.streetWidth;
  const float totalZ = c.blocksZ * c.blockSize + (c.blocksZ + 1) * c.streetWidth;

  for (int bx = 0; bx < c.blocksX; ++bx) {
    for (int bz = 0; bz < c.blocksZ; ++bz) {
      const float blockX0 = -totalX * 0.5f + c.streetWidth + bx * (c.blockSize + c.streetWidth);
      const float blockZ0 = -totalZ * 0.5f + c.streetWidth + bz * (c.blockSize + c.streetWidth);
      // Outer edge of the building ring, behind the sidewalk setback.
      const float x0 = blockX0 + c.sidewalkWidth, x1 = blockX0 + c.blockSize - c.sidewalkWidth;
      const float z0 = blockZ0 + c.sidewalkWidth, z1 = blockZ0 + c.blockSize - c.sidewalkWidth;
      const float d = c.buildingDepth;

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
        AddBuilding(&scene, a, z0, b, z0 + d, c.buildingHeight);
      });
      SplitRun(x0, x1, sides[1].inner, c.gapWidth, false, false, [&](float a, float b) {
        AddBuilding(&scene, a, z1 - d, b, z1, c.buildingHeight);
      });
      SplitRun(z0 + d, z1 - d, sides[2].inner, c.gapWidth, sides[2].startGap, sides[2].endGap,
               [&](float a, float b) { AddBuilding(&scene, x0, a, x0 + d, b, c.buildingHeight); });
      SplitRun(z0 + d, z1 - d, sides[3].inner, c.gapWidth, sides[3].startGap, sides[3].endGap,
               [&](float a, float b) { AddBuilding(&scene, x1 - d, a, x1, b, c.buildingHeight); });
    }
  }

  // Squads line up on the west (Blue) and east (Red) rim streets.
  const float spawnX = scene.mapHalfExtent - c.streetWidth * 0.5f;
  const float rows[3] = {-6.0f, 0.0f, 6.0f};
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
