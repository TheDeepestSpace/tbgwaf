// Seeded regression tests for the procedural urban map generator and the
// range-scoped navmesh it feeds. Headless: no SDL/GL/ImGui.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "game/GameLogic.h"
#include "game/MapGenerator.h"
#include "game/NavMesh.h"
#include "game/Scene.h"
#include "game/Visibility.h"

namespace {

int g_failures = 0;

void ReportFailure(const char* file, int line, const char* expr) {
  std::fprintf(stderr, "CHECK FAILED at %s:%d: %s\n", file, line, expr);
  ++g_failures;
}

#define CHECK(expr) \
  do { \
    if (!(expr)) ReportFailure(__FILE__, __LINE__, #expr); \
  } while (0)

using namespace tactics;

constexpr float kEps = 1.0e-3f;
const uint32_t kSeeds[] = {1u, 42u, 2024u};

bool SameScene(const Scene& a, const Scene& b) {
  if (a.mapHalfExtent != b.mapHalfExtent || a.obstacles.size() != b.obstacles.size() ||
      a.units.size() != b.units.size()) {
    return false;
  }
  for (size_t i = 0; i < a.obstacles.size(); ++i) {
    const auto& x = a.obstacles[i];
    const auto& y = b.obstacles[i];
    if (x.bounds.min != y.bounds.min || x.bounds.max != y.bounds.max ||
        x.climbable != y.climbable) {
      return false;
    }
  }
  for (size_t i = 0; i < a.units.size(); ++i) {
    if (a.units[i].position != b.units[i].position || a.units[i].team != b.units[i].team ||
        a.units[i].facingYaw != b.units[i].facingYaw) {
      return false;
    }
  }
  return true;
}

void TestDeterminismAndVariety() {
  for (uint32_t seed : kSeeds) {
    CHECK(SameScene(GenerateUrbanMap(seed), GenerateUrbanMap(seed)));
  }
  CHECK(!SameScene(GenerateUrbanMap(1), GenerateUrbanMap(42)));
  CHECK(!SameScene(GenerateUrbanMap(42), GenerateUrbanMap(2024)));
}

// Pinned per-seed output size: a generator change that alters layouts shows
// up here first (update the numbers deliberately when it's intended).
void TestPinnedSeedFingerprints() {
  struct Expected { uint32_t seed; size_t buildings; };
  const Expected expected[] = {{1u, 115}, {42u, 120}, {2024u, 126}};
  for (const auto& e : expected) {
    const Scene scene = GenerateUrbanMap(e.seed);
    if (scene.obstacles.size() != e.buildings) {
      std::fprintf(stderr, "seed %u: %zu buildings (expected %zu)\n", e.seed,
                   scene.obstacles.size(), e.buildings);
    }
    CHECK(scene.obstacles.size() == e.buildings);
  }
}

void TestMapIsMuchLargerThanDefault() {
  const Scene scene = GenerateUrbanMap(1);
  CHECK(scene.mapHalfExtent >= 4.0f * constants::kMapHalfExtent);
  CHECK(BuildDefaultScene().mapHalfExtent == constants::kMapHalfExtent);
  for (const auto& o : scene.obstacles) {
    CHECK(o.bounds.min.x >= -scene.mapHalfExtent && o.bounds.max.x <= scene.mapHalfExtent);
    CHECK(o.bounds.min.z >= -scene.mapHalfExtent && o.bounds.max.z <= scene.mapHalfExtent);
    CHECK(o.bounds.max.x - o.bounds.min.x > kEps && o.bounds.max.z - o.bounds.min.z > kEps);
  }
}

using BlockRect = UrbanBlock;

float Area(const BlockRect& r) { return (r.x1 - r.x0) * (r.z1 - r.z0); }

std::vector<BlockRect> Blocks(uint32_t seed, const MapGeneratorConfig& c) {
  return UrbanBlocks(seed, c);
}

// Built lots (merged cells, L-shapes, open areas) as bounding rectangles.
std::vector<BlockRect> Lots(uint32_t seed, const MapGeneratorConfig& c, bool buildingsOnly) {
  std::vector<BlockRect> out;
  for (const auto& l : UrbanLots(seed, c)) {
    if (buildingsOnly && l.empty) continue;
    out.push_back({l.x0, l.x1, l.z0, l.z1});
  }
  return out;
}

bool Inside(const AABB& b, const BlockRect& r, float inset) {
  return b.min.x >= r.x0 + inset - kEps && b.max.x <= r.x1 - inset + kEps &&
         b.min.z >= r.z0 + inset - kEps && b.max.z <= r.z1 - inset + kEps;
}

void TestStreetsAndSidewalksAreObstacleFree() {
  const MapGeneratorConfig c;
  for (uint32_t seed : kSeeds) {
    const auto blocks = Lots(seed, c, /*buildingsOnly=*/true);
    const Scene scene = GenerateUrbanMap(seed);
    size_t perBlock[16] = {};
    for (const auto& o : scene.obstacles) {
      int owner = -1;
      for (size_t i = 0; i < blocks.size(); ++i) {
        // Fully inside a block, behind the sidewalk setback: so it neither
        // touches a street corridor nor a sidewalk strip.
        if (Inside(o.bounds, blocks[i], c.sidewalkWidth) &&
            (owner < 0 || Area(blocks[i]) < Area(blocks[owner]))) {
          owner = static_cast<int>(i);
        }
      }
      CHECK(owner >= 0);
      if (owner >= 0) ++perBlock[owner];
      CHECK(!o.climbable);
    }
    for (size_t i = 0; i < blocks.size(); ++i) CHECK(perBlock[i] >= 6);
  }
}

// Junction between two buildings: they line up along one axis with a
// separation on the other. Returns the separation, or -1 if not adjacent.
float Separation(const AABB& a, const AABB& b) {
  const bool zOverlap = a.min.z < b.max.z - kEps && b.min.z < a.max.z - kEps;
  const bool xOverlap = a.min.x < b.max.x - kEps && b.min.x < a.max.x - kEps;
  if (zOverlap) {
    const float s1 = b.min.x - a.max.x, s2 = a.min.x - b.max.x;
    if (s1 > -kEps) return s1;
    if (s2 > -kEps) return s2;
  }
  if (xOverlap) {
    const float s1 = b.min.z - a.max.z, s2 = a.min.z - b.max.z;
    if (s1 > -kEps) return s1;
    if (s2 > -kEps) return s2;
  }
  return -1.0f;
}

void TestEveryBlockHasWallToWallAndGappedRuns() {
  const MapGeneratorConfig c;
  for (uint32_t seed : kSeeds) {
    const auto blocks = Lots(seed, c, /*buildingsOnly=*/true);
    const Scene scene = GenerateUrbanMap(seed);
    for (const auto& block : blocks) {
      std::vector<AABB> in;
      for (const auto& o : scene.obstacles) {
        if (Inside(o.bounds, block, 0.0f)) in.push_back(o.bounds);
      }
      bool wall = false, gap = false;
      for (size_t i = 0; i < in.size(); ++i) {
        for (size_t j = i + 1; j < in.size(); ++j) {
          const float sep = Separation(in[i], in[j]);
          if (sep < 0.0f) continue;
          if (sep < kEps) wall = true;
          if (std::fabs(sep - c.gapWidth) < kEps) gap = true;
        }
      }
      CHECK(wall);
      CHECK(gap);
    }
  }
}

void TestVariedHeightsWithFewTowers() {
  const MapGeneratorConfig c;
  for (uint32_t seed : kSeeds) {
    const Scene scene = GenerateUrbanMap(seed);
    int towers = 0, medium = 0;
    float lo = 1e9f, hi = 0.0f;
    for (const auto& o : scene.obstacles) {
      const float h = o.bounds.max.y;
      lo = std::min(lo, h);
      hi = std::max(hi, h);
      CHECK(h >= c.minBuildingHeight - kEps);
      if (h >= c.towerMinHeight - kEps) ++towers;
      else if (h >= 4.5f && h <= 7.5f) ++medium;
    }
    CHECK(towers == c.towerCount);
    CHECK(hi - lo > 8.0f);
    CHECK(medium * 2 > static_cast<int>(scene.obstacles.size()) / 2);  // Mostly medium.
  }
}

void TestBlocksAndStreetsVary() {
  const MapGeneratorConfig c;
  for (uint32_t seed : kSeeds) {
    const auto blocks = Blocks(seed, c);
    float minW = 1e9f, maxW = 0.0f, minGap = 1e9f, maxGap = 0.0f;
    for (const auto& b : blocks) {
      minW = std::min(minW, b.x1 - b.x0);
      maxW = std::max(maxW, b.x1 - b.x0);
      CHECK(b.x1 - b.x0 >= c.minBlockSize - kEps && b.z1 - b.z0 >= c.minBlockSize - kEps);
    }
    CHECK(maxW - minW > 1.0f);
    // Straight streets: columns share x extents, rows share z extents.
    for (const auto& a : blocks) {
      for (const auto& b : blocks) {
        if (std::fabs(a.x0 - b.x0) < kEps) CHECK(std::fabs(a.x1 - b.x1) < kEps);
        if (std::fabs(a.z0 - b.z0) < kEps) CHECK(std::fabs(a.z1 - b.z1) < kEps);
      }
    }
    for (int i = 1; i < c.blocksX; ++i) {
      const float w = blocks[i * c.blocksZ].x0 - blocks[(i - 1) * c.blocksZ].x1;
      minGap = std::min(minGap, w);
      maxGap = std::max(maxGap, w);
    }
    CHECK(maxGap - minGap > 0.5f);
    CHECK(minGap >= c.streetWidth * c.streetWidthMin - kEps);
    CHECK(maxGap <= c.streetWidth * c.streetWidthMax + kEps);
  }
}

void TestSidewalksLineEveryBlock() {
  const MapGeneratorConfig c;
  for (uint32_t seed : kSeeds) {
    const Scene scene = GenerateUrbanMap(seed);
    const auto blocks = Lots(seed, c, /*buildingsOnly=*/false);
    size_t expected = 0;
    for (const auto& l : UrbanLots(seed, c)) expected += l.notch ? 6 : 4;
    CHECK(scene.sidewalks.size() == expected);
    for (const AABB& s : scene.sidewalks) {
      CHECK(s.max.y > 0.0f && s.max.y < 0.3f);  // Curb, not a wall.
      bool inBlock = false;
      for (const auto& b : blocks) inBlock |= Inside(s, b, 0.0f);
      CHECK(inBlock);
      // Never overlaps a building footprint.
      for (const auto& o : scene.obstacles) {
        const bool overlap = s.min.x < o.bounds.max.x - kEps && o.bounds.min.x < s.max.x - kEps &&
                             s.min.z < o.bounds.max.z - kEps && o.bounds.min.z < s.max.z - kEps;
        CHECK(!overlap);
      }
    }
  }
}

// Over many seeds the generator produces merged blocks, L-shapes and open
// areas, and every lot stays within the grid of streets.
void TestLotVariety() {
  const MapGeneratorConfig c;
  int merged = 0, ls = 0, empties = 0, plain = 0;
  for (uint32_t seed = 1; seed <= 40; ++seed) {
    for (const auto& l : UrbanLots(seed, c)) {
      const bool multi = l.x1 - l.x0 > c.blockSize * 1.6f + c.streetWidth * 0.5f ||
                         l.z1 - l.z0 > c.blockSize * 1.6f + c.streetWidth * 0.5f;
      if (l.empty) ++empties;
      else if (l.notch) ++ls;
      else if (multi) ++merged;
      else ++plain;
    }
    // Middle east-west street stays clear of buildings for the spawn rows.
    const Scene scene = GenerateUrbanMap(seed);
    const Unit& blue = scene.units[1];
    for (const auto& o : scene.obstacles) {
      CHECK(!(o.bounds.min.z < blue.position.z + 0.5f && o.bounds.max.z > blue.position.z - 0.5f));
    }
  }
  CHECK(merged > 0);
  CHECK(ls > 0);
  CHECK(empties > 0);
  CHECK(plain > merged + ls + empties);
}

void TestNavMeshFullyReachableFromSpawns() {
  for (uint32_t seed : kSeeds) {
    const Scene scene = GenerateUrbanMap(seed);
    NavMesh nav;
    nav.Build(scene.obstacles, scene.mapHalfExtent, constants::kAgentRadius);
    const glm::vec3 blue = scene.units.front().position;
    const glm::vec3 red = scene.units.back().position;
    std::vector<glm::vec3> path;
    CHECK(nav.FindPath(blue, red, &path));
    int unreachable = 0;
    for (const NavCell& cell : nav.Cells()) {
      if (!nav.FindPath(blue, cell.Center(), &path)) ++unreachable;
    }
    if (unreachable) std::fprintf(stderr, "seed %u: %d unreachable cells\n", seed, unreachable);
    CHECK(unreachable == 0);
  }
}

void TestWindowedNavMeshMatchesGlobalWithinBudget() {
  const Scene scene = GenerateUrbanMap(42);
  NavMesh global;
  global.Build(scene.obstacles, scene.mapHalfExtent, constants::kAgentRadius);

  const Unit& unit = scene.units.front();
  const float reach = unit.MoveBudget() + 2.0f * constants::kAgentRadius;
  NavRegion region{unit.position.x - reach, unit.position.x + reach, unit.position.z - reach,
                   unit.position.z + reach};
  NavMesh local;
  local.Build(scene.obstacles, region, constants::kAgentRadius);
  CHECK(local.Cells().size() < global.Cells().size());

  auto length = [](const std::vector<glm::vec3>& p) {
    float l = 0.0f;
    for (size_t i = 0; i + 1 < p.size(); ++i) l += glm::distance(p[i], p[i + 1]);
    return l;
  };
  int compared = 0;
  for (const NavCell& cell : global.Cells()) {
    std::vector<glm::vec3> gp, lp;
    if (!global.FindPath(unit.position, cell.Center(), &gp)) continue;
    if (length(gp) > unit.MoveBudget()) continue;
    ++compared;
    CHECK(local.FindPath(unit.position, cell.Center(), &lp));
    CHECK(std::fabs(length(lp) - length(gp)) < 1.0e-2f);
  }
  CHECK(compared > 0);
}

void TestGameLogicUsesRangeScopedNavMesh() {
  Scene scene = GenerateUrbanMap(42);
  NavMesh global;
  global.Build(scene.obstacles, scene.mapHalfExtent, constants::kAgentRadius);
  GameLogic game(scene);
  CHECK(game.GetNavMesh().Cells().empty());

  const Unit blue = game.GetScene().units.front();
  game.ClickUnit(blue.id, Team::Blue);
  game.ChooseMove();
  // Along the rim street, within budget: plannable.
  game.ClickGround(blue.position + glm::vec3(0.0f, 0.0f, 15.0f), Team::Blue);
  CHECK(game.FindUnit(blue.id)->plan.type == PlannedActionType::Move);
  CHECK(!game.GetNavMesh().Cells().empty());
  CHECK(game.GetNavMesh().Cells().size() < global.Cells().size());

  // Beyond the window (across the map): rejected.
  game.ClickUnit(blue.id, Team::Blue);
  game.ChooseMove();
  game.ClickGround(scene.units.back().position, Team::Blue);
  CHECK(game.FindUnit(blue.id)->plan.movePath.size() >= 2);
  CHECK(glm::distance(game.FindUnit(blue.id)->plan.movePath.back(),
                      scene.units.front().position + glm::vec3(0.0f, 0.0f, 15.0f)) < 0.1f);
}

void TestVisibilityStillSpansWholeMap() {
  // Spawns sit on the same middle east-west street, ~128 units apart.
  const Scene scene = GenerateUrbanMap(42);
  const Unit& blue = scene.units[1];  // Middle street row.
  const Unit& red = scene.units[4];
  CHECK(glm::distance(blue.position, red.position) > 100.0f);
  CHECK(IsPointVisibleToTeam(Team::Blue, red.position + glm::vec3(0, 1.0f, 0), scene.units,
                             ObstacleBounds(scene.obstacles)));
}

}  // namespace

int main() {
  TestDeterminismAndVariety();
  TestPinnedSeedFingerprints();
  TestMapIsMuchLargerThanDefault();
  TestStreetsAndSidewalksAreObstacleFree();
  TestEveryBlockHasWallToWallAndGappedRuns();
  TestVariedHeightsWithFewTowers();
  TestBlocksAndStreetsVary();
  TestSidewalksLineEveryBlock();
  TestLotVariety();
  TestNavMeshFullyReachableFromSpawns();
  TestWindowedNavMeshMatchesGlobalWithinBudget();
  TestGameLogicUsesRangeScopedNavMesh();
  TestVisibilityStillSpansWholeMap();
  if (g_failures) {
    std::fprintf(stderr, "%d check(s) failed\n", g_failures);
    return 1;
  }
  std::printf("map generator tests passed\n");
  return 0;
}
