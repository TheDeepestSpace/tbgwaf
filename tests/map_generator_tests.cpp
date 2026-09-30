// Seeded regression tests for the procedural urban map generator and the
// range-scoped navmesh it feeds. Headless: no SDL/GL/ImGui.

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
  const Expected expected[] = {{1u, 124}, {42u, 126}, {2024u, 129}};
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

struct BlockRect { float x0, x1, z0, z1; };

std::vector<BlockRect> Blocks(const MapGeneratorConfig& c) {
  const float totalX = c.blocksX * c.blockSize + (c.blocksX + 1) * c.streetWidth;
  const float totalZ = c.blocksZ * c.blockSize + (c.blocksZ + 1) * c.streetWidth;
  std::vector<BlockRect> blocks;
  for (int bx = 0; bx < c.blocksX; ++bx) {
    for (int bz = 0; bz < c.blocksZ; ++bz) {
      const float x0 = -totalX * 0.5f + c.streetWidth + bx * (c.blockSize + c.streetWidth);
      const float z0 = -totalZ * 0.5f + c.streetWidth + bz * (c.blockSize + c.streetWidth);
      blocks.push_back({x0, x0 + c.blockSize, z0, z0 + c.blockSize});
    }
  }
  return blocks;
}

bool Inside(const AABB& b, const BlockRect& r, float inset) {
  return b.min.x >= r.x0 + inset - kEps && b.max.x <= r.x1 - inset + kEps &&
         b.min.z >= r.z0 + inset - kEps && b.max.z <= r.z1 - inset + kEps;
}

void TestStreetsAndSidewalksAreObstacleFree() {
  const MapGeneratorConfig c;
  const auto blocks = Blocks(c);
  for (uint32_t seed : kSeeds) {
    const Scene scene = GenerateUrbanMap(seed);
    size_t perBlock[16] = {};
    for (const auto& o : scene.obstacles) {
      int owner = -1;
      for (size_t i = 0; i < blocks.size(); ++i) {
        // Fully inside a block, behind the sidewalk setback: so it neither
        // touches a street corridor nor a sidewalk strip.
        if (Inside(o.bounds, blocks[i], c.sidewalkWidth)) owner = static_cast<int>(i);
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
  const auto blocks = Blocks(c);
  for (uint32_t seed : kSeeds) {
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
  // Spawns sit on the same central east-west street, ~128 units apart.
  const Scene scene = GenerateUrbanMap(42);
  const Unit& blue = scene.units[1];  // z == 0 row.
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
