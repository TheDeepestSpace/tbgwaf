// Seeded regression tests for the procedural urban map generator and the
// range-scoped navmesh it feeds. Headless: no SDL/GL/ImGui.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <set>
#include <vector>

#include "game/GameLogic.h"
#include "game/Geometry.h"
#include "game/MapGenerator.h"
#include "game/NavMesh.h"
#include "game/Raycast.h"
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

// The original orthogonal grid generator stays available behind
// arteryCount=0; the structural grid/lot tests run against this config.
MapGeneratorConfig LegacyConfig() {
  MapGeneratorConfig c;
  c.arteryCount = 0;
  return c;
}

bool SameScene(const Scene& a, const Scene& b) {
  if (a.mapHalfExtent != b.mapHalfExtent || a.obstacles.size() != b.obstacles.size() ||
      a.units.size() != b.units.size()) {
    return false;
  }
  for (size_t i = 0; i < a.obstacles.size(); ++i) {
    const auto& x = a.obstacles[i];
    const auto& y = b.obstacles[i];
    if (x.bounds.min != y.bounds.min || x.bounds.max != y.bounds.max ||
        x.climbable != y.climbable || x.footprint != y.footprint) {
      return false;
    }
  }
  if (a.roads.size() != b.roads.size() ||
      a.sidewalkSurfaces.size() != b.sidewalkSurfaces.size() ||
      a.walkSurfaces.size() != b.walkSurfaces.size()) {
    return false;
  }
  for (size_t i = 0; i < a.roads.size(); ++i) {
    if (a.roads[i].vertices != b.roads[i].vertices) return false;
  }
  for (size_t i = 0; i < a.sidewalkSurfaces.size(); ++i) {
    if (a.sidewalkSurfaces[i].vertices != b.sidewalkSurfaces[i].vertices) return false;
  }
  for (size_t i = 0; i < a.walkSurfaces.size(); ++i) {
    if (a.walkSurfaces[i].vertices != b.walkSurfaces[i].vertices ||
        a.walkSurfaces[i].neighbors != b.walkSurfaces[i].neighbors ||
        a.walkSurfaces[i].connectsToGround != b.walkSurfaces[i].connectsToGround) {
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
    const Scene scene = GenerateUrbanMap(e.seed, LegacyConfig());
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
  const MapGeneratorConfig c = LegacyConfig();
  for (uint32_t seed : kSeeds) {
    const auto blocks = Lots(seed, c, /*buildingsOnly=*/true);
    const Scene scene = GenerateUrbanMap(seed, LegacyConfig());
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
  const MapGeneratorConfig c = LegacyConfig();
  for (uint32_t seed : kSeeds) {
    const auto blocks = Lots(seed, c, /*buildingsOnly=*/true);
    const Scene scene = GenerateUrbanMap(seed, LegacyConfig());
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
      if (o.footprint.empty()) continue;  // Pier columns (elevated seeds), not buildings.
      const float h = o.bounds.max.y;
      lo = std::min(lo, h);
      hi = std::max(hi, h);
      CHECK(h >= c.minBuildingHeight - kEps);
      if (h >= c.towerMinHeight - kEps) ++towers;
      else if (h >= 4.5f && h <= 7.5f) ++medium;
    }
    CHECK(towers == c.towerCount);
    CHECK(hi - lo > 8.0f);
    CHECK(medium * 10 > static_cast<int>(scene.obstacles.size()));  // Medium heights still appear.
  }
}

void TestBlocksAndStreetsVary() {
  const MapGeneratorConfig c = LegacyConfig();
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

void TestSidewalksCoverEveryBlock() {
  const MapGeneratorConfig c = LegacyConfig();
  for (uint32_t seed : kSeeds) {
    const Scene scene = GenerateUrbanMap(seed, LegacyConfig());
    const auto blocks = Lots(seed, c, /*buildingsOnly=*/false);
    size_t expected = 0;
    for (const auto& l : UrbanLots(seed, c)) expected += l.notch ? 2 : 1;
    CHECK(scene.sidewalks.size() == expected);
    for (const AABB& s : scene.sidewalks) {
      CHECK(s.max.y > 0.0f && s.max.y < 0.3f);  // Curb, not a wall.
      bool inBlock = false;
      for (const auto& b : blocks) inBlock |= Inside(s, b, 0.0f);
      CHECK(inBlock);
    }
  }
}

// Over many seeds the generator produces merged blocks, L-shapes and open
// areas, and every lot stays within the grid of streets.
void TestLotVariety() {
  const MapGeneratorConfig c = LegacyConfig();
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
    const Scene scene = GenerateUrbanMap(seed, LegacyConfig());
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
    const Scene scene = GenerateUrbanMap(seed, LegacyConfig());
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
  const Scene scene = GenerateUrbanMap(42, LegacyConfig());
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
  Scene scene = GenerateUrbanMap(42, LegacyConfig());
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
  // Spawns face each other down the oblique artery, far apart with a clear
  // corridor between them; LOS uses the real polygon prisms.
  const Scene scene = GenerateUrbanMap(42);
  const Unit& blue = scene.units[1];  // Center of the spawn row.
  const Unit& red = scene.units[4];
  CHECK(glm::distance(blue.position, red.position) > 100.0f);
  CHECK(IsPointVisibleToTeam(Team::Blue, red.position + glm::vec3(0, 1.0f, 0), scene.units,
                             scene.obstacles));
}

// --- Hilly terrain generator (issue #90). ---

bool SameHillyScene(const Scene& a, const Scene& b) {
  return SameScene(a, b) && a.ground.heights == b.ground.heights && a.ground.nx == b.ground.nx &&
         a.ground.nz == b.ground.nz && a.ground.step == b.ground.step;
}

void TestHillyDeterminismAndVariety() {
  for (uint32_t seed : kSeeds) {
    CHECK(SameHillyScene(GenerateHillyMap(seed), GenerateHillyMap(seed)));
  }
  CHECK(!SameHillyScene(GenerateHillyMap(1), GenerateHillyMap(42)));
  CHECK(!SameHillyScene(GenerateHillyMap(42), GenerateHillyMap(2024)));
}

void TestHillyTerrainIsGenuinelyUneven() {
  for (uint32_t seed : kSeeds) {
    const Scene scene = GenerateHillyMap(seed);
    CHECK(!scene.ground.Empty());
    float lo = scene.ground.heights[0], hi = scene.ground.heights[0];
    for (float h : scene.ground.heights) {
      lo = std::min(lo, h);
      hi = std::max(hi, h);
    }
    // Real relief (several units), and nothing below the map's base plane.
    CHECK(hi - lo > 3.0f);
    CHECK(lo >= 0.0f);
  }
}

void TestHillyUnitsSpawnOnTerrain() {
  for (uint32_t seed : kSeeds) {
    const Scene scene = GenerateHillyMap(seed);
    CHECK(scene.units.size() == 6);
    for (const Unit& unit : scene.units) {
      const float h = scene.ground.HeightAt(unit.position.x, unit.position.z);
      CHECK(std::fabs(unit.position.y - h) < kEps);
      // Spawns must not be buried inside a rock's padded footprint.
      NavMesh nav;
      nav.Build(scene.obstacles, scene.mapHalfExtent, constants::kAgentRadius, &scene.ground);
      CHECK(nav.IsWalkable(unit.position.x, unit.position.z));
    }
  }
}

void TestHillyPathsFollowTerrain() {
  for (uint32_t seed : kSeeds) {
    const Scene scene = GenerateHillyMap(seed);
    NavMesh nav;
    nav.Build(scene.obstacles, scene.mapHalfExtent, constants::kAgentRadius, &scene.ground);

    // A cross-map path exists (west spawn to east spawn) and every waypoint
    // sits on the terrain surface.
    std::vector<glm::vec3> path;
    CHECK(nav.FindPath(scene.units[0].position, scene.units[3].position, &path));
    CHECK(path.size() >= 2);
    float maxSegment = 0.0f;
    for (size_t i = 0; i < path.size(); ++i) {
      const float h = scene.ground.HeightAt(path[i].x, path[i].z);
      CHECK(std::fabs(path[i].y - h) < kEps);
      if (i + 1 < path.size()) maxSegment = std::max(maxSegment, glm::distance(path[i], path[i + 1]));
    }
    // Ground segments are densified so linear interpolation between
    // waypoints hugs the slope.
    CHECK(maxSegment < 1.5f);
  }
}

void TestHillyReachFieldCostsIncludeSlope() {
  // The frontier distance between two points on a slope is the 3D walked
  // distance, so it exceeds the flat XZ distance.
  const Scene scene = GenerateHillyMap(7);
  NavMesh nav;
  nav.Build(scene.obstacles, scene.mapHalfExtent, constants::kAgentRadius, &scene.ground);
  const glm::vec3 start = scene.units[0].position;
  const ReachField field = nav.ComputeReachField(start, /*budget=*/20.0f);
  CHECK(field.nx > 0);
  float best = 0.0f;
  for (int iz = 0; iz < field.nz; ++iz) {
    for (int ix = 0; ix < field.nx; ++ix) {
      const float d = field.Dist(ix, iz);
      if (d > field.budget) continue;
      const glm::vec3 node = field.Node(ix, iz);
      const float flat = glm::length(glm::vec2(node.x - start.x, node.z - start.z));
      CHECK(d > flat - kEps);  // Routed 3D distance can't undercut straight-line XZ.
      best = std::max(best, d);
    }
  }
  CHECK(best > 15.0f);  // The field actually extends outward.
}

void TestHillyGameLogicMoveLandsOnTerrain() {
  // Full click-flow move across a hillside through GameLogic: the mover must
  // end at the destination standing on the terrain.
  Scene scene = GenerateHillyMap(7);
  GameLogic game(scene);
  const glm::vec3 start = game.GetScene().units[0].position;
  game.ClickUnit(0, Team::Blue);
  game.ChooseMove();
  const glm::vec3 dest(start.x + 14.0f, 0.0f, start.z);
  game.ClickGround(dest, Team::Blue);
  game.FinishMovePlan();
  const Unit* mover = game.FindUnit(0);
  CHECK(mover->plan.type == PlannedActionType::Move);
  CHECK(!mover->plan.movePath.empty());
  for (int id = 1; id < 6; ++id) {
    game.ClickUnit(id, id < 3 ? Team::Blue : Team::Red);
    game.ChoosePass();
  }
  CHECK(game.CanCommitRound());
  game.CommitRound();
  game.Update(1.0e6f);
  const glm::vec3 end = game.FindUnit(0)->position;
  CHECK(std::fabs(end.x - dest.x) < 0.05f && std::fabs(end.z - dest.z) < 0.05f);
  CHECK(std::fabs(end.y - game.GetScene().ground.HeightAt(end.x, end.z)) < kEps);
}

void TestHillyShortSameCellMoveStaysOnTerrain() {
  // Regression: a path whose start and goal share one navmesh slab cell took
  // an early exit that skipped the terrain lift, so the figure's first short
  // step on open hills walked a straight flat-space chord at y = 0 --
  // underground. Short moves inside the rock-free spawn strip stay within a
  // single cell, so they exercise exactly that early exit.
  for (uint32_t seed : kSeeds) {
    const Scene scene = GenerateHillyMap(seed);
    NavMesh nav;
    nav.Build(scene.obstacles, scene.mapHalfExtent, constants::kAgentRadius, &scene.ground);
    const glm::vec3 start = scene.units[0].position;
    glm::vec3 goal = start + glm::vec3(3.0f, 0.0f, 1.0f);
    goal.y = scene.ground.HeightAt(goal.x, goal.z);
    std::vector<glm::vec3> path;
    CHECK(nav.FindPath(start, goal, &path));
    CHECK(path.size() >= 2);
    for (size_t i = 0; i < path.size(); ++i) {
      const float h = scene.ground.HeightAt(path[i].x, path[i].z);
      CHECK(std::fabs(path[i].y - h) < kEps);
      if (i + 1 < path.size()) CHECK(glm::distance(path[i], path[i + 1]) < 1.5f);
    }
  }

  // Same thing through the full GameLogic click flow: after committing a
  // short first-step move, the figure must still stand on the terrain.
  GameLogic game(GenerateHillyMap(2024));
  const glm::vec3 start = game.GetScene().units[0].position;
  game.ClickUnit(0, Team::Blue);
  game.ChooseMove();
  game.ClickGround(start + glm::vec3(3.0f, 0.0f, 1.0f), Team::Blue);
  game.FinishMovePlan();
  CHECK(game.FindUnit(0)->plan.type == PlannedActionType::Move);
  for (int id = 1; id < 6; ++id) {
    game.ClickUnit(id, id < 3 ? Team::Blue : Team::Red);
    game.ChoosePass();
  }
  CHECK(game.CanCommitRound());
  game.CommitRound();
  game.Update(1.0e6f);
  const glm::vec3 end = game.FindUnit(0)->position;
  CHECK(std::fabs(end.y - game.GetScene().ground.HeightAt(end.x, end.z)) < kEps);
}

// --- Hierarchical oblique polygon city (issue #92). ---

float DistanceToSegment(glm::vec2 p, glm::vec2 a, glm::vec2 b) {
  const glm::vec2 ab = b - a;
  const float t = glm::dot(ab, ab) > 0.0f
                      ? std::clamp(glm::dot(p - a, ab) / glm::dot(ab, ab), 0.0f, 1.0f)
                      : 0.0f;
  return glm::distance(p, a + ab * t);
}

void TestObliqueArteryAndBranchMerge() {
  MapGeneratorConfig one;
  one.arteryCount = 1;
  one.elevatedHighway = OverpassMode::Off;
  const float half = UrbanMapHalfExtent(one);
  const auto oneRoads = UrbanRoads(42, one);
  CHECK(oneRoads.size() == 1);
  CHECK(oneRoads[0].artery);
  CHECK(oneRoads[0].width > one.localStreetWidth * 1.8f);
  auto onBoundary = [&](const glm::vec3& p) {
    return std::fabs(std::fabs(p.x) - half) < kEps || std::fabs(std::fabs(p.z) - half) < kEps;
  };
  CHECK(onBoundary(oneRoads[0].centerline.front()));
  CHECK(onBoundary(oneRoads[0].centerline.back()));
  // A genuine diagonal (and straight), not an axis-aligned road.
  const glm::vec2 a0(oneRoads[0].centerline.front().x, oneRoads[0].centerline.front().z);
  const glm::vec2 a1(oneRoads[0].centerline.back().x, oneRoads[0].centerline.back().z);
  const glm::vec2 arteryDir = glm::normalize(a1 - a0);
  CHECK(std::fabs(arteryDir.x) > 0.05f && std::fabs(arteryDir.y) > 0.05f);
  for (const glm::vec3& p : oneRoads[0].centerline) {
    CHECK(DistanceToSegment(glm::vec2(p.x, p.z), a0, a1) < 1.0e-2f);
    CHECK(p.y < 0.1f);  // No deck without elevatedHighway.
  }

  MapGeneratorConfig two = one;
  two.arteryCount = 2;
  const auto roads = UrbanRoads(42, two);
  CHECK(roads.size() == 2);
  CHECK(roads[1].artery);
  CHECK(roads[1].width > one.localStreetWidth);
  // The branch runs from the map boundary to a merge point exactly on the
  // artery centerline, meeting it at an oblique angle.
  CHECK(onBoundary(roads[1].centerline.front()));
  const glm::vec3 mergePoint = roads[1].centerline.back();
  CHECK(DistanceToSegment(glm::vec2(mergePoint.x, mergePoint.z), a0, a1) < 1.0e-2f);
  const glm::vec2 branchDir = glm::normalize(
      glm::vec2(mergePoint.x - roads[1].centerline.front().x,
                mergePoint.z - roads[1].centerline.front().z));
  const float cosMerge = std::fabs(glm::dot(arteryDir, branchDir));
  CHECK(cosMerge > 0.6f && cosMerge < 0.95f);  // ~30..44 degrees off the artery.
}

void TestPolygonBlocksAdaptToAngledStreets() {
  MapGeneratorConfig config;
  config.arteryCount = 2;
  config.elevatedHighway = OverpassMode::Off;
  const Scene scene = GenerateUrbanMap(42, config);
  const auto blocks = UrbanBlocks(42, config);
  const auto roads = UrbanRoads(42, config);
  CHECK(!blocks.empty());
  CHECK(!scene.obstacles.empty());
  bool angledBlock = false;
  int triangles = 0, quads = 0, fivePlus = 0;
  for (const UrbanBlock& block : blocks) {
    CHECK(block.vertices.size() >= 3);
    CHECK(PolygonArea(block.vertices) >= config.minPolygonArea);
    if (block.vertices.size() == 3) ++triangles;
    else if (block.vertices.size() == 4) ++quads;
    else ++fivePlus;
    for (size_t i = 0; i < block.vertices.size(); ++i) {
      const glm::vec2 edge = block.vertices[(i + 1) % block.vertices.size()] - block.vertices[i];
      angledBlock |= std::fabs(edge.x) > 0.1f && std::fabs(edge.y) > 0.1f;
    }
  }
  CHECK(angledBlock);
  // Genuine shape variety from the angled streets, not a uniform grid.
  CHECK(triangles > 0);
  CHECK(quads > 0);
  CHECK(fivePlus > 0);
  bool angledBuilding = false;
  bool acuteBuilding = false;
  for (const Obstacle& obstacle : scene.obstacles) {
    if (obstacle.footprint.empty()) continue;  // Elevated pier columns are boxes.
    CHECK(PolygonArea(obstacle.footprint) > 1.0f);
    CHECK(obstacle.bounds.min.x >= -scene.mapHalfExtent - kEps);
    CHECK(obstacle.bounds.max.x <= scene.mapHalfExtent + kEps);
    for (size_t i = 0; i < obstacle.footprint.size(); ++i) {
      const glm::vec2 prev = obstacle.footprint[(i + obstacle.footprint.size() - 1) %
                                                obstacle.footprint.size()];
      const glm::vec2 cur = obstacle.footprint[i];
      const glm::vec2 next = obstacle.footprint[(i + 1) % obstacle.footprint.size()];
      const glm::vec2 a = glm::normalize(prev - cur), b = glm::normalize(next - cur);
      const float angle = glm::degrees(std::acos(std::clamp(glm::dot(a, b), -1.0f, 1.0f)));
      acuteBuilding |= angle < 60.0f;
      const glm::vec2 edge = next - cur;
      angledBuilding |= std::fabs(edge.x) > 0.1f && std::fabs(edge.y) > 0.1f;
      for (const UrbanRoad& road : roads) {
        for (size_t segment = 0; segment + 1 < road.centerline.size(); ++segment) {
          const glm::vec2 p0(road.centerline[segment].x, road.centerline[segment].z);
          const glm::vec2 p1(road.centerline[segment + 1].x, road.centerline[segment + 1].z);
          CHECK(DistanceToSegment(cur, p0, p1) >= road.width * 0.5f - kEps);
        }
      }
    }
  }
  for (size_t i = 0; i < scene.obstacles.size(); ++i) {
    if (scene.obstacles[i].footprint.empty()) continue;
    for (size_t j = i + 1; j < scene.obstacles.size(); ++j) {
      if (scene.obstacles[j].footprint.empty()) continue;
      for (size_t edge = 0; edge < scene.obstacles[i].footprint.size(); ++edge) {
        const glm::vec2 a = scene.obstacles[i].footprint[edge];
        const glm::vec2 b = scene.obstacles[i].footprint[
            (edge + 1) % scene.obstacles[i].footprint.size()];
        CHECK(!SegmentEntersConvexPolygon(a, b, scene.obstacles[j].footprint));
      }
    }
  }
  CHECK(angledBuilding);
  CHECK(acuteBuilding);
}

void TestPolygonCollisionLosAndNavigationUseRealFootprint() {
  Obstacle wedge;
  wedge.bounds = AABB{glm::vec3(-2.0f, 0.0f, -2.0f), glm::vec3(2.0f, 4.0f, 2.0f)};
  wedge.footprint = {{-2.0f, -2.0f}, {2.0f, -2.0f}, {-2.0f, 2.0f}};
  const std::vector<Obstacle> obstacles = {wedge};
  // z=1.5 through x>0 lies inside the AABB but outside the triangular prism.
  CHECK(LineOfSightClear(glm::vec3(0.8f, 1.5f, 1.5f), glm::vec3(1.8f, 1.5f, 1.5f), obstacles));
  CHECK(!LineOfSightClear(glm::vec3(-3.0f, 1.5f, -1.0f),
                          glm::vec3(1.0f, 1.5f, -1.0f), obstacles));
  NavMesh nav;
  nav.Build(obstacles, 6.0f, 0.1f);
  std::vector<glm::vec3> path;
  CHECK(nav.FindPath(glm::vec3(0.8f, 0.0f, 1.5f), glm::vec3(1.8f, 0.0f, 1.5f), &path));
  CHECK(path.size() == 2);
}

void TestElevatedOverpassRampsPiersAndDistinctLayers() {
  MapGeneratorConfig config;
  config.arteryCount = 2;
  config.elevatedHighway = OverpassMode::On;
  config.overpassLayout = OverpassLayout::RampUpRampDown;
  const Scene scene = GenerateUrbanMap(7, config);
  const auto roads = UrbanRoads(7, config);

  // Overpass profile: at grade on both map boundaries, at full elevation
  // over the center, climbing/descending smoothly in between.
  const auto& deck = roads[0].centerline;
  CHECK(deck.front().y < 0.1f);
  CHECK(deck.back().y < 0.1f);
  float peak = 0.0f;
  for (const glm::vec3& p : deck) peak = std::max(peak, p.y);
  CHECK(std::fabs(peak - config.highwayElevation) < 0.1f);
  for (size_t i = 0; i + 1 < deck.size(); ++i) {
    CHECK(std::fabs(deck[i + 1].y - deck[i].y) < 1.0f);
  }
  // The branching on-ramp climbs from grade at the boundary to deck height
  // at its merge point.
  const auto& ramp = roads[1].centerline;
  CHECK(ramp.front().y < 0.1f);
  CHECK(std::fabs(ramp.back().y - config.highwayElevation) < 0.1f);

  // Bridge stands: pier columns exist, stop below the slab above them, and
  // stand under a highway corridor rather than inside a block.
  int piers = 0;
  for (const Obstacle& obstacle : scene.obstacles) {
    if (!obstacle.footprint.empty()) continue;
    ++piers;
    CHECK(obstacle.bounds.max.y > 1.0f);
    CHECK(obstacle.bounds.max.y <
          config.highwayElevation - config.highwayThickness + kEps);
    const glm::vec2 center(obstacle.bounds.Center().x, obstacle.bounds.Center().z);
    float nearest = std::numeric_limits<float>::infinity();
    for (const UrbanRoad& road : roads) {
      for (size_t i = 0; i + 1 < road.centerline.size(); ++i) {
        nearest = std::min(
            nearest, DistanceToSegment(center,
                                       glm::vec2(road.centerline[i].x, road.centerline[i].z),
                                       glm::vec2(road.centerline[i + 1].x,
                                                 road.centerline[i + 1].z)));
      }
    }
    CHECK(nearest < config.arteryWidth * 0.5f);
  }
  CHECK(piers >= 6);

  // Both artery ramp feet plus the on-ramp foot declare ground connections,
  // and the on-ramp merges mid-deck (a span with a third neighbor).
  CHECK(scene.walkSurfaces.size() > 10);
  int groundConnections = 0;
  bool mergeNode = false;
  for (const WalkSurface& surface : scene.walkSurfaces) {
    groundConnections += surface.connectsToGround ? 1 : 0;
    mergeNode |= surface.neighbors.size() >= 3;
  }
  CHECK(groundConnections == 3);
  CHECK(mergeNode);

  NavMesh nav;
  nav.Build(scene.obstacles, scene.mapHalfExtent, constants::kAgentRadius, &scene.ground,
            &scene.walkSurfaces);

  // The deck blocks LOS vertically; the air gap under the high span (away
  // from the piers, by the merge) stays clear for ground shots.
  glm::vec3 top(0.0f);
  for (const WalkSurface& surface : scene.walkSurfaces) {
    const glm::vec3 center = SurfaceCenter(surface);
    if (center.y > top.y) top = center;
  }
  CHECK(!LineOfSightClear(glm::vec3(top.x, 1.0f, top.z),
                          glm::vec3(top.x, top.y + 2.0f, top.z), scene.obstacles,
                          scene.walkSurfaces));
  const glm::vec2 a0(deck.front().x, deck.front().z);
  const glm::vec2 a1(deck.back().x, deck.back().z);
  const glm::vec2 arteryDir = glm::normalize(a1 - a0);
  const glm::vec2 arteryNormal(-arteryDir.y, arteryDir.x);
  const glm::vec2 merge(ramp.back().x, ramp.back().z);
  const glm::vec2 underA = merge + arteryNormal * 3.0f;
  const glm::vec2 underB = merge - arteryNormal * 3.0f;
  CHECK(LineOfSightClear(glm::vec3(underA.x, 1.5f, underA.y),
                         glm::vec3(underB.x, 1.5f, underB.y), scene.obstacles,
                         scene.walkSurfaces));

  // Up the west ramp onto the deck: the path's Y climbs the ramp smoothly.
  const WalkSurface& foot = scene.walkSurfaces[0];
  CHECK(foot.connectsToGround);
  float lowest = foot.vertices.front().y;
  for (const glm::vec3& v : foot.vertices) lowest = std::min(lowest, v.y);
  glm::vec3 connection(0.0f);
  int lowCount = 0;
  for (const glm::vec3& v : foot.vertices) {
    if (v.y < lowest + 1.0e-3f) {
      connection += v;
      ++lowCount;
    }
  }
  connection /= static_cast<float>(lowCount);
  const glm::vec3 start(connection.x - arteryDir.x * 5.0f, 0.0f,
                        connection.z - arteryDir.y * 5.0f);
  std::vector<glm::vec3> up;
  CHECK(nav.FindPath(start, top, &up));
  CHECK(up.size() > 3);
  CHECK(std::fabs(up.front().y) < kEps);
  CHECK(std::fabs(up.back().y - config.highwayElevation) < 0.1f);
  for (size_t i = 0; i + 1 < up.size(); ++i) {
    CHECK(std::fabs(up[i + 1].y - up[i].y) < 1.5f);
  }

  // Reaching the deck from the ground directly below takes a real route out
  // to a ramp foot; XZ overlap alone never teleports between layers.
  std::vector<glm::vec3> noTeleport;
  CHECK(nav.FindPath(glm::vec3(top.x, 0.0f, top.z), top, &noTeleport));
  float horizontalTravel = 0.0f;
  for (size_t i = 0; i + 1 < noTeleport.size(); ++i) {
    horizontalTravel += glm::length(glm::vec2(noTeleport[i + 1].x - noTeleport[i].x,
                                               noTeleport[i + 1].z - noTeleport[i].z));
  }
  CHECK(horizontalTravel > scene.mapHalfExtent * 0.5f);

  // A ground route crossing beneath the high span stays on the ground layer.
  std::vector<glm::vec3> under;
  CHECK(nav.FindPath(glm::vec3(underA.x + arteryNormal.x * 3.0f, 0.0f,
                               underA.y + arteryNormal.y * 3.0f),
                     glm::vec3(underB.x - arteryNormal.x * 3.0f, 0.0f,
                               underB.y - arteryNormal.y * 3.0f),
                     &under));
  for (const glm::vec3& p : under) CHECK(std::fabs(p.y) < kEps);

  // The movement frontier from the ramp foot climbs onto the ramp layer.
  const ReachField frontier = nav.ComputeReachField(start, 20.0f, 1.0f);
  bool frontierClimbs = false;
  for (int iz = 0; iz < frontier.nz; ++iz) {
    for (int ix = 0; ix < frontier.nx; ++ix) {
      if (frontier.Reached(ix, iz) && frontier.Node(ix, iz).y > 0.2f) frontierClimbs = true;
    }
  }
  CHECK(frontierClimbs);

  // A committed gameplay move lands exactly on a mid-ramp surface.
  Scene playable = scene;
  playable.units[0].position = start;
  const glm::vec3 rampGoal = SurfaceCenter(playable.walkSurfaces[4]);
  GameLogic game(playable);
  game.ClickUnit(0, Team::Blue);
  game.ChooseMove();
  game.ClickGround(rampGoal, Team::Blue);
  game.FinishMovePlan();
  CHECK(game.FindUnit(0)->plan.type == PlannedActionType::Move);
  for (int id = 1; id < 6; ++id) {
    game.ClickUnit(id, id < 3 ? Team::Blue : Team::Red);
    game.ChoosePass();
  }
  game.CommitRound();
  game.Update(1.0e6f);
  CHECK(glm::distance(game.FindUnit(0)->position, rampGoal) < 0.05f);
}

// --- Random overpass presence and off-map layouts (issue #130). ---

void TestOverpassPresenceAndLayoutsAreSeedDrivenAndForceable() {
  const MapGeneratorConfig autoConfig;  // Auto presence + Auto layout.
  int present = 0, absent = 0, layoutCounts[4] = {};
  for (uint32_t seed = 1; seed <= 120; ++seed) {
    const OverpassChoice choice = UrbanOverpass(seed, autoConfig);
    const OverpassChoice again = UrbanOverpass(seed, autoConfig);
    CHECK(choice.elevated == again.elevated);
    CHECK(choice.layout == again.layout);
    // The generated scene agrees with the reported choice.
    if (seed <= 8) {
      const auto roads = UrbanRoads(seed, autoConfig);
      CHECK(roads[0].elevated == choice.elevated);
      float peak = 0.0f;
      for (const glm::vec3& p : roads[0].centerline) peak = std::max(peak, p.y);
      CHECK((peak > 1.0f) == choice.elevated);
    }
    if (!choice.elevated) {
      ++absent;
      continue;
    }
    ++present;
    ++layoutCounts[static_cast<int>(choice.layout) - 1];
  }
  CHECK(present > 30 && absent > 30);               // Roughly 50/50 presence.
  for (int count : layoutCounts) CHECK(count > 0);  // Every layout occurs.

  // Forcing presence works both ways, a forced layout is honored, and the
  // legacy grid never has an overpass.
  MapGeneratorConfig off = autoConfig;
  off.elevatedHighway = OverpassMode::Off;
  MapGeneratorConfig on = autoConfig;
  on.elevatedHighway = OverpassMode::On;
  for (uint32_t seed : kSeeds) {
    CHECK(!UrbanOverpass(seed, off).elevated);
    CHECK(UrbanOverpass(seed, on).elevated);
  }
  on.overpassLayout = OverpassLayout::Through;
  CHECK(UrbanOverpass(7, on).layout == OverpassLayout::Through);
  CHECK(!UrbanOverpass(7, LegacyConfig()).elevated);

  // The seeds the overpass goldens/scenarios use produce their layout
  // naturally under the default (Auto) config, and seed 7 has no overpass.
  MapGeneratorConfig twoArteries;
  twoArteries.arteryCount = 2;
  const std::pair<uint32_t, OverpassLayout> kNatural[] = {
      {33, OverpassLayout::RampUpRampDown}, {4, OverpassLayout::Through},
      {16, OverpassLayout::EnterRampUp},    {22, OverpassLayout::EnterRampDown},
      {26, OverpassLayout::RampUpRampDown},
  };
  for (const auto& [seed, layout] : kNatural) {
    const OverpassChoice choice = UrbanOverpass(seed, twoArteries);
    CHECK(choice.elevated);
    CHECK(choice.layout == layout);
  }
  CHECK(!UrbanOverpass(7, twoArteries).elevated);
}

void TestOverpassLayoutGeometryAndBranchRampOff() {
  const struct {
    OverpassLayout layout;
    bool gradeStart, gradeEnd;
  } cases[] = {
      {OverpassLayout::RampUpRampDown, true, true},
      {OverpassLayout::Through, false, false},
      {OverpassLayout::EnterRampUp, true, false},
      {OverpassLayout::EnterRampDown, false, true},
  };
  MapGeneratorConfig config;
  config.elevatedHighway = OverpassMode::On;
  config.branchEnd = BranchEnd::Ramp;
  const float half = UrbanMapHalfExtent(config);
  for (const auto& tc : cases) {
    config.overpassLayout = tc.layout;
    for (uint32_t seed : kSeeds) {
      CHECK(UrbanOverpass(seed, config).layout == tc.layout);
      const Scene scene = GenerateUrbanMap(seed, config);
      const auto roads = UrbanRoads(seed, config);
      const auto& deck = roads[0].centerline;

      // Both end kinds are cut off exactly at the map boundary; an elevated
      // end stays at deck height there.
      auto checkEnd = [&](const glm::vec3& p, bool grade) {
        if (grade) {
          CHECK(std::fabs(std::max(std::fabs(p.x), std::fabs(p.z)) - half) < kEps);
          CHECK(p.y < 0.1f);
        } else {
          CHECK(std::fabs(std::max(std::fabs(p.x), std::fabs(p.z)) - half) < kEps);
          CHECK(std::fabs(p.y - config.highwayElevation) < 0.1f);
        }
      };
      checkEnd(deck.front(), tc.gradeStart);
      checkEnd(deck.back(), tc.gradeEnd);
      float peak = 0.0f;
      for (size_t i = 0; i < deck.size(); ++i) {
        peak = std::max(peak, deck[i].y);
        if (i) CHECK(std::fabs(deck[i].y - deck[i - 1].y) < 1.0f);  // Smooth profile.
      }
      CHECK(std::fabs(peak - config.highwayElevation) < 0.1f);

      // Each off-map end's deck surfaces straddle the boundary, so the
      // geometry is continuous across the map edge (no gap at the rim).
      int straddling = 0;
      for (const WalkSurface& surface : scene.walkSurfaces) {
        bool inside = false, outside = false;
        for (const glm::vec3& v : surface.vertices) {
          (std::max(std::fabs(v.x), std::fabs(v.z)) < half ? inside : outside) = true;
        }
        straddling += inside && outside ? 1 : 0;
      }
      const int elevatedEnds = (tc.gradeStart ? 0 : 1) + (tc.gradeEnd ? 0 : 1);
      CHECK(straddling >= elevatedEnds);
      if (elevatedEnds == 0) CHECK(straddling == 0);

      // Pier columns stay on the map even when the deck runs past it.
      for (const Obstacle& o : scene.obstacles) {
        if (!o.footprint.empty()) continue;
        CHECK(std::max({std::fabs(o.bounds.min.x), std::fabs(o.bounds.max.x),
                        std::fabs(o.bounds.min.z), std::fabs(o.bounds.max.z)}) < half);
      }

      // Ground transitions: one per on-map ramp foot plus the branch's foot.
      int groundConnections = 0;
      for (const WalkSurface& surface : scene.walkSurfaces) {
        groundConnections += surface.connectsToGround ? 1 : 0;
      }
      CHECK(groundConnections == 1 + (tc.gradeStart ? 1 : 0) + (tc.gradeEnd ? 1 : 0));

      // The branch ramps off the elevated artery down to grade: it starts at
      // grade on the boundary, tops out at deck height on the artery, and a
      // ground unit can walk up it onto the deck in every layout (for
      // `through` it is the only way up).
      CHECK(roads.size() == 2);
      const auto& ramp = roads[1].centerline;
      CHECK(ramp.front().y < 0.1f);
      CHECK(std::fabs(ramp.back().y - config.highwayElevation) < 0.1f);

      NavMesh nav;
      nav.Build(scene.obstacles, scene.mapHalfExtent, constants::kAgentRadius, &scene.ground,
                &scene.walkSurfaces);
      glm::vec3 top(0.0f);  // Highest deck surface center still on the map.
      for (const WalkSurface& surface : scene.walkSurfaces) {
        const glm::vec3 center = SurfaceCenter(surface);
        if (std::max(std::fabs(center.x), std::fabs(center.z)) > half - 2.0f) continue;
        if (center.y > top.y) top = center;
      }
      const glm::vec2 inward =
          glm::normalize(glm::vec2(ramp[1].x - ramp[0].x, ramp[1].z - ramp[0].z));
      const glm::vec3 start(ramp.front().x + inward.x * 4.0f, 0.0f,
                            ramp.front().z + inward.y * 4.0f);
      std::vector<glm::vec3> up;
      CHECK(nav.FindPath(start, top, &up));
      if (!up.empty()) {
        CHECK(std::fabs(up.front().y) < 0.1f);
        CHECK(std::fabs(up.back().y - top.y) < 0.1f);
      }
    }
  }
}

void TestBranchMayRunOffMapWithoutRamp() {
  MapGeneratorConfig config;
  config.elevatedHighway = OverpassMode::On;
  const float half = UrbanMapHalfExtent(config);

  // Auto draws both endings across seeds, never off-map under `through`,
  // and ramp lengths vary per seed.
  int offMap = 0, ramp = 0;
  std::set<int> rampLengths;
  for (uint32_t seed = 1; seed <= 60; ++seed) {
    for (const OverpassLayout layout :
         {OverpassLayout::RampUpRampDown, OverpassLayout::Through}) {
      config.overpassLayout = layout;
      const auto roads = UrbanRoads(seed, config);
      const auto& branch = roads[1].centerline;
      const bool flat = std::fabs(branch.front().y - config.highwayElevation) < 0.1f;
      if (layout == OverpassLayout::Through) CHECK(!flat);
      if (layout != OverpassLayout::RampUpRampDown) continue;
      (flat ? offMap : ramp) += 1;
      if (!flat) {
        size_t flatFrom = 0;
        while (flatFrom < branch.size() && branch[flatFrom].y < 0.1f) ++flatFrom;
        rampLengths.insert(static_cast<int>(flatFrom));
      }
    }
  }
  CHECK(offMap > 5 && ramp > 5);
  CHECK(rampLengths.size() > 3);

  // Forced off-map: deck height from end to end, past the boundary, no
  // ground connection, and still a walkable deck connected to the artery.
  config.overpassLayout = OverpassLayout::RampUpRampDown;
  config.branchEnd = BranchEnd::OffMap;
  for (uint32_t seed : kSeeds) {
    const Scene scene = GenerateUrbanMap(seed, config);
    const auto roads = UrbanRoads(seed, config);
    const auto& branch = roads[1].centerline;
    for (const glm::vec3& p : branch) CHECK(std::fabs(p.y - config.highwayElevation) < 0.1f);
    CHECK(std::fabs(std::max(std::fabs(branch.front().x), std::fabs(branch.front().z)) - half) < kEps);
    int groundConnections = 0;
    for (const WalkSurface& surface : scene.walkSurfaces) {
      groundConnections += surface.connectsToGround ? 1 : 0;
    }
    CHECK(groundConnections == 2);  // The artery's two ramp feet only.
    for (const Obstacle& o : scene.obstacles) {
      if (!o.footprint.empty()) continue;
      CHECK(std::max({std::fabs(o.bounds.min.x), std::fabs(o.bounds.max.x),
                      std::fabs(o.bounds.min.z), std::fabs(o.bounds.max.z)}) < half);
    }
  }
}

void TestSpawnReachabilityAndSurfaceSnapshotSynchronization() {
  MapGeneratorConfig config;
  config.arteryCount = 2;
  config.elevatedHighway = OverpassMode::Off;
  Scene scene = GenerateUrbanMap(2024, config);
  NavMesh nav;
  nav.Build(scene.obstacles, scene.mapHalfExtent, constants::kAgentRadius, &scene.ground,
            &scene.walkSurfaces);
  std::vector<glm::vec3> path;
  CHECK(nav.FindPath(scene.units[0].position, scene.units[3].position, &path));

  config.elevatedHighway = OverpassMode::On;
  config.overpassLayout = OverpassLayout::RampUpRampDown;
  scene = GenerateUrbanMap(7, config);
  const WalkSurface& patch = scene.walkSurfaces[3];
  const glm::vec3 surfacePoint = SurfaceCenter(patch);
  scene.units[0].position = surfacePoint;
  GameLogic simulator(scene);
  const GameSnapshot snapshot = simulator.ExportState();
  GameLogic follower(scene);
  CHECK(follower.ImportState(snapshot));
  CHECK(glm::distance(follower.FindUnit(0)->position, surfacePoint) < kEps);
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
  TestSidewalksCoverEveryBlock();
  TestLotVariety();
  TestNavMeshFullyReachableFromSpawns();
  TestWindowedNavMeshMatchesGlobalWithinBudget();
  TestGameLogicUsesRangeScopedNavMesh();
  TestVisibilityStillSpansWholeMap();
  TestObliqueArteryAndBranchMerge();
  TestPolygonBlocksAdaptToAngledStreets();
  TestPolygonCollisionLosAndNavigationUseRealFootprint();
  TestElevatedOverpassRampsPiersAndDistinctLayers();
  TestOverpassPresenceAndLayoutsAreSeedDrivenAndForceable();
  TestOverpassLayoutGeometryAndBranchRampOff();
  TestBranchMayRunOffMapWithoutRamp();
  TestSpawnReachabilityAndSurfaceSnapshotSynchronization();
  TestHillyDeterminismAndVariety();
  TestHillyTerrainIsGenuinelyUneven();
  TestHillyUnitsSpawnOnTerrain();
  TestHillyPathsFollowTerrain();
  TestHillyReachFieldCostsIncludeSlope();
  TestHillyGameLogicMoveLandsOnTerrain();
  TestHillyShortSameCellMoveStaysOnTerrain();
  if (g_failures) {
    std::fprintf(stderr, "%d check(s) failed\n", g_failures);
    return 1;
  }
  std::printf("map generator tests passed\n");
  return 0;
}
