// Seeded regression tests for the procedural urban map generator and the
// range-scoped navmesh it feeds. Headless: no SDL/GL/ImGui.

#include <algorithm>
#include <cmath>
#include <cstdio>
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

void TestSidewalksCoverEveryBlock() {
  const MapGeneratorConfig c;
  for (uint32_t seed : kSeeds) {
    const Scene scene = GenerateUrbanMap(seed);
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

// --- Hierarchical polygon city (issue #92). ---

float DistanceToSegment(glm::vec2 p, glm::vec2 a, glm::vec2 b) {
  const glm::vec2 ab = b - a;
  const float t = glm::dot(ab, ab) > 0.0f
                      ? std::clamp(glm::dot(p - a, ab) / glm::dot(ab, ab), 0.0f, 1.0f)
                      : 0.0f;
  return glm::distance(p, a + ab * t);
}

void TestArteriesAreFirstWideAndSmooth() {
  MapGeneratorConfig one;
  one.arteryCount = 1;
  const auto oneRoads = UrbanRoads(42, one);
  CHECK(oneRoads.size() == 1);
  CHECK(oneRoads[0].artery);
  CHECK(oneRoads[0].width > one.localStreetWidth * 1.8f);
  const float half = UrbanMapHalfExtent(one);
  CHECK(std::fabs(oneRoads[0].centerline.front().x + half) < kEps);
  CHECK(std::fabs(oneRoads[0].centerline.back().x - half) < kEps);

  MapGeneratorConfig two = one;
  two.arteryCount = 2;
  const auto roads = UrbanRoads(42, two);
  CHECK(roads.size() == 2);
  CHECK(roads[1].centerline.size() > 10);
  glm::vec2 primary = glm::normalize(glm::vec2(
      roads[0].centerline.back().x - roads[0].centerline.front().x,
      roads[0].centerline.back().z - roads[0].centerline.front().z));
  const size_t n = roads[1].centerline.size();
  glm::vec2 mergeTangent = glm::normalize(glm::vec2(
      roads[1].centerline[n - 1].x - roads[1].centerline[n - 2].x,
      roads[1].centerline[n - 1].z - roads[1].centerline[n - 2].z));
  CHECK(glm::dot(primary, mergeTangent) > 0.995f);
  for (size_t i = 1; i + 1 < n; ++i) {
    const glm::vec2 a = glm::normalize(glm::vec2(
        roads[1].centerline[i].x - roads[1].centerline[i - 1].x,
        roads[1].centerline[i].z - roads[1].centerline[i - 1].z));
    const glm::vec2 b = glm::normalize(glm::vec2(
        roads[1].centerline[i + 1].x - roads[1].centerline[i].x,
        roads[1].centerline[i + 1].z - roads[1].centerline[i].z));
    CHECK(glm::dot(a, b) > 0.97f);
  }
}

void TestPolygonBlocksBuildingsAndRoadClearance() {
  MapGeneratorConfig config;
  config.arteryCount = 2;
  const Scene scene = GenerateUrbanMap(42, config);
  const auto blocks = UrbanBlocks(42, config);
  const auto roads = UrbanRoads(42, config);
  CHECK(!blocks.empty());
  CHECK(!scene.obstacles.empty());
  bool angledBlock = false;
  bool angledBuilding = false;
  bool acuteBuilding = false;
  for (const UrbanBlock& block : blocks) {
    CHECK(block.vertices.size() >= 3);
    CHECK(PolygonArea(block.vertices) >= config.minPolygonArea);
    for (size_t i = 0; i < block.vertices.size(); ++i) {
      const glm::vec2 edge = block.vertices[(i + 1) % block.vertices.size()] - block.vertices[i];
      angledBlock |= std::fabs(edge.x) > 0.1f && std::fabs(edge.y) > 0.1f;
    }
  }
  for (const Obstacle& obstacle : scene.obstacles) {
    if (obstacle.footprint.empty()) continue;  // Elevated supports are boxes.
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
  CHECK(angledBlock);
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

void TestElevatedRampDeckAndGroundRemainDistinct() {
  MapGeneratorConfig config;
  config.arteryCount = 2;
  config.elevatedHighway = true;
  const Scene scene = GenerateUrbanMap(7, config);
  CHECK(scene.walkSurfaces.size() > 10);
  CHECK(scene.walkSurfaces[1].connectsToGround);
  CHECK(std::fabs(scene.walkSurfaces[0].vertices.front().x + scene.mapHalfExtent) < kEps);
  CHECK(std::fabs(scene.walkSurfaces[0].vertices[2].x - scene.mapHalfExtent) < kEps);

  NavMesh nav;
  nav.Build(scene.obstacles, scene.mapHalfExtent, constants::kAgentRadius, &scene.ground,
            &scene.walkSurfaces);
  const WalkSurface& rampFoot = scene.walkSurfaces[1];
  glm::vec3 start = (rampFoot.vertices[0] + rampFoot.vertices[1]) * 0.5f;
  const glm::vec3 deckGoal = SurfaceCenter(scene.walkSurfaces[0]);
  CHECK(!LineOfSightClear(glm::vec3(deckGoal.x, 1.0f, deckGoal.z),
                          glm::vec3(deckGoal.x, 7.0f, deckGoal.z), scene.obstacles,
                          scene.walkSurfaces));
  CHECK(LineOfSightClear(glm::vec3(deckGoal.x - 2.0f, 1.5f, deckGoal.z + 10.0f),
                         glm::vec3(deckGoal.x + 2.0f, 1.5f, deckGoal.z + 10.0f), scene.obstacles,
                         scene.walkSurfaces));
  std::vector<glm::vec3> up;
  CHECK(nav.FindPath(start, deckGoal, &up));
  CHECK(up.size() > 3);
  CHECK(std::fabs(up.front().y) < kEps);
  CHECK(std::fabs(up.back().y - config.highwayElevation) < kEps);
  for (size_t i = 0; i + 1 < up.size(); ++i) {
    CHECK(std::fabs(up[i + 1].y - up[i].y) < 1.5f);
  }

  std::vector<glm::vec3> noTeleport;
  CHECK(nav.FindPath(glm::vec3(deckGoal.x, 0.0f, deckGoal.z), deckGoal, &noTeleport));
  float horizontalTravel = 0.0f;
  for (size_t i = 0; i + 1 < noTeleport.size(); ++i) {
    horizontalTravel += glm::length(glm::vec2(noTeleport[i + 1].x - noTeleport[i].x,
                                               noTeleport[i + 1].z - noTeleport[i].z));
  }
  CHECK(horizontalTravel > scene.mapHalfExtent * 0.5f);

  // A route under the deck stays on ground; XZ overlap alone never snaps it
  // onto the elevated surface.
  std::vector<glm::vec3> under;
  CHECK(nav.FindPath(glm::vec3(-4.0f, 0.0f, -12.0f),
                     glm::vec3(-4.0f, 0.0f, 12.0f), &under));
  for (const glm::vec3& p : under) CHECK(std::fabs(p.y) < kEps);

  const ReachField frontier = nav.ComputeReachField(start, 20.0f, 1.0f);
  bool frontierClimbs = false;
  for (int iz = 0; iz < frontier.nz; ++iz) {
    for (int ix = 0; ix < frontier.nx; ++ix) {
      if (frontier.Reached(ix, iz) && frontier.Node(ix, iz).y > 0.2f) frontierClimbs = true;
    }
  }
  CHECK(frontierClimbs);

  Scene playable = scene;
  playable.units[0].position = start;
  const glm::vec3 nearbyRampGoal = SurfaceCenter(playable.walkSurfaces[4]);
  GameLogic game(playable);
  game.ClickUnit(0, Team::Blue);
  game.ChooseMove();
  game.ClickGround(nearbyRampGoal, Team::Blue);
  game.FinishMovePlan();
  CHECK(game.FindUnit(0)->plan.type == PlannedActionType::Move);
  for (int id = 1; id < 6; ++id) {
    game.ClickUnit(id, id < 3 ? Team::Blue : Team::Red);
    game.ChoosePass();
  }
  game.CommitRound();
  game.Update(1.0e6f);
  CHECK(glm::distance(game.FindUnit(0)->position, nearbyRampGoal) < 0.05f);
}

void TestSpawnReachabilityAndSurfaceSnapshotSynchronization() {
  MapGeneratorConfig config;
  config.arteryCount = 2;
  Scene scene = GenerateUrbanMap(2024, config);
  NavMesh nav;
  nav.Build(scene.obstacles, scene.mapHalfExtent, constants::kAgentRadius, &scene.ground,
            &scene.walkSurfaces);
  std::vector<glm::vec3> path;
  CHECK(nav.FindPath(scene.units[0].position, scene.units[3].position, &path));

  config.elevatedHighway = true;
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
  TestMapIsMuchLargerThanDefault();
  TestVariedHeightsWithFewTowers();
  TestVisibilityStillSpansWholeMap();
  TestArteriesAreFirstWideAndSmooth();
  TestPolygonBlocksBuildingsAndRoadClearance();
  TestPolygonCollisionLosAndNavigationUseRealFootprint();
  TestElevatedRampDeckAndGroundRemainDistinct();
  TestSpawnReachabilityAndSurfaceSnapshotSynchronization();
  TestHillyDeterminismAndVariety();
  TestHillyTerrainIsGenuinelyUneven();
  TestHillyUnitsSpawnOnTerrain();
  TestHillyPathsFollowTerrain();
  TestHillyReachFieldCostsIncludeSlope();
  TestHillyGameLogicMoveLandsOnTerrain();
  if (g_failures) {
    std::fprintf(stderr, "%d check(s) failed\n", g_failures);
    return 1;
  }
  std::printf("map generator tests passed\n");
  return 0;
}
