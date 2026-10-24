// Terrain controls, seeded invariants, cliff collision and real navigation.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "game/GameLogic.h"
#include "game/Geometry.h"
#include "game/MapGenerator.h"
#include "game/NavMesh.h"
#include "game/Raycast.h"

namespace {
using namespace tactics;
int failures = 0;
uint32_t currentSeed = 0;
#define CHECK(expr) do { if (!(expr)) { \
  std::fprintf(stderr, "seed %u line %d: %s\n", currentSeed, __LINE__, #expr); ++failures; \
} } while (0)

MapGeneratorConfig TerrainConfig() {
  MapGeneratorConfig c;
  c.features.terrain.mode = FeatureMode::On;
  c.elevatedHighway = OverpassMode::Off;
  c.maxZiplinesPerBlock = 0;
  return c;
}
glm::vec3 OnGround(const Scene& scene, glm::vec3 p) {
  p.y = scene.ground.HeightAt(p.x, p.z);
  return p;
}
float FlatDistance(glm::vec3 a, glm::vec3 b) {
  return glm::length(glm::vec2(a.x - b.x, a.z - b.z));
}
void CheckGroundPath(const Scene& scene, const std::vector<glm::vec3>& path) {
  CHECK(path.size() >= 2);
  for (size_t i = 0; i < path.size(); ++i) {
    CHECK(std::fabs(path[i].y - scene.ground.HeightAt(path[i].x, path[i].z)) < 0.001f);
    if (i) CHECK(FlatDistance(path[i - 1], path[i]) < 1.001f);
  }
}

void TestControlsAndDeterminism() {
  auto c = TerrainConfig();
  const Scene on = GenerateUrbanMap(7, c), repeat = GenerateUrbanMap(7, c);
  CHECK(on.ground.heights == repeat.ground.heights);
  CHECK(on.terraces.size() == repeat.terraces.size());
  for (size_t i = 0; i < on.terraces.size(); ++i) {
    CHECK(on.terraces[i].footprint == repeat.terraces[i].footprint);
    CHECK(on.terraces[i].ramp.bottom == repeat.terraces[i].ramp.bottom);
  }
  CHECK(on.ground.heights != GenerateUrbanMap(42, c).ground.heights);
  c.features.terrain.mode = FeatureMode::Off;
  const Scene off = GenerateUrbanMap(7, c);
  CHECK(off.ground.Empty() && off.terraces.empty());
  c.features.terrain.mode = FeatureMode::Auto;
  c.features.terrain.chance = 0.0f;
  CHECK(GenerateUrbanMap(7, c).ground.Empty());
  c.features.terrain.chance = 1.0f;
  CHECK(GenerateUrbanMap(7, c).ground.heights == on.ground.heights);
  c.features.terrain.mode = FeatureMode::On;
  c.terraceCount = 0;
  CHECK(GenerateUrbanMap(7, c).terraces.empty());
  CHECK(!GenerateUrbanMap(7, c).ground.Empty());
  c.terrainAmplitude = 0.0f;
  CHECK(GenerateUrbanMap(7, c).ground.Empty());
  c = TerrainConfig();
  c.terraceCount = 2;
  for (float chance : {0.0f, 1.0f}) {
    c.cliffChance = chance;
    const Scene s = GenerateUrbanMap(7, c);
    CHECK(s.terraces.size() == 2);
    for (const auto& t : s.terraces) CHECK(t.cliff == (chance == 1.0f));
  }
  c.arteryCount = 0;
  CHECK(GenerateUrbanMap(7, c).ground.Empty());
}

void TestPinnedTerrainFingerprints() {
  struct Expected { uint32_t seed; size_t terraces; float minHeight, maxHeight; };
  for (const Expected& e : {Expected{7, 4, 0.070610f, 3.443202f},
                            Expected{42, 2, 0.068733f, 3.251197f},
                            Expected{2024, 4, 0.067942f, 3.265492f}}) {
    currentSeed = e.seed;
    const Scene s = GenerateUrbanMap(e.seed, TerrainConfig());
    const auto range = std::minmax_element(s.ground.heights.begin(), s.ground.heights.end());
    CHECK(s.terraces.size() == e.terraces);
    CHECK(std::fabs(*range.first - e.minHeight) < 0.00001f);
    CHECK(std::fabs(*range.second - e.maxHeight) < 0.00001f);
    // Street inspection and emitted pavement use the same height sampler.
    for (const auto& road : UrbanRoads(e.seed, TerrainConfig())) {
      for (auto p : road.centerline) CHECK(std::fabs(p.y - s.ground.HeightAt(p.x, p.z) - 0.025f) < 0.001f);
    }
  }
}

void TestSeedInvariants() {
  const auto c = TerrainConfig();
  for (currentSeed = 1; currentSeed <= 200; ++currentSeed) {
    const Scene s = GenerateUrbanMap(currentSeed, c);
    CHECK(s.terraces.size() >= 2 && s.terraces.size() <= 4);
    CHECK(!s.ground.Empty());
    NavMesh nav;
    nav.Build(s.obstacles, s.mapHalfExtent, constants::kAgentRadius, &s.ground, &s.walkSurfaces);
    for (const Unit& unit : s.units) {
      CHECK(nav.IsWalkable(unit.position.x, unit.position.z));
      CHECK(unit.position == OnGround(s, unit.position));
    }
    for (const TerrainTerrace& terrace : s.terraces) {
      CHECK(terrace.height > terrace.baseHeight);
      for (const TerrainAccess& access : {terrace.ramp, terrace.stair}) {
        for (int sample = 0; sample <= 10; ++sample) {
          const glm::vec3 p = glm::mix(access.bottom, access.top, sample / 10.0f);
          CHECK(nav.IsWalkable(p.x, p.z));
        }
      }
      CHECK((terrace.ramp.top.y - terrace.ramp.bottom.y) /
            FlatDistance(terrace.ramp.top, terrace.ramp.bottom) <= 0.301f);
    }
    // Buildings have a level foundation, including each footprint corner.
    // Retaining walls are deliberately below the minimum building height.
    for (const Obstacle& o : s.obstacles) {
      if (o.bounds.max.y - o.bounds.min.y < c.minBuildingHeight - 0.001f) continue;
      for (glm::vec2 p : ObstacleFootprint(o)) {
        CHECK(std::fabs(s.ground.HeightAt(p.x, p.y) - o.bounds.min.y) < 0.03f);
      }
    }
  }
}

void TestReachabilityAndGroundPaths() {
  for (uint32_t seed : {7u, 42u, 2024u, 16u}) {
    currentSeed = seed;
    auto c = TerrainConfig();
    if (seed == 16) { c.elevatedHighway = OverpassMode::On; c.arteryCount = 2; }
    const Scene s = GenerateUrbanMap(seed, c);
    NavMesh nav;
    nav.Build(s.obstacles, s.mapHalfExtent, constants::kAgentRadius, &s.ground, &s.walkSurfaces);
    std::vector<glm::vec3> path;
    // Every original low sidewalk ring remains reachable from BOTH squads.
    for (const UrbanBlock& block : UrbanBlocks(seed, c)) {
      bool checked = false;
      for (size_t edge = 0; edge < block.vertices.size() && !checked; ++edge) {
        const auto a = block.vertices[edge], b = block.vertices[(edge + 1) % block.vertices.size()];
        const glm::vec2 d = glm::normalize(b - a), p = (a + b) * 0.5f + glm::vec2(-d.y, d.x) * 0.75f;
        if (!nav.IsWalkable(p.x, p.y)) continue;
        for (int id : {0, 3}) {
          CHECK(nav.FindPath(s.units[id].position, OnGround(s, {p.x, 0.0f, p.y}), &path));
          CheckGroundPath(s, path);
        }
        checked = true;
      }
      CHECK(checked);
    }
    // Both entrances reach the high pad, and the explicit ramp/landing
    // surfaces also connect to the ground at each end.
    for (const auto& t : s.terraces) {
      for (const auto& access : {t.ramp, t.stair}) {
        for (int id : {0, 3}) {
          CHECK(nav.FindPath(s.units[id].position, OnGround(s, access.top), &path));
          CheckGroundPath(s, path);
          CHECK(nav.FindPath(s.units[id].position, access.top + glm::vec3(0, 0.025f, 0), &path));
        }
        const auto direction = glm::normalize(glm::vec3(access.top.x - access.bottom.x, 0,
                                                         access.top.z - access.bottom.z));
        CHECK(nav.FindPath(access.top + glm::vec3(0, 0.025f, 0),
                           OnGround(s, access.top + direction * 1.2f), &path));
      }
    }
  }
}

void TestCliffCollisionAndCover() {
  auto c = TerrainConfig();
  c.cliffChance = 1.0f;
  const Scene s = GenerateUrbanMap(7, c);
  NavMesh nav;
  nav.Build(s.obstacles, s.mapHalfExtent, constants::kAgentRadius, &s.ground, &s.walkSurfaces);
  int checked = 0;
  for (const auto& t : s.terraces) {
    for (size_t e = 0; e < t.footprint.size(); ++e) {
      const auto a = t.footprint[e], b = t.footprint[(e + 1) % t.footprint.size()];
      const glm::vec2 p = glm::mix(a, b, 0.2f), d = glm::normalize(b - a), n(-d.y, d.x);
      if (nav.IsWalkable(p.x, p.y)) continue;  // Entrance cut.
      const glm::vec3 low(p.x - n.x * 1.4f, t.baseHeight + 0.5f, p.y - n.y * 1.4f);
      const glm::vec3 high(p.x + n.x * 1.4f, low.y, p.y + n.y * 1.4f);
      CHECK(!LineOfSightClear(low, high, s.obstacles));
      CHECK(LineOfSightClear({low.x, t.height + 0.1f, low.z},
                            {high.x, t.height + 0.1f, high.z}, s.obstacles));
      ++checked;
    }
  }
  CHECK(checked >= 4);
}

void TestTerrainZiplines() {
  auto c = TerrainConfig();
  c.maxZiplinesPerBlock = 2;
  for (uint32_t seed : {7u, 42u, 2024u}) {
    const Scene s = GenerateUrbanMap(seed, c);
    CHECK(!s.ziplines.empty());
    for (const auto& line : s.ziplines) {
      const glm::vec3 lift(0.0f, constants::kZiplinePostHeight, 0.0f);
      CHECK(LineOfSightClear(line.a + lift, line.b + lift, s.ground));
    }
  }
}

void TestMovementAndSlopeCost() {
  Scene s = GenerateUrbanMap(7, TerrainConfig());
  const auto it = std::find_if(s.terraces.begin(), s.terraces.end(), [](const auto& t) { return t.cliff; });
  CHECK(it != s.terraces.end());
  if (it == s.terraces.end()) return;
  const auto access = it->ramp;
  const glm::vec3 start = OnGround(s, access.bottom), goal = OnGround(s, access.top);
  NavMesh nav;
  nav.Build(s.obstacles, s.mapHalfExtent, constants::kAgentRadius, &s.ground, &s.walkSurfaces);
  const ReachField reach = nav.ComputeReachField(start, 15.0f, 0.5f);
  const int ix = static_cast<int>(std::round((goal.x - reach.minX) / reach.step));
  const int iz = static_cast<int>(std::round((goal.z - reach.minZ) / reach.step));
  CHECK(reach.Reached(ix, iz));
  CHECK(reach.Dist(ix, iz) > FlatDistance(start, reach.Node(ix, iz)) + 0.1f);
  s.units[0].position = start;
  GameLogic game(s);
  game.ClickUnit(0, Team::Blue);
  game.ChooseMove();
  game.ClickGround(goal, Team::Blue);
  game.FinishMovePlan();
  CHECK(game.FindUnit(0)->plan.type == PlannedActionType::Move);
  for (int id = 1; id < 6; ++id) {
    game.ClickUnit(id, id < 3 ? Team::Blue : Team::Red);
    game.ChoosePass();
  }
  CHECK(game.CanCommitRound());
  game.CommitRound();
  game.Update(1.0e6f);
  CHECK(glm::distance(game.FindUnit(0)->position, goal) < 0.05f);
}
}  // namespace

int main() {
  TestControlsAndDeterminism();
  TestPinnedTerrainFingerprints();
  TestSeedInvariants();
  TestReachabilityAndGroundPaths();
  TestCliffCollisionAndCover();
  TestTerrainZiplines();
  TestMovementAndSlopeCost();
  if (failures) { std::fprintf(stderr, "%d terrain check(s) failed\n", failures); return 1; }
  std::puts("urban terrain tests passed (200 seeds)");
}
