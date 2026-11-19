// Terrain level controls, seeded invariants, wall cover and real navigation
// over the map-spanning levels (issue #155).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include "game/GameLogic.h"
#include "test_shard.h"
#include "game/Geometry.h"
#include "game/MapGenerator.h"
#include "OldSizeConfig.h"
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
  MapGeneratorConfig c = OldSizeConfig();
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
bool InsideConvex(const std::vector<glm::vec2>& polygon, glm::vec2 p) {
  float sign = 0.0f;
  for (size_t i = 0; i < polygon.size(); ++i) {
    const glm::vec2 a = polygon[i], e = polygon[(i + 1) % polygon.size()] - a;
    const float side = e.x * (p.y - a.y) - e.y * (p.x - a.x);
    if (std::fabs(side) < 1e-4f) continue;
    if (sign == 0.0f) sign = side;
    else if (side * sign < 0.0f) return false;
  }
  return true;
}

bool Planar(const std::vector<glm::vec3>& polygon, float tolerance = 0.01f) {
  if (polygon.size() < 4) return true;
  glm::vec3 normal(0.0f);
  for (size_t i = 1; i + 1 < polygon.size(); ++i) {
    const glm::vec3 candidate =
        glm::cross(polygon[i] - polygon[0], polygon[i + 1] - polygon[0]);
    if (glm::length(candidate) > glm::length(normal)) normal = candidate;
  }
  if (glm::length(normal) < 1e-5f) return false;
  normal = glm::normalize(normal);
  for (const glm::vec3& p : polygon) {
    if (std::fabs(glm::dot(normal, p - polygon[0])) > tolerance) return false;
  }
  return true;
}
// Width estimate of an obstacle footprint: area over its longest edge.
// Retaining walls and ramp flanks are thin; buildings are not.
float FootprintWidth(const Obstacle& o) {
  const auto footprint = ObstacleFootprint(o);
  float area = 0.0f, longest = 0.0f;
  for (size_t i = 0; i < footprint.size(); ++i) {
    const glm::vec2 a = footprint[i], b = footprint[(i + 1) % footprint.size()];
    area += a.x * b.y - b.x * a.y;
    longest = std::max(longest, glm::distance(a, b));
  }
  return longest > 1e-3f ? std::fabs(area) * 0.5f / longest : 0.0f;
}

void TestControlsAndDeterminism() {
  auto c = TerrainConfig();
  const Scene on = GenerateUrbanMap(7, c), repeat = GenerateUrbanMap(7, c);
  CHECK(on.ground.heights == repeat.ground.heights);
  CHECK(on.terrainStairs.size() == repeat.terrainStairs.size());
  for (size_t i = 0; i < on.terrainStairs.size(); ++i) {
    CHECK(on.terrainStairs[i].flight == repeat.terrainStairs[i].flight);
    CHECK(on.terrainStairs[i].landing == repeat.terrainStairs[i].landing);
  }
  CHECK(on.terrainWalls.size() == repeat.terrainWalls.size());
  for (size_t i = 0; i < on.terrainWalls.size(); ++i) {
    CHECK(on.terrainWalls[i].origin == repeat.terrainWalls[i].origin);
    CHECK(on.terrainWalls[i].rise == repeat.terrainWalls[i].rise);
    CHECK(on.terrainWalls[i].accesses.size() == repeat.terrainWalls[i].accesses.size());
    for (size_t a = 0; a < on.terrainWalls[i].accesses.size(); ++a) {
      CHECK(on.terrainWalls[i].accesses[a].bottom == repeat.terrainWalls[i].accesses[a].bottom);
    }
  }
  CHECK(on.terrainTunnels.size() == repeat.terrainTunnels.size());
  for (size_t i = 0; i < on.terrainTunnels.size(); ++i) {
    CHECK(on.terrainTunnels[i].bore == repeat.terrainTunnels[i].bore);
    CHECK(on.terrainTunnels[i].roof == repeat.terrainTunnels[i].roof);
    CHECK(on.terrainTunnels[i].entrance == repeat.terrainTunnels[i].entrance);
    CHECK(on.terrainTunnels[i].exit == repeat.terrainTunnels[i].exit);
  }
  CHECK(on.ground.heights != GenerateUrbanMap(42, c).ground.heights);
  c.features.terrain.mode = FeatureMode::Off;
  const Scene off = GenerateUrbanMap(7, c);
  CHECK(off.ground.Empty() && off.terrainWalls.empty());
  c.features.terrain.mode = FeatureMode::Auto;
  c.features.terrain.chance = 0.0f;
  CHECK(GenerateUrbanMap(7, c).ground.Empty());
  c.features.terrain.chance = 1.0f;
  CHECK(GenerateUrbanMap(7, c).ground.heights == on.ground.heights);
  c.features.terrain.mode = FeatureMode::On;
  c.terrainLevelCount = 0;  // No levels: the city stays flat.
  CHECK(GenerateUrbanMap(7, c).terrainWalls.empty());
  CHECK(GenerateUrbanMap(7, c).ground.Empty());
  c = TerrainConfig();
  c.terrainLevelCount = 2;
  CHECK(GenerateUrbanMap(7, c).terrainWalls.size() == 2);
  c.arteryCount = 0;
  CHECK(GenerateUrbanMap(7, c).ground.Empty());
}

void TestPinnedTerrainFingerprints() {
  struct Expected { uint32_t seed; size_t walls; float minHeight, maxHeight; };
  for (const Expected& e : {Expected{7, 2, 0.000000f, 4.680593f},
                            Expected{42, 1, 0.000000f, 5.304327f},
                            Expected{2024, 2, 0.000000f, 8.247633f}}) {
    currentSeed = e.seed;
    const Scene s = GenerateUrbanMap(e.seed, TerrainConfig());
    const auto range = std::minmax_element(s.ground.heights.begin(), s.ground.heights.end());
    CHECK(s.terrainWalls.size() == e.walls);
    CHECK(std::fabs(*range.first - e.minHeight) < 0.00001f);
    CHECK(std::fabs(*range.second - e.maxHeight) < 0.00001f);
    // At grade the artery rides its exact road grade, which the heightfield
    // under its corridor samples (bilinear smoothing at a ramp's ends is a
    // few centimetres off): it follows the terrain, climbing through each wall.
    for (const auto& road : UrbanRoads(e.seed, TerrainConfig())) {
      for (auto p : road.centerline) CHECK(std::fabs(p.y - s.ground.HeightAt(p.x, p.z) - 0.025f) < 0.06f);
    }
  }
}

void TestSeedInvariants() {
  const auto c = TerrainConfig();
  for (currentSeed = 1; currentSeed <= 200; ++currentSeed) {
    const Scene s = GenerateUrbanMap(currentSeed, c);
    CHECK(s.terrainWalls.size() >= 1 && s.terrainWalls.size() <= 3);
    CHECK(!s.ground.Empty());
    size_t accessCount = 0;
    NavMesh nav;
    nav.Build(s.obstacles, s.mapHalfExtent, constants::kAgentRadius, &s.ground, &s.walkSurfaces);
    for (const Unit& unit : s.units) {
      CHECK(nav.IsWalkable(unit.position.x, unit.position.z));
      CHECK(unit.position == OnGround(s, unit.position));
    }
    for (const TerrainWall& wall : s.terrainWalls) {
      CHECK(wall.rise >= constants::kUnitHeight * 0.99f);
      CHECK(wall.rise <= constants::kUnitHeight * 5.01f);
      CHECK(!wall.accesses.empty());
      accessCount += wall.accesses.size();
      for (const TerrainAccess& access : wall.accesses) {
        CHECK(std::fabs(access.top.y - access.bottom.y - wall.rise) < 0.3f);
        CHECK(nav.IsWalkable(access.bottom.x, access.bottom.z));
        CHECK(nav.IsWalkable(access.top.x, access.top.z));
        // The flight is carried by the ground: a direct route climbs it
        // (foot, straight up the wall, onto the level) at stair grade.
        std::vector<glm::vec3> path;
        CHECK(nav.FindPath(access.bottom, access.top, &path));
        float length = 0.0f;
        for (size_t i = 0; i + 1 < path.size(); ++i) length += glm::distance(path[i], path[i + 1]);
        CHECK(length < wall.rise / 0.8f + 12.0f);
      }
    }
    // Each access renders as one wedge: a planar flight and a flat landing
    // at the level's height. Roads are likewise planar patches instead of
    // being tessellated over the grid.
    CHECK(s.terrainStairs.size() == accessCount);
    for (const TerrainStair& stair : s.terrainStairs) {
      CHECK(stair.flight.size() == 4 && Planar(stair.flight));
      CHECK(stair.landing.size() == 4);
      for (const glm::vec3& v : stair.landing) {
        CHECK(std::fabs(v.y - stair.landing[0].y) < 1e-4f);
        CHECK(v.y > stair.base + 0.4f);
      }
    }
    for (const RoadSurface& road : s.roads) CHECK(Planar(road.vertices, 0.03f));
    // Buildings have a level foundation, including each footprint corner.
    // Thin obstacles are retaining walls / ramp flanks standing on slopes.
    for (size_t oi = 0; oi < s.obstacles.size(); ++oi) {
      const Obstacle& o = s.obstacles[oi];
      if (o.bounds.max.y - o.bounds.min.y < c.minBuildingHeight - 0.001f) continue;
      if (FootprintWidth(o) < 3.0f) continue;
      // A set-back top floor stands on its building's base prism, not the ground.
      if (o.building >= 0 && s.buildings[o.building].upperObstacle == static_cast<int>(oi)) continue;
      for (glm::vec2 p : ObstacleFootprint(o)) {
        // Over a tunnel bore the building stands on the roof, not the road.
        float ground = s.ground.HeightAt(p.x, p.y);
        for (const TerrainTunnel& tunnel : s.terrainTunnels) {
          if (InsideConvex(tunnel.bore, p)) ground = tunnel.roof[0].y;
        }
        CHECK(std::fabs(ground - o.bounds.min.y) < 0.03f);
      }
    }
  }
}

void TestReachabilitySeed(uint32_t only) {
  for (uint32_t seed : {7u, 42u, 2024u, 16u}) {
    if (seed != only) continue;
    currentSeed = seed;
    auto c = TerrainConfig();
    if (seed == 16) { c.elevatedHighway = OverpassMode::On; c.arteryCount = 2; }
    const Scene s = GenerateUrbanMap(seed, c);
    NavMesh nav;
    nav.Build(s.obstacles, s.mapHalfExtent, constants::kAgentRadius, &s.ground, &s.walkSurfaces);
    std::vector<glm::vec3> path;
    // Every block keeps a sidewalk spot reachable from BOTH squads, over
    // whatever stairs and ramps the route needs.
    for (const UrbanBlock& block : UrbanBlocks(seed, c)) {
      bool reachable = false, walkable = false;
      for (size_t edge = 0; edge < block.vertices.size() && !reachable; ++edge) {
        const auto a = block.vertices[edge], b = block.vertices[(edge + 1) % block.vertices.size()];
        const glm::vec2 d = glm::normalize(b - a), p = (a + b) * 0.5f + glm::vec2(-d.y, d.x) * 0.75f;
        if (!nav.IsWalkable(p.x, p.y)) continue;
        walkable = true;
        bool ok = true;
        for (int id : {0, 3}) {
          ok = ok && nav.FindPath(s.units[id].position, OnGround(s, {p.x, 0.0f, p.y}), &path);
        }
        reachable = ok;
      }
      CHECK(walkable);
      CHECK(reachable);
    }
    // Both squads reach both ends of every access, and a through-wall
    // stair carries a direct route over its wall.
    for (const TerrainWall& wall : s.terrainWalls) {
      for (const TerrainAccess& access : wall.accesses) {
        for (const glm::vec3& point : {access.bottom, access.top}) {
          for (int id : {0, 3}) {
            CHECK(nav.FindPath(s.units[id].position, OnGround(s, point), &path));
          }
        }
        CHECK(nav.FindPath(OnGround(s, access.bottom), OnGround(s, access.top), &path));
        float length = 0.0f;
        for (size_t i = 0; i + 1 < path.size(); ++i) length += glm::distance(path[i], path[i + 1]);
        CHECK(length < wall.rise / 0.8f + 12.0f);
      }
    }
  }
}

// One case per seed so CI can spread the sweep over shards.
void TestReachability_7() { TestReachabilitySeed(7); }
void TestReachability_42() { TestReachabilitySeed(42); }
void TestReachability_2024() { TestReachabilitySeed(2024); }
void TestReachability_16() { TestReachabilitySeed(16); }

void TestWallCoverAndCollision() {
  currentSeed = 7;
  const Scene s = GenerateUrbanMap(7, TerrainConfig());
  NavMesh nav;
  nav.Build(s.obstacles, s.mapHalfExtent, constants::kAgentRadius, &s.ground, &s.walkSurfaces);
  int checked = 0;
  for (const TerrainWall& wall : s.terrainWalls) {
    const glm::vec2 n = wall.raisedNormal;
    for (float t = -s.mapHalfExtent; t <= s.mapHalfExtent; t += 4.0f) {
      // Walk the step line; wherever the cliff stands (its nav wall blocks
      // the line), it blocks low LOS across and leaves the top line clear.
      const glm::vec2 onLine = wall.origin + wall.dir * t;
      // The wall's origin is the step line, where the unseen cliff wall stands.
      const glm::vec2 box = onLine;
      if (std::max(std::fabs(box.x), std::fabs(box.y)) > s.mapHalfExtent - 10.0f) continue;
      if (nav.IsWalkable(box.x, box.y)) continue;  // A gap or off the cliff.
      // A sliver beside a ramp opening is unwalkable too (the cliff beside
      // the ramp) but has no wall: only a continuous cliff counts.
      const glm::vec2 before = box - wall.dir * 1.5f, after = box + wall.dir * 1.5f;
      if (nav.IsWalkable(before.x, before.y) || nav.IsWalkable(after.x, after.y)) continue;
      // Probes just inside the level's sidewalk ring and just outside the
      // stair strip, where the profile must be a clean full step: a flight,
      // a ramp channel mouth or a building spoils the sample.
      const glm::vec2 low2 = box - n * 1.2f, high2 = box + n * 1.2f;
      const float lowY = s.ground.HeightAt(low2.x, low2.y);
      const float highY = s.ground.HeightAt(high2.x, high2.y);
      if (highY - lowY < wall.rise * 0.9f) continue;
      bool clean = true;
      for (float k = 0.6f; k <= 1.2f; k += 0.2f) {
        const glm::vec2 lo = box - n * k, hi = box + n * k;
        clean &= std::fabs(s.ground.HeightAt(lo.x, lo.y) - lowY) < 0.3f &&
                 std::fabs(s.ground.HeightAt(hi.x, hi.y) - highY) < 0.3f;
      }
      if (!clean) continue;
      const glm::vec3 low(low2.x, lowY + 0.6f, low2.y);
      const glm::vec3 high(high2.x, lowY + 0.6f, high2.y);
      CHECK(!LineOfSightClear(low, high, s.obstacles));
      CHECK(LineOfSightClear({low.x, highY + 0.2f, low.z},
                             {high.x, highY + 0.2f, high.z}, s.obstacles));
      ++checked;
    }
  }
  CHECK(checked >= 8);
}

void TestHighwayTerrainInterplay() {
  // At grade the artery rides the terrain, ramping through each wall: its
  // centerline spans the levels it crosses.
  currentSeed = 7;
  const Scene s = GenerateUrbanMap(7, TerrainConfig());
  const auto roads = UrbanRoads(7, TerrainConfig());
  float minY = 1e9f, maxY = -1e9f;
  for (const auto& p : roads[0].centerline) {
    minY = std::min(minY, p.y);
    maxY = std::max(maxY, p.y);
  }
  CHECK(maxY - minY > 1.5f);

  // Elevated, the deck starts at one level, flies clear over the walls it
  // crosses and settles onto the level at its far end; it never dips
  // below the ground beneath it.
  currentSeed = 16;
  auto elevated = TerrainConfig();
  elevated.elevatedHighway = OverpassMode::On;
  elevated.arteryCount = 2;
  const Scene e = GenerateUrbanMap(16, elevated);
  const auto elevatedRoads = UrbanRoads(16, elevated);
  float deckTop = -1e9f;
  for (const auto& p : elevatedRoads[0].centerline) {
    CHECK(p.y >= e.ground.HeightAt(p.x, p.z) - 0.01f);
    deckTop = std::max(deckTop, p.y - e.ground.HeightAt(p.x, p.z));
  }
  CHECK(deckTop > 3.0f);

  // Terrain-enabled seeds draw the overpass with a boosted chance: raised
  // highways are more common over the levels.
  MapGeneratorConfig withTerrain = OldSizeConfig();
  withTerrain.features.terrain.mode = FeatureMode::On;
  MapGeneratorConfig withoutTerrain = OldSizeConfig();
  withoutTerrain.features.terrain.mode = FeatureMode::Off;
  int boosted = 0, plain = 0;
  for (uint32_t seed = 1; seed <= 80; ++seed) {
    boosted += UrbanOverpass(seed, withTerrain).elevated ? 1 : 0;
    plain += UrbanOverpass(seed, withoutTerrain).elevated ? 1 : 0;
  }
  CHECK(boosted > plain);
}

void TestHighwayProfilesOverLevels() {
  // With the overpass forced on, the deck is planned against the walls:
  // it never dips below the ground, flies clear over every wall it crosses
  // on the deck, ramps through the others at grade, and the layouts vary.
  MapGeneratorConfig c = OldSizeConfig();
  c.elevatedHighway = OverpassMode::On;
  c.arteryCount = 2;
  int layouts[5] = {};
  for (currentSeed = 1; currentSeed <= 200; ++currentSeed) {
    const Scene s = GenerateUrbanMap(currentSeed, c);
    const auto roads = UrbanRoads(currentSeed, c);
    ++layouts[static_cast<int>(UrbanOverpass(currentSeed, c).layout)];
    float deckTop = -1e9f;
    for (const auto& p : roads[0].centerline) {
      const float lift = p.y - s.ground.HeightAt(p.x, p.z);
      CHECK(lift > -0.01f);
      deckTop = std::max(deckTop, lift);
    }
    CHECK(deckTop > 3.0f);
    // No kinks: the grade between consecutive centerline segments of either
    // highway road changes by no more than a road ramp's own grade (0.3),
    // so a deck flying over a wall or an on-ramp climbing to a tall deck
    // never folds.
    for (const auto& road : roads) {
      const auto& line = road.centerline;
      for (size_t i = 1; i + 1 < line.size(); ++i) {
        const float g0 = (line[i].y - line[i - 1].y) / FlatDistance(line[i], line[i - 1]);
        const float g1 = (line[i + 1].y - line[i].y) / FlatDistance(line[i + 1], line[i]);
        CHECK(std::fabs(g1 - g0) <= 0.31f);
      }
    }
    for (const TerrainWall& wall : s.terrainWalls) {
      // The step line is the wall strip's raised edge; the artery sample
      // nearest it is either at grade (ramped through) or well up on the
      // deck (flown over).
      const glm::vec2 line = wall.origin;
      float best = 1e9f, lift = 0.0f;
      for (const auto& p : roads[0].centerline) {
        const float d = std::fabs(glm::dot(glm::vec2(p.x, p.z) - line, wall.raisedNormal));
        if (d < best) {
          best = d;
          lift = p.y - s.ground.HeightAt(p.x, p.z);
        }
      }
      CHECK(lift < 0.1f || lift > 2.5f);
    }
  }
  CHECK(layouts[static_cast<int>(OverpassLayout::Through)] > 0);
  CHECK(layouts[static_cast<int>(OverpassLayout::RampUpRampDown)] > 0);
  CHECK(layouts[static_cast<int>(OverpassLayout::EnterRampUp)] > 0);
  CHECK(layouts[static_cast<int>(OverpassLayout::EnterRampDown)] > 0);
  // At grade, the artery climbs through every wall it meets, unless it
  // holds its grade through a raised band as a tunnel (issue #156): then
  // its centerline spans only the difference between the exterior levels.
  c.elevatedHighway = OverpassMode::Off;
  int tunnelled = 0;
  for (currentSeed = 1; currentSeed <= 200; ++currentSeed) {
    const auto roads = UrbanRoads(currentSeed, c);
    float minY = 1e9f, maxY = -1e9f;
    for (const auto& p : roads[0].centerline) {
      minY = std::min(minY, p.y);
      maxY = std::max(maxY, p.y);
    }
    if (maxY - minY > constants::kUnitHeight * 0.9f) continue;
    const Scene s = GenerateUrbanMap(currentSeed, c);
    bool highwayTunnel = false;
    for (const TerrainTunnel& tunnel : s.terrainTunnels) highwayTunnel |= tunnel.highway;
    CHECK(highwayTunnel);
    tunnelled += highwayTunnel ? 1 : 0;
  }
  CHECK(tunnelled > 0);
}

void TestStreetsKeepClearOfLevels() {
  // A local street's pavement never reaches a level band, except the avenue
  // ramped through that level, which crosses it climbing the full rise.
  const auto c = TerrainConfig();
  for (currentSeed = 1; currentSeed <= 200; ++currentSeed) {
    const Scene s = GenerateUrbanMap(currentSeed, c);
    for (const UrbanStreet& street : UrbanStreets(currentSeed, c)) {
      if (street.cls == UrbanStreetClass::Highway) continue;
      const glm::vec2 a(street.a.x, street.a.z), b(street.b.x, street.b.z);
      float expectedRise = 0.0f;  // Over every level it ramps through.
      for (const TerrainWall& wall : s.terrainWalls) {
        const float da = glm::dot(wall.raisedNormal, a - wall.origin);
        const float db = glm::dot(wall.raisedNormal, b - wall.origin);
        if ((da < 0.0f) != (db < 0.0f)) {
          expectedRise += wall.rise * ((db >= 0.0f ? 1.0f : 0.0f) - (da >= 0.0f ? 1.0f : 0.0f));
          continue;
        }
        // Both ends on one side: the band (3.2) plus the street's half width.
        CHECK(std::min(std::fabs(da), std::fabs(db)) >= 3.2f + street.width * 0.5f - 0.05f);
      }
      CHECK(std::fabs((street.b.y - street.a.y) - expectedRise) < 0.3f);
    }
  }
}

// Convex intersection of `subject` with `clip` (Sutherland-Hodgman).
std::vector<glm::vec2> ClipConvex(std::vector<glm::vec2> subject,
                                  const std::vector<glm::vec2>& clip) {
  const float orient = PolygonSignedArea(clip) >= 0.0f ? 1.0f : -1.0f;
  for (size_t i = 0; i < clip.size() && subject.size() >= 3; ++i) {
    const glm::vec2 a = clip[i], b = clip[(i + 1) % clip.size()];
    const glm::vec2 inward = glm::vec2(-(b - a).y, (b - a).x) * orient;
    const float d = glm::dot(inward, a);
    std::vector<glm::vec2> out;
    for (size_t j = 0; j < subject.size(); ++j) {
      const glm::vec2 p = subject[j], q = subject[(j + 1) % subject.size()];
      const float sp = glm::dot(inward, p) - d, sq = glm::dot(inward, q) - d;
      if (sp >= 0.0f) out.push_back(p);
      if ((sp < 0.0f && sq > 0.0f) || (sp > 0.0f && sq < 0.0f)) {
        out.push_back(p + (q - p) * (sp / (sp - sq)));
      }
    }
    subject = std::move(out);
  }
  return subject;
}

void TestSurfacesSitOnTiers() {
  // Every pavement patch lies on the levels as drawn: no road or sidewalk
  // area under a tier's top (a buried road, or a level's face standing
  // across it), every sidewalk strip flat on its tier, and every ramp
  // corridor cut out of a level carries its road. At grade and with the
  // seed-driven overpass.
  for (const bool overpass : {false, true}) {
    auto c = TerrainConfig();
    if (overpass) c.elevatedHighway = OverpassMode::Auto;
    for (currentSeed = 1; currentSeed <= 200; ++currentSeed) {
      const Scene s = GenerateUrbanMap(currentSeed, c);
      const float h = s.mapHalfExtent;
      const std::vector<TerrainTier> tiers =
          TerrainTiers(s.terrainWalls, s.terrainCutouts, {{-h, -h}, {h, -h}, {h, h}, {-h, h}});
      // No sliver pieces: a zero-width tier piece along a step line would
      // stand as a thin wall across the ramp where the tier is cut away.
      for (const TerrainTier& tier : tiers) {
        float width = std::numeric_limits<float>::infinity();
        for (size_t i = 0; i < tier.polygon.size(); ++i) {
          const glm::vec2 a = tier.polygon[i], e = tier.polygon[(i + 1) % tier.polygon.size()] - a;
          const float length = glm::length(e);
          if (length < 1e-6f) continue;
          float farthest = 0.0f;
          for (const glm::vec2& p : tier.polygon) {
            farthest = std::max(farthest, std::fabs(e.x * (p.y - a.y) - e.y * (p.x - a.x)) / length);
          }
          width = std::min(width, farthest);
        }
        CHECK(tier.polygon.size() >= 3 && width >= 0.05f);
      }
      const auto onTiers = [&](const RoadSurface& patch) {
        if (patch.vertices.size() < 3) return;
        std::vector<glm::vec2> flat;
        for (const glm::vec3& v : patch.vertices) flat.emplace_back(v.x, v.z);
        const glm::vec3 p0 = patch.vertices[0];
        glm::vec3 normal(0.0f);
        for (size_t k = 2; k < patch.vertices.size() && glm::length(normal) < 1e-4f; ++k) {
          normal = glm::cross(patch.vertices[1] - p0, patch.vertices[k] - p0);
        }
        const auto planeY = [&](glm::vec2 q) {
          if (std::fabs(normal.y) < 1e-6f) return p0.y;
          return p0.y - (normal.x * (q.x - p0.x) + normal.z * (q.y - p0.z)) / normal.y;
        };
        for (const TerrainTier& tier : tiers) {
          if (tier.height < 0.01f) continue;
          const std::vector<glm::vec2> overlap = ClipConvex(flat, tier.polygon);
          if (overlap.size() < 3 || PolygonArea(overlap) < 0.05f) continue;
          for (const glm::vec2& q : overlap) CHECK(planeY(q) >= tier.height - 0.05f);
        }
      };
      for (const RoadSurface& road : s.roads) onTiers(road);
      for (const RoadSurface& strip : s.sidewalkSurfaces) {
        float lo = std::numeric_limits<float>::infinity(), hi = -lo;
        for (const glm::vec3& v : strip.vertices) {
          lo = std::min(lo, v.y);
          hi = std::max(hi, v.y);
        }
        CHECK(hi - lo < 0.05f);
        onTiers(strip);
      }
      for (const auto& cutout : s.terrainCutouts) {
        glm::vec2 center(0.0f);
        for (const glm::vec2& p : cutout) center += p;
        center /= static_cast<float>(cutout.size());
        bool paved = false;
        for (const RoadSurface& road : s.roads) {
          std::vector<glm::vec2> flat;
          for (const glm::vec3& v : road.vertices) flat.emplace_back(v.x, v.z);
          if (flat.size() >= 3 && PointInConvexPolygon(center, flat, 1e-3f)) paved = true;
        }
        CHECK(paved);
      }
    }
  }
}

void TestTerrainZiplines() {
  auto c = TerrainConfig();
  c.maxZiplinesPerBlock = 2;
  c.enterableChance = 0.0f;  // Zipline roofs assume solid prisms.
  for (uint32_t seed : {7u, 42u, 2024u}) {
    currentSeed = seed;
    const Scene s = GenerateUrbanMap(seed, c);
    CHECK(!s.ziplines.empty());
    for (const auto& line : s.ziplines) {
      const glm::vec3 lift(0.0f, constants::kZiplinePostHeight, 0.0f);
      CHECK(LineOfSightClear(line.a + lift, line.b + lift, s.ground));
    }
  }
}

void TestTerrainTunnels() {
  int highway = 0, local = 0, downhillExit = 0;
  for (const uint32_t seed : {31u, 72u}) {
    currentSeed = seed;
    const Scene s = GenerateUrbanMap(seed, TerrainConfig());
    CHECK(!s.terrainTunnels.empty());
    NavMesh nav;
    nav.Build(s.obstacles, s.mapHalfExtent, constants::kAgentRadius, &s.ground,
              &s.walkSurfaces, &s.terrainTunnels);
    for (const TerrainTunnel& tunnel : s.terrainTunnels) {
      highway += tunnel.highway ? 1 : 0;
      local += tunnel.highway ? 0 : 1;
      downhillExit += tunnel.entrance.y > tunnel.exit.y + 0.1f ? 1 : 0;
      CHECK(tunnel.bore.size() == 4 && tunnel.roof.size() == 4);
      CHECK(tunnel.roofSurface >= 0 &&
            tunnel.roofSurface < static_cast<int>(s.walkSurfaces.size()));
      if (tunnel.roofSurface < 0 ||
          tunnel.roofSurface >= static_cast<int>(s.walkSurfaces.size())) {
        continue;
      }
      const WalkSurface& roof = s.walkSurfaces[tunnel.roofSurface];
      CHECK(roof.terrain && !roof.connectsToGround);
      CHECK(roof.groundConnections.size() >= 2);
      const float roofY = tunnel.roof.front().y;
      CHECK(roofY - std::max(tunnel.entrance.y, tunnel.exit.y) >=
            constants::kMinDeckHeadroom + 0.35f);

      const glm::vec2 a(tunnel.entrance.x, tunnel.entrance.z);
      const glm::vec2 b(tunnel.exit.x, tunnel.exit.z);
      const glm::vec2 dir = glm::normalize(b - a);
      CHECK(std::fabs(s.ground.HeightAt((a - dir * 0.5f).x, (a - dir * 0.5f).y) +
                          0.025f - tunnel.entrance.y) < 0.12f);
      CHECK(std::fabs(s.ground.HeightAt((b + dir * 0.5f).x, (b + dir * 0.5f).y) +
                          0.025f - tunnel.exit.y) < 0.12f);

      // The road stays walkable portal-to-portal, never exceeds its normal
      // grade, and the shortest route actually uses the bore below the roof.
      float last = s.ground.HeightAt(a.x, a.y);
      const float length = glm::distance(a, b);
      for (int i = 1; i <= 100; ++i) {
        const glm::vec2 p = glm::mix(a, b, static_cast<float>(i) / 100.0f);
        const float y = s.ground.HeightAt(p.x, p.y);
        CHECK(std::fabs(y - last) / (length / 100.0f) <= 0.32f);
        last = y;
      }
      std::vector<glm::vec3> path;
      glm::vec3 start = tunnel.entrance, goal = tunnel.exit;
      start.y -= 0.025f;
      goal.y -= 0.025f;
      CHECK(nav.FindPath(start, goal, &path));
      bool inBore = false;
      for (const glm::vec3& p : path) {
        inBore |= PointInConvexPolygon({p.x, p.z}, tunnel.bore) &&
                  p.y < roofY - constants::kMinDeckHeadroom;
      }
      CHECK(inBore);

      // The bore's own centerline stays walkable the whole way: a building
      // or cover item standing on the roof above must not block the road
      // below (its padded footprint used to seal the tunnel mid-way).
      for (int i = 0; i <= 100; ++i) {
        const glm::vec2 p = glm::mix(a, b, static_cast<float>(i) / 100.0f);
        if (!PointInConvexPolygon(p, tunnel.bore, -0.3f)) continue;
        CHECK(nav.IsWalkable(p.x, p.y));
      }

      // The restored level above is independently reachable from the
      // adjacent high terrain (the sidewalk strip beside the bore, with the
      // block's buildings right behind it, or a street crossing the roof);
      // it never connects vertically to the road.
      if (roof.groundConnections.size() >= 2) {
        const glm::vec3 center = SurfaceCenter(roof);
        bool reachable = false;
        for (const glm::vec3& connection : roof.groundConnections) {
          glm::vec2 outward(connection.x - center.x, connection.z - center.z);
          outward = glm::normalize(outward) * 0.3f;
          glm::vec3 outside(connection.x + outward.x, 0.0f, connection.z + outward.y);
          outside.y = s.ground.HeightAt(outside.x, outside.z);
          CHECK(std::fabs(outside.y - roofY) < 0.3f);
          reachable |= nav.FindPath(center, outside, &path) && nav.FindPath(outside, center, &path);
        }
        CHECK(reachable);
      }
    }
  }
  CHECK(highway > 0);
  CHECK(local > 0);
  CHECK(downhillExit > 0);

  // Cover items and furniture over a bore belong to the level above: they
  // stand on the roof, not on the road in the tunnel below.
  MapGeneratorConfig city;
  city.features.terrain.mode = FeatureMode::On;
  currentSeed = 31;
  const Scene s = GenerateUrbanMap(31, city);
  int overBore = 0;
  for (const Obstacle& o : s.obstacles) {
    if (!IsStreetFixture(o.kind)) continue;
    const glm::vec2 p(0.5f * (o.bounds.min.x + o.bounds.max.x),
                      0.5f * (o.bounds.min.z + o.bounds.max.z));
    for (const TerrainTunnel& tunnel : s.terrainTunnels) {
      if (!InsideConvex(tunnel.bore, p)) continue;
      ++overBore;
      CHECK(std::fabs(o.bounds.min.y - tunnel.roof.front().y) < 0.3f);
    }
  }
  CHECK(overBore > 0);
}

void TestMovementUpStair() {
  currentSeed = 7;
  Scene s = GenerateUrbanMap(7, TerrainConfig());
  const TerrainAccess* stair = nullptr;
  for (const TerrainWall& wall : s.terrainWalls) {
    for (const TerrainAccess& access : wall.accesses) {
      if (!stair || access.top.y - access.bottom.y > stair->top.y - stair->bottom.y) {
        stair = &access;
      }
    }
  }
  CHECK(stair != nullptr);
  if (!stair) return;
  const glm::vec3 start = OnGround(s, stair->bottom), goal = OnGround(s, stair->top);
  NavMesh nav;
  nav.Build(s.obstacles, s.mapHalfExtent, constants::kAgentRadius, &s.ground, &s.walkSurfaces);
  const ReachField reach = nav.ComputeReachField(start, 30.0f, 0.5f);
  const int ix = static_cast<int>(std::round((goal.x - reach.minX) / reach.step));
  const int iz = static_cast<int>(std::round((goal.z - reach.minZ) / reach.step));
  CHECK(reach.Reached(ix, iz));
  // The climb costs more than the flat distance.
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

void TestStairSidesSealed() {
  // A stair's open sides are cliffs: its outer edge is walled along the
  // whole flight and landing, however short the wall pieces are, so nothing
  // steps off onto the lower ground beside it.
  int checked = 0;
  for (currentSeed = 1; currentSeed <= 40; ++currentSeed) {
    const Scene s = GenerateUrbanMap(currentSeed, TerrainConfig());
    NavMesh nav;
    nav.Build(s.obstacles, s.mapHalfExtent, constants::kAgentRadius, &s.ground, &s.walkSurfaces);
    for (const TerrainStair& stair : s.terrainStairs) {
      const glm::vec3 a = stair.flight[1], b = stair.landing[2], inner = stair.flight[0];
      const glm::vec2 n = glm::normalize(glm::vec2(a.x - inner.x, a.z - inner.z));
      for (float k = 0.0f; k <= 1.0f; k += 0.05f) {
        const glm::vec2 e = glm::mix(glm::vec2(a.x, a.z), glm::vec2(b.x, b.z), k);
        const glm::vec2 on = e - n * 0.9f, off = e + n * 1.1f;
        if (s.ground.HeightAt(on.x, on.y) - s.ground.HeightAt(off.x, off.y) < 1.0f) continue;
        ++checked;
        CHECK(!nav.IsWalkable(e.x, e.y));
      }
    }
  }
  CHECK(checked > 0);
}

void TestNoWalkableCliffLinksSeeds(uint32_t lo, uint32_t hi) {
  // No two neighbouring walkable points straddle a cliff: every drop is
  // walled, including the wall stretch beside a road ramping through a step
  // (seed 7 left it open). The map rim is skipped.
  int checked = 0;
  for (currentSeed = lo; currentSeed <= hi; ++currentSeed) {
    MapGeneratorConfig c;
    c.elevatedHighway = OverpassMode::Off;
    const Scene s = GenerateUrbanMap(currentSeed, c);
    NavMesh nav;
    nav.Build(s.obstacles, s.mapHalfExtent, constants::kAgentRadius, &s.ground, &s.walkSurfaces);
    const float lim = s.mapHalfExtent - 4.0f;
    for (float x = -lim; x < lim; x += 0.5f) {
      for (float z = -lim; z < lim; z += 0.5f) {
        if (!nav.IsWalkable(x, z)) continue;
        const float y = s.ground.HeightAt(x, z);
        for (const glm::vec2 d : {glm::vec2(0.5f, 0.0f), glm::vec2(0.0f, 0.5f)}) {
          if (!nav.IsWalkable(x + d.x, z + d.y)) continue;
          ++checked;
          CHECK(std::fabs(y - s.ground.HeightAt(x + d.x, z + d.y)) < 1.2f);
        }
      }
    }
  }
  CHECK(checked > 0);
}

// Split into seed ranges so CI can spread the (slowest) sweep over shards.
void TestNoWalkableCliffLinks_1_3() { TestNoWalkableCliffLinksSeeds(1, 3); }
void TestNoWalkableCliffLinks_4_6() { TestNoWalkableCliffLinksSeeds(4, 6); }
void TestNoWalkableCliffLinks_7_9() { TestNoWalkableCliffLinksSeeds(7, 9); }
void TestNoWalkableCliffLinks_10_12() { TestNoWalkableCliffLinksSeeds(10, 12); }

// Sheer faces are not walkable (issue #221). A plateau (x >= 6, 3 high)
// stands over flat ground behind a cliff; with `ramp` a 2-wide strip at
// z in [4, 6] climbs it at grade 0.5. No obstacles, so only the slope rule
// can keep a figure off the cliff and off the strip's sheer sides.
HeightField PlateauField(bool ramp) {
  HeightField field;
  field.minX = field.minZ = -10.0f;
  field.step = 0.5f;
  field.nx = field.nz = 41;
  field.heights.assign(static_cast<size_t>(field.nx) * field.nz, 0.0f);
  for (int iz = 0; iz < field.nz; ++iz) {
    for (int ix = 0; ix < field.nx; ++ix) {
      const float x = field.minX + ix * field.step, z = field.minZ + iz * field.step;
      float h = x >= 6.0f ? 3.0f : 0.0f;
      if (ramp && z >= 4.0f && z <= 6.0f && x < 6.0f) h = std::clamp(x * 0.5f, 0.0f, 3.0f);
      field.heights[static_cast<size_t>(iz) * field.nx + ix] = h;
    }
  }
  return field;
}
float WorstGrade(const HeightField& field, const std::vector<glm::vec3>& path) {
  float worst = 0.0f;
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    const glm::vec2 a(path[i].x, path[i].z), b(path[i + 1].x, path[i + 1].z);
    const int pieces = std::max(1, static_cast<int>(std::ceil(glm::distance(a, b) / 0.25f)));
    const float ds = glm::distance(a, b) / pieces;
    for (int k = 0; k < pieces; ++k) {
      const glm::vec2 p = a + (b - a) * (static_cast<float>(k) / pieces);
      const glm::vec2 q = a + (b - a) * (static_cast<float>(k + 1) / pieces);
      worst = std::max(worst, std::fabs(field.HeightAt(q.x, q.y) - field.HeightAt(p.x, p.y)) / ds);
    }
  }
  return worst;
}

void TestSheerFacesAreImpassable() {
  currentSeed = 0;
  const glm::vec3 start(0.0f, 0.0f, 0.0f), top(10.0f, 3.0f, 0.0f);
  std::vector<glm::vec3> path;

  // No ramp: the plateau is sealed off, for paths and for the reach field.
  HeightField sheer = PlateauField(false);
  NavMesh sealed;
  sealed.Build({}, 10.0f, constants::kAgentRadius, &sheer, nullptr);
  CHECK(!sealed.FindPath(start, top, &path));
  const auto reaches = [](const ReachField& field, float x, float z) {
    return field.Reached(static_cast<int>(std::lround((x - field.minX) / field.step)),
                         static_cast<int>(std::lround((z - field.minZ) / field.step)));
  };
  CHECK(reaches(sealed.ComputeReachField(start, 40.0f), -5.0f, 0.0f));
  CHECK(!reaches(sealed.ComputeReachField(start, 40.0f), 7.0f, 0.0f));

  // With the ramp the straight line is refused but the ramp reaches the top,
  // never steeper than the walkable grade, entering at its low end.
  HeightField stepped = PlateauField(true);
  NavMesh open;
  open.Build({}, 10.0f, constants::kAgentRadius, &stepped, nullptr);
  CHECK(open.FindPath(start, top, &path));
  CHECK(path.size() >= 2);
  CHECK(WorstGrade(stepped, path) <= constants::kMaxWalkSlope + 0.05f);
  bool throughRamp = false;
  for (const glm::vec3& p : path) throughRamp |= p.x > 1.0f && p.x < 5.0f && p.z > 3.9f && p.z < 6.1f;
  CHECK(throughRamp);
  CHECK(reaches(open.ComputeReachField(start, 40.0f), 7.0f, 0.0f));
  // And back down the same way: no stepping off the cliff or the strip's side.
  CHECK(open.FindPath(top, start, &path));
  CHECK(WorstGrade(stepped, path) <= constants::kMaxWalkSlope + 0.05f);
  // A hop onto the ramp from its sheer side is refused when it is the only way.
  CHECK(!sealed.FindPath(start, glm::vec3(7.0f, 3.0f, 0.0f), &path));
}

// Seed 87 raises a stair-and-shell building over a tunnel bore: its ground
// floor is the roof level and its stair foot stands on the roof, not on the
// road in the bore. A route to the upper floor must climb the stair from the
// roof (never hop up from the road below), and a figure on the roof must not
// walk through the building's walls.
void TestBuildingOverTunnelRoof() {
  currentSeed = 87;
  const Scene s = GenerateUrbanMap(87, TerrainConfig());
  NavMesh nav;
  nav.Build(s.obstacles, s.mapHalfExtent, constants::kAgentRadius, &s.ground,
            &s.walkSurfaces, &s.terrainTunnels);
  int checkedFlights = 0;
  for (const TerrainTunnel& tunnel : s.terrainTunnels) {
    if (tunnel.roofSurface < 0) continue;
    const WalkSurface& roof = s.walkSurfaces[tunnel.roofSurface];
    const float roofY = roof.vertices.front().y;
    std::vector<glm::vec2> outline;
    for (const glm::vec3& v : roof.vertices) outline.emplace_back(v.x, v.z);
    for (size_t i = 0; i < s.walkSurfaces.size(); ++i) {
      const WalkSurface& flight = s.walkSurfaces[i];
      if (!flight.stair || !flight.connectsToGround || flight.building < 0) continue;
      float lowest = 1e9f, highest = -1e9f;
      glm::vec3 foot(0.0f), top(0.0f);
      for (const glm::vec3& v : flight.vertices) {
        lowest = std::min(lowest, v.y);
        highest = std::max(highest, v.y);
      }
      int feet = 0, tops = 0;
      for (const glm::vec3& v : flight.vertices) {
        if (v.y < lowest + 1e-3f) { foot += v; ++feet; }
        if (v.y > highest - 1e-3f) { top += v; ++tops; }
      }
      foot /= static_cast<float>(feet);
      top /= static_cast<float>(tops);
      if (std::fabs(lowest - roofY) > 0.3f ||
          std::fabs(s.ground.HeightAt(foot.x, foot.z) - foot.y) < 1.0f ||
          !PointInConvexPolygon({foot.x, foot.z}, outline)) {
        continue;
      }
      ++checkedFlights;
      // From a roof point beside the building to the top of the flight.
      const glm::vec3 center = SurfaceCenter(roof);
      std::vector<glm::vec3> path;
      CHECK(nav.FindPath(center, top + glm::vec3(0.0f, 0.1f, 0.0f), &path));
      for (size_t k = 0; k + 1 < path.size(); ++k) {
        const float run = glm::length(glm::vec2(path[k + 1].x - path[k].x,
                                                 path[k + 1].z - path[k].z));
        CHECK(std::fabs(path[k + 1].y - path[k].y) <= 1.5f * run + 0.05f);
        // Never through a wall standing on the roof level.
        if (path[k].y > roofY + 0.3f || path[k + 1].y > roofY + 0.3f) continue;
        for (const Obstacle& o : s.obstacles) {
          if (o.footprint.size() < 3 || std::fabs(o.bounds.min.y - roofY) > 0.3f ||
              o.bounds.max.y - roofY < 0.9f) {
            continue;
          }
          CHECK(!SegmentEntersConvexPolygon({path[k].x, path[k].z},
                                            {path[k + 1].x, path[k + 1].z}, o.footprint));
        }
      }
    }
  }
  CHECK(checkedFlights > 0);
}

// A figure on the tier beside a tunnel roof steps onto the slab anywhere along
// its long edges, not only at the declared connection points: the movement
// frontier must carry across the slab edge (seed 87, start near an edge end).
void TestRoofReachableFromAnyEdgePoint() {
  currentSeed = 87;
  const Scene s = GenerateUrbanMap(87, TerrainConfig());
  NavMesh nav;
  nav.Build(s.obstacles, s.mapHalfExtent, constants::kAgentRadius, &s.ground,
            &s.walkSurfaces, &s.terrainTunnels);
  int probes = 0;
  for (const TerrainTunnel& tunnel : s.terrainTunnels) {
    if (tunnel.roofSurface < 0) continue;
    const WalkSurface& roof = s.walkSurfaces[tunnel.roofSurface];
    const float roofY = roof.vertices.front().y;
    std::vector<glm::vec2> outline;
    glm::vec2 center(0.0f);
    for (const glm::vec3& v : roof.vertices) {
      outline.emplace_back(v.x, v.z);
      center += outline.back() * 0.25f;
    }
    for (const size_t e : {size_t{0}, size_t{2}}) {
      const glm::vec3 a = roof.vertices[e], b = roof.vertices[e + 1];
      const glm::vec2 dir = glm::normalize(glm::vec2(b.x - a.x, b.z - a.z));
      glm::vec2 out(-dir.y, dir.x);
      const glm::vec3 m = a + (b - a) * 0.1f;
      if (glm::dot(out, glm::vec2(m.x, m.z) - center) < 0.0f) out = -out;
      glm::vec3 start(m.x + out.x * 0.8f, 0.0f, m.z + out.y * 0.8f);
      start.y = s.ground.HeightAt(start.x, start.z);
      if (!nav.IsWalkable(start.x, start.z) || std::fabs(start.y - roofY) > 0.3f) continue;
      ++probes;
      const ReachField field = nav.ComputeReachField(start, 5.0f);
      for (int iz = 0; iz < field.nz; ++iz) {
        for (int ix = 0; ix < field.nx; ++ix) {
          const glm::vec3 p = field.Node(ix, iz);
          if (!PointInConvexPolygon({p.x, p.z}, outline)) continue;
          if (glm::length(glm::vec2(p.x - start.x, p.z - start.z)) > 2.5f) continue;
          bool blocked = false;  // Walls standing on the roof level.
          for (const Obstacle& o : s.obstacles) {
            blocked |= std::fabs(o.bounds.min.y - roofY) < 0.3f &&
                       p.x > o.bounds.min.x - 0.5f && p.x < o.bounds.max.x + 0.5f &&
                       p.z > o.bounds.min.z - 0.5f && p.z < o.bounds.max.z + 0.5f;
          }
          if (!blocked) CHECK(field.Reached(ix, iz));
        }
      }
    }
  }
  CHECK(probes > 0);
}

int main(int argc, char** argv) {
  const std::vector<TestCase> cases = {
      TBG_TEST(TestStairSidesSealed),
      TBG_TEST(TestNoWalkableCliffLinks_1_3),
      TBG_TEST(TestNoWalkableCliffLinks_4_6),
      TBG_TEST(TestNoWalkableCliffLinks_7_9),
      TBG_TEST(TestNoWalkableCliffLinks_10_12),
      TBG_TEST(TestControlsAndDeterminism),
      TBG_TEST(TestPinnedTerrainFingerprints),
      TBG_TEST(TestSeedInvariants),
      TBG_TEST(TestReachability_7),
      TBG_TEST(TestReachability_42),
      TBG_TEST(TestReachability_2024),
      TBG_TEST(TestReachability_16),
      TBG_TEST(TestWallCoverAndCollision),
      TBG_TEST(TestHighwayTerrainInterplay),
      TBG_TEST(TestHighwayProfilesOverLevels),
      TBG_TEST(TestStreetsKeepClearOfLevels),
      TBG_TEST(TestSurfacesSitOnTiers),
      TBG_TEST(TestTerrainTunnels),
      TBG_TEST(TestBuildingOverTunnelRoof),
      TBG_TEST(TestRoofReachableFromAnyEdgePoint),
      TBG_TEST(TestTerrainZiplines),
      TBG_TEST(TestMovementUpStair),
      TBG_TEST(TestSheerFacesAreImpassable),
  };
  if (!RunSelectedTests(argc, argv, cases)) return 2;
  if (failures) { std::fprintf(stderr, "%d terrain check(s) failed\n", failures); return 1; }
  std::puts("urban terrain tests passed (200 seeds)");
}
