// Terrain level controls, seeded invariants, wall cover and real navigation
// over the map-spanning levels (issue #155).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include "game/GameLogic.h"
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
        CHECK(std::fabs(s.ground.HeightAt(p.x, p.y) - o.bounds.min.y) < 0.03f);
      }
    }
  }
}

void TestReachability() {
  for (uint32_t seed : {7u, 42u, 2024u, 16u}) {
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
  // At grade, the artery climbs through every wall it meets.
  c.elevatedHighway = OverpassMode::Off;
  for (currentSeed = 1; currentSeed <= 200; ++currentSeed) {
    const auto roads = UrbanRoads(currentSeed, c);
    float minY = 1e9f, maxY = -1e9f;
    for (const auto& p : roads[0].centerline) {
      minY = std::min(minY, p.y);
      maxY = std::max(maxY, p.y);
    }
    CHECK(maxY - minY > constants::kUnitHeight * 0.9f);
  }
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

int main() {
  TestControlsAndDeterminism();
  TestPinnedTerrainFingerprints();
  TestSeedInvariants();
  TestReachability();
  TestWallCoverAndCollision();
  TestHighwayTerrainInterplay();
  TestHighwayProfilesOverLevels();
  TestStreetsKeepClearOfLevels();
  TestSurfacesSitOnTiers();
  TestTerrainZiplines();
  TestMovementUpStair();
  if (failures) { std::fprintf(stderr, "%d terrain check(s) failed\n", failures); return 1; }
  std::puts("urban terrain tests passed (200 seeds)");
}
