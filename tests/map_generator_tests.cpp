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
#include "OldSizeConfig.h"
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
float DistanceToSegment(glm::vec2 p, glm::vec2 a, glm::vec2 b);

// The original orthogonal grid generator stays available behind
// arteryCount=0; the structural grid/lot tests run against this config.
MapGeneratorConfig LegacyConfig() {
  MapGeneratorConfig c = OldSizeConfig();
  c.arteryCount = 0;
  return c;
}

// The default city with the terrain levels off: the archetype and parcel
// tests assume one building per parcel on flat ground.
MapGeneratorConfig FlatConfig() {
  MapGeneratorConfig c = OldSizeConfig();
  c.features.terrain.mode = FeatureMode::Off;
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
        x.climbable != y.climbable || x.footprint != y.footprint || x.kind != y.kind ||
        x.fixtureModel != y.fixtureModel || x.fixtureYaw != y.fixtureYaw ||
        x.fixtureBaseY != y.fixtureBaseY || x.fixtureRoll != y.fixtureRoll) {
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
  if (a.facadeOpenings.size() != b.facadeOpenings.size()) return false;
  for (size_t i = 0; i < a.facadeOpenings.size(); ++i) {
    const FacadeOpening& x = a.facadeOpenings[i];
    const FacadeOpening& y = b.facadeOpenings[i];
    if (x.kind != y.kind || x.obstacle != y.obstacle || x.wallEdge != y.wallEdge ||
        x.u0 != y.u0 || x.u1 != y.u1 || x.y0 != y.y0 || x.y1 != y.y1) {
      return false;
    }
  }
  if (a.ziplines.size() != b.ziplines.size()) return false;
  for (size_t i = 0; i < a.ziplines.size(); ++i) {
    if (a.ziplines[i].a != b.ziplines[i].a || a.ziplines[i].b != b.ziplines[i].b) return false;
  }
  if (a.props.size() != b.props.size()) return false;
  for (size_t i = 0; i < a.props.size(); ++i) {
    if (a.props[i].kind != b.props[i].kind || a.props[i].position != b.props[i].position ||
        a.props[i].yaw != b.props[i].yaw || a.props[i].secondaryYaw != b.props[i].secondaryYaw) {
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
    CHECK(SameScene(GenerateUrbanMap(seed, OldSizeConfig()), GenerateUrbanMap(seed, OldSizeConfig())));
  }
  CHECK(!SameScene(GenerateUrbanMap(1, OldSizeConfig()), GenerateUrbanMap(42, OldSizeConfig())));
  CHECK(!SameScene(GenerateUrbanMap(42, OldSizeConfig()), GenerateUrbanMap(2024, OldSizeConfig())));
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

// Issue #151: parcels are the unit the plan sizes buildings by. Each maps
// 1:1 onto a Building/Obstacle. Frontage-ring parcels lie on the block's
// building line and match the street alongside it; detached apartment slabs
// use a principal facade inside the block instead.
void TestParcelsMatchBuildingsAndFrontStreets() {
  const MapGeneratorConfig c = OldSizeConfig();
  for (uint32_t seed : kSeeds) {
    const Scene scene = GenerateUrbanMap(seed, FlatConfig());
    const std::vector<UrbanParcel> parcels = UrbanParcels(seed, FlatConfig());
    const std::vector<UrbanBlock> blocks = UrbanBlocks(seed, FlatConfig());
    const std::vector<UrbanStreet> streets = UrbanStreets(seed, FlatConfig());
    CHECK(!parcels.empty());
    CHECK(parcels.size() == scene.buildings.size());
    int highway = 0, avenue = 0, cross = 0, corners = 0;
    for (size_t i = 0; i < parcels.size() && i < scene.buildings.size(); ++i) {
      const UrbanParcel& p = parcels[i];
      const Building& b = scene.buildings[i];
      CHECK(b.parcel == static_cast<int>(i));
      CHECK(b.block == p.block);
      CHECK(b.obstacle >= 0 && b.obstacle < static_cast<int>(scene.obstacles.size()));
      const Obstacle& o = scene.obstacles[b.obstacle];
      CHECK(o.building == static_cast<int>(i));
      CHECK(o.footprint == p.footprint);
      CHECK(PolygonSignedArea(p.footprint) > 0.0f);  // Counter-clockwise.
      CHECK(p.depth > 1.0f);
      // Frontage on the building line (the block outline inset by the
      // sidewalk), and the nearest street of the reported class alongside.
      const glm::vec2 a = p.footprint[p.frontageEdge];
      const glm::vec2 bq = p.footprint[(p.frontageEdge + 1) % p.footprint.size()];
      const std::vector<glm::vec2> line =
          InsetConvexPolygon(blocks[p.block].vertices, c.sidewalkWidth);
      const bool detached = blocks[p.block].apartmentRow;
      auto lineDistance = [&](glm::vec2 q) {
        float best = 1e9f;
        for (size_t e = 0; e < line.size(); ++e) {
          best = std::min(best, DistanceToSegment(q, line[e], line[(e + 1) % line.size()]));
        }
        return best;
      };
      if (!detached) {
        CHECK(lineDistance(a) < 0.01f);
        CHECK(lineDistance(bq) < 0.01f);
        CHECK(lineDistance((a + bq) * 0.5f) < 0.01f);
      }
      const glm::vec2 mid = (a + bq) * 0.5f;
      const glm::vec2 dir = glm::normalize(bq - a);
      const glm::vec2 inward(-dir.y, dir.x);  // Left of a CCW edge: into the parcel.
      // The street a frontage faces runs parallel to it: the nearest parallel
      // street band (local streets are reported as full map-spanning lines,
      // so a perpendicular street ending at this block never counts), or the
      // rim street when the frontage is axis-aligned at the map edge.
      const float half = UrbanMapHalfExtent(c);
      const float rim = std::max(c.streetWidth, c.localStreetWidth);
      const glm::vec2 outside = mid - inward * (c.sidewalkWidth + 1.0f);
      const bool axisAligned = std::fabs(dir.x) > 0.99f || std::fabs(dir.y) > 0.99f;
      const bool onRim =
          axisAligned && half - std::max(std::fabs(outside.x), std::fabs(outside.y)) < rim;
      bool pedestrianFrontage = false;
      for (size_t other = 0; other < blocks.size(); ++other) {
        if (other == static_cast<size_t>(p.block)) continue;
        pedestrianFrontage |= PointInConvexPolygon(outside, blocks[other].vertices);
      }
      float nearest = 1e9f;
      UrbanStreetClass nearestClass = UrbanStreetClass::Cross;
      for (const UrbanStreet& st : streets) {
        const glm::vec2 sa(st.a.x, st.a.z), sb(st.b.x, st.b.z);
        if (glm::length(sb - sa) < 1e-3f ||
            std::fabs(glm::dot(dir, glm::normalize(sb - sa))) < 0.8f) {
          continue;
        }
        const float d = DistanceToSegment(mid, sa, sb) - st.width * 0.5f;
        if (d < nearest) {
          nearest = d;
          nearestClass = st.cls;
        }
      }
      if (detached) {
        // Its nominal street class informs styling, but the building itself
        // is deliberately set back and rotated away from the building line.
      } else if (pedestrianFrontage) {
        // Long blocks may be divided by a double sidewalk with no road.
        // Frontages along that shared edge intentionally have no street.
      } else if (onRim) {
        CHECK(p.street == UrbanStreetClass::Cross);
      } else {
        // On the building line: a sidewalk from a local street, a sidewalk
        // plus the artery's verge from the highway.
        CHECK(nearest < c.sidewalkWidth + c.localStreetWidth * 0.5f + 0.5f);
        if (p.street != nearestClass) {
          std::fprintf(stderr, "seed %u parcel %zu: class %d, nearest street class %d (%.2f)\n",
                       seed, i, static_cast<int>(p.street), static_cast<int>(nearestClass),
                       nearest);
        }
        CHECK(p.street == nearestClass);
      }
      highway += p.street == UrbanStreetClass::Highway;
      avenue += p.street == UrbanStreetClass::Avenue;
      cross += p.street == UrbanStreetClass::Cross;
      corners += p.corner;
    }
    CHECK(highway > 0 && avenue > 0 && cross > 0);
    CHECK(corners > 0 && corners < static_cast<int>(parcels.size()));
    // Parcels don't move anything: the layout rng's draws are unchanged.
    CHECK(UrbanParcels(seed, FlatConfig()).size() == parcels.size());
  }
  CHECK(UrbanParcels(7, LegacyConfig()).empty());
}

// Issue #151: kinds are drawn per parcel from a street-class-weighted table
// with a per-block character bias; floors per kind set the height; the two
// landmark towers are the Civic buildings; set-back offices stack a second
// prism on their base.
void TestBuildingKindsFloorsAndHeights() {
  const MapGeneratorConfig c = OldSizeConfig();
  int highwayDowntown = 0, highwayTotal = 0, crossResidential = 0, crossTotal = 0;
  for (uint32_t seed : kSeeds) {
    const Scene scene = GenerateUrbanMap(seed, FlatConfig());
    const std::vector<UrbanParcel> parcels = UrbanParcels(seed, FlatConfig());
    int perKind[kBuildingKindCount] = {};
    int setbacks = 0;
    for (size_t i = 0; i < scene.buildings.size(); ++i) {
      const Building& b = scene.buildings[i];
      ++perKind[static_cast<int>(b.kind)];
      const Obstacle& base = scene.obstacles[b.obstacle];
      CHECK(b.floorCount >= 1);
      CHECK(b.groundFloorHeight > 2.0f && b.floorHeight > 2.0f);
      const float height = b.Height();
      if (b.kind == BuildingKind::Civic) {
        CHECK(height >= c.towerMinHeight - kEps && height <= c.towerMaxHeight + kEps);
      } else {
        CHECK(height >= c.minBuildingHeight - kEps && height <= c.skylineMaxHeight + kEps);
        if (b.kind == BuildingKind::Garage || b.kind == BuildingKind::Commercial) {
          CHECK(height <= c.maxBuildingHeight + kEps);
        }
      }
      if (b.kind == BuildingKind::Commercial) CHECK(b.floorCount == 1);
      if (b.upperObstacle >= 0) {
        ++setbacks;
        CHECK(b.kind == BuildingKind::Office);
        const Obstacle& upper = scene.obstacles[b.upperObstacle];
        CHECK(upper.building == static_cast<int>(i));
        CHECK(std::fabs(upper.bounds.min.y - base.bounds.max.y) < kEps);
        CHECK(std::fabs(upper.bounds.max.y - (base.bounds.min.y + height)) < kEps);
        CHECK(PolygonArea(upper.footprint) < PolygonArea(base.footprint));
        for (const glm::vec2& v : upper.footprint) CHECK(PointInConvexPolygon(v, base.footprint));
      } else {
        CHECK(std::fabs(base.bounds.max.y - base.bounds.min.y - height) < kEps);
      }
    }
    CHECK(perKind[static_cast<int>(BuildingKind::Civic)] == c.towerCount);
    for (int k = 0; k < kBuildingKindCount; ++k) CHECK(perKind[k] > 0);
    (void)setbacks;  // Seed-dependent; asserted on the forced-downtown city below.
    // Every obstacle with a footprint is a building or a pier column/footing.
    for (const Obstacle& o : scene.obstacles) {
      CHECK(o.footprint.empty() || o.building >= 0 || o.kind == ObstacleKind::Pier ||
            o.kind == ObstacleKind::Cover);  // Cover: artery wrecks.
    }

    // Street weighting: artery frontage is mostly office/mixed-use, back
    // streets mostly residential/commercial/garage, across the seeds.
    for (size_t i = 0; i < scene.buildings.size(); ++i) {
      const BuildingKind kind = scene.buildings[i].kind;
      if (kind == BuildingKind::Civic) continue;
      if (parcels[i].street == UrbanStreetClass::Highway) {
        ++highwayTotal;
        highwayDowntown += kind == BuildingKind::Office || kind == BuildingKind::MixedUse;
      } else if (parcels[i].street == UrbanStreetClass::Cross) {
        ++crossTotal;
        crossResidential += kind == BuildingKind::Residential || kind == BuildingKind::Commercial ||
                            kind == BuildingKind::Garage;
      }
    }
  }
  // Pooled over the seeds: a single seed has too few artery-fronting parcels
  // for the majority to be stable.
  CHECK(highwayTotal > 0 && 2 * highwayDowntown > highwayTotal);
  CHECK(crossTotal > 0 && 2 * crossResidential > crossTotal);

  // Forcing a character biases every block the same way without moving a
  // wall: identical footprints, different kind mix.
  MapGeneratorConfig downtown = OldSizeConfig(), residential = OldSizeConfig();
  downtown.character = BlockCharacter::Downtown;
  residential.character = BlockCharacter::Residential;
  const Scene base = GenerateUrbanMap(42, OldSizeConfig());
  const Scene town = GenerateUrbanMap(42, downtown);
  const Scene homes = GenerateUrbanMap(42, residential);
  CHECK(town.buildings.size() == base.buildings.size());
  CHECK(homes.buildings.size() == base.buildings.size());
  int townOffice = 0, homesOffice = 0, townRes = 0, homesRes = 0;
  for (size_t i = 0; i < base.buildings.size(); ++i) {
    CHECK(town.obstacles[town.buildings[i].obstacle].footprint ==
          base.obstacles[base.buildings[i].obstacle].footprint);
    CHECK(homes.obstacles[homes.buildings[i].obstacle].footprint ==
          base.obstacles[base.buildings[i].obstacle].footprint);
    townOffice += town.buildings[i].kind == BuildingKind::Office;
    homesOffice += homes.buildings[i].kind == BuildingKind::Office;
    townRes += town.buildings[i].kind == BuildingKind::Residential;
    homesRes += homes.buildings[i].kind == BuildingKind::Residential;
  }
  CHECK(townOffice > homesOffice);
  CHECK(homesRes > townRes);
  int townSetbacks = 0;
  for (const Building& b : town.buildings) townSetbacks += b.upperObstacle >= 0;
  CHECK(townSetbacks > 0);
  CHECK(homesRes * 2 > static_cast<int>(base.buildings.size()));
  BlockCharacter parsed = BlockCharacter::Auto;
  CHECK(ParseBlockCharacter("industrial", &parsed) && parsed == BlockCharacter::Industrial);
  CHECK(!ParseBlockCharacter("suburb", &parsed));
}

// Massing blocks sight lines and FOV (not movement): a segment through any
// parapet, fixture or awning is blocked by SightObstacles, and the real
// obstacle list is its unchanged prefix.
void TestMassingBlocksSight() {
  const Scene scene = GenerateUrbanMap(kSeeds[0]);
  const std::vector<Obstacle> sight = SightObstacles(scene);
  CHECK(sight.size() == scene.obstacles.size() + scene.massing.size());
  for (size_t i = 0; i < scene.massing.size(); ++i) {
    const AABB& box = sight[scene.obstacles.size() + i].bounds;
    const glm::vec3 c = box.Center();
    const float reach = 0.5f * (box.max.x - box.min.x) + 0.5f;  // Endpoints outside the prism.
    CHECK(!LineOfSightClear(glm::vec3(c.x - reach, c.y, c.z), glm::vec3(c.x + reach, c.y, c.z),
                            sight));
  }
}

// Parapets and roof furniture block walking: every one is a blocker on a roof
// slab, and a path across a roof never enters a blocker.
void TestRoofMassingBlocksWalking() {
  int blockers = 0, crossings = 0;
  for (uint32_t seed : kSeeds) {
    const Scene scene = GenerateUrbanMap(seed, FlatConfig());
    NavMesh mesh;
    mesh.Build(scene.obstacles, scene.mapHalfExtent, constants::kAgentRadius, &scene.ground,
               &scene.walkSurfaces);
    for (const WalkSurface& slab : scene.walkSurfaces) {
      blockers += static_cast<int>(slab.blockers.size());
      if (slab.blockers.empty()) continue;
      // Straight across the slab, between points outside every blocker.
      const glm::vec3 a = slab.vertices[0], b = slab.vertices[2];
      for (float t : {0.1f, 0.2f, 0.3f}) {
        const glm::vec3 from = glm::mix(a, SurfaceCenter(slab), t);
        const glm::vec3 to = glm::mix(b, SurfaceCenter(slab), t);
        // Endpoints clear of every blocker (by a full agent radius, at least
        // the navmesh's own padding) must reach each other: a detour around
        // the blocker always exists on the open roof.
        bool endpointsClear = true;
        for (const auto& blocker : slab.blockers) {
          const std::vector<glm::vec2> padded =
              ExpandConvexPolygon(blocker, constants::kAgentRadius);
          endpointsClear &= !PointInConvexPolygon({from.x, from.z}, padded) &&
                            !PointInConvexPolygon({to.x, to.z}, padded);
        }
        std::vector<glm::vec3> path;
        const bool found = mesh.FindPath(from, to, &path);
        if (endpointsClear) CHECK(found);
        // An endpoint inside a blocker (a parapet hugging the roof edge) has
        // no guarantee of a path around it.
        if (!found || !endpointsClear) continue;
        ++crossings;
        // An endpoint inside a blocker's padding may legitimately leave it.
        if (!endpointsClear) continue;
        for (size_t i = 0; i + 1 < path.size(); ++i) {
          for (const auto& blocker : slab.blockers) {
            CHECK(!SegmentEntersConvexPolygon({path[i].x, path[i].z},
                                              {path[i + 1].x, path[i + 1].z}, blocker));
          }
        }
      }
    }
  }
  CHECK(blockers > 0 && crossings > 0);
}

// Issue #151: massing. Most roofs (zipline roofs included) get a parapet ring,
// roof furniture stays inside the roof and clear of zipline landings, and
// awnings hang over the sidewalk from shopfronts only.
void TestMassingStaysOnItsBuilding() {
  for (uint32_t seed : kSeeds) {
    const Scene scene = GenerateUrbanMap(seed, FlatConfig());
    CHECK(!scene.massing.empty());
    int parapets = 0, fixtures = 0, awnings = 0;
    std::vector<int> parapetsPerBuilding(scene.buildings.size(), 0);
    for (const MassingDetail& d : scene.massing) {
      CHECK(d.footprint.size() >= 3 && d.y1 > d.y0);
      CHECK(PolygonSignedArea(d.footprint) > 0.0f);
      // Overpass details (issue #223) belong to the highway, not a building.
      if (d.kind == MassingDetail::Kind::PierCap || d.kind == MassingDetail::Kind::DeckRail) {
        CHECK(d.building == -1);
        continue;
      }
      CHECK(d.building >= 0 && d.building < static_cast<int>(scene.buildings.size()));
      const Building& b = scene.buildings[d.building];
      const Obstacle& roof = scene.obstacles[b.upperObstacle >= 0 ? b.upperObstacle : b.obstacle];
      switch (d.kind) {
        case MassingDetail::Kind::Parapet:
        case MassingDetail::Kind::RoofFixture: {
          CHECK(std::fabs(d.y0 - roof.bounds.max.y) < kEps);
          CHECK(d.y1 - d.y0 <= 1.6f);
          const std::vector<glm::vec2> tolerant = ExpandConvexPolygon(roof.footprint, 1e-3f);
          for (const glm::vec2& v : d.footprint) CHECK(PointInConvexPolygon(v, tolerant));
          if (d.kind == MassingDetail::Kind::Parapet) {
            ++parapets;
            ++parapetsPerBuilding[d.building];
          } else {
            ++fixtures;
          }
          // Furniture keeps clear of a zipline landing on its roof.
          if (d.kind == MassingDetail::Kind::RoofFixture) {
            for (const Zipline& line : scene.ziplines) {
              for (const glm::vec3& foot : {line.a, line.b}) {
                if (std::fabs(foot.y - roof.bounds.max.y) < 0.2f &&
                    PointInConvexPolygon(glm::vec2(foot.x, foot.z), roof.footprint)) {
                  glm::vec2 centre(0.0f);
                  for (const glm::vec2& v : d.footprint) centre += v;
                  centre /= static_cast<float>(d.footprint.size());
                  CHECK(glm::distance(centre, glm::vec2(foot.x, foot.z)) >= 1.5f);
                }
              }
            }
          }
          break;
        }
        case MassingDetail::Kind::Awning: {
          ++awnings;
          CHECK(b.kind == BuildingKind::MixedUse);  // Warehouse loading doors get none.
          CHECK(d.y0 > 2.4f && d.y0 < b.groundFloorHeight);  // Above head height, below floor 2.
          // Over the sidewalk: outside the building, inside its block.
          const std::vector<glm::vec2> blockOutline = UrbanBlocks(seed, FlatConfig())[b.block].vertices;
          bool anyOutside = false;
          for (const glm::vec2& v : d.footprint) {
            anyOutside |= !PointInConvexPolygon(v, InsetConvexPolygon(roof.footprint, 1e-3f));
            CHECK(PointInConvexPolygon(v, ExpandConvexPolygon(blockOutline, 1e-3f)));
          }
          CHECK(anyOutside);
          break;
        }
      }
    }
    CHECK(parapets > 0 && fixtures > 0 && awnings > 0);
    // A parapet ring has one piece per roof edge.
    int ringed = 0;
    for (size_t i = 0; i < scene.buildings.size(); ++i) {
      const Building& b = scene.buildings[i];
      const Obstacle& roof = scene.obstacles[b.upperObstacle >= 0 ? b.upperObstacle : b.obstacle];
      if (parapetsPerBuilding[i] == 0) continue;
      ++ringed;
      CHECK(parapetsPerBuilding[i] <= static_cast<int>(roof.footprint.size()));
    }
    CHECK(ringed * 2 > static_cast<int>(scene.buildings.size()));
  }
}

// Pinned per-seed kind histogram for the default (arterial) city: a change
// to the archetype tables or the draw order shows up here first (update the
// numbers deliberately when it's intended).
void TestPinnedKindHistograms() {
  struct Expected { uint32_t seed; size_t buildings; int kinds[kBuildingKindCount]; };
  const Expected expected[] = {
      // residential, mixed-use, office, commercial, civic, garage
      {1u, 63, {10, 22, 13, 7, 2, 9}},
      {42u, 73, {29, 18, 11, 6, 2, 7}},
      {2024u, 80, {21, 18, 15, 11, 2, 13}},
  };
  for (const auto& e : expected) {
    const Scene scene = GenerateUrbanMap(e.seed, FlatConfig());
    int kinds[kBuildingKindCount] = {};
    for (const Building& b : scene.buildings) ++kinds[static_cast<int>(b.kind)];
    bool same = scene.buildings.size() == e.buildings;
    for (int k = 0; k < kBuildingKindCount; ++k) same &= kinds[k] == e.kinds[k];
    if (!same) {
      std::fprintf(stderr, "seed %u: %zu buildings, kinds", e.seed, scene.buildings.size());
      for (int k = 0; k < kBuildingKindCount; ++k) {
        std::fprintf(stderr, " %s=%d", BuildingKindName(static_cast<BuildingKind>(k)), kinds[k]);
      }
      std::fprintf(stderr, "\n");
    }
    CHECK(same);
  }
}

void TestMapIsMuchLargerThanDefault() {
  const Scene scene = GenerateUrbanMap(1, OldSizeConfig());
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
  MapGeneratorConfig c = OldSizeConfig();
  c.features.terrain.mode = FeatureMode::Off;  // Buildings only: no retaining walls.
  for (uint32_t seed : kSeeds) {
    const Scene scene = GenerateUrbanMap(seed, c);
    int towers = 0, medium = 0, buildings = 0;
    float lo = 1e9f, hi = 0.0f;
    for (const auto& o : scene.obstacles) {
      if (o.kind == ObstacleKind::Pier) continue;  // Pier bents (elevated seeds), not buildings.
      if (o.footprint.empty() || o.kind == ObstacleKind::Cover) continue;  // Wrecks aren't buildings.
      ++buildings;
      const float h = o.bounds.max.y;
      lo = std::min(lo, h);
      hi = std::max(hi, h);
      CHECK(h >= c.minBuildingHeight - kEps);
      if (h >= c.towerMinHeight - kEps) ++towers;
      else if (h >= 4.5f && h <= 7.5f) ++medium;
    }
    CHECK(towers == c.towerCount);
    CHECK(hi - lo > 8.0f);
    CHECK(medium * 10 > buildings);  // Medium heights still appear.
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
  const Scene scene = GenerateUrbanMap(42, OldSizeConfig());
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
  MapGeneratorConfig one = OldSizeConfig();
  one.features.terrain.mode = FeatureMode::Off;  // Flat-city road geometry.
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
  MapGeneratorConfig config = OldSizeConfig();
  config.features.terrain.mode = FeatureMode::Off;  // Flat-city blocks.
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
    if (obstacle.kind == ObstacleKind::Pier) continue;  // Elevated pier bents, not buildings.
    if (obstacle.footprint.empty() || obstacle.kind == ObstacleKind::Cover) continue;
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

// A block corner without an alley is one building wrapping the corner: no
// two footprints may both have a vertex at the same frontage corner.
void TestNoCornerSplitBuildings() {
  MapGeneratorConfig config = OldSizeConfig();
  config.arteryCount = 2;
  config.elevatedHighway = OverpassMode::Off;
  int wrapped = 0;
  for (uint32_t seed : kSeeds) {
    const Scene scene = GenerateUrbanMap(seed, config);
    for (const UrbanBlock& block : UrbanBlocks(seed, config)) {
      const std::vector<glm::vec2> outer = InsetConvexPolygon(block.vertices, config.sidewalkWidth);
      for (const glm::vec2& corner : outer) {
        int touching = 0;
        for (const Obstacle& o : scene.obstacles) {
          for (const glm::vec2& v : o.footprint) {
            if (glm::distance(v, corner) < 0.05f) {
              ++touching;
              break;
            }
          }
        }
        CHECK(touching <= 1);
        wrapped += touching;
      }
    }
  }
  CHECK(wrapped > 0);  // Corner buildings still exist.
}

// True when two convex CCW footprints share interior area (touching edges
// do not count).
bool FootprintsOverlap(const std::vector<glm::vec2>& a, const std::vector<glm::vec2>& b) {
  for (const std::vector<glm::vec2>* poly : {&a, &b}) {
    for (size_t i = 0; i < poly->size(); ++i) {
      const glm::vec2 edge = (*poly)[(i + 1) % poly->size()] - (*poly)[i];
      const glm::vec2 axis(-edge.y, edge.x);
      const float len = glm::length(axis);
      if (len < 1.0e-6f) continue;
      float loA = 1.0e30f, hiA = -1.0e30f, loB = 1.0e30f, hiB = -1.0e30f;
      for (const glm::vec2& p : a) {
        const float d = glm::dot(p, axis) / len;
        loA = std::min(loA, d);
        hiA = std::max(hiA, d);
      }
      for (const glm::vec2& p : b) {
        const float d = glm::dot(p, axis) / len;
        loB = std::min(loB, d);
        hiB = std::max(hiB, d);
      }
      if (hiA <= loB + 0.02f || hiB <= loA + 0.02f) return false;
    }
  }
  return true;
}

// Seed 7 used to turn a too-short corner alley into a merge after frontage
// pieces had been planned. One piece then belonged to both neighboring
// corner buildings, chaining three footprints into a giant U. Pieces must
// never share area, and ordinary two-piece L buildings must still appear.
void TestCornerBuildingsDoNotChainAcrossAFrontage() {
  MapGeneratorConfig config = OldSizeConfig();
  config.elevatedHighway = OverpassMode::Off;
  const Scene scene = GenerateUrbanMap(7, config);
  std::vector<const Obstacle*> buildings;
  for (const Obstacle& obstacle : scene.obstacles) {
    if (obstacle.building >= 0 && !obstacle.footprint.empty()) buildings.push_back(&obstacle);
  }

  int pairs = 0;
  for (size_t i = 0; i < buildings.size(); ++i) {
    for (size_t j = i + 1; j < buildings.size(); ++j) {
      if (buildings[i]->building == buildings[j]->building) continue;
      CHECK(!FootprintsOverlap(buildings[i]->footprint, buildings[j]->footprint));
    }
  }
  for (const Obstacle* b : buildings) pairs += b->footprint.size() > 4;
  CHECK(pairs > 0);
}

float MinimumPolygonWidth(const std::vector<glm::vec2>& polygon) {
  float result = std::numeric_limits<float>::infinity();
  for (size_t i = 0; i < polygon.size(); ++i) {
    const glm::vec2 edge = polygon[(i + 1) % polygon.size()] - polygon[i];
    const float length = glm::length(edge);
    if (length < 1.0e-4f) continue;
    const glm::vec2 normal(-edge.y / length, edge.x / length);
    float lo = std::numeric_limits<float>::infinity(), hi = -lo;
    for (const glm::vec2& point : polygon) {
      const float projection = glm::dot(point, normal);
      lo = std::min(lo, projection);
      hi = std::max(hi, projection);
    }
    result = std::min(result, hi - lo);
  }
  return result;
}

// Seed 7 also left a long, tapered remnant at an acute corner. It was large
// enough for the clipping machinery but too thin to be a useful building.
void TestThinCornerBuildingSliversAreDropped() {
  MapGeneratorConfig config = OldSizeConfig();
  config.elevatedHighway = OverpassMode::Off;
  const Scene scene = GenerateUrbanMap(7, config);
  for (const Obstacle& obstacle : scene.obstacles) {
    if (obstacle.building < 0 || obstacle.footprint.empty()) continue;
    CHECK(MinimumPolygonWidth(obstacle.footprint) >= config.buildingDepth * 0.25f);
  }
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

// A fork rail must remain in the narrow strip along its own deck edge.
// Checking only whether the two caps touch misses a join that detours off
// the deck (especially a twin deck over raised terrain).
void TestForkRailsFollowDeckEdges() {
  for (const FeatureMode terrain : {FeatureMode::Off, FeatureMode::On}) {
    for (const PierForm form : {PierForm::TBent, PierForm::Twin}) {
      for (const uint32_t seed : {7u, 26u, 42u}) {
        MapGeneratorConfig config;
        config.arteryCount = 2;
        config.elevatedHighway = OverpassMode::On;
        config.pierForm = form;
        config.features.terrain.mode = terrain;
        config.features.facades.mode = FeatureMode::Off;
        const Scene scene = GenerateUrbanMap(seed, config);
        int checked = 0;
        for (const WalkSurface& span : scene.walkSurfaces) {
          if (span.vertices.size() != 4) continue;
          const auto xz = [](glm::vec3 p) { return glm::vec2(p.x, p.z); };
          for (const auto& blocker : span.blockers) {
            bool isRail = false;
            for (const MassingDetail& d : scene.massing) {
              isRail |= d.kind == MassingDetail::Kind::DeckRail && d.footprint == blocker;
            }
            if (!isRail) continue;
            ++checked;
            bool followsEdge = false;
            for (const auto ends : {std::pair<int, int>{0, 3}, {2, 1}}) {
              const glm::vec2 a = xz(span.vertices[ends.first]);
              const glm::vec2 b = xz(span.vertices[ends.second]);
              const glm::vec2 dir = glm::normalize(b - a);
              const glm::vec2 inward(dir.y, -dir.x);
              bool inStrip = true;
              for (const glm::vec2 p : blocker) {
                const float d = glm::dot(p - a, inward);
                inStrip &= d >= -kEps && d <= 0.3f + kEps;
              }
              followsEdge |= inStrip;
            }
            CHECK(followsEdge);
          }
        }
        CHECK(checked > 0);
      }
    }
  }
}

void TestElevatedOverpassRampsPiersAndDistinctLayers() {
  MapGeneratorConfig config = OldSizeConfig();
  config.arteryCount = 2;
  config.elevatedHighway = OverpassMode::On;
  config.overpassLayout = OverpassLayout::RampUpRampDown;
  // The pier checks below identify piers by their obstacle kind; furniture
  // (issue #150) is pinned off so the scene stays piers + buildings only.
  config.features.furniture.mode = FeatureMode::Off;
  config.features.terrain.mode = FeatureMode::Off;  // Overpass vs terrain is roadmap 4.4.
  config.features.facades.mode = FeatureMode::Off;  // Count highway transitions only.
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
    if (obstacle.kind != ObstacleKind::Pier) continue;
    ++piers;
    CHECK(!obstacle.footprint.empty());
    // Shafts reach just below the slab; footings stay knee-high plinths.
    const float height = obstacle.bounds.max.y - obstacle.bounds.min.y;
    CHECK(height > 1.0f || height <= config.pierBaseHeight + kEps);
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

  // Rails are true sloped prisms, not a staircase of flat-topped boxes.
  // At the fork, each branch wall and its artery continuation share the
  // exact same mitered end-cap edge, at the acute fork too.
  std::vector<const MassingDetail*> rails;
  bool slopedRail = false;
  for (const MassingDetail& d : scene.massing) {
    if (d.kind != MassingDetail::Kind::DeckRail) continue;
    rails.push_back(&d);
    CHECK(d.top.size() == d.footprint.size());
    CHECK(std::fabs(d.depth - (config.deckRailHeight + 0.25f)) < kEps);
    float lo = 1e9f, hi = -1e9f;
    for (size_t i = 0; i < d.top.size(); ++i) {
      CHECK(glm::distance(glm::vec2(d.top[i].x, d.top[i].z), d.footprint[i]) < kEps);
      lo = std::min(lo, d.top[i].y);
      hi = std::max(hi, d.top[i].y);
    }
    slopedRail |= hi - lo > kEps;
  }
  CHECK(!rails.empty() && slopedRail);
  const auto samePoint = [](glm::vec2 a, glm::vec2 b) {
    return glm::distance(a, b) < kEps;
  };
  // The shared cap edge of two rails, if any: a[i] -> a[i + 1] traversed the
  // other way round in b.
  const auto sharedEdge = [&](const std::vector<glm::vec2>& a, const std::vector<glm::vec2>& b,
                              size_t* ia, size_t* ib) {
    for (size_t i = 0; i < a.size(); ++i) {
      for (size_t j = 0; j < b.size(); ++j) {
        if (samePoint(a[i], b[(j + 1) % b.size()]) &&
            samePoint(a[(i + 1) % a.size()], b[j])) {
          *ia = i;
          *ib = j;
          return true;
        }
      }
    }
    return false;
  };
  // Direction of a rail's wall: its longest edge (a junction remnant can be
  // shorter along the road than across its cap).
  const auto wallDir = [](const std::vector<glm::vec2>& f) {
    glm::vec2 best(1.0f, 0.0f);
    float longest = 0.0f;
    for (size_t e = 0; e < f.size(); ++e) {
      const glm::vec2 d = f[(e + 1) % f.size()] - f[e];
      if (glm::length(d) > longest) {
        longest = glm::length(d);
        best = glm::normalize(d);
      }
    }
    return best;
  };
  int forkJoins = 0;
  for (size_t i = 0; i < rails.size(); ++i) {
    for (size_t j = i + 1; j < rails.size(); ++j) {
      const std::vector<glm::vec2>& a = rails[i]->footprint;
      const std::vector<glm::vec2>& b = rails[j]->footprint;
      const glm::vec2 ai = wallDir(a), aj = wallDir(b);
      if (std::fabs(ai.x * aj.y - ai.y * aj.x) < 0.2f) continue;
      size_t ia = 0, ib = 0;
      if (sharedEdge(a, b, &ia, &ib)) {
        // Both forks are plain miters: two quads sharing the whole cap, so
        // no gap opens between the walls.
        ++forkJoins;
        CHECK(a.size() == 4 && b.size() == 4);
      }
    }
  }
  CHECK(forkJoins == 2);

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
    if (surface.neighbors.empty()) continue;  // A zipline's roof slab, not the deck.
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
  MapGeneratorConfig autoConfig = OldSizeConfig();  // Auto presence + Auto layout.
  autoConfig.features.terrain.mode = FeatureMode::Off;  // Flat: the deck is the only climb.
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
  // Terrain is pinned off: a terrain-enabled seed boosts the presence draw.
  MapGeneratorConfig twoArteries = OldSizeConfig();
  twoArteries.arteryCount = 2;
  twoArteries.features.terrain.mode = FeatureMode::Off;
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
  MapGeneratorConfig config = OldSizeConfig();
  config.features.terrain.mode = FeatureMode::Off;  // Legacy flat-city layouts.
  config.features.facades.mode = FeatureMode::Off;  // Count highway transitions only.
  config.elevatedHighway = OverpassMode::On;
  config.branchEnd = BranchEnd::Ramp;
  const float half = UrbanMapHalfExtent(config);
  bool clippedPier = false;
  bool clippedCap = false;
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
        if (o.kind != ObstacleKind::Pier) continue;
        const float edge = std::max({std::fabs(o.bounds.min.x), std::fabs(o.bounds.max.x),
                                     std::fabs(o.bounds.min.z), std::fabs(o.bounds.max.z)});
        CHECK(edge <= half + kEps);
        clippedPier |= edge >= half - kEps;
      }
      for (const MassingDetail& d : scene.massing) {
        if (d.kind != MassingDetail::Kind::PierCap) continue;
        for (const glm::vec2& p : d.footprint) {
          const float edge = std::max(std::fabs(p.x), std::fabs(p.y));
          CHECK(edge <= half + kEps);
          clippedCap |= edge >= half - kEps;
        }
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

      // The ramp tops out before its slab reaches the artery's edge, so the
      // junction sits entirely at deck height and the descent only starts
      // clear of the deck (no wedge cutting under the artery slab).
      const glm::vec2 arteryOrigin(deck.front().x, deck.front().z);
      const glm::vec2 arteryDir = glm::normalize(
          glm::vec2(deck.back().x, deck.back().z) - arteryOrigin);
      const glm::vec2 arteryNormal(-arteryDir.y, arteryDir.x);
      const float junctionBand = config.arteryWidth * 0.5f + roads[1].width * 0.5f;
      for (const glm::vec3& p : ramp) {
        const float lateral =
            std::fabs(glm::dot(glm::vec2(p.x, p.z) - arteryOrigin, arteryNormal));
        if (lateral < junctionBand) {
          CHECK(std::fabs(p.y - config.highwayElevation) < 0.1f);
        }
      }

      NavMesh nav;
      nav.Build(scene.obstacles, scene.mapHalfExtent, constants::kAgentRadius, &scene.ground,
                &scene.walkSurfaces);
      glm::vec3 top(0.0f);  // Highest deck surface center still on the map.
      for (const WalkSurface& surface : scene.walkSurfaces) {
        if (surface.neighbors.empty()) continue;  // A zipline's roof slab, not the deck.
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
  CHECK(clippedPier);
  CHECK(clippedCap);
}

void TestBranchMayRunOffMapWithoutRamp() {
  MapGeneratorConfig config = OldSizeConfig();
  config.features.terrain.mode = FeatureMode::Off;  // Flat-city deck heights.
  config.features.facades.mode = FeatureMode::Off;  // Count highway transitions only.
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
  MapGeneratorConfig config = OldSizeConfig();
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

// Issue #135: a figure on the elevated deck near the junction must see the
// branch ramp inside its movement frontier. The on-ramp rides level with the
// deck across the whole junction overlap, so cells a few meters away on the
// branch cost roughly their straight-line distance -- never a detour through
// a single merge-span waypoint that prices them out of the budget.
void TestMoveFrontierCoversBranchRampAtJunction() {
  MapGeneratorConfig config = OldSizeConfig();
  config.arteryCount = 2;
  config.elevatedHighway = OverpassMode::On;
  config.overpassLayout = OverpassLayout::RampUpRampDown;
  config.features.terrain.mode = FeatureMode::Off;  // Pin the flat-grade junction.
  Scene scene = GenerateUrbanMap(7, config);
  const auto roads = UrbanRoads(7, config);
  const auto& deck = roads[0].centerline;
  const auto& ramp = roads[1].centerline;
  const glm::vec3 merge = ramp.back();
  const glm::vec2 arteryDir =
      glm::normalize(glm::vec2(deck.back().x - deck.front().x, deck.back().z - deck.front().z));
  const glm::vec2 arteryNormal(-arteryDir.y, arteryDir.x);
  // The deck side opposite the branch, so the start never sits on a branch
  // span itself.
  const glm::vec2 toBranchFoot(ramp.front().x - merge.x, ramp.front().z - merge.z);
  const float branchSide = glm::dot(arteryNormal, toBranchFoot) > 0.0f ? 1.0f : -1.0f;
  const glm::vec2 start2 =
      glm::vec2(merge.x, merge.z) - arteryDir * 10.0f - arteryNormal * (branchSide * 3.0f);
  const glm::vec3 start(start2.x, merge.y, start2.y);

  NavMesh nav;
  nav.Build(scene.obstacles, scene.mapHalfExtent, constants::kAgentRadius, &scene.ground,
            &scene.walkSurfaces);
  CHECK(nav.ResolveSurfaceY(start.x, start.z, start.y) > 2.0f);  // Really on the deck.
  const float budget = 20.0f;
  const ReachField frontier = nav.ComputeReachField(start, budget, 0.5f);
  int rampNodes = 0;
  for (size_t i = 0; i + 1 < ramp.size(); ++i) {
    const glm::vec3 p = (ramp[i] + ramp[i + 1]) * 0.5f;
    const float straight = glm::length(glm::vec2(p.x - start.x, p.z - start.z));
    if (straight > budget * 0.7f) continue;
    const int ix = static_cast<int>(std::round((p.x - frontier.minX) / frontier.step));
    const int iz = static_cast<int>(std::round((p.z - frontier.minZ) / frontier.step));
    CHECK(frontier.Reached(ix, iz));
    CHECK(std::fabs(frontier.surfaceY[iz * frontier.nx + ix] - p.y) < 0.6f);
    CHECK(frontier.Dist(ix, iz) < straight * 1.8f + 2.0f);
    ++rampNodes;
  }
  CHECK(rampNodes >= 4);

  // The same holds through gameplay (windowed navmesh): selecting the figure
  // and entering move mode must put those branch cells inside the frontier.
  scene.units[0].position = start;
  GameLogic game(scene);
  game.ClickUnit(0, Team::Blue);
  game.ChooseMove();
  const ReachField* gameFrontier = game.MoveFrontier();
  CHECK(gameFrontier != nullptr);
  if (gameFrontier) {
    int gameRampNodes = 0;
    for (size_t i = 0; i + 1 < ramp.size(); ++i) {
      const glm::vec3 p = (ramp[i] + ramp[i + 1]) * 0.5f;
      const float straight = glm::length(glm::vec2(p.x - start.x, p.z - start.z));
      if (straight > game.FindUnit(0)->MoveBudget() * 0.6f) continue;
      const int ix = static_cast<int>(std::round((p.x - gameFrontier->minX) / gameFrontier->step));
      const int iz = static_cast<int>(std::round((p.z - gameFrontier->minZ) / gameFrontier->step));
      if (gameFrontier->Reached(ix, iz) &&
          std::fabs(gameFrontier->surfaceY[iz * gameFrontier->nx + ix] - p.y) < 0.6f) {
        ++gameRampNodes;
      }
    }
    CHECK(gameRampNodes >= 4);
  }

  // From the ground near the branch ramp's foot, the frontier climbs the
  // branch -- the reach field must cost surface cells via the cheapest ramp
  // foot, not whichever ground connection comes first in surface order.
  glm::vec3 groundStart(0.0f);
  bool haveGroundStart = false;
  for (size_t i = 0; i + 1 < ramp.size(); ++i) {
    if (ramp[i + 1].y > 0.05f) {  // Last at-grade sample before the climb.
      groundStart = ramp[i];
      haveGroundStart = true;
      break;
    }
  }
  CHECK(haveGroundStart);
  const ReachField climb = nav.ComputeReachField(groundStart, budget, 0.5f);
  int climbedNodes = 0;
  for (size_t i = 0; i + 1 < ramp.size(); ++i) {
    const glm::vec3 p = (ramp[i] + ramp[i + 1]) * 0.5f;
    if (p.y < 0.5f) continue;
    const float straight = glm::length(glm::vec2(p.x - groundStart.x, p.z - groundStart.z));
    if (straight > budget * 0.6f) continue;
    const int ix = static_cast<int>(std::round((p.x - climb.minX) / climb.step));
    const int iz = static_cast<int>(std::round((p.z - climb.minZ) / climb.step));
    CHECK(climb.Reached(ix, iz));
    CHECK(std::fabs(climb.surfaceY[iz * climb.nx + ix] - p.y) < 0.6f);
    ++climbedNodes;
  }
  CHECK(climbedNodes >= 2);
}

// Issue #135 across the #130 seed/layout mix: wherever an elevated artery has
// a branch, a short hop from the deck by the merge onto the nearby branch top
// stays a short path (no detour through a lone junction link).
void TestBranchJunctionCrossingsStayLocalAcrossLayouts() {
  for (uint32_t seed = 1; seed <= 12; ++seed) {
    MapGeneratorConfig config = OldSizeConfig();
    config.arteryCount = 2;
    config.elevatedHighway = OverpassMode::On;
    const Scene scene = GenerateUrbanMap(seed, config);
    const auto roads = UrbanRoads(seed, config);
    if (roads.size() < 2) continue;
    const auto& deck = roads[0].centerline;
    const auto& ramp = roads[1].centerline;
    const glm::vec3 merge = ramp.back();
    const glm::vec2 arteryDir =
        glm::normalize(glm::vec2(deck.back().x - deck.front().x, deck.back().z - deck.front().z));
    const glm::vec2 arteryNormal(-arteryDir.y, arteryDir.x);
    const glm::vec2 toBranchFoot(ramp.front().x - merge.x, ramp.front().z - merge.z);
    const float branchSide = glm::dot(arteryNormal, toBranchFoot) > 0.0f ? 1.0f : -1.0f;

    NavMesh nav;
    nav.Build(scene.obstacles, scene.mapHalfExtent, constants::kAgentRadius, &scene.ground,
              &scene.walkSurfaces);
    // Deck point shortly "upstream" of the merge on the far side of the deck,
    // and a branch-top point a few meters down the branch.
    const glm::vec2 from2 =
        glm::vec2(merge.x, merge.z) - arteryDir * 8.0f - arteryNormal * (branchSide * 3.0f);
    glm::vec3 from(from2.x, merge.y, from2.y);
    glm::vec3 to(0.0f);
    bool haveTo = false;
    for (size_t i = ramp.size(); i-- > 0;) {
      const float d = glm::length(glm::vec2(ramp[i].x - merge.x, ramp[i].z - merge.z));
      if (d >= 6.0f && std::fabs(ramp[i].y - merge.y) < 0.3f) {
        to = ramp[i];
        haveTo = true;
        break;
      }
    }
    if (!haveTo) continue;
    if (nav.ResolveSurfaceY(from.x, from.z, from.y) < merge.y - 0.6f) continue;
    from.y = nav.ResolveSurfaceY(from.x, from.z, from.y);
    std::vector<glm::vec3> path;
    CHECK(nav.FindPath(from, to, &path));
    float length = 0.0f;
    for (size_t i = 0; i + 1 < path.size(); ++i) length += glm::distance(path[i], path[i + 1]);
    const float straight = glm::distance(from, to);
    CHECK(length < straight * 2.0f + 4.0f);
  }
}

// --- Ziplines (issue #166). ---

bool InsideConvex(glm::vec2 p, const std::vector<glm::vec2>& poly) {
  bool allLeft = true, allRight = true;
  for (size_t i = 0; i < poly.size(); ++i) {
    const glm::vec2 a = poly[i], b = poly[(i + 1) % poly.size()];
    const float cross = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
    allLeft &= cross >= -kEps;
    allRight &= cross <= kEps;
  }
  return allLeft || allRight;
}

std::vector<glm::vec2> BlockPolygon(const UrbanBlock& block) {
  if (!block.vertices.empty()) return block.vertices;
  return {{block.x0, block.z0}, {block.x1, block.z0}, {block.x1, block.z1}, {block.x0, block.z1}};
}

glm::vec2 XZ(const glm::vec3& p) { return glm::vec2(p.x, p.z); }

float MinSegmentSeparation(const Zipline& l, const Zipline& m) {
  float best = 1.0e9f;
  for (int i = 0; i <= 40; ++i) {
    const glm::vec2 p = XZ(glm::mix(l.a, l.b, i / 40.0f));
    best = std::min(best, DistanceToSegment(p, XZ(m.a), XZ(m.b)));
  }
  for (int i = 0; i <= 40; ++i) {
    const glm::vec2 p = XZ(glm::mix(m.a, m.b, i / 40.0f));
    best = std::min(best, DistanceToSegment(p, XZ(l.a), XZ(l.b)));
  }
  return best;
}

void TestZiplinesAreDeterministicBoundedAndValid() {
  for (const bool legacy : {false, true}) {
    for (uint32_t seed : kSeeds) {
      MapGeneratorConfig config = legacy ? LegacyConfig() : OldSizeConfig();
      config.features.terrain.mode = FeatureMode::Off;  // Anchors at y = 0.
      const Scene scene = GenerateUrbanMap(seed, config);
      CHECK(SameScene(scene, GenerateUrbanMap(seed, config)));
      CHECK(!scene.ziplines.empty());

      // Placing ziplines never perturbs the rest of the seed's map: it only
      // appends the ziplines and one roof slab per line.
      MapGeneratorConfig off = config;
      off.maxZiplinesPerBlock = 0;
      Scene bare = GenerateUrbanMap(seed, off);
      CHECK(bare.ziplines.empty());
      CHECK(scene.walkSurfaces.size() == bare.walkSurfaces.size() + scene.ziplines.size());
      bare.ziplines = scene.ziplines;
      bare.walkSurfaces = scene.walkSurfaces;
      CHECK(SameScene(scene, bare));

      const std::vector<UrbanBlock> blocks = UrbanBlocks(seed, config);
      std::vector<int> perBlock(blocks.size(), 0);
      NavMesh nav;
      nav.Build(scene.obstacles, scene.mapHalfExtent, constants::kAgentRadius);
      for (size_t i = 0; i < scene.ziplines.size(); ++i) {
        const Zipline& line = scene.ziplines[i];
        // Cable length bounded, so ride cost stays sensible.
        CHECK(line.Length() >= config.ziplineMinLength - kEps);
        CHECK(line.Length() <= config.ziplineMaxLength + kEps);

        // `a` is walkable street level; `b` is on the flat roof slab of a building
        // in a block, so the line goes from the ground up to a rooftop.
        CHECK(std::fabs(line.a.y) < kEps);
        CHECK(nav.IsWalkable(line.a.x, line.a.z));
        CHECK(std::fabs(line.a.x) < scene.mapHalfExtent && std::fabs(line.a.z) < scene.mapHalfExtent);
        const WalkSurface& roof = scene.walkSurfaces[scene.walkSurfaces.size() - scene.ziplines.size() + i];
        CHECK(!roof.connectsToGround && roof.neighbors.empty());
        CHECK(SurfaceContainsXZ(roof, line.b.x, line.b.z));
        CHECK(std::fabs(SurfaceHeightAt(roof, line.b.x, line.b.z) - line.b.y) < kEps);
        CHECK(line.b.y > line.a.y + 2.0f);
        bool onBuilding = false;
        for (const Obstacle& o : scene.obstacles) {
          onBuilding |= InsideConvex(XZ(line.b), ObstacleFootprint(o)) &&
                        std::fabs(o.bounds.max.y + 0.05f - line.b.y) < kEps;
        }
        CHECK(onBuilding);
        for (size_t k = 0; k < blocks.size(); ++k) {
          if (InsideConvex(XZ(line.b), BlockPolygon(blocks[k]))) ++perBlock[k];
        }

        // The cable (post tops) never passes through an obstacle.
        const glm::vec3 up(0.0f, constants::kZiplinePostHeight, 0.0f);
        for (const Obstacle& o : scene.obstacles) {
          const auto fp = ObstacleFootprint(o);
          for (int k = 0; k <= 80; ++k) {
            const glm::vec3 p = glm::mix(line.a, line.b, k / 80.0f) + up;
            CHECK(!InsideConvex(XZ(p), fp) || p.y > o.bounds.max.y);
          }
        }
        for (size_t j = i + 1; j < scene.ziplines.size(); ++j) {
          CHECK(MinSegmentSeparation(line, scene.ziplines[j]) >= 2.0f);
        }
      }
      for (int count : perBlock) CHECK(count <= config.maxZiplinesPerBlock);
    }
  }
  // Different seeds draw different lines.
  CHECK(!SameScene(GenerateUrbanMap(1, OldSizeConfig()), GenerateUrbanMap(42, OldSizeConfig())));
}

// The street end is reachable on foot from the spawns; the roof end only by
// the line itself (the roof slab joins nothing).
void TestZiplineStreetEndsReachableRoofsOnlyByLine() {
  const Scene scene = GenerateUrbanMap(42, OldSizeConfig());
  NavMesh nav;
  nav.Build(scene.obstacles, scene.mapHalfExtent, constants::kAgentRadius, &scene.ground,
            &scene.walkSurfaces);
  std::vector<glm::vec3> path;
  for (const Zipline& line : scene.ziplines) {
    CHECK(nav.FindPath(scene.units.front().position, line.a, &path));
    CHECK(!nav.FindPath(scene.units.front().position, line.b, &path));
  }
}

}  // namespace

// Issue #146: feature flags are plumbed (the ones without an emitter yet stay
// inert; `furniture` is live, issue #150), and the street graph covers the
// highway plus every local street.
void TestFeatureFlagsAndStreetGraph() {
  MapGeneratorConfig full = OldSizeConfig();
  for (const char* name :
       {"lanes", "curbs", "furniture", "terrain", "interiors", "rail", "facades"}) {
    FeatureToggle* toggle = UrbanFeatureByName(&full.features, name);
    CHECK(toggle != nullptr);
    if (toggle) toggle->mode = FeatureMode::On;
  }
  CHECK(UrbanFeatureByName(&full.features, "bogus") == nullptr);
  FeatureMode mode = FeatureMode::Auto;
  CHECK(ParseFeatureMode("on", &mode) && mode == FeatureMode::On);
  CHECK(!ParseFeatureMode("maybe", &mode));
  // Every feature but the live `furniture` is inert: forcing the others on
  // leaves the scene identical as long as furniture is pinned the same way.
  full.features.furniture.mode = FeatureMode::Off;
  MapGeneratorConfig noFurniture = OldSizeConfig();
  noFurniture.features.furniture.mode = FeatureMode::Off;
  for (uint32_t seed : kSeeds) {
    CHECK(SameScene(GenerateUrbanMap(seed, noFurniture), GenerateUrbanMap(seed, full)));
    const std::vector<UrbanStreet> streets = UrbanStreets(seed);
    CHECK(SameScene(GenerateUrbanMap(seed, OldSizeConfig()), GenerateUrbanMap(seed, OldSizeConfig())));
    int highway = 0, local = 0;
    for (const UrbanStreet& street : streets) {
      CHECK(street.width > 0.0f);
      (street.cls == UrbanStreetClass::Highway ? highway : local)++;
    }
    CHECK(highway > 0 && local > 0);
    CHECK(streets.size() == UrbanStreets(seed).size());
  }
  MapGeneratorConfig legacy = OldSizeConfig();
  legacy.arteryCount = 0;
  CHECK(UrbanStreets(1, legacy).empty());
}

void TestLocalStreetsPavedAndSidewalkIsRing() {
  const MapGeneratorConfig c = OldSizeConfig();
  for (uint32_t seed : kSeeds) {
    size_t local = 0;
    for (const UrbanStreet& street : UrbanStreets(seed, c)) {
      if (street.cls != UrbanStreetClass::Highway) ++local;
    }
    const Scene scene = GenerateUrbanMap(seed, c);
    CHECK(local > 0);
    CHECK(scene.roads.size() >= local);
    // Sidewalk ring: no paved strip covers the middle of a block.
    for (const UrbanBlock& block : UrbanBlocks(seed, c)) {
      glm::vec2 centroid(0.0f);
      for (const glm::vec2& v : block.vertices) centroid += v;
      centroid /= static_cast<float>(block.vertices.size());
      float inradius = 1e9f;  // Ring is only guaranteed for blocks with room inside.
      for (size_t e = 0; e < block.vertices.size(); ++e) {
        const glm::vec2 a = block.vertices[e];
        const glm::vec2 b = block.vertices[(e + 1) % block.vertices.size()];
        const glm::vec2 d = b - a, r = centroid - a;
        inradius = std::min(inradius, std::fabs(d.x * r.y - d.y * r.x) / glm::length(d));
      }
      if (inradius < c.sidewalkWidth * 2.0f) continue;
      for (const RoadSurface& sw : scene.sidewalkSurfaces) {
        std::vector<glm::vec2> poly;
        for (const glm::vec3& v : sw.vertices) poly.emplace_back(v.x, v.z);
        CHECK(!PointInConvexPolygon(centroid, poly));
      }
    }
  }
}

// A local street that exits obliquely must carry its full width through the
// boundary. Clipping the centerline first produces the half-width notch from
// PR #192's review screenshot.
void TestLocalStreetBandsReachMapEdgeAtFullWidth() {
  MapGeneratorConfig c = OldSizeConfig();
  c.features.terrain.mode = FeatureMode::Off;  // One pavement band per street.
  c.elevatedHighway = OverpassMode::Off;
  for (const uint32_t seed : kSeeds) {
    const std::vector<UrbanStreet> streets = UrbanStreets(seed, c);
    const Scene scene = GenerateUrbanMap(seed, c);
    CHECK(scene.roads.size() == streets.size());
    if (scene.roads.size() != streets.size()) continue;

    for (size_t i = 0; i < streets.size(); ++i) {
      const UrbanStreet& street = streets[i];
      if (street.cls == UrbanStreetClass::Highway) continue;
      const glm::vec2 a(street.a.x, street.a.z), b(street.b.x, street.b.z);
      const glm::vec2 dir = glm::normalize(b - a);
      const glm::vec2 normal(-dir.y, dir.x);
      float minOffset = 1e9f, maxOffset = -1e9f;
      for (const glm::vec3& vertex : scene.roads[i].vertices) {
        const glm::vec2 p(vertex.x, vertex.z);
        const bool onBoundary =
            std::fabs(std::fabs(p.x) - scene.mapHalfExtent) < kEps ||
            std::fabs(std::fabs(p.y) - scene.mapHalfExtent) < kEps;
        if (!onBoundary) continue;
        const float offset = glm::dot(p - a, normal);
        minOffset = std::min(minOffset, offset);
        maxOffset = std::max(maxOffset, offset);
      }
      if (minOffset > maxOffset) continue;  // An interior segment (block split).
      CHECK(minOffset <= -street.width * 0.5f + kEps);
      CHECK(maxOffset >= street.width * 0.5f - kEps);
    }
  }
}

void TestSpawnHintsPlaceOppositeCornersAndCenterFlag() {
  for (uint32_t seed : {1u, 7u, 42u, 1234u}) {
    for (int hilly = 0; hilly < 2; ++hilly) {
      tactics::Scene scene = hilly ? tactics::GenerateHillyMap(seed) : tactics::GenerateUrbanMap(seed);
      CHECK(scene.spawnHints.blue && scene.spawnHints.red && scene.spawnHints.flag);
      tactics::ApplySpawnHints(&scene);
      const float e = scene.mapHalfExtent;
      glm::vec2 blue(0.0f), red(0.0f);
      for (const tactics::Unit& u : scene.units) {
        CHECK(tactics::IsFreeSpot(scene, u.position.x, u.position.z));
        (u.team == tactics::Team::Blue ? blue : red) += glm::vec2(u.position.x, u.position.z) / 3.0f;
      }
      // Opposite corners: each squad in its own quadrant, far from center.
      CHECK(blue.x < -0.5f * e && blue.y < -0.5f * e);
      CHECK(red.x > 0.5f * e && red.y > 0.5f * e);
      tactics::Scene ctf = scene;
      ctf.flag.enabled = true;
      tactics::GameLogic logic(ctf);
      const glm::vec3 flag = logic.FlagPosition();
      CHECK(std::abs(flag.x) < 0.1f * e && std::abs(flag.z) < 0.1f * e);
    }
  }
  // Without hints, units are untouched.
  tactics::Scene plain = tactics::BuildDefaultScene();
  const glm::vec3 before = plain.units[0].position;
  tactics::ApplySpawnHints(&plain);
  CHECK(plain.units[0].position == before);
}

// Issue #152 (roadmap 3.4): windows and doors are recorded per building as
// FacadeOpening data on a wall edge, inside the wall's bounds. Most
// buildings get a street entrance on their frontage; some entrances are
// raised on a stoop walk-up while others open straight off the ground; rear
// doors are usually raised on a little platform whose ramp sits on one of
// its three free sides. Every raised door has a walk surface at its base
// that reaches the ground.
void TestFacadeOpeningsDoorsStoopsAndPlatforms() {
  int stoops = 0, grade = 0, rearDoors = 0, platforms = 0, rearAtGrade = 0, loadingBays = 0;
  std::set<int> rampSides;
  for (uint32_t seed : kSeeds) {
    const Scene scene = GenerateUrbanMap(seed, FlatConfig());
    const std::vector<UrbanParcel> parcels = UrbanParcels(seed, FlatConfig());
    CHECK(!scene.facadeOpenings.empty());
    // Determinism: the same seed re-rolls the same openings; forcing the
    // feature off removes them (and their stoop/platform surfaces) without
    // moving anything else.
    {
      const Scene again = GenerateUrbanMap(seed, FlatConfig());
      bool same = again.facadeOpenings.size() == scene.facadeOpenings.size();
      for (size_t i = 0; same && i < scene.facadeOpenings.size(); ++i) {
        const FacadeOpening& x = scene.facadeOpenings[i];
        const FacadeOpening& y = again.facadeOpenings[i];
        same = x.kind == y.kind && x.building == y.building && x.obstacle == y.obstacle &&
               x.wallEdge == y.wallEdge && x.u0 == y.u0 && x.u1 == y.u1 && x.y0 == y.y0 &&
               x.y1 == y.y1 && x.floor == y.floor;
      }
      CHECK(same);
      MapGeneratorConfig bare = FlatConfig();
      bare.features.facades.mode = FeatureMode::Off;
      const Scene bareScene = GenerateUrbanMap(seed, bare);
      CHECK(bareScene.facadeOpenings.empty());
      CHECK(bareScene.walkSurfaces.size() <= scene.walkSurfaces.size());
      CHECK(bareScene.obstacles.size() == scene.obstacles.size());
    }
    std::vector<int> frontDoors(scene.buildings.size(), 0);
    int windows = 0;
    for (const FacadeOpening& o : scene.facadeOpenings) {
      CHECK(o.building >= 0 && o.building < static_cast<int>(scene.buildings.size()));
      const Building& b = scene.buildings[o.building];
      CHECK(o.obstacle == b.obstacle);
      const Obstacle& wall = scene.obstacles[o.obstacle];
      const std::vector<glm::vec2>& fp = wall.footprint;
      CHECK(o.wallEdge >= 0 && o.wallEdge < static_cast<int>(fp.size()));
      const glm::vec2 a = fp[o.wallEdge], q = fp[(o.wallEdge + 1) % fp.size()];
      const float len = glm::distance(a, q);
      // The opening lies within its wall edge, horizontally and vertically.
      CHECK(o.u0 >= -kEps && o.u1 <= len + kEps && o.u0 < o.u1);
      CHECK(o.y0 >= wall.bounds.min.y - kEps && o.y1 <= wall.bounds.max.y + kEps);
      CHECK(o.y0 < o.y1);
      if (o.kind == FacadeOpening::Kind::Window) {
        // Warehouses (commercial halls) have no windows.
        CHECK(b.kind != BuildingKind::Commercial);
        ++windows;
        continue;
      }
      if (b.kind == BuildingKind::Commercial && o.wallEdge == parcels[b.parcel].frontageEdge) {
        if (o.u1 - o.u0 > 4.0f) ++loadingBays;
      }
      CHECK(o.floor == 0);
      const float baseY = wall.bounds.min.y;
      const bool raised = o.y0 > baseY + 0.05f;
      const bool front = o.wallEdge == parcels[b.parcel].frontageEdge;
      if (front) {
        ++frontDoors[o.building];
        (raised ? stoops : grade) += 1;
      } else {
        ++rearDoors;
        (raised ? platforms : rearAtGrade) += 1;
      }
      if (!raised) continue;
      // A raised door's threshold: a walk surface right outside the wall at
      // the door's base height, connected (itself or via a neighbour ramp)
      // to the ground.
      const glm::vec2 dir = (q - a) / len;
      const glm::vec2 out(dir.y, -dir.x);
      // Probe just outside the wall: the stoop ramp top / platform sits
      // there at the door's base height (a stoop already slopes away, so
      // allow its grade over the probe distance).
      const glm::vec2 probe = a + dir * (0.5f * (o.u0 + o.u1)) + out * 0.12f;
      int threshold = -1;
      for (size_t s = 0; s < scene.walkSurfaces.size(); ++s) {
        const WalkSurface& surface = scene.walkSurfaces[s];
        if (!SurfaceContainsXZ(surface, probe.x, probe.y)) continue;
        if (std::fabs(SurfaceHeightAt(surface, probe.x, probe.y) - o.y0) < 0.15f) {
          threshold = static_cast<int>(s);
          break;
        }
      }
      CHECK(threshold >= 0);
      if (threshold < 0) continue;
      const WalkSurface& surface = scene.walkSurfaces[threshold];
      bool grounded = surface.connectsToGround;
      for (int ni : surface.neighbors) grounded |= scene.walkSurfaces[ni].connectsToGround;
      CHECK(grounded);
      if (!front && surface.neighbors.size() == 1) {
        // Which of the platform's three free sides carries the ramp.
        const WalkSurface& ramp = scene.walkSurfaces[surface.neighbors[0]];
        const glm::vec3 pc = SurfaceCenter(surface), rc = SurfaceCenter(ramp);
        const glm::vec2 d(rc.x - pc.x, rc.z - pc.z);
        const float along = glm::dot(d, dir), outward = glm::dot(d, out);
        rampSides.insert(std::fabs(along) > std::fabs(outward) ? (along < 0.0f ? 0 : 1) : 2);
      }
    }
    CHECK(windows > 50);
    int withDoor = 0;
    for (int doors : frontDoors) {
      CHECK(doors <= 1);
      withDoor += doors;
    }
    // A street entrance per building, bar the rare frontage too small for one.
    CHECK(withDoor > static_cast<int>(scene.buildings.size()) * 3 / 4);
  }
  // Across the pinned seeds: both entrance styles occur, rear doors exist
  // and are usually raised, and ramps land on more than one platform side.
  CHECK(stoops > 0 && grade > 0);
  CHECK(rearDoors > 0 && platforms > rearAtGrade);
  CHECK(rampSides.size() >= 2);
  CHECK(loadingBays > 0);  // Warehouses get wide loading entrances.
}

// --- Street furniture and traffic props (issue #150, roadmap 2.4/2.6). ---

constexpr float kPiTest = 3.14159265f;

MapGeneratorConfig FurnitureConfig(FeatureMode mode) {
  MapGeneratorConfig c = OldSizeConfig();
  c.elevatedHighway = OverpassMode::Off;
  c.features.terrain.mode = FeatureMode::Off;  // Flat-city street graph; furniture on terrain is later work.
  c.features.furniture.mode = mode;
  return c;
}

float DistanceToConvexPolygon(glm::vec2 p, const std::vector<glm::vec2>& poly) {
  if (InsideConvex(p, poly)) return 0.0f;
  float best = 1e9f;
  for (size_t i = 0; i < poly.size(); ++i) {
    best = std::min(best, DistanceToSegment(p, poly[i], poly[(i + 1) % poly.size()]));
  }
  return best;
}

bool SharedBoundaryEdge(const UrbanBlock& a, const UrbanBlock& b, glm::vec2* from,
                        glm::vec2* to) {
  for (size_t i = 0; i < a.vertices.size(); ++i) {
    const glm::vec2 a0 = a.vertices[i], a1 = a.vertices[(i + 1) % a.vertices.size()];
    for (size_t j = 0; j < b.vertices.size(); ++j) {
      const glm::vec2 b0 = b.vertices[j], b1 = b.vertices[(j + 1) % b.vertices.size()];
      if (glm::distance(a0, b1) < kEps && glm::distance(a1, b0) < kEps) {
        *from = a0;
        *to = a1;
        return true;
      }
    }
  }
  return false;
}

// Taller skylines cluster: heights beyond the old 11-unit cap appear, and
// buildings of one block are far closer in height than random pairs.
void TestSkylineReachesHigherAndClusters() {
  const MapGeneratorConfig c = FlatConfig();
  float tallest = 0.0f;
  double within = 0.0, across = 0.0;
  int withinN = 0, acrossN = 0;
  for (const uint32_t seed : kSeeds) {
    const Scene scene = GenerateUrbanMap(seed, c);
    std::vector<const Building*> ordinary;
    for (const Building& b : scene.buildings) {
      if (b.kind == BuildingKind::Civic || b.kind == BuildingKind::Commercial ||
          b.kind == BuildingKind::Garage) {
        continue;
      }
      CHECK(b.Height() <= c.skylineMaxHeight + kEps);
      tallest = std::max(tallest, b.Height());
      ordinary.push_back(&b);
    }
    for (size_t i = 0; i < ordinary.size(); ++i) {
      for (size_t j = i + 1; j < ordinary.size(); ++j) {
        const float diff = std::fabs(ordinary[i]->Height() - ordinary[j]->Height());
        if (ordinary[i]->block == ordinary[j]->block) {
          within += diff;
          ++withinN;
        } else {
          across += diff;
          ++acrossN;
        }
      }
    }
  }
  CHECK(tallest > c.maxBuildingHeight + 6.0f);
  CHECK(withinN > 0 && acrossN > 0);
  CHECK(within / withinN < 0.7 * across / acrossN);
}

// Extra arteries: wide avenues beside the main road, none when pinned to 0,
// and 0-2 drawn per seed otherwise (so one to three arteries in all).
void TestExtraArteries() {
  auto wide = [](uint32_t seed, const MapGeneratorConfig& c) {
    int count = 0;
    std::set<std::pair<int, int>> lines;
    for (const UrbanStreet& street : UrbanStreets(seed, c)) {
      if (street.cls != UrbanStreetClass::Avenue || street.width < c.arteryWidth * 0.75f - kEps) {
        continue;
      }
      // One avenue can be several trimmed segments: count distinct lines.
      const glm::vec2 a(street.a.x, street.a.z), b(street.b.x, street.b.z);
      const glm::vec2 n = glm::normalize(glm::vec2(-(b - a).y, (b - a).x));
      lines.insert({static_cast<int>(std::lround(glm::dot(n, a) * 0.5f)),
                    static_cast<int>(std::lround(std::atan2(n.y, n.x) * 20.0f))});
    }
    count = static_cast<int>(lines.size());
    return count;
  };
  int none = 0, some = 0;
  for (const uint32_t seed : kSeeds) {
    MapGeneratorConfig off = FlatConfig();
    off.extraArteries = 0;
    CHECK(wide(seed, off) == 0);
    MapGeneratorConfig two = FlatConfig();
    two.extraArteries = 2;
    CHECK(wide(seed, two) >= 1);
  }
  for (uint32_t seed = 1; seed <= 30; ++seed) {
    const MapGeneratorConfig autoConfig = FlatConfig();
    const int arteries = wide(seed, autoConfig);
    CHECK(arteries <= 2);
    (arteries == 0 ? none : some)++;
  }
  CHECK(none > 0 && some > 0);
}

// Long polygons no longer produce one empty perimeter-ring block. One seeded
// variant is a length-scaled row of short, angled, widely separated apartment
// slabs; the other is 2-3 touching blocks whose shared edges make a double
// sidewalk without a road or signal.
void TestLongBlockApartmentRowsAndPedestrianSplits() {
  {
    // Rows need blocks big enough for 5-window-wide slabs, so use the
    // default-size city and the first seed that has any (they're rare).
    MapGeneratorConfig c;
    c.features.furniture.mode = FeatureMode::Off;
    uint32_t seed = 1;
    for (; seed < 300; ++seed) {
      bool any = false;
      for (const UrbanBlock& b : UrbanBlocks(seed, c)) any |= b.apartmentRow;
      if (any) break;
    }
    const std::vector<UrbanBlock> blocks = UrbanBlocks(seed, c);
    const std::vector<UrbanParcel> parcels = UrbanParcels(seed, c);
    const Scene scene = GenerateUrbanMap(seed, c);
    int rows = 0, minCount = 100;
    for (size_t block = 0; block < blocks.size(); ++block) {
      if (!blocks[block].apartmentRow) continue;
      ++rows;
      int parcelCount = 0, buildingCount = 0;
      std::vector<const UrbanParcel*> row;
      for (const UrbanParcel& parcel : parcels) {
        if (parcel.block != static_cast<int>(block)) continue;
        ++parcelCount;
        row.push_back(&parcel);
      }
      for (const Building& building : scene.buildings) {
        if (building.block != static_cast<int>(block)) continue;
        ++buildingCount;
        CHECK(building.kind == BuildingKind::Residential);
      }
      if (parcelCount < 3 || parcelCount > 6) {
        std::fprintf(stderr, "apartment-row block %zu: %d parcels\n", block, parcelCount);
      }
      CHECK(parcelCount >= 3 && parcelCount <= 6);
      CHECK(buildingCount == parcelCount);

      glm::vec2 rowAxis(1.0f, 0.0f);
      float longest = 0.0f;
      for (size_t edge = 0; edge < blocks[block].vertices.size(); ++edge) {
        const glm::vec2 delta = blocks[block].vertices[(edge + 1) % blocks[block].vertices.size()] -
                                blocks[block].vertices[edge];
        if (glm::length(delta) > longest) {
          longest = glm::length(delta);
          rowAxis = delta / longest;
        }
      }
      const glm::vec2 crossAxis(-rowAxis.y, rowAxis.x);
      float blockCrossLo = 1.0e9f, blockCrossHi = -1.0e9f;
      for (const glm::vec2& point : blocks[block].vertices) {
        blockCrossLo = std::min(blockCrossLo, glm::dot(crossAxis, point));
        blockCrossHi = std::max(blockCrossHi, glm::dot(crossAxis, point));
      }
      std::vector<std::pair<float, float>> intervals;
      for (const UrbanParcel* parcel : row) {
        float parcelRowLo = 1.0e9f, parcelRowHi = -1.0e9f;
        float parcelCrossLo = 1.0e9f, parcelCrossHi = -1.0e9f;
        float longestEdge = 0.0f;
        glm::vec2 slabAxis(1.0f, 0.0f);
        for (size_t edge = 0; edge < parcel->footprint.size(); ++edge) {
          const glm::vec2 point = parcel->footprint[edge];
          CHECK(PointInConvexPolygon(point, blocks[block].vertices));
          parcelRowLo = std::min(parcelRowLo, glm::dot(rowAxis, point));
          parcelRowHi = std::max(parcelRowHi, glm::dot(rowAxis, point));
          parcelCrossLo = std::min(parcelCrossLo, glm::dot(crossAxis, point));
          parcelCrossHi = std::max(parcelCrossHi, glm::dot(crossAxis, point));
          const glm::vec2 delta =
              parcel->footprint[(edge + 1) % parcel->footprint.size()] - point;
          if (glm::length(delta) > longestEdge) {
            longestEdge = glm::length(delta);
            slabAxis = delta / longestEdge;
          }
        }
        // Five residential windows (1.1 wide, 1.0 gap, 0.7 margins) across.
        float shortestEdge = 1.0e9f;
        for (size_t edge = 0; edge < parcel->footprint.size(); ++edge) {
          shortestEdge = std::min(
              shortestEdge, glm::length(parcel->footprint[(edge + 1) % parcel->footprint.size()] -
                                        parcel->footprint[edge]));
        }
        CHECK(shortestEdge >= 10.9f - 0.01f);
        const float alignment = std::fabs(glm::dot(rowAxis, slabAxis));
        CHECK(alignment > 0.2f && alignment < 0.55f);  // Neither square nor lengthwise.
        CHECK(parcelCrossHi - parcelCrossLo < (blockCrossHi - blockCrossLo) * 0.75f);
        intervals.emplace_back(parcelRowLo, parcelRowHi);
      }
      std::sort(intervals.begin(), intervals.end());
      for (size_t i = 1; i < intervals.size(); ++i) {
        CHECK(intervals[i].first - intervals[i - 1].second > c.gapWidth * 1.5f);
      }
      minCount = std::min(minCount, parcelCount);
    }
    CHECK(rows >= 1);
    CHECK(minCount >= 3);  // A row is only spawned when 3+ decent slabs fit.
  }

  {
    const MapGeneratorConfig c = FurnitureConfig(FeatureMode::On);
    const std::vector<UrbanBlock> blocks = UrbanBlocks(42, c);
    // Split blocks are divided by a real street segment (the local street
    // width exactly), not by touching double sidewalks.
    const std::vector<UrbanStreet> streets = UrbanStreets(42, c);
    const Scene scene = GenerateUrbanMap(42, c);
    CHECK(scene.roads.size() == streets.size());
    int splitRoads = 0;
    for (size_t i = 0; i < blocks.size(); ++i) {
      for (size_t j = i + 1; j < blocks.size(); ++j) {
        glm::vec2 from, to;
        CHECK(!SharedBoundaryEdge(blocks[i], blocks[j], &from, &to));
      }
    }
    const float half = UrbanMapHalfExtent(c);
    for (const UrbanStreet& street : streets) {
      if (street.cls != UrbanStreetClass::Cross || std::fabs(street.width - c.localStreetWidth) > 1e-4f) {
        continue;
      }
      const float reach = std::max({std::fabs(street.a.x), std::fabs(street.a.z),
                                    std::fabs(street.b.x), std::fabs(street.b.z)});
      if (reach > half - 4.0f) continue;  // Runs to the map edge: an ordinary street.
      ++splitRoads;
      const glm::vec2 a(street.a.x, street.a.z), b(street.b.x, street.b.z);
      CHECK(glm::distance(a, b) < c.blockSize * 2.0f);
      // Both sides of it are blocks: it is a street between two blocks.
      const glm::vec2 mid = (a + b) * 0.5f;
      const glm::vec2 side = glm::normalize(glm::vec2(-(b - a).y, (b - a).x)) *
                             (c.localStreetWidth * 0.5f + 0.5f);
      bool left = false, right = false;
      for (const UrbanBlock& block : blocks) {
        left |= PointInConvexPolygon(mid + side, block.vertices);
        right |= PointInConvexPolygon(mid - side, block.vertices);
      }
      CHECK(left && right);
    }
    CHECK(splitRoads >= 1);
  }
}

// Furniture only appends small curb-side obstacles: the rest of the scene is
// untouched, every item hugs a sidewalk strip, keeps 2 * kAgentRadius of
// clear sidewalk to every building, and stays off the spawn rows. The Auto
// mode draws presence per seed, and a drawn seed matches the forced-on map.
void TestFurnitureHugsSidewalksAndRespectsClearances() {
  const MapGeneratorConfig on = FurnitureConfig(FeatureMode::On);
  const MapGeneratorConfig off = FurnitureConfig(FeatureMode::Off);
  for (uint32_t seed : kSeeds) {
    const Scene bare = GenerateUrbanMap(seed, off);
    const Scene scene = GenerateUrbanMap(seed, on);
    CHECK(bare.props.empty());
    CHECK(scene.obstacles.size() > bare.obstacles.size() + 20);
    for (size_t i = 0; i < bare.obstacles.size(); ++i) {
      CHECK(scene.obstacles[i].bounds.min == bare.obstacles[i].bounds.min);
      CHECK(scene.obstacles[i].bounds.max == bare.obstacles[i].bounds.max);
      CHECK(scene.obstacles[i].footprint == bare.obstacles[i].footprint);
      CHECK(scene.obstacles[i].kind == bare.obstacles[i].kind);
    }
    CHECK(SameScene(scene, GenerateUrbanMap(seed, on)));

    const std::vector<UrbanBlock> blocks = UrbanBlocks(seed, on);
    for (size_t i = bare.obstacles.size(); i < scene.obstacles.size(); ++i) {
      const Obstacle& o = scene.obstacles[i];
      if (o.kind == ObstacleKind::Cover) continue;  // Covered by the cover-item test.
      CHECK(o.footprint.empty());
      CHECK(!o.climbable);
      CHECK(o.kind == ObstacleKind::StreetFurniture);
      const glm::vec3 size = o.bounds.max - o.bounds.min;
      CHECK(size.x <= 0.4f && size.z <= 0.4f);  // Post-sized, never cover-sized.
      const glm::vec2 center(o.bounds.Center().x, o.bounds.Center().z);
      bool signalPost = false;  // Traffic-control poles sit at junctions, not on sidewalks only.
      for (const StreetProp& prop : scene.props) {
        signalPost |= prop.kind != StreetPropKind::LampPost &&
                      glm::distance(glm::vec2(prop.position.x, prop.position.z), center) < 1e-4f;
      }
      if (signalPost) continue;
      const float halfOut = 0.5f * std::max(size.x, size.z);
      bool onSidewalk = false;
      for (const UrbanBlock& block : blocks) {
        if (!InsideConvex(center, block.vertices)) continue;
        float edgeDistance = 1e9f;
        for (size_t e = 0; e < block.vertices.size(); ++e) {
          edgeDistance = std::min(
              edgeDistance, DistanceToSegment(center, block.vertices[e],
                                              block.vertices[(e + 1) % block.vertices.size()]));
        }
        onSidewalk |= edgeDistance <= on.sidewalkWidth;
      }
      CHECK(onSidewalk);
      for (const Obstacle& building : bare.obstacles) {
        if (building.footprint.empty()) continue;
        const float gap = DistanceToConvexPolygon(center, building.footprint) - halfOut;
        CHECK(gap >= 2.0f * constants::kAgentRadius - kEps);
      }
      for (const Unit& unit : scene.units) {
        CHECK(glm::distance(glm::vec2(unit.position.x, unit.position.z), center) >= 2.5f);
      }
    }
  }

  // Auto is unconditional: it matches the forced-On scene for every seed.
  for (uint32_t seed = 1; seed <= 12; ++seed) {
    MapGeneratorConfig base = OldSizeConfig();
    base.elevatedHighway = OverpassMode::Off;
    base.features.terrain.mode = FeatureMode::Off;
    const Scene scene = GenerateUrbanMap(seed, base);
    CHECK(!scene.props.empty());
    CHECK(SameScene(scene, GenerateUrbanMap(seed, FurnitureConfig(FeatureMode::On))));
  }
}

// The required connectivity property (roadmap 2.4): everything walkable
// without furniture stays walkable with it -- items never seal a sidewalk,
// alley or opening.
void TestFurnitureKeepsSidewalksConnected() {
  for (uint32_t seed : kSeeds) {
    const Scene bare = GenerateUrbanMap(seed, FurnitureConfig(FeatureMode::Off));
    const Scene scene = GenerateUrbanMap(seed, FurnitureConfig(FeatureMode::On));
    NavMesh bareNav, nav;
    bareNav.Build(bare.obstacles, bare.mapHalfExtent, constants::kAgentRadius);
    nav.Build(scene.obstacles, scene.mapHalfExtent, constants::kAgentRadius);
    const glm::vec3 blue = scene.units.front().position;
    std::vector<glm::vec3> path;
    CHECK(nav.FindPath(blue, scene.units.back().position, &path));
    // One routed-distance flood per mesh (per-point FindPath would cost
    // seconds each), then probe rings around every item: whatever was
    // reachable right next to it without furniture must stay reachable.
    const float budget = 4.0f * scene.mapHalfExtent;
    const ReachField bareField = bareNav.ComputeReachField(blue, budget, 0.5f);
    const ReachField field = nav.ComputeReachField(blue, budget, 0.5f);
    auto reachedAt = [](const ReachField& f, glm::vec2 p) {
      const int ix = static_cast<int>(std::round((p.x - f.minX) / f.step));
      const int iz = static_cast<int>(std::round((p.y - f.minZ) / f.step));
      return f.Reached(ix, iz);
    };
    int compared = 0, unreachable = 0;
    for (size_t i = bare.obstacles.size(); i < scene.obstacles.size(); ++i) {
      const glm::vec3 center = scene.obstacles[i].bounds.Center();
      for (const float radius : {1.2f, 2.5f}) {
        for (int k = 0; k < 8; ++k) {
          const float angle = static_cast<float>(k) * 0.25f * kPiTest;
          const glm::vec2 target(center.x + radius * std::cos(angle),
                                 center.z + radius * std::sin(angle));
          // Keep clear of every item so the furniture's own inflated
          // footprint never explains a miss.
          bool nearItem = false;
          for (size_t j = bare.obstacles.size(); j < scene.obstacles.size(); ++j) {
            const glm::vec3 other = scene.obstacles[j].bounds.Center();
            nearItem |= j != i && glm::distance(glm::vec2(other.x, other.z), target) < 1.0f;
          }
          // Cover-sized items swallow the ring points that land inside them.
          for (size_t j = bare.obstacles.size(); j < scene.obstacles.size(); ++j) {
            const Obstacle& other = scene.obstacles[j];
            nearItem |= other.kind == ObstacleKind::Cover &&
                        target.x > other.bounds.min.x - 0.9f && target.x < other.bounds.max.x + 0.9f &&
                        target.y > other.bounds.min.z - 0.9f && target.y < other.bounds.max.z + 0.9f;
          }
          if (nearItem || !reachedAt(bareField, target)) continue;
          ++compared;
          if (!reachedAt(field, target)) ++unreachable;
        }
      }
    }
    CHECK(compared > 300);
    if (unreachable) {
      std::fprintf(stderr, "seed %u: furniture cut off %d probe points\n", seed, unreachable);
    }
    CHECK(unreachable == 0);
  }
}

// Traffic lights stand on junction corners (two streets of distinct heading
// within reach), stop/yield signs appear at artery tees, and every control
// faces along one of those streets. Street-name blades reproduce both actual
// corner headings (including skewed angles), while lamp arms point out over
// the road from a taller collision pole.
void TestTrafficPropsSitAtJunctionsAndStayVisualOnly() {
  const MapGeneratorConfig on = FurnitureConfig(FeatureMode::On);
  int lights = 0, stopYield = 0, names = 0, lamps = 0, skewedNames = 0;
  const auto direction = [](float yaw) { return glm::vec2(std::cos(yaw), std::sin(yaw)); };
  const auto parallel = [](glm::vec2 a, glm::vec2 b) {
    return std::fabs(a.x * b.y - a.y * b.x) < 1.0e-4f;
  };
  for (uint32_t seed : kSeeds) {
    const Scene scene = GenerateUrbanMap(seed, on);
    CHECK(!scene.props.empty());
    const std::vector<UrbanStreet> streets = UrbanStreets(seed, on);
    const std::vector<UrbanBlock> blocks = UrbanBlocks(seed, on);
    for (const StreetProp& prop : scene.props) {
      const glm::vec2 p(prop.position.x, prop.position.z);
      CHECK(std::fabs(p.x) < scene.mapHalfExtent - 0.4f);
      CHECK(std::fabs(p.y) < scene.mapHalfExtent - 0.4f);
      if (prop.kind == StreetPropKind::LampPost) {
        ++lamps;
        bool hasPole = false;
        for (const Obstacle& o : scene.obstacles) {
          const glm::vec3 center = o.bounds.Center();
          if (glm::distance(glm::vec2(center.x, center.z), p) > 1.0e-3f) continue;
          hasPole |= o.bounds.max.y - prop.position.y >= 5.39f;
        }
        CHECK(hasPole);
        // The arm crosses its curb at a right angle and points out of the
        // block over the road, including along oblique streets.
        bool hasCurb = false;
        float curbDistance = 1e9f;
        glm::vec2 curbDirection(0.0f), blockInterior(0.0f);
        for (const UrbanBlock& block : blocks) {
          if (!InsideConvex(p, block.vertices)) continue;
          glm::vec2 centroid(0.0f);
          for (const glm::vec2& vertex : block.vertices) centroid += vertex;
          centroid /= static_cast<float>(block.vertices.size());
          for (size_t e = 0; e < block.vertices.size(); ++e) {
            const glm::vec2 a = block.vertices[e];
            const glm::vec2 b = block.vertices[(e + 1) % block.vertices.size()];
            const float distance = DistanceToSegment(p, a, b);
            if (distance >= curbDistance) continue;
            curbDistance = distance;
            curbDirection = glm::normalize(b - a);
            blockInterior = centroid - p;
            hasCurb = true;
          }
        }
        CHECK(hasCurb);
        if (hasCurb) {
          const glm::vec2 armDirection = direction(prop.yaw);
          CHECK(std::fabs(glm::dot(armDirection, curbDirection)) < 1.0e-4f);
          CHECK(glm::dot(armDirection, blockInterior) < 0.0f);
        }
        continue;
      }
      for (const Obstacle& o : scene.obstacles) {
        // A prop's own collision post sits exactly under it.
        if (glm::distance(p, glm::vec2(o.bounds.Center().x, o.bounds.Center().z)) < 1e-4f) continue;
        CHECK(DistanceToConvexPolygon(p, ObstacleFootprint(o)) > 0.2f);
      }
      // Signs and lights stand on a sidewalk corner, never out in the street.
      bool onSidewalk = false;
      for (const UrbanBlock& block : blocks) onSidewalk |= InsideConvex(p, block.vertices);
      CHECK(onSidewalk);
      if (prop.kind == StreetPropKind::StreetNameSign) {
        ++names;
        float corner = 1e9f;
        const UrbanBlock* closestBlock = nullptr;
        size_t closestVertex = 0;
        for (const UrbanBlock& block : blocks) {
          for (size_t v = 0; v < block.vertices.size(); ++v) {
            const float distance = glm::distance(p, block.vertices[v]);
            if (distance >= corner) continue;
            corner = distance;
            closestBlock = &block;
            closestVertex = v;
          }
        }
        CHECK(corner <= 1.0f);
        CHECK(closestBlock != nullptr);
        if (closestBlock) {
          const std::vector<glm::vec2>& poly = closestBlock->vertices;
          const glm::vec2 next =
              glm::normalize(poly[(closestVertex + 1) % poly.size()] - poly[closestVertex]);
          const glm::vec2 previous = glm::normalize(
              poly[closestVertex] - poly[(closestVertex + poly.size() - 1) % poly.size()]);
          CHECK(parallel(direction(prop.yaw), next));
          CHECK(parallel(direction(prop.secondaryYaw), previous));
          if (std::fabs(glm::dot(next, previous)) > 0.05f) ++skewedNames;
        }
        continue;
      }
      if (prop.kind == StreetPropKind::TrafficLight) {
        ++lights;
      } else {
        CHECK(prop.kind == StreetPropKind::StopSign || prop.kind == StreetPropKind::YieldSign);
        ++stopYield;
      }
      // A junction: two nearby streets with clearly different headings.
      glm::vec2 firstDir(0.0f);
      bool junction = false, aligned = false;
      for (const UrbanStreet& street : streets) {
        const glm::vec2 a(street.a.x, street.a.z), b(street.b.x, street.b.z);
        if (DistanceToSegment(p, a, b) > street.width * 0.5f + 4.0f) continue;
        const glm::vec2 dir = glm::normalize(b - a);
        aligned |= parallel(direction(prop.yaw), dir);
        if (firstDir == glm::vec2(0.0f)) {
          firstDir = dir;
        } else if (std::fabs(firstDir.x * dir.y - firstDir.y * dir.x) > 0.25f) {
          junction = true;
        }
      }
      CHECK(junction);
      CHECK(aligned);
    }
  }
  CHECK(lights > 0);
  CHECK(stopYield > 0);
  CHECK(names > 0);
  CHECK(lamps > 0);
  CHECK(skewedNames > 0);

  // Traffic/sign fixtures add no gameplay surfaces; lamp collision remains
  // the already-tested furniture pole. The off scene carries neither kind.
  CHECK(GenerateUrbanMap(kSeeds[0], FurnitureConfig(FeatureMode::Off)).props.empty());
}

// Cover-sized street items (issue #153, roadmap 2.5): every item is a box
// whose top clears a foot probe but not a standing eye, so a standing eye
// sees over it while a low probe is hidden behind it; the Off switch removes
// them all; and the layout is deterministic. Parked vehicles (sedans, SUVs,
// pickups) are climbable. The coach bus is the one full-height exception,
// covered by TestCoachBusesStopMidStreet.
void TestCoverItemsAreEyeLowAndFootHigh() {
  const MapGeneratorConfig on = FurnitureConfig(FeatureMode::On);
  const MapGeneratorConfig off = FurnitureConfig(FeatureMode::Off);
  std::set<StreetFixtureModel> allModels;
  for (uint32_t seed : {1u, 7u, 42u, 2024u}) {
    const Scene scene = GenerateUrbanMap(seed, on);
    CHECK(SameScene(scene, GenerateUrbanMap(seed, on)));
    for (const Obstacle& o : GenerateUrbanMap(seed, off).obstacles) {
      CHECK(o.kind != ObstacleKind::Cover);
    }
    int cover = 0, climbable = 0, large = 0;
    std::set<StreetFixtureModel> models;
    for (const Obstacle& o : scene.obstacles) {
      if (o.fixtureModel != StreetFixtureModel::None) {
        models.insert(o.fixtureModel);
        CHECK(IsStreetFixture(o.kind));
        CHECK(o.fixtureBaseY >= o.bounds.min.y);
        CHECK(o.fixtureBaseY < o.bounds.max.y);
      }
      if (o.kind != ObstacleKind::Cover) continue;
      CHECK(o.fixtureModel != StreetFixtureModel::None);
      if (!o.footprint.empty()) continue;  // Artery wrecks: TestArteryWrecks.
      if (o.fixtureModel == StreetFixtureModel::CoachBus) continue;
      ++cover;
      climbable += o.climbable;
      CHECK(o.climbable == IsCurbVehicle(o.fixtureModel));
      const glm::vec3 size = o.bounds.max - o.bounds.min;
      large += std::max(size.x, size.z) > 1.0f;
      const float base = o.bounds.min.y;
      CHECK(size.y > 0.5f);                     // Hides a low probe...
      CHECK(o.bounds.max.y < base + 0.12f + constants::kEyeHeight);  // ...under eye level.
      const glm::vec3 c = o.bounds.Center();
      const float reachX = size.x * 0.5f + 1.0f, reachZ = size.z * 0.5f + 1.0f;
      // Cross the box along its short axis at eye height (clear) and at
      // foot-probe height (blocked). Other obstacles near the item could
      // interfere, so test against the item alone.
      const std::vector<Obstacle> alone = {o};
      const bool acrossX = size.x <= size.z;
      const glm::vec3 d = acrossX ? glm::vec3(reachX, 0.0f, 0.0f) : glm::vec3(0.0f, 0.0f, reachZ);
      CHECK(LineOfSightClear(glm::vec3(c.x, base + constants::kEyeHeight, c.z) - d,
                             glm::vec3(c.x, base + constants::kEyeHeight, c.z) + d, alone));
      CHECK(!LineOfSightClear(glm::vec3(c.x, base + 0.3f, c.z) - d,
                              glm::vec3(c.x, base + 0.3f, c.z) + d, alone));
    }
    CHECK(cover > 10);
    CHECK(climbable > 0);
    CHECK(large > 0);
    allModels.insert(models.begin(), models.end());
  }
  // Every kind shows up somewhere across the seeds (a single layout may skip one).
  for (StreetFixtureModel model : {StreetFixtureModel::ParkedCar, StreetFixtureModel::Suv,
                                     StreetFixtureModel::Pickup, StreetFixtureModel::Bench,
                                     StreetFixtureModel::Dumpster, StreetFixtureModel::TrashBin}) {
    CHECK(allModels.count(model) == 1);
  }
}

// A coach bus stopped mid-street is a seeded rarity: some maps have one or
// two, many have none. Each one is a full-height wall of a vehicle (taller
// than a standing eye, not climbable) standing on level road grade with a
// walkable lane to every building and other obstacle, clear of the spawn
// rows, and placed deterministically.
void TestCoachBusesStopMidStreet() {
  const MapGeneratorConfig on = FurnitureConfig(FeatureMode::On);
  const float lane = 2.0f * constants::kAgentRadius + 0.2f;
  int mapsWithBus = 0, mapsWithout = 0;
  for (uint32_t seed = 1; seed <= 24; ++seed) {
    const Scene scene = GenerateUrbanMap(seed, on);
    std::vector<const Obstacle*> buses;
    for (const Obstacle& o : scene.obstacles) {
      // Artery buses sit at any angle on the road (TestArteryWrecks).
      if (o.fixtureModel == StreetFixtureModel::CoachBus && o.footprint.empty()) buses.push_back(&o);
    }
    CHECK(buses.size() <= 2);
    (buses.empty() ? mapsWithout : mapsWithBus) += 1;
    for (const Obstacle* bus : buses) {
      CHECK(bus->kind == ObstacleKind::Cover);
      CHECK(!bus->climbable);
      CHECK(bus->footprint.empty());
      const glm::vec3 size = bus->bounds.max - bus->bounds.min;
      CHECK(size.y > constants::kEyeHeight + 1.0f);
      CHECK(std::max(size.x, size.z) > 10.0f);  // A coach, not a car.
      CHECK(std::fabs(bus->fixtureBaseY - bus->bounds.min.y) < 0.1f);
      const glm::vec2 lo(bus->bounds.min.x, bus->bounds.min.z);
      const glm::vec2 hi(bus->bounds.max.x, bus->bounds.max.z);
      // Level road under every corner and the middle.
      const float base = bus->bounds.min.y;
      for (const glm::vec2& q : {lo, hi, glm::vec2(lo.x, hi.y), glm::vec2(hi.x, lo.y),
                                 (lo + hi) * 0.5f}) {
        CHECK(std::fabs(scene.ground.HeightAt(q.x, q.y) - base) < 0.05f);
      }
      for (const Obstacle& o : scene.obstacles) {
        if (&o == bus) continue;
        float gap = 1e9f;
        for (const glm::vec2& v : ObstacleFootprint(o)) {
          const glm::vec2 d = glm::max(glm::max(lo - v, v - hi), glm::vec2(0.0f));
          gap = std::min(gap, glm::length(d));
        }
        for (const glm::vec2& q : {lo, hi, glm::vec2(lo.x, hi.y), glm::vec2(hi.x, lo.y),
                                   (lo + hi) * 0.5f}) {
          gap = std::min(gap, DistanceToConvexPolygon(q, ObstacleFootprint(o)));
        }
        if (gap < lane - 0.01f) {
          std::fprintf(stderr, "seed %u: bus at (%.1f, %.1f) only %.2f from an obstacle\n", seed,
                       (lo.x + hi.x) * 0.5f, (lo.y + hi.y) * 0.5f, gap);
        }
        CHECK(gap >= lane - 0.01f);
      }
      for (const Unit& unit : scene.units) {
        const glm::vec2 u(unit.position.x, unit.position.z);
        const glm::vec2 d = glm::max(glm::max(lo - u, u - hi), glm::vec2(0.0f));
        CHECK(glm::length(d) >= 5.0f - 0.01f);
      }
    }
    if (!buses.empty()) CHECK(SameScene(scene, GenerateUrbanMap(seed, on)));
  }
  CHECK(mapsWithBus >= 4);
  CHECK(mapsWithout >= 4);
}

// Wrecks on the arteries: cars at any heading (upright, overturned or on a
// side) and a coach bus more often than on a local street. Each is a rotated
// convex footprint on level road at grade, clear of every other obstacle, the
// spawn rows and the map edge, and generation is deterministic.
void TestArteryWrecks() {
  const float kPi = 3.14159265358979f;
  const MapGeneratorConfig on = FurnitureConfig(FeatureMode::On);
  const float lane = 2.0f * constants::kAgentRadius + 0.2f;
  int wrecks = 0, upright = 0, overturned = 0, onSide = 0, angled = 0, arteryBuses = 0;
  int mapsWithArteryBus = 0;
  for (uint32_t seed = 1; seed <= 24; ++seed) {
    const Scene scene = GenerateUrbanMap(seed, on);
    CHECK(SameScene(scene, GenerateUrbanMap(seed, on)));
    const std::vector<UrbanRoad> roads = UrbanRoads(seed, on);
    bool hasBus = false;
    for (size_t i = 0; i < scene.obstacles.size(); ++i) {
      const Obstacle& o = scene.obstacles[i];
      if (o.kind != ObstacleKind::Cover || o.footprint.empty()) continue;
      ++wrecks;
      CHECK(!o.climbable);
      CHECK(o.footprint.size() == 4);
      CHECK(o.fixtureModel != StreetFixtureModel::None);
      const bool bus = o.fixtureModel == StreetFixtureModel::CoachBus;
      arteryBuses += bus;
      hasBus |= bus;
      const float roll = o.fixtureRoll;
      const bool side = std::fabs(std::fabs(roll) - 0.5f * kPi) < 1e-4f;
      CHECK(roll == 0.0f || side || std::fabs(roll - kPi) < 1e-4f);
      CHECK(!bus || roll == 0.0f);
      upright += roll == 0.0f;
      overturned += std::fabs(roll - kPi) < 1e-4f;
      onSide += side;
      const glm::vec3 extent = StreetFixtureExtent(o.fixtureModel);
      const float width = side ? extent.y : extent.z;
      const float height = side ? extent.z : extent.y;
      CHECK(std::fabs(PolygonArea(o.footprint) - extent.x * width) < 0.05f);
      CHECK(std::fabs(o.bounds.max.y - o.fixtureBaseY - height) < 0.05f);
      const glm::vec2 heading(std::cos(o.fixtureYaw), std::sin(o.fixtureYaw));
      angled += std::fabs(heading.x) > 0.2f && std::fabs(heading.y) > 0.2f;
      const glm::vec3 mid = o.bounds.Center();
      CHECK(std::fabs(scene.ground.HeightAt(mid.x, mid.z) - o.bounds.min.y) < 0.05f);
      for (const glm::vec2& v : o.footprint) {
        CHECK(std::fabs(v.x) <= scene.mapHalfExtent && std::fabs(v.y) <= scene.mapHalfExtent);
        // On the pavement of a road (the artery or its branch).
        bool onRoad = false;
        for (const UrbanRoad& road : roads) {
          for (size_t k = 0; k + 1 < road.centerline.size(); ++k) {
            onRoad |= DistanceToSegment(v, glm::vec2(road.centerline[k].x, road.centerline[k].z),
                                        glm::vec2(road.centerline[k + 1].x,
                                                  road.centerline[k + 1].z)) <= road.width * 0.5f;
          }
        }
        CHECK(onRoad);
      }
      for (size_t j = 0; j < scene.obstacles.size(); ++j) {
        if (j == i) continue;
        const std::vector<glm::vec2> other = ObstacleFootprint(scene.obstacles[j]);
        float gap = 1e9f;
        for (const glm::vec2& v : o.footprint) gap = std::min(gap, DistanceToConvexPolygon(v, other));
        for (const glm::vec2& v : other) gap = std::min(gap, DistanceToConvexPolygon(v, o.footprint));
        CHECK(!FootprintsOverlap(o.footprint, other));
        if (bus) CHECK(gap >= lane - 0.01f);
      }
      for (const Unit& unit : scene.units) {
        CHECK(DistanceToConvexPolygon(glm::vec2(unit.position.x, unit.position.z), o.footprint) >=
              5.0f - 0.01f);
      }
    }
    mapsWithArteryBus += hasBus;
  }
  CHECK(wrecks > 40);
  CHECK(upright > 0 && overturned > 0 && onSide > 0);
  CHECK(angled > 0);
  CHECK(arteryBuses >= 4);
  CHECK(mapsWithArteryBus >= 4);
}

// Pinned per-seed furniture output (issue #150): a generator change that
// alters item or prop placement shows up here first.
void TestPinnedFurnitureFingerprints() {
  struct Expected { uint32_t seed; size_t furniture; size_t props; };
  const Expected expected[] = {{1u, 154, 75}, {42u, 160, 67}, {2024u, 166, 75}};
  const MapGeneratorConfig on = FurnitureConfig(FeatureMode::On);
  const MapGeneratorConfig off = FurnitureConfig(FeatureMode::Off);
  for (const auto& e : expected) {
    const Scene scene = GenerateUrbanMap(e.seed, on);
    size_t furniture = 0;
    for (const Obstacle& o : scene.obstacles) furniture += o.kind == ObstacleKind::StreetFurniture;
    if (furniture != e.furniture || scene.props.size() != e.props) {
      std::fprintf(stderr, "seed %u: %zu furniture, %zu props (expected %zu, %zu)\n", e.seed,
                   furniture, scene.props.size(), e.furniture, e.props);
    }
    CHECK(furniture == e.furniture);
    CHECK(scene.props.size() == e.props);
  }
}

int main() {
  TestFeatureFlagsAndStreetGraph();
  TestLongBlockApartmentRowsAndPedestrianSplits();
  TestFurnitureHugsSidewalksAndRespectsClearances();
  TestFurnitureKeepsSidewalksConnected();
  TestTrafficPropsSitAtJunctionsAndStayVisualOnly();
  TestCoverItemsAreEyeLowAndFootHigh();
  TestCoachBusesStopMidStreet();
  TestArteryWrecks();
  TestPinnedFurnitureFingerprints();
  TestDeterminismAndVariety();
  TestZiplinesAreDeterministicBoundedAndValid();
  TestZiplineStreetEndsReachableRoofsOnlyByLine();
  TestPinnedSeedFingerprints();
  TestParcelsMatchBuildingsAndFrontStreets();
  TestBuildingKindsFloorsAndHeights();
  TestSkylineReachesHigherAndClusters();
  TestExtraArteries();
  TestFacadeOpeningsDoorsStoopsAndPlatforms();
  TestMassingStaysOnItsBuilding();
  TestRoofMassingBlocksWalking();
  TestMassingBlocksSight();
  TestPinnedKindHistograms();
  TestMapIsMuchLargerThanDefault();
  TestStreetsAndSidewalksAreObstacleFree();
  TestEveryBlockHasWallToWallAndGappedRuns();
  TestVariedHeightsWithFewTowers();
  TestBlocksAndStreetsVary();
  TestSidewalksCoverEveryBlock();
  TestLocalStreetsPavedAndSidewalkIsRing();
  TestLocalStreetBandsReachMapEdgeAtFullWidth();
  TestLotVariety();
  TestNavMeshFullyReachableFromSpawns();
  TestWindowedNavMeshMatchesGlobalWithinBudget();
  TestGameLogicUsesRangeScopedNavMesh();
  TestVisibilityStillSpansWholeMap();
  TestObliqueArteryAndBranchMerge();
  TestPolygonBlocksAdaptToAngledStreets();
  TestNoCornerSplitBuildings();
  TestCornerBuildingsDoNotChainAcrossAFrontage();
  TestThinCornerBuildingSliversAreDropped();
  TestPolygonCollisionLosAndNavigationUseRealFootprint();
  TestForkRailsFollowDeckEdges();
  TestElevatedOverpassRampsPiersAndDistinctLayers();
  TestOverpassPresenceAndLayoutsAreSeedDrivenAndForceable();
  TestOverpassLayoutGeometryAndBranchRampOff();
  TestBranchMayRunOffMapWithoutRamp();
  TestMoveFrontierCoversBranchRampAtJunction();
  TestBranchJunctionCrossingsStayLocalAcrossLayouts();
  TestSpawnReachabilityAndSurfaceSnapshotSynchronization();
  TestHillyDeterminismAndVariety();
  TestHillyTerrainIsGenuinelyUneven();
  TestHillyUnitsSpawnOnTerrain();
  TestHillyPathsFollowTerrain();
  TestHillyReachFieldCostsIncludeSlope();
  TestHillyGameLogicMoveLandsOnTerrain();
  TestHillyShortSameCellMoveStaysOnTerrain();
  TestSpawnHintsPlaceOppositeCornersAndCenterFlag();
  if (g_failures) {
    std::fprintf(stderr, "%d check(s) failed\n", g_failures);
    return 1;
  }
  std::printf("map generator tests passed\n");
  return 0;
}
