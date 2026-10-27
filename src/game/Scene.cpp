#include "game/Scene.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace tactics {

const char* BuildingKindName(BuildingKind kind) {
  switch (kind) {
    case BuildingKind::Residential: return "residential";
    case BuildingKind::MixedUse: return "mixed-use";
    case BuildingKind::Office: return "office";
    case BuildingKind::Commercial: return "commercial";
    case BuildingKind::Civic: return "civic";
    case BuildingKind::Garage: return "garage";
  }
  return "unknown";
}
namespace {

constexpr float kPi = 3.14159265358979323846f;

Obstacle MakeObstacle(float centerX, float centerZ, float halfWidthX, float halfWidthZ,
                       float height, bool climbable) {
  return Obstacle{AABB{glm::vec3(centerX - halfWidthX, 0.0f, centerZ - halfWidthZ),
                        glm::vec3(centerX + halfWidthX, height, centerZ + halfWidthZ)},
                   climbable};
}

}  // namespace

Scene BuildDefaultScene() {
  Scene scene;

  // 30x30 field. Non-climbable structures (taller than a figure can climb)
  // are fully impassable to the navmesh and block shots; the squads start at
  // opposite ends with no clear sightline between them.
  constexpr float kWallHeight = 2.5f;
  constexpr float kFenceHeight = 1.7f;
  auto wall = [&](float cx, float cz, float hx, float hz) {
    scene.obstacles.push_back(MakeObstacle(cx, cz, hx, hz, kWallHeight, /*climbable=*/false));
  };
  auto fence = [&](float cx, float cz, float hx, float hz) {
    scene.obstacles.push_back(MakeObstacle(cx, cz, hx, hz, kFenceHeight, /*climbable=*/false));
  };

  // Central building: a walled room with a door gap (x in [-1, 1]) in its
  // south wall, blocking the middle lane.
  wall(0.0f, -3.5f, 3.75f, 0.25f);   // North wall.
  wall(-3.5f, 0.0f, 0.25f, 3.25f);   // West wall.
  wall(3.5f, 0.0f, 0.25f, 3.25f);    // East wall.
  wall(-2.25f, 3.5f, 1.5f, 0.25f);   // South wall, west of the door.
  wall(2.25f, 3.5f, 1.5f, 0.25f);    // South wall, east of the door.

  // Two small solid buildings in the outer corners.
  wall(-8.0f, -11.5f, 2.0f, 1.5f);
  wall(8.0f, 11.5f, 2.0f, 1.5f);

  // Fences screening the outer lanes near each squad, plus a pair guarding
  // the approach to the central building.
  fence(-7.0f, -6.0f, 0.15f, 2.5f);
  fence(7.0f, 6.0f, 0.15f, 2.5f);
  fence(-7.0f, 6.0f, 2.4f, 0.15f);
  fence(7.0f, -6.0f, 2.4f, 0.15f);

  // A handful of standalone crates for extra path variety. Low enough to be
  // climbable: the navmesh connects ground level to their tops (Stage C).
  scene.obstacles.push_back(MakeObstacle(-4.0f, 6.5f, 0.6f, 0.6f, 1.2f, /*climbable=*/true));
  scene.obstacles.push_back(MakeObstacle(4.0f, -6.5f, 0.6f, 0.6f, 1.2f, /*climbable=*/true));
  scene.obstacles.push_back(MakeObstacle(-3.5f, -7.5f, 0.6f, 0.6f, 1.2f, /*climbable=*/true));
  scene.obstacles.push_back(MakeObstacle(3.5f, 7.5f, 0.6f, 0.6f, 1.2f, /*climbable=*/true));
  scene.obstacles.push_back(MakeObstacle(0.0f, 0.0f, 0.6f, 0.6f, 1.2f, /*climbable=*/true));

  const float spawnX = 12.0f;
  const float rows[3] = {-6.0f, 0.0f, 6.0f};

  for (int i = 0; i < 3; ++i) {
    Unit blue;
    blue.id = i;
    blue.team = Team::Blue;
    blue.position = glm::vec3(-spawnX, 0.0f, rows[i]);
    blue.facingYaw = 0.0f;  // Faces +X, toward the Red side.
    blue.weapon = DefaultWeaponForUnit(blue.id);
    scene.units.push_back(blue);
  }
  for (int i = 0; i < 3; ++i) {
    Unit red;
    red.id = 3 + i;
    red.team = Team::Red;
    red.position = glm::vec3(spawnX, 0.0f, rows[i]);
    red.facingYaw = kPi;  // Faces -X, toward the Blue side.
    red.weapon = DefaultWeaponForUnit(red.id);
    scene.units.push_back(red);
  }

  return scene;
}

bool IsFreeSpot(const Scene& scene, float x, float z) {
  if (std::abs(x) > scene.mapHalfExtent || std::abs(z) > scene.mapHalfExtent) return false;
  const float m = constants::kAgentRadius;
  for (const Obstacle& o : scene.obstacles) {
    if (x > o.bounds.min.x - m && x < o.bounds.max.x + m && z > o.bounds.min.z - m &&
        z < o.bounds.max.z + m) {
      return false;
    }
  }
  return true;
}

glm::vec2 NearestFreeSpot(const Scene& scene, glm::vec2 target) {
  constexpr float kStep = 0.5f;
  if (IsFreeSpot(scene, target.x, target.y)) return target;
  for (int ring = 1; ring * kStep <= scene.mapHalfExtent * 2.0f; ++ring) {
    float bestD = 1e30f;
    glm::vec2 best = target;
    for (int ix = -ring; ix <= ring; ++ix) {
      for (int iz = -ring; iz <= ring; ++iz) {
        if (std::max(std::abs(ix), std::abs(iz)) != ring) continue;
        const glm::vec2 p = target + glm::vec2(ix, iz) * kStep;
        const float d = ix * ix + iz * iz;
        if (d < bestD && IsFreeSpot(scene, p.x, p.y)) {
          bestD = d;
          best = p;
        }
      }
    }
    if (bestD < 1e29f) return best;
  }
  return target;
}

void SuggestSpawnHints(Scene* scene) {
  const float corner = scene->mapHalfExtent * 0.8f;
  scene->spawnHints.blue = glm::vec2(-corner, -corner);
  scene->spawnHints.red = glm::vec2(corner, corner);
  scene->spawnHints.flag = glm::vec2(0.0f, 0.0f);
}

void ApplySpawnHints(Scene* scene) {
  const auto& hints = scene->spawnHints;
  for (const Team team : {Team::Blue, Team::Red}) {
    const auto& hint = team == Team::Blue ? hints.blue : hints.red;
    const auto& other = team == Team::Blue ? hints.red : hints.blue;
    if (!hint) continue;
    // Squad row perpendicular to the corner's diagonal, 2 units apart.
    const glm::vec2 toCenter = glm::normalize(-*hint + glm::vec2(1e-3f));
    const glm::vec2 across(-toCenter.y, toCenter.x);
    const float yaw = other ? std::atan2(other->y - hint->y, other->x - hint->x)
                            : std::atan2(toCenter.y, toCenter.x);
    std::vector<glm::vec2> taken;
    int slot = 0;
    for (Unit& unit : scene->units) {
      if (unit.team != team) continue;
      const float offset = (slot++ % 3 - 1) * 2.0f;
      glm::vec2 want = *hint + across * offset;
      glm::vec2 p = NearestFreeSpot(*scene, want);
      // Keep figures apart when a blocked spot snaps onto a neighbour.
      for (int tries = 0; tries < 8; ++tries) {
        bool clash = false;
        for (const glm::vec2& t : taken) clash |= glm::distance(t, p) < 1.0f;
        if (!clash) break;
        want += toCenter * 1.0f;
        p = NearestFreeSpot(*scene, want);
      }
      taken.push_back(p);
      unit.position = glm::vec3(p.x, scene->ground.HeightAt(p.x, p.y), p.y);
      unit.facingYaw = yaw;
    }
  }
}

namespace {

// Sutherland-Hodgman clip of a convex polygon to the half-plane
// side * (dot(n, p) - d) >= 0.
std::vector<glm::vec2> ClipHalfPlane(const std::vector<glm::vec2>& polygon, glm::vec2 n, float d,
                                     float side) {
  std::vector<glm::vec2> out;
  for (size_t i = 0; i < polygon.size(); ++i) {
    const glm::vec2 a = polygon[i], b = polygon[(i + 1) % polygon.size()];
    const float sa = side * (glm::dot(n, a) - d), sb = side * (glm::dot(n, b) - d);
    if (sa >= 0.0f) out.push_back(a);
    if ((sa < 0.0f && sb > 0.0f) || (sa > 0.0f && sb < 0.0f)) {
      out.push_back(a + (b - a) * (sa / (sa - sb)));
    }
  }
  return out;
}

// A clipped piece worth keeping: at least three vertices and a real width.
// Clipping a polygon against a line that runs along its own edge (a ramp
// corridor's start edge lies on the step line that already bounds the
// tier) leaves a zero-width sliver of vertices on that line; an area test
// alone keeps it when it is long, and its prism would stand as a thin
// wall right across the road where the real tier is cut away.
bool UsablePiece(const std::vector<glm::vec2>& polygon) {
  if (polygon.size() < 3) return false;
  constexpr float kMinWidth = 0.05f;
  float width = std::numeric_limits<float>::infinity();
  for (size_t i = 0; i < polygon.size(); ++i) {
    const glm::vec2 a = polygon[i], e = polygon[(i + 1) % polygon.size()] - a;
    const float length = glm::length(e);
    if (length < 1e-6f) continue;
    float farthest = 0.0f;
    for (const glm::vec2& p : polygon) {
      farthest = std::max(farthest, std::fabs(e.x * (p.y - a.y) - e.y * (p.x - a.x)) / length);
    }
    width = std::min(width, farthest);
  }
  return width >= kMinWidth;
}

}  // namespace

std::vector<TerrainTier> TerrainTiers(const std::vector<TerrainWall>& walls,
                                      const std::vector<std::vector<glm::vec2>>& cutouts,
                                      const std::vector<glm::vec2>& square) {
  // Split by each step line, then cut by each ramp corridor (kept as the
  // convex pieces outside it).
  std::vector<std::vector<glm::vec2>> tiers = {square};
  for (const TerrainWall& wall : walls) {
    const float d = glm::dot(wall.raisedNormal, wall.lineOrigin);
    std::vector<std::vector<glm::vec2>> split;
    for (const auto& polygon : tiers) {
      for (const float side : {-1.0f, 1.0f}) {
        auto piece = ClipHalfPlane(polygon, wall.raisedNormal, d, side);
        if (UsablePiece(piece)) split.push_back(std::move(piece));
      }
    }
    tiers = std::move(split);
  }
  for (const auto& cutout : cutouts) {
    if (cutout.size() < 3) continue;
    float signedArea = 0.0f;
    for (size_t i = 0; i < cutout.size(); ++i) {
      const glm::vec2 a = cutout[i], b = cutout[(i + 1) % cutout.size()];
      signedArea += a.x * b.y - b.x * a.y;
    }
    std::vector<std::vector<glm::vec2>> outside;
    for (const auto& polygon : tiers) {
      std::vector<glm::vec2> inside = polygon;
      for (size_t i = 0; i < cutout.size() && inside.size() >= 3; ++i) {
        const glm::vec2 a = cutout[i], b = cutout[(i + 1) % cutout.size()];
        const glm::vec2 edge = b - a;
        glm::vec2 inward(-edge.y, edge.x);
        if (signedArea < 0.0f) inward = -inward;
        const float d = glm::dot(inward, a);
        auto piece = ClipHalfPlane(inside, inward, d, -1.0f);
        if (UsablePiece(piece)) outside.push_back(std::move(piece));
        inside = ClipHalfPlane(inside, inward, d, 1.0f);
      }
    }
    tiers = std::move(outside);
  }
  std::vector<TerrainTier> out;
  out.reserve(tiers.size());
  for (auto& polygon : tiers) {
    glm::vec2 center(0.0f);
    for (glm::vec2 p : polygon) center += p;
    center /= static_cast<float>(polygon.size());
    float height = 0.0f;
    for (const TerrainWall& wall : walls) {
      if (glm::dot(wall.raisedNormal, center - wall.lineOrigin) >= 0.0f) height += wall.rise;
    }
    out.push_back({std::move(polygon), height});
  }
  return out;
}

std::vector<Obstacle> SightObstacles(const Scene& scene) {
  std::vector<Obstacle> out = scene.obstacles;
  out.reserve(out.size() + scene.massing.size());
  for (const MassingDetail& d : scene.massing) {
    if (d.footprint.size() < 3 || d.y1 <= d.y0) continue;
    Obstacle prism;
    prism.footprint = d.footprint;
    prism.bounds.min.y = d.y0;
    prism.bounds.max.y = d.y1;
    constexpr float kInf = std::numeric_limits<float>::infinity();
    prism.bounds.min.x = prism.bounds.min.z = kInf;
    prism.bounds.max.x = prism.bounds.max.z = -kInf;
    for (const glm::vec2& p : d.footprint) {
      prism.bounds.min.x = std::min(prism.bounds.min.x, p.x);
      prism.bounds.max.x = std::max(prism.bounds.max.x, p.x);
      prism.bounds.min.z = std::min(prism.bounds.min.z, p.y);
      prism.bounds.max.z = std::max(prism.bounds.max.z, p.y);
    }
    prism.building = d.building;
    out.push_back(std::move(prism));
  }
  return out;
}

}  // namespace tactics
