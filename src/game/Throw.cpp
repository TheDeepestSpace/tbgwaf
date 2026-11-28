#include "game/Throw.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "game/Geometry.h"

namespace tactics {

float ThrowDistance(const Unit& unit, const glm::vec3& point) {
  return std::hypot(point.x - unit.position.x, point.z - unit.position.z);
}

bool CanThrowAt(const Unit& unit, const glm::vec3& point) {
  return unit.alive && unit.grenades > 0 && std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z) && ThrowDistance(unit, point) <= constants::kThrowRange;
}

namespace {

// Earliest surface the segment from -> to runs into: entry fraction along
// the segment, the impact point and the surface normal (oriented against the
// direction of travel).
struct SegmentHit {
  bool hit = false;
  float t = std::numeric_limits<float>::infinity();
  glm::vec3 point{0.0f};
  glm::vec3 normal{0.0f, 1.0f, 0.0f};
};

// Outward XZ normal of the footprint edge nearest to `p` (the wall face the
// grenade struck). Vertical hits (top/bottom of the prism) are resolved
// before calling this.
glm::vec3 NearestFaceNormal(const Obstacle& obstacle, const glm::vec3& p) {
  const std::vector<glm::vec2> fp = ObstacleFootprint(obstacle);
  float best = std::numeric_limits<float>::infinity();
  glm::vec2 normal(1.0f, 0.0f);
  const glm::vec2 q(p.x, p.z);
  for (size_t i = 0; i < fp.size(); ++i) {
    const glm::vec2 a = fp[i];
    const glm::vec2 b = fp[(i + 1) % fp.size()];
    const glm::vec2 e = b - a;
    const float len2 = glm::dot(e, e);
    if (len2 < 1e-10f) continue;
    const float s = glm::clamp(glm::dot(q - a, e) / len2, 0.0f, 1.0f);
    const float d = glm::distance(q, a + e * s);
    if (d < best) {
      best = d;
      normal = glm::normalize(glm::vec2(e.y, -e.x));  // Perpendicular; orientation fixed below.
    }
  }
  return glm::vec3(normal.x, 0.0f, normal.y);
}

void ConsiderObstacles(const glm::vec3& from, const glm::vec3& delta, float length,
                       const std::vector<Obstacle>& obstacles, SegmentHit* out) {
  const glm::vec3 dir = delta / length;
  for (const Obstacle& obstacle : obstacles) {
    if (obstacle.dead) continue;
    float t = 0.0f;
    if (!RayIntersectsObstacle(from, dir, obstacle, &t)) continue;
    const float frac = t / length;
    if (frac > 1.0f || frac >= out->t) continue;
    const glm::vec3 p = from + dir * t;
    glm::vec3 normal;
    constexpr float kFaceEps = 1e-3f;
    if (p.y >= obstacle.bounds.max.y - kFaceEps) {
      normal = glm::vec3(0.0f, 1.0f, 0.0f);
    } else if (p.y <= obstacle.bounds.min.y + kFaceEps) {
      normal = glm::vec3(0.0f, -1.0f, 0.0f);
    } else {
      normal = NearestFaceNormal(obstacle, p);
    }
    if (glm::dot(normal, dir) > 0.0f) normal = -normal;
    out->hit = true;
    out->t = frac;
    out->point = p;
    out->normal = normal;
  }
}

void ConsiderWalkSurfaces(const glm::vec3& from, const glm::vec3& delta, float length,
                          const std::vector<WalkSurface>& walkSurfaces, SegmentHit* out) {
  const glm::vec3 dir = delta / length;
  for (const WalkSurface& surface : walkSurfaces) {
    if (surface.dead || surface.vertices.size() < 3) continue;
    float t = 0.0f;
    if (!RayIntersectsWalkSurface(from, dir, surface, 0.2f, &t)) continue;
    const float frac = t / length;
    if (frac > 1.0f || frac >= out->t) continue;
    const glm::vec3 p0 = surface.vertices[0];
    glm::vec3 normal(0.0f);
    for (size_t i = 1; i + 1 < surface.vertices.size() && std::fabs(normal.y) < 1e-6f; ++i) {
      normal = glm::cross(surface.vertices[i] - p0, surface.vertices[i + 1] - p0);
    }
    if (glm::length(normal) < 1e-8f) continue;
    normal = glm::normalize(normal);
    if (normal.y < 0.0f) normal = -normal;
    if (glm::dot(normal, dir) > 0.0f) normal = -normal;
    out->hit = true;
    out->t = frac;
    out->point = from + dir * t;
    out->normal = normal;
  }
}

void ConsiderTerrain(const glm::vec3& from, const glm::vec3& delta, const HeightField& terrain,
                     SegmentHit* out) {
  // The step segments are short (1/30 s of flight), so an endpoint test plus
  // a few bisection steps pins the crossing closely enough. Works unchanged
  // on flat maps, where MeshHeightAt is 0 everywhere.
  const glm::vec3 to = from + delta;
  if (to.y - terrain.MeshHeightAt(to.x, to.z) >= constants::kGrenadeRadius) return;
  float lo = 0.0f, hi = 1.0f;  // from is assumed clear; to is embedded.
  for (int i = 0; i < 8; ++i) {
    const float mid = 0.5f * (lo + hi);
    const glm::vec3 p = from + delta * mid;
    if (p.y - terrain.MeshHeightAt(p.x, p.z) >= constants::kGrenadeRadius) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  if (lo >= out->t) return;
  const glm::vec3 p = from + delta * lo;
  constexpr float kProbe = 0.25f;
  const float hx0 = terrain.MeshHeightAt(p.x - kProbe, p.z);
  const float hx1 = terrain.MeshHeightAt(p.x + kProbe, p.z);
  const float hz0 = terrain.MeshHeightAt(p.x, p.z - kProbe);
  const float hz1 = terrain.MeshHeightAt(p.x, p.z + kProbe);
  glm::vec3 normal(-(hx1 - hx0) / (2.0f * kProbe), 1.0f, -(hz1 - hz0) / (2.0f * kProbe));
  out->hit = true;
  out->t = lo;
  out->point = glm::vec3(p.x, terrain.MeshHeightAt(p.x, p.z) + constants::kGrenadeRadius, p.z);
  out->normal = glm::normalize(normal);
}

}  // namespace

glm::vec3 ThrowPositionAt(const ThrowPreview& flight, float t) {
  if (flight.arc.empty()) return glm::vec3(0.0f);
  const float step = constants::kSimStepSeconds;
  const float f = glm::clamp(t / step, 0.0f, static_cast<float>(flight.arc.size() - 1));
  const size_t i = static_cast<size_t>(f);
  if (i + 1 >= flight.arc.size()) return flight.arc.back();
  return glm::mix(flight.arc[i], flight.arc[i + 1], f - static_cast<float>(i));
}

ThrowPreview SimulateThrow(const Unit& unit, const glm::vec3& point,
                           const std::vector<Obstacle>& obstacles,
                           const std::vector<WalkSurface>& walkSurfaces,
                           const HeightField& terrain) {
  ThrowPreview out;
  const glm::vec3 from = unit.position + glm::vec3(0.0f, constants::kThrowReleaseHeight, 0.0f);

  // Constant gravity and a flight time proportional to the 3D distance to the
  // target, so the grenade moves at the same pace whatever the target's
  // height. Solving y(T) = point.y under gravity g gives
  // vy0 = (point.y - from.y)/T + g T / 2. Unobstructed, the grenade passes
  // exactly through `point` at t = T.
  const float horizontal = ThrowDistance(unit, point);
  const float dy = point.y - from.y;
  const float T = std::max(constants::kThrowMinFlightSeconds,
                           std::hypot(horizontal, dy) / constants::kThrowFlightSpeed);
  const float gravity = constants::kThrowGravity;
  glm::vec3 velocity = (point - from) / T;
  velocity.y = dy / T + 0.5f * gravity * T;

  const float dt = constants::kSimStepSeconds;
  const int steps = static_cast<int>(std::lround(constants::kGrenadeFuseSeconds / dt));
  constexpr float kSkin = 0.02f;
  constexpr size_t kMaxBounceMarks = 8;
  glm::vec3 pos = from;
  bool resting = false;
  out.arc.reserve(static_cast<size_t>(steps) + 1);
  out.arc.push_back(pos);
  for (int i = 1; i <= steps; ++i) {
    if (!resting) {
      velocity.y -= gravity * dt;
      const glm::vec3 delta = velocity * dt;
      const float length = glm::length(delta);
      SegmentHit hit;
      if (length > 1e-6f) {
        ConsiderObstacles(pos, delta, length, obstacles, &hit);
        ConsiderWalkSurfaces(pos, delta, length, walkSurfaces, &hit);
        ConsiderTerrain(pos, delta, terrain, &hit);
      }
      if (hit.hit) {
        // Rebound: the normal component restituted, the tangential one
        // damped. The step ends at the impact (the sliver of travel left in
        // this 1/30 s is negligible), keeping samples time-uniform.
        pos = hit.point + hit.normal * kSkin;
        const float vn = glm::dot(velocity, hit.normal);
        const glm::vec3 tangent = velocity - hit.normal * vn;
        velocity = tangent * constants::kGrenadeFriction -
                   hit.normal * vn * constants::kGrenadeRestitution;
        if (out.bounces.size() < kMaxBounceMarks) out.bounces.push_back(hit.point);
        if (hit.normal.y > 0.7f && glm::length(velocity) < constants::kGrenadeRestSpeed) {
          velocity = glm::vec3(0.0f);
          resting = true;
        }
      } else {
        pos += delta;
      }
    }
    out.arc.push_back(pos);
  }
  out.landing = out.arc.back();
  return out;
}

}  // namespace tactics
