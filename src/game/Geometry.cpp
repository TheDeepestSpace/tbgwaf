#include "game/Geometry.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tactics {
namespace {

constexpr float kEps = 1e-6f;

float Cross(glm::vec2 a, glm::vec2 b) { return a.x * b.y - a.y * b.x; }

std::vector<glm::vec2> Ccw(std::vector<glm::vec2> polygon) {
  if (PolygonSignedArea(polygon) < 0.0f) std::reverse(polygon.begin(), polygon.end());
  return polygon;
}

bool LineIntersection(glm::vec2 p, glm::vec2 r, glm::vec2 q, glm::vec2 s,
                      glm::vec2* out) {
  const float denominator = Cross(r, s);
  if (std::fabs(denominator) < kEps) return false;
  *out = p + r * (Cross(q - p, s) / denominator);
  return true;
}

// Clips p(t)=origin+t*direction to a CCW convex polygon. Bounds may start
// finite (segments / 3D Y clip) or infinite (rays).
bool ClipParametric(glm::vec2 origin, glm::vec2 direction,
                    const std::vector<glm::vec2>& polygon, float* enter, float* exit) {
  const std::vector<glm::vec2> ccw = Ccw(polygon);
  for (size_t i = 0; i < ccw.size(); ++i) {
    const glm::vec2 a = ccw[i];
    const glm::vec2 edge = ccw[(i + 1) % ccw.size()] - a;
    const float value = Cross(edge, origin - a);
    const float rate = Cross(edge, direction);
    if (std::fabs(rate) < kEps) {
      if (value < 0.0f) return false;
      continue;
    }
    const float t = -value / rate;
    if (rate > 0.0f) *enter = std::max(*enter, t);
    else *exit = std::min(*exit, t);
    if (*enter > *exit) return false;
  }
  return true;
}

}  // namespace

float PolygonSignedArea(const std::vector<glm::vec2>& polygon) {
  float twice = 0.0f;
  for (size_t i = 0; i < polygon.size(); ++i) {
    twice += Cross(polygon[i], polygon[(i + 1) % polygon.size()]);
  }
  return 0.5f * twice;
}

float PolygonArea(const std::vector<glm::vec2>& polygon) {
  return std::fabs(PolygonSignedArea(polygon));
}

bool PointInConvexPolygon(const glm::vec2& point, const std::vector<glm::vec2>& polygon,
                          float epsilon) {
  if (polygon.size() < 3) return false;
  const std::vector<glm::vec2> ccw = Ccw(polygon);
  for (size_t i = 0; i < ccw.size(); ++i) {
    if (Cross(ccw[(i + 1) % ccw.size()] - ccw[i], point - ccw[i]) < -epsilon) return false;
  }
  return true;
}

bool PointStrictlyInConvexPolygon(const glm::vec2& point,
                                  const std::vector<glm::vec2>& polygon, float epsilon) {
  if (polygon.size() < 3) return false;
  const std::vector<glm::vec2> ccw = Ccw(polygon);
  for (size_t i = 0; i < ccw.size(); ++i) {
    if (Cross(ccw[(i + 1) % ccw.size()] - ccw[i], point - ccw[i]) <= epsilon) return false;
  }
  return true;
}

std::vector<glm::vec2> ExpandConvexPolygon(const std::vector<glm::vec2>& input,
                                           float amount) {
  std::vector<glm::vec2> polygon = Ccw(input);
  if (polygon.size() < 3 || PolygonArea(polygon) < kEps) return {};
  if (std::fabs(amount) < kEps) return polygon;
  std::vector<glm::vec2> result;
  result.reserve(polygon.size());
  for (size_t i = 0; i < polygon.size(); ++i) {
    const size_t prev = (i + polygon.size() - 1) % polygon.size();
    const glm::vec2 e0 = glm::normalize(polygon[i] - polygon[prev]);
    const glm::vec2 e1 = glm::normalize(polygon[(i + 1) % polygon.size()] - polygon[i]);
    const glm::vec2 outward0(e0.y, -e0.x);
    const glm::vec2 outward1(e1.y, -e1.x);
    glm::vec2 intersection;
    if (!LineIntersection(polygon[prev] + outward0 * amount, e0,
                          polygon[i] + outward1 * amount, e1, &intersection)) {
      intersection = polygon[i] + glm::normalize(outward0 + outward1) * amount;
    }
    result.push_back(intersection);
  }
  return PolygonArea(result) < kEps ? std::vector<glm::vec2>{} : result;
}

std::vector<glm::vec2> InsetConvexPolygon(const std::vector<glm::vec2>& polygon,
                                          float amount) {
  const std::vector<glm::vec2> inset = ExpandConvexPolygon(polygon, -amount);
  if (inset.size() < 3) return {};
  // An over-large inset flips or escapes the original polygon.
  for (const glm::vec2& p : inset) {
    if (!PointInConvexPolygon(p, polygon, 1e-3f)) return {};
  }
  return inset;
}

bool SegmentEntersConvexPolygon(glm::vec2 from, glm::vec2 to,
                                const std::vector<glm::vec2>& polygon) {
  float enter = 0.0f, exit = 1.0f;
  if (!ClipParametric(from, to - from, polygon, &enter, &exit)) return false;
  if (exit - enter <= 1e-5f) return false;
  const glm::vec2 mid = from + (to - from) * ((enter + exit) * 0.5f);
  return PointStrictlyInConvexPolygon(mid, polygon, 1e-5f);
}

bool RayIntersectsObstacle(const glm::vec3& origin, const glm::vec3& direction,
                           const Obstacle& obstacle, float* outT) {
  float enter = 0.0f;
  float exit = std::numeric_limits<float>::infinity();
  if (std::fabs(direction.y) < kEps) {
    if (origin.y < obstacle.bounds.min.y || origin.y > obstacle.bounds.max.y) return false;
  } else {
    float a = (obstacle.bounds.min.y - origin.y) / direction.y;
    float b = (obstacle.bounds.max.y - origin.y) / direction.y;
    if (a > b) std::swap(a, b);
    enter = std::max(enter, a);
    exit = std::min(exit, b);
  }
  if (enter > exit) return false;
  if (!ClipParametric({origin.x, origin.z}, {direction.x, direction.z},
                      ObstacleFootprint(obstacle), &enter, &exit)) {
    return false;
  }
  if (exit < 0.0f) return false;
  if (outT) *outT = std::max(0.0f, enter);
  return true;
}

bool RayIntersectsWalkSurface(const glm::vec3& origin, const glm::vec3& direction,
                              const WalkSurface& surface, float thickness, float* outT) {
  if (surface.vertices.size() < 3) return false;
  std::vector<glm::vec2> footprint;
  for (const glm::vec3& v : surface.vertices) footprint.emplace_back(v.x, v.z);
  float enter = 0.0f, exit = std::numeric_limits<float>::infinity();
  if (!ClipParametric({origin.x, origin.z}, {direction.x, direction.z}, footprint, &enter, &exit)) {
    return false;
  }
  const glm::vec3 p0 = surface.vertices[0];
  glm::vec3 normal(0.0f);
  for (size_t i = 1; i + 1 < surface.vertices.size() && std::fabs(normal.y) < kEps; ++i) {
    normal = glm::cross(surface.vertices[i] - p0, surface.vertices[i + 1] - p0);
  }
  if (std::fabs(normal.y) < kEps) return false;
  // topY(x,z) = c + gx*x + gz*z.
  const float gx = -normal.x / normal.y;
  const float gz = -normal.z / normal.y;
  const float c = p0.y - gx * p0.x - gz * p0.z;
  auto clipPositive = [&](float value, float rate) {
    if (std::fabs(rate) < kEps) return value >= 0.0f;
    const float t = -value / rate;
    if (rate > 0.0f) enter = std::max(enter, t);
    else exit = std::min(exit, t);
    return enter <= exit;
  };
  // Below top, above underside.
  const float topValue = c + gx * origin.x + gz * origin.z - origin.y;
  const float topRate = gx * direction.x + gz * direction.z - direction.y;
  if (!clipPositive(topValue, topRate)) return false;
  const float bottomValue = origin.y - (c + gx * origin.x + gz * origin.z - thickness);
  const float bottomRate = direction.y - gx * direction.x - gz * direction.z;
  if (!clipPositive(bottomValue, bottomRate) || exit < 0.0f) return false;
  if (outT) *outT = std::max(0.0f, enter);
  return true;
}

bool SurfaceContainsXZ(const WalkSurface& surface, float x, float z) {
  std::vector<glm::vec2> footprint;
  footprint.reserve(surface.vertices.size());
  for (const glm::vec3& v : surface.vertices) footprint.emplace_back(v.x, v.z);
  return PointInConvexPolygon({x, z}, footprint);
}

float SurfaceHeightAt(const WalkSurface& surface, float x, float z) {
  if (surface.vertices.empty()) return 0.0f;
  const glm::vec3 origin = surface.vertices[0];
  for (size_t i = 1; i + 1 < surface.vertices.size(); ++i) {
    const glm::vec3 normal = glm::cross(surface.vertices[i] - origin,
                                        surface.vertices[i + 1] - origin);
    if (std::fabs(normal.y) < kEps) continue;
    return origin.y - (normal.x * (x - origin.x) + normal.z * (z - origin.z)) / normal.y;
  }
  return origin.y;
}

glm::vec3 SurfaceCenter(const WalkSurface& surface) {
  glm::vec3 center(0.0f);
  if (surface.vertices.empty()) return center;
  for (const glm::vec3& v : surface.vertices) center += v;
  return center / static_cast<float>(surface.vertices.size());
}

}  // namespace tactics
