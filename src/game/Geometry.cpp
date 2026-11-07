#include "game/Geometry.h"

#include <algorithm>
#include <array>
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

// Vertex `i` of `polygon` read in CCW order without copying it: these are hot
// inner loops (navmesh visibility tests), so no per-call allocation.
glm::vec2 CcwAt(const std::vector<glm::vec2>& polygon, bool reversed, size_t i) {
  return reversed ? polygon[polygon.size() - 1 - i] : polygon[i];
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
  const bool reversed = PolygonSignedArea(polygon) < 0.0f;
  const size_t n = polygon.size();
  for (size_t i = 0; i < n; ++i) {
    const glm::vec2 a = CcwAt(polygon, reversed, i);
    const glm::vec2 edge = CcwAt(polygon, reversed, (i + 1) % n) - a;
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
  const bool reversed = PolygonSignedArea(polygon) < 0.0f;
  const size_t n = polygon.size();
  for (size_t i = 0; i < n; ++i) {
    const glm::vec2 a = CcwAt(polygon, reversed, i);
    if (Cross(CcwAt(polygon, reversed, (i + 1) % n) - a, point - a) < -epsilon) return false;
  }
  return true;
}

bool PointStrictlyInConvexPolygon(const glm::vec2& point,
                                  const std::vector<glm::vec2>& polygon, float epsilon) {
  if (polygon.size() < 3) return false;
  const bool reversed = PolygonSignedArea(polygon) < 0.0f;
  const size_t n = polygon.size();
  for (size_t i = 0; i < n; ++i) {
    const glm::vec2 a = CcwAt(polygon, reversed, i);
    if (Cross(CcwAt(polygon, reversed, (i + 1) % n) - a, point - a) <= epsilon) return false;
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

ConvexDecomposition DecomposeToConvex(const std::vector<glm::vec2>& input) {
  ConvexDecomposition result;
  // Clean input: counter-clockwise, no repeated or collinear vertices (a
  // vertex in the middle of a straight edge would be a degenerate ear).
  std::vector<glm::vec2> polygon = Ccw(input);
  for (bool changed = true; changed && polygon.size() >= 3;) {
    changed = false;
    for (size_t i = 0; i < polygon.size() && polygon.size() >= 3; ++i) {
      const glm::vec2 prev = polygon[(i + polygon.size() - 1) % polygon.size()];
      const glm::vec2 next = polygon[(i + 1) % polygon.size()];
      const glm::vec2 e0 = polygon[i] - prev, e1 = next - polygon[i];
      if (glm::length(e1) < 1e-5f ||
          (std::fabs(Cross(e0, e1)) < 1e-6f && glm::dot(e0, e1) > 0.0f)) {
        polygon.erase(polygon.begin() + static_cast<long>(i));
        changed = true;
        break;
      }
    }
  }
  const size_t n = polygon.size();
  if (n < 3 || PolygonArea(polygon) < kEps) return result;

  // Ear clipping triangulation over vertex indices. O(n^3) worst case; floor
  // plans have a handful of vertices.
  std::vector<int> ring(n);
  for (size_t i = 0; i < n; ++i) ring[i] = static_cast<int>(i);
  std::vector<std::array<int, 3>> triangles;
  const auto inTriangle = [&](glm::vec2 p, glm::vec2 a, glm::vec2 b, glm::vec2 c) {
    return Cross(b - a, p - a) >= -1e-7f && Cross(c - b, p - b) >= -1e-7f &&
           Cross(a - c, p - c) >= -1e-7f;
  };
  while (ring.size() > 3) {
    bool clipped = false;
    for (size_t i = 0; i < ring.size(); ++i) {
      const int ia = ring[(i + ring.size() - 1) % ring.size()], ib = ring[i],
                ic = ring[(i + 1) % ring.size()];
      const glm::vec2 a = polygon[ia], b = polygon[ib], c = polygon[ic];
      if (Cross(b - a, c - b) <= 1e-7f) continue;  // Reflex (or flat) corner: not an ear.
      bool empty = true;
      for (int other : ring) {
        if (other == ia || other == ib || other == ic) continue;
        if (inTriangle(polygon[other], a, b, c)) {
          empty = false;
          break;
        }
      }
      if (!empty) continue;
      triangles.push_back({ia, ib, ic});
      ring.erase(ring.begin() + static_cast<long>(i));
      clipped = true;
      break;
    }
    if (!clipped) {
      // Numerically stuck (near-degenerate input): take the least reflex
      // corner so the loop always terminates with a full cover.
      size_t best = 0;
      float bestTurn = -std::numeric_limits<float>::infinity();
      for (size_t i = 0; i < ring.size(); ++i) {
        const glm::vec2 a = polygon[ring[(i + ring.size() - 1) % ring.size()]];
        const glm::vec2 b = polygon[ring[i]], c = polygon[ring[(i + 1) % ring.size()]];
        const float turn = Cross(b - a, c - b);
        if (turn > bestTurn) {
          bestTurn = turn;
          best = i;
        }
      }
      triangles.push_back({ring[(best + ring.size() - 1) % ring.size()], ring[best],
                           ring[(best + 1) % ring.size()]});
      ring.erase(ring.begin() + static_cast<long>(best));
    }
  }
  triangles.push_back({ring[0], ring[1], ring[2]});

  // Hertel-Mehlhorn: merge pieces across a shared diagonal while the union
  // stays convex. Pieces are index rings in counter-clockwise order.
  std::vector<std::vector<int>> pieces;
  for (const auto& t : triangles) pieces.push_back({t[0], t[1], t[2]});
  const auto convex = [&](const std::vector<int>& ring) {
    for (size_t i = 0; i < ring.size(); ++i) {
      const glm::vec2 a = polygon[ring[(i + ring.size() - 1) % ring.size()]];
      const glm::vec2 b = polygon[ring[i]], c = polygon[ring[(i + 1) % ring.size()]];
      if (Cross(b - a, c - b) < -1e-6f) return false;
    }
    return true;
  };
  // Merge ring `q` into `p` across the directed edge p[i] -> p[i+1], which q
  // holds reversed as q[j] -> q[j+1]: p's vertices up to p[i], then q's from
  // q[j+1] around to q[j] (exclusive of the shared pair), then the rest of p.
  const auto merged = [&](const std::vector<int>& p, size_t i, const std::vector<int>& q,
                          size_t j) {
    std::vector<int> out;
    for (size_t k = 0; k <= i; ++k) out.push_back(p[k]);
    for (size_t k = 2; k < q.size(); ++k) out.push_back(q[(j + k) % q.size()]);
    for (size_t k = i + 1; k < p.size(); ++k) out.push_back(p[k]);
    return out;
  };
  for (bool mergedAny = true; mergedAny;) {
    mergedAny = false;
    for (size_t a = 0; a < pieces.size() && !mergedAny; ++a) {
      for (size_t b = a + 1; b < pieces.size() && !mergedAny; ++b) {
        const std::vector<int>& p = pieces[a];
        const std::vector<int>& q = pieces[b];
        for (size_t i = 0; i < p.size() && !mergedAny; ++i) {
          const int u = p[i], v = p[(i + 1) % p.size()];
          for (size_t j = 0; j < q.size(); ++j) {
            if (q[j] != v || q[(j + 1) % q.size()] != u) continue;
            std::vector<int> candidate = merged(p, i, q, j);
            if (!convex(candidate)) break;
            pieces[a] = std::move(candidate);
            pieces.erase(pieces.begin() + static_cast<long>(b));
            mergedAny = true;
            break;
          }
        }
      }
    }
  }

  for (const std::vector<int>& ring : pieces) {
    std::vector<glm::vec2> piece;
    for (int index : ring) piece.push_back(polygon[index]);
    result.pieces.push_back(std::move(piece));
  }
  for (size_t a = 0; a < pieces.size(); ++a) {
    for (size_t b = a + 1; b < pieces.size(); ++b) {
      bool shared = false;
      for (size_t i = 0; i < pieces[a].size() && !shared; ++i) {
        const int u = pieces[a][i], v = pieces[a][(i + 1) % pieces[a].size()];
        for (size_t j = 0; j < pieces[b].size(); ++j) {
          if (pieces[b][j] == v && pieces[b][(j + 1) % pieces[b].size()] == u) {
            shared = true;
            break;
          }
        }
      }
      if (shared) result.links.emplace_back(static_cast<int>(a), static_cast<int>(b));
    }
  }
  return result;
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
