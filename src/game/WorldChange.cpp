#include "game/WorldChange.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <istream>
#include <ostream>

#include "game/Geometry.h"

namespace tactics {

namespace {

constexpr size_t kMaxPieces = 64;
constexpr size_t kMaxPolygon = 64;
constexpr float kParkedY = -1000.0f;
constexpr float kMinPieceSide = 0.02f;

bool ValidObstacle(const Scene& scene, int index) {
  return index >= 0 && index < static_cast<int>(scene.obstacles.size());
}

bool SameObstacle(const Obstacle& a, const Obstacle& b) {
  return a.bounds.min == b.bounds.min && a.bounds.max == b.bounds.max &&
         a.footprint == b.footprint && a.kind == b.kind && a.building == b.building &&
         a.navOnly == b.navOnly && a.fixtureModel == b.fixtureModel &&
         a.fixtureYaw == b.fixtureYaw && a.fixtureBaseY == b.fixtureBaseY &&
         a.fixtureRoll == b.fixtureRoll && a.dead == b.dead && a.profile == b.profile;
}

void RefreshBuildingLiveness(Scene* scene, int building) {
  if (building < 0 || building >= static_cast<int>(scene->buildings.size())) return;
  Building& b = scene->buildings[building];
  if (b.dead) return;
  for (const Obstacle& o : scene->obstacles) {
    if (!o.dead && o.building == building) return;  // Something of it still stands.
  }
  b.dead = true;
  for (const int s : b.floorSurfaces) KillWalkSurface(scene, s);
  for (const int s : b.stairSurfaces) KillWalkSurface(scene, s);
  for (MassingDetail& d : scene->massing) {
    if (d.building != building) continue;
    d.dead = true;
    d.footprint.clear();
    d.top.clear();
  }
}

// Clips a planar 3D polygon (XZ-convex) against the half-plane
// eval(x, z) <= 0, interpolating full vertices along crossing edges.
std::vector<glm::vec3> ClipPolygon3(const std::vector<glm::vec3>& polygon,
                                    const std::function<float(float, float)>& eval) {
  std::vector<glm::vec3> out;
  const size_t n = polygon.size();
  for (size_t i = 0; i < n; ++i) {
    const glm::vec3& p = polygon[i];
    const glm::vec3& q = polygon[(i + 1) % n];
    const float dp = eval(p.x, p.z), dq = eval(q.x, q.z);
    if (dp <= 0.0f) out.push_back(p);
    if ((dp < 0.0f) != (dq < 0.0f) && std::fabs(dq - dp) > 1e-9f) {
      const float t = dp / (dp - dq);
      out.push_back(p + (q - p) * t);
    }
  }
  return out;
}

float PolygonAreaXZ(const std::vector<glm::vec3>& polygon) {
  float doubled = 0.0f;
  const size_t n = polygon.size();
  for (size_t i = 0; i < n; ++i) {
    const glm::vec3& p = polygon[i];
    const glm::vec3& q = polygon[(i + 1) % n];
    doubled += p.x * q.z - q.x * p.z;
  }
  return 0.5f * std::fabs(doubled);
}

// True when two walkable patches touch along a boundary segment of positive
// length (colinear overlapping edges in XZ whose heights agree there).
bool SurfacesShareBoundary(const std::vector<glm::vec3>& a, const std::vector<glm::vec3>& b) {
  constexpr float kLineSlack = 0.03f;
  constexpr float kHeightSlack = 0.35f;
  constexpr float kMinOverlap = 0.05f;
  for (size_t i = 0; i < a.size(); ++i) {
    const glm::vec3 p = a[i], q = a[(i + 1) % a.size()];
    const glm::vec2 pq(q.x - p.x, q.z - p.z);
    const float len = glm::length(pq);
    if (len < 1e-5f) continue;
    const glm::vec2 dir = pq / len;
    const glm::vec2 normal(-dir.y, dir.x);
    for (size_t j = 0; j < b.size(); ++j) {
      const glm::vec3 r = b[j], s = b[(j + 1) % b.size()];
      const glm::vec2 rs(s.x - r.x, s.z - r.z);
      const float lenB = glm::length(rs);
      if (lenB < 1e-5f) continue;
      // Colinear: both endpoints of the other edge on this edge's line.
      if (std::fabs(glm::dot(glm::vec2(r.x - p.x, r.z - p.z), normal)) > kLineSlack ||
          std::fabs(glm::dot(glm::vec2(s.x - p.x, s.z - p.z), normal)) > kLineSlack) {
        continue;
      }
      const float t0 = glm::dot(glm::vec2(r.x - p.x, r.z - p.z), dir);
      const float t1 = glm::dot(glm::vec2(s.x - p.x, s.z - p.z), dir);
      const float lo = std::max(0.0f, std::min(t0, t1));
      const float hi = std::min(len, std::max(t0, t1));
      if (hi - lo < kMinOverlap) continue;
      // Heights agree at the overlap midpoint.
      const float mid = 0.5f * (lo + hi);
      const glm::vec3 onA = p + (q - p) * (mid / len);
      const float tB = std::clamp((mid - std::min(t0, t1)) / std::max(1e-5f, std::fabs(t1 - t0)),
                                  0.0f, 1.0f);
      const glm::vec3 onB = t0 <= t1 ? r + (s - r) * tB : s + (r - s) * tB;
      if (std::fabs(onA.y - onB.y) <= kHeightSlack) return true;
    }
  }
  return false;
}

bool ApplyFloorHole(Scene* scene, const WorldChange& c) {
  if (c.target < 0 || c.target >= static_cast<int>(scene->walkSurfaces.size())) return false;
  const WalkSurface base = scene->walkSurfaces[c.target];  // Copy: the cut kills it.
  if (base.dead || base.vertices.size() < 3) return false;
  const float x0 = std::min(c.rect.x, c.rect.z), x1 = std::max(c.rect.x, c.rect.z);
  const float z0 = std::min(c.rect.y, c.rect.w), z1 = std::max(c.rect.y, c.rect.w);
  const bool cells = !c.pieces.empty();
  if (cells && c.pieces.size() > kMaxPieces) return false;
  if (!cells && (x1 - x0 < kMinPieceSide || z1 - z0 < kMinPieceSide)) return false;
  // Floors only: the piece clipping below keeps each vertex's own height, but
  // a steep ramp/stair cut this way would leave inconsistent seams.
  float yMin = base.vertices.front().y, yMax = yMin;
  for (const glm::vec3& v : base.vertices) {
    yMin = std::min(yMin, v.y);
    yMax = std::max(yMax, v.y);
  }
  if (yMax - yMin > 0.5f) return false;
  // Disjoint convex partition of the patch minus the hole: everything left
  // of the hole, right of it, and the strips before/behind it in between.
  const auto& v = base.vertices;
  std::vector<std::vector<glm::vec3>> pieces;
  if (cells) {
    // Fracture cells left standing: the patch clipped to each convex
    // footprint, so every piece keeps the patch's own heights.
    for (const Obstacle& cell : c.pieces) {
      const std::vector<glm::vec2>& f = cell.footprint;
      if (f.size() < 3) return false;
      const float winding = PolygonSignedArea(f) < 0.0f ? -1.0f : 1.0f;
      std::vector<glm::vec3> piece = v;
      for (size_t i = 0; i < f.size() && piece.size() >= 3; ++i) {
        const glm::vec2 p = f[i], q = f[(i + 1) % f.size()];
        piece = ClipPolygon3(piece, [&](float x, float z) {
          return -winding * ((q.x - p.x) * (z - p.y) - (q.y - p.y) * (x - p.x));
        });
      }
      pieces.push_back(std::move(piece));
    }
  } else {
    pieces.push_back(ClipPolygon3(v, [&](float x, float) { return x - x0; }));
    pieces.push_back(ClipPolygon3(v, [&](float x, float) { return x1 - x; }));
    std::vector<glm::vec3> mid = ClipPolygon3(v, [&](float x, float) { return x0 - x; });
    mid = ClipPolygon3(mid, [&](float x, float) { return x - x1; });
    pieces.push_back(ClipPolygon3(mid, [&](float, float z) { return z - z0; }));
    pieces.push_back(ClipPolygon3(mid, [&](float, float z) { return z1 - z; }));
  }
  float keptArea = 0.0f;
  std::vector<std::vector<glm::vec3>> live;
  for (std::vector<glm::vec3>& piece : pieces) {
    const float area = piece.size() >= 3 ? PolygonAreaXZ(piece) : 0.0f;
    if (area < 1e-3f) continue;
    keptArea += area;
    live.push_back(std::move(piece));
  }
  // The hole must actually remove some of the patch.
  if (keptArea > PolygonAreaXZ(v) - 1e-2f) return false;
  const std::vector<int> oldNeighbors = base.neighbors;
  KillWalkSurface(scene, c.target);
  const int first = static_cast<int>(scene->walkSurfaces.size());
  for (std::vector<glm::vec3>& piece : live) {
    WalkSurface s = base;
    s.dead = false;
    s.vertices = std::move(piece);
    s.neighbors.clear();
    s.groundConnections.clear();
    scene->walkSurfaces.push_back(std::move(s));
  }
  const int count = static_cast<int>(scene->walkSurfaces.size()) - first;
  const auto link = [scene](int i, int j) {
    auto& ni = scene->walkSurfaces[i].neighbors;
    if (std::find(ni.begin(), ni.end(), j) == ni.end()) ni.push_back(j);
    auto& nj = scene->walkSurfaces[j].neighbors;
    if (std::find(nj.begin(), nj.end(), i) == nj.end()) nj.push_back(i);
  };
  // New pieces link to each other across the cut lines they share.
  for (int i = 0; i < count; ++i) {
    for (int j = i + 1; j < count; ++j) {
      if (SurfacesShareBoundary(scene->walkSurfaces[first + i].vertices,
                                scene->walkSurfaces[first + j].vertices)) {
        link(first + i, first + j);
      }
    }
  }
  // Old neighbours relink to the piece(s) still touching them; if numerics
  // leave one orphaned, it keeps the connection via the nearest piece so a
  // stair/floor link never silently breaks.
  for (const int n : oldNeighbors) {
    if (n < 0 || n >= static_cast<int>(scene->walkSurfaces.size())) continue;
    const WalkSurface& other = scene->walkSurfaces[n];
    if (other.dead || other.vertices.empty()) continue;
    bool linked = false;
    for (int i = 0; i < count; ++i) {
      if (SurfacesShareBoundary(scene->walkSurfaces[first + i].vertices, other.vertices)) {
        link(first + i, n);
        linked = true;
      }
    }
    if (!linked && count > 0) {
      glm::vec3 center(0.0f);
      for (const glm::vec3& p : other.vertices) center += p;
      center /= static_cast<float>(other.vertices.size());
      int best = first;
      float bestDist = 1e30f;
      for (int i = 0; i < count; ++i) {
        glm::vec3 c2(0.0f);
        for (const glm::vec3& p : scene->walkSurfaces[first + i].vertices) c2 += p;
        c2 /= static_cast<float>(scene->walkSurfaces[first + i].vertices.size());
        const float dist = glm::distance(glm::vec2(center.x, center.z), glm::vec2(c2.x, c2.z));
        if (dist < bestDist) {
          bestDist = dist;
          best = first + i;
        }
      }
      link(best, n);
    }
  }
  // Explicit ground transitions follow the piece they lie on.
  for (const glm::vec3& p : base.groundConnections) {
    for (int i = 0; i < count; ++i) {
      std::vector<glm::vec2> xz;
      for (const glm::vec3& q : scene->walkSurfaces[first + i].vertices) xz.emplace_back(q.x, q.z);
      if (PointInConvexPolygon({p.x, p.z}, xz)) {
        scene->walkSurfaces[first + i].groundConnections.push_back(p);
        break;
      }
    }
  }
  return true;
}

// --- Wall-plane fracture helpers (angled WallHole outlines) ------------------

// Sutherland-Hodgman step: keeps the part of `poly` where eval(p) <= 0.
// Convex results stay convex; a u-monotone subject clipped by an axis-aligned
// half-plane stays u-monotone.
std::vector<glm::vec2> ClipPoly2(const std::vector<glm::vec2>& poly,
                                 const std::function<float(const glm::vec2&)>& eval) {
  std::vector<glm::vec2> out;
  const size_t n = poly.size();
  for (size_t i = 0; i < n; ++i) {
    const glm::vec2 p = poly[i], q = poly[(i + 1) % n];
    const float dp = eval(p), dq = eval(q);
    if (dp <= 0.0f) out.push_back(p);
    if ((dp < 0.0f) != (dq < 0.0f) && std::fabs(dq - dp) > 1e-9f) {
      out.push_back(p + (q - p) * (dp / (dp - dq)));
    }
  }
  return out;
}

// Splits a u-monotone outline polygon into its lower and upper chains, both
// running left to right. False when the polygon is degenerate or either
// chain backtracks in u (not one y run per u).
bool SplitChains(const std::vector<glm::vec2>& outline, std::vector<glm::vec2>* lower,
                 std::vector<glm::vec2>* upper) {
  const size_t n = outline.size();
  if (n < 3) return false;
  size_t iMin = 0, iMax = 0;
  for (size_t i = 1; i < n; ++i) {
    if (outline[i].x < outline[iMin].x) iMin = i;
    if (outline[i].x > outline[iMax].x) iMax = i;
  }
  if (outline[iMax].x - outline[iMin].x < 1e-4f) return false;
  std::vector<glm::vec2> a, b;  // iMin -> iMax forward, and backward.
  for (size_t i = iMin;; i = (i + 1) % n) {
    a.push_back(outline[i]);
    if (i == iMax) break;
  }
  for (size_t i = iMin;; i = (i + n - 1) % n) {
    b.push_back(outline[i]);
    if (i == iMax) break;
  }
  const auto monotone = [](const std::vector<glm::vec2>& chain) {
    for (size_t i = 0; i + 1 < chain.size(); ++i) {
      if (chain[i + 1].x < chain[i].x - 1e-4f) return false;
    }
    return true;
  };
  if (!monotone(a) || !monotone(b)) return false;
  const auto meanY = [](const std::vector<glm::vec2>& chain) {
    float sum = 0.0f;
    for (const glm::vec2& p : chain) sum += p.y;
    return sum / static_cast<float>(chain.size());
  };
  if (meanY(a) <= meanY(b)) {
    *lower = std::move(a);
    *upper = std::move(b);
  } else {
    *lower = std::move(b);
    *upper = std::move(a);
  }
  return true;
}

// True when `poly` is (within eps) exactly the axis-aligned rectangle of its
// own bounding box -- such a piece is a plain prism and needs no profile.
bool IsBoundingRect(const std::vector<glm::vec2>& poly) {
  if (poly.size() != 4) return false;
  glm::vec2 lo = poly[0], hi = poly[0];
  for (const glm::vec2& p : poly) {
    lo = glm::min(lo, p);
    hi = glm::max(hi, p);
  }
  constexpr float kEps = 1e-3f;
  for (const glm::vec2& p : poly) {
    if (std::fabs(p.x - lo.x) > kEps && std::fabs(p.x - hi.x) > kEps) return false;
    if (std::fabs(p.y - lo.y) > kEps && std::fabs(p.y - hi.y) > kEps) return false;
  }
  return true;
}

bool ApplyWallHole(Scene* scene, const WorldChange& c) {
  if (!ValidObstacle(*scene, c.target) || scene->obstacles[c.target].dead) return false;
  if (c.wallEdge < 0 || c.wallEdge >= 4) return false;
  const Obstacle wall = scene->obstacles[c.target];  // Copy: the cut kills it.
  const std::vector<glm::vec2> f = ObstacleFootprint(wall);
  if (f.size() != 4) return false;
  const size_t e = static_cast<size_t>(c.wallEdge);
  const glm::vec2 a = f[e], b = f[(e + 1) % 4];
  const float len = glm::distance(a, b);
  if (len < kMinPieceSide) return false;
  const glm::vec2 d = (b - a) / len;
  const glm::vec2 tv = f[(e + 3) % 4] - f[e];  // Through the wall's thickness.
  const float ylo = wall.bounds.min.y, yhi = wall.bounds.max.y;

  // The solid silhouette of this piece in the edge's (u, y) frame: its
  // fracture profile (stored in the frame of footprint edge 0) or the full
  // rect. A profiled piece can only be re-cut along its own wall plane.
  std::vector<glm::vec2> solid;
  if (wall.profile.size() >= 3) {
    if (e == 0) {
      solid = wall.profile;
    } else if (e == 2) {
      for (auto it = wall.profile.rbegin(); it != wall.profile.rend(); ++it) {
        solid.emplace_back(len - it->x, it->y);
      }
    } else {
      return false;
    }
  } else {
    solid = {{0.0f, ylo}, {len, ylo}, {len, yhi}, {0.0f, yhi}};
  }

  // The requested hole outline (legacy rect when no polygon), clipped to
  // the part actually on this piece.
  std::vector<glm::vec2> hole;
  if (c.polygon.size() >= 3) {
    for (const glm::vec3& p : c.polygon) hole.emplace_back(p.x, p.y);
  } else {
    if (!(c.u1 > c.u0) || !(c.y1 > c.y0)) return false;
    hole = {{c.u0, c.y0}, {c.u1, c.y0}, {c.u1, c.y1}, {c.u0, c.y1}};
  }
  hole = ClipPoly2(hole, [&](const glm::vec2& p) { return -p.x; });
  hole = ClipPoly2(hole, [&](const glm::vec2& p) { return p.x - len; });
  hole = ClipPoly2(hole, [&](const glm::vec2& p) { return ylo - p.y; });
  hole = ClipPoly2(hole, [&](const glm::vec2& p) { return p.y - yhi; });
  glm::vec2 holeLo(1e30f), holeHi(-1e30f);
  for (const glm::vec2& p : hole) {
    holeLo = glm::min(holeLo, p);
    holeHi = glm::max(holeHi, p);
  }
  if (hole.size() < 3 || holeHi.x - holeLo.x < kMinPieceSide ||
      holeHi.y - holeLo.y < kMinPieceSide) {
    return false;
  }
  std::vector<glm::vec2> lower, upper;
  if (!SplitChains(hole, &lower, &upper)) return false;

  // The leftover silhouette around the hole as convex pieces: jambs beside
  // it, one piece under each lower-chain edge and over each upper-chain edge.
  std::vector<std::vector<glm::vec2>> pieces;
  const auto push = [&](std::vector<glm::vec2> piece) {
    if (piece.size() >= 3 && PolygonArea(piece) >= 0.01f) pieces.push_back(std::move(piece));
  };
  push(ClipPoly2(solid, [&](const glm::vec2& p) { return p.x - holeLo.x; }));
  push(ClipPoly2(solid, [&](const glm::vec2& p) { return holeHi.x - p.x; }));
  const auto chainPieces = [&](const std::vector<glm::vec2>& chain, bool below) {
    for (size_t i = 0; i + 1 < chain.size(); ++i) {
      const glm::vec2 p = chain[i], q = chain[i + 1];
      if (q.x - p.x < 1e-4f) continue;
      std::vector<glm::vec2> piece =
          ClipPoly2(solid, [&](const glm::vec2& v) { return p.x - v.x; });
      piece = ClipPoly2(piece, [&](const glm::vec2& v) { return v.x - q.x; });
      // Keep the side of the edge's line away from the hole.
      const glm::vec2 dir = q - p;
      piece = ClipPoly2(piece, [&](const glm::vec2& v) {
        const float side = dir.x * (v.y - p.y) - dir.y * (v.x - p.x);
        return below ? side : -side;
      });
      push(std::move(piece));
    }
  };
  chainPieces(lower, true);
  chainPieces(upper, false);
  if (pieces.size() > kMaxPieces) return false;
  float keptArea = 0.0f;
  for (const auto& piece : pieces) keptArea += PolygonArea(piece);
  if (keptArea > PolygonArea(solid) - 0.01f) return false;  // Hole misses the solid.

  KillObstacle(scene, c.target);
  for (const std::vector<glm::vec2>& piece : pieces) {
    glm::vec2 lo = piece[0], hi = piece[0];
    for (const glm::vec2& p : piece) {
      lo = glm::min(lo, p);
      hi = glm::max(hi, p);
    }
    if (hi.x - lo.x < kMinPieceSide || hi.y - lo.y < kMinPieceSide) continue;
    const glm::vec2 s0 = a + d * lo.x, s1 = a + d * hi.x;
    Obstacle out = wall;
    out.dead = false;
    out.profile.clear();
    out.footprint = {s0, s1, s1 + tv, s0 + tv};
    out.bounds.min = glm::vec3(1e30f, lo.y, 1e30f);
    out.bounds.max = glm::vec3(-1e30f, hi.y, -1e30f);
    for (const glm::vec2& v : out.footprint) {
      out.bounds.min.x = std::min(out.bounds.min.x, v.x);
      out.bounds.max.x = std::max(out.bounds.max.x, v.x);
      out.bounds.min.z = std::min(out.bounds.min.z, v.y);
      out.bounds.max.z = std::max(out.bounds.max.z, v.y);
    }
    if (!IsBoundingRect(piece)) {
      out.profile.reserve(piece.size());
      for (const glm::vec2& p : piece) out.profile.emplace_back(p.x - lo.x, p.y);
    }
    scene->obstacles.push_back(std::move(out));
  }
  // The surviving pieces keep the building alive.
  if (wall.building >= 0 && wall.building < static_cast<int>(scene->buildings.size())) {
    for (const Obstacle& o : scene->obstacles) {
      if (!o.dead && o.building == wall.building) {
        scene->buildings[wall.building].dead = false;
        break;
      }
    }
  }
  FacadeOpening opening;
  opening.kind = FacadeOpening::Kind::Breach;
  opening.building = wall.building;
  opening.obstacle = c.target;
  opening.wallEdge = c.wallEdge;
  opening.u0 = c.polygon.size() >= 3 ? holeLo.x : c.u0;
  opening.u1 = c.polygon.size() >= 3 ? holeHi.x : c.u1;
  opening.y0 = c.polygon.size() >= 3 ? holeLo.y : c.y0;
  opening.y1 = c.polygon.size() >= 3 ? holeHi.y : c.y1;
  scene->facadeOpenings.push_back(opening);
  return true;
}

// Clips a convex `footprint` to the part outside rect (x0,z0)-(x1,z1), as up
// to four disjoint convex pieces (left of it, right of it, and the strips
// before/behind it in between).
std::vector<std::vector<glm::vec2>> ClipConvexAroundRect(const std::vector<glm::vec2>& footprint,
                                                         float x0, float z0, float x1, float z1) {
  std::vector<std::vector<glm::vec2>> pieces;
  pieces.push_back(ClipPoly2(footprint, [&](const glm::vec2& p) { return p.x - x0; }));
  pieces.push_back(ClipPoly2(footprint, [&](const glm::vec2& p) { return x1 - p.x; }));
  std::vector<glm::vec2> mid = ClipPoly2(footprint, [&](const glm::vec2& p) { return x0 - p.x; });
  mid = ClipPoly2(mid, [&](const glm::vec2& p) { return p.x - x1; });
  pieces.push_back(ClipPoly2(mid, [&](const glm::vec2& p) { return p.y - z0; }));
  pieces.push_back(ClipPoly2(mid, [&](const glm::vec2& p) { return z1 - p.y; }));
  pieces.erase(std::remove_if(pieces.begin(), pieces.end(),
                              [](const std::vector<glm::vec2>& p) {
                                return p.size() < 3 || PolygonArea(p) < 0.01f;
                              }),
               pieces.end());
  return pieces;
}

bool ApplyMassingCut(Scene* scene, const WorldChange& c) {
  if (c.target < 0 || c.target >= static_cast<int>(scene->massing.size())) return false;
  const MassingDetail detail = scene->massing[c.target];  // Copy: the cut kills it.
  if (detail.dead || detail.footprint.size() < 3 || detail.y1 <= detail.y0) return false;
  const float x0 = std::min(c.rect.x, c.rect.z), x1 = std::max(c.rect.x, c.rect.z);
  const float z0 = std::min(c.rect.y, c.rect.w), z1 = std::max(c.rect.y, c.rect.w);
  std::vector<std::vector<glm::vec2>> pieces;
  bool wholeInside = true;
  for (const glm::vec2& p : detail.footprint) {
    wholeInside &= p.x >= x0 - 1e-3f && p.x <= x1 + 1e-3f && p.y >= z0 - 1e-3f &&
                   p.y <= z1 + 1e-3f;
  }
  if (!c.pieces.empty()) {
    // Fracture cells left standing (see PlanExplosion): taken as planned,
    // as long as they really are a smaller part of this detail.
    if (c.pieces.size() > kMaxPieces || !detail.top.empty()) return false;
    float keptArea = 0.0f;
    for (const Obstacle& cell : c.pieces) {
      if (cell.footprint.size() < 3) return false;
      keptArea += PolygonArea(cell.footprint);
      pieces.push_back(cell.footprint);
    }
    if (keptArea > PolygonArea(detail.footprint) - 0.01f) return false;
  } else if (x1 - x0 < kMinPieceSide || z1 - z0 < kMinPieceSide) {
    return false;
  } else if (!wholeInside) {
    // Sloped-top details (deck rails) cannot be partially cut.
    if (!detail.top.empty()) return false;
    pieces = ClipConvexAroundRect(detail.footprint, x0, z0, x1, z1);
    float keptArea = 0.0f;
    for (const auto& piece : pieces) keptArea += PolygonArea(piece);
    if (keptArea > PolygonArea(detail.footprint) - 0.01f) return false;  // Misses it.
  }
  MassingDetail& dead = scene->massing[c.target];
  dead.dead = true;
  dead.footprint.clear();
  dead.top.clear();
  for (std::vector<glm::vec2>& piece : pieces) {
    MassingDetail out;
    out.kind = detail.kind;
    out.footprint = std::move(piece);
    out.y0 = detail.y0;
    out.y1 = detail.y1;
    out.building = detail.building;
    scene->massing.push_back(std::move(out));
  }
  return true;
}

bool ApplyDebris(Scene* scene, const WorldChange& c) {
  if (c.pieces.empty() || c.pieces.size() > kMaxPieces) return false;
  if (c.flights.size() != c.pieces.size()) return false;
  for (const Obstacle& piece : c.pieces) {
    if (!(piece.bounds.max.y > piece.bounds.min.y) || piece.footprint.size() < 3) return false;
  }
  for (const DebrisFlight& flight : c.flights) {
    if (!(flight.duration >= 0.0f) || !std::isfinite(flight.duration)) return false;
  }
  // Solid chunks are rubble obstacles from here on; litter is only drawn.
  for (size_t i = 0; i < c.pieces.size(); ++i) {
    if (!c.flights[i].solid) continue;
    Obstacle p = c.pieces[i];
    p.dead = false;
    scene->obstacles.push_back(std::move(p));
  }
  return true;
}

void WriteObstacle(std::ostream& out, const Obstacle& o) {
  out << ' ' << o.bounds.min.x << ' ' << o.bounds.min.y << ' ' << o.bounds.min.z << ' '
      << o.bounds.max.x << ' ' << o.bounds.max.y << ' ' << o.bounds.max.z << ' '
      << static_cast<int>(o.kind) << ' ' << o.building << ' ' << (o.navOnly ? 1 : 0) << ' '
      << static_cast<int>(o.fixtureModel) << ' ' << o.fixtureYaw << ' ' << o.fixtureBaseY << ' '
      << o.fixtureRoll << ' ' << o.footprint.size();
  for (const glm::vec2& p : o.footprint) out << ' ' << p.x << ' ' << p.y;
  out << ' ' << o.profile.size();
  for (const glm::vec2& p : o.profile) out << ' ' << p.x << ' ' << p.y;
}

bool ReadObstacle(std::istream& in, Obstacle* o) {
  int kind = 0, navOnly = 0, model = 0;
  size_t count = 0;
  if (!(in >> o->bounds.min.x >> o->bounds.min.y >> o->bounds.min.z >> o->bounds.max.x >>
        o->bounds.max.y >> o->bounds.max.z >> kind >> o->building >> navOnly >> model >>
        o->fixtureYaw >> o->fixtureBaseY >> o->fixtureRoll >> count) ||
      count > kMaxPolygon || kind < 0 || model < 0) {
    return false;
  }
  o->kind = static_cast<ObstacleKind>(kind);
  o->fixtureModel = static_cast<StreetFixtureModel>(model);
  o->navOnly = navOnly != 0;
  o->footprint.resize(count);
  for (glm::vec2& p : o->footprint) {
    if (!(in >> p.x >> p.y)) return false;
  }
  size_t profileCount = 0;
  if (!(in >> profileCount) || profileCount > kMaxPolygon) return false;
  o->profile.resize(profileCount);
  for (glm::vec2& p : o->profile) {
    if (!(in >> p.x >> p.y)) return false;
  }
  return true;
}

}  // namespace

bool WorldChange::operator==(const WorldChange& o) const {
  if (id != o.id || round != o.round || at != o.at || kind != o.kind || target != o.target ||
      polygon != o.polygon || rect != o.rect || delta != o.delta || wallEdge != o.wallEdge ||
      u0 != o.u0 || u1 != o.u1 || y0 != o.y0 || y1 != o.y1 || flights != o.flights ||
      pieces.size() != o.pieces.size()) {
    return false;
  }
  for (size_t i = 0; i < pieces.size(); ++i) {
    if (!SameObstacle(pieces[i], o.pieces[i])) return false;
  }
  return true;
}

void KillObstacle(Scene* scene, int index) {
  if (!ValidObstacle(*scene, index)) return;
  Obstacle& o = scene->obstacles[index];
  if (o.dead) return;
  const glm::vec3 c = o.bounds.Center();
  o.dead = true;
  o.navOnly = true;  // Never drawn.
  o.footprint.clear();
  // Parked below the world as a zero-size box: any consumer that has not
  // learned about `dead` still finds nothing to hit, collide with, or see.
  o.bounds.min = o.bounds.max = glm::vec3(c.x, kParkedY, c.z);
  for (FacadeOpening& f : scene->facadeOpenings) {
    if (f.obstacle == index) f.dead = true;
  }
  RefreshBuildingLiveness(scene, o.building);
  // Buildings that name this prism as their roof/base but whose own prisms
  // carry another (or no) building index.
  for (size_t b = 0; b < scene->buildings.size(); ++b) {
    const Building& building = scene->buildings[b];
    if (building.obstacle == index || building.upperObstacle == index) {
      RefreshBuildingLiveness(scene, static_cast<int>(b));
    }
  }
}

void KillWalkSurface(Scene* scene, int index) {
  if (index < 0 || index >= static_cast<int>(scene->walkSurfaces.size())) return;
  WalkSurface& s = scene->walkSurfaces[index];
  if (s.dead) return;
  s.dead = true;
  s.vertices.clear();
  s.neighbors.clear();
  s.groundConnections.clear();
  s.blockers.clear();
  s.connectsToGround = false;
  for (WalkSurface& other : scene->walkSurfaces) {
    other.neighbors.erase(std::remove(other.neighbors.begin(), other.neighbors.end(), index),
                          other.neighbors.end());
  }
}

bool ApplyWorldChange(Scene* scene, const WorldChange& c) {
  switch (c.kind) {
    case WorldChangeKind::ObstacleRemoved:
      if (!ValidObstacle(*scene, c.target) || scene->obstacles[c.target].dead) return false;
      KillObstacle(scene, c.target);
      return true;
    case WorldChangeKind::ObstacleReplaced: {
      if (!ValidObstacle(*scene, c.target) || scene->obstacles[c.target].dead) return false;
      if (c.pieces.size() > kMaxPieces) return false;
      KillObstacle(scene, c.target);
      for (const Obstacle& piece : c.pieces) {
        Obstacle p = piece;
        p.dead = false;
        scene->obstacles.push_back(std::move(p));
      }
      // Pieces carrying the building index keep it alive.
      for (const Obstacle& piece : c.pieces) {
        if (piece.building >= 0 && piece.building < static_cast<int>(scene->buildings.size())) {
          scene->buildings[piece.building].dead = false;
        }
      }
      return true;
    }
    case WorldChangeKind::SurfaceRemoved:
      if (c.target < 0 || c.target >= static_cast<int>(scene->walkSurfaces.size()) ||
          scene->walkSurfaces[c.target].dead) {
        return false;
      }
      KillWalkSurface(scene, c.target);
      return true;
    case WorldChangeKind::SurfaceCut:
      if (c.target < 0 || c.target >= static_cast<int>(scene->walkSurfaces.size()) ||
          scene->walkSurfaces[c.target].dead || c.polygon.size() < 3 ||
          c.polygon.size() > kMaxPolygon) {
        return false;
      }
      scene->walkSurfaces[c.target].vertices = c.polygon;
      scene->walkSurfaces[c.target].groundConnections.clear();
      return true;
    case WorldChangeKind::HeightPatch: {
      HeightField& g = scene->ground;
      if (g.Empty() || g.step <= 0.0f || !std::isfinite(c.delta)) return false;
      const float x0 = std::min(c.rect.x, c.rect.z), x1 = std::max(c.rect.x, c.rect.z);
      const float z0 = std::min(c.rect.y, c.rect.w), z1 = std::max(c.rect.y, c.rect.w);
      bool any = false;
      for (int iz = 0; iz < g.nz; ++iz) {
        for (int ix = 0; ix < g.nx; ++ix) {
          const float x = g.minX + ix * g.step, z = g.minZ + iz * g.step;
          if (x < x0 || x > x1 || z < z0 || z > z1) continue;
          g.heights[static_cast<size_t>(iz) * g.nx + ix] += c.delta;
          any = true;
        }
      }
      return any;
    }
    case WorldChangeKind::WallHole:
      return ApplyWallHole(scene, c);
    case WorldChangeKind::FloorHole:
      return ApplyFloorHole(scene, c);
    case WorldChangeKind::MassingCut:
      return ApplyMassingCut(scene, c);
    case WorldChangeKind::Debris:
      return ApplyDebris(scene, c);
  }
  return false;
}

void WriteWorldChange(std::ostream& out, const WorldChange& c) {
  out << ' ' << c.id << ' ' << c.round << ' ' << c.at << ' ' << static_cast<int>(c.kind) << ' '
      << c.target << ' ' << c.rect.x << ' ' << c.rect.y << ' ' << c.rect.z << ' ' << c.rect.w
      << ' ' << c.delta << ' ' << c.wallEdge << ' ' << c.u0 << ' ' << c.u1 << ' ' << c.y0 << ' '
      << c.y1 << ' ' << c.polygon.size();
  for (const glm::vec3& p : c.polygon) out << ' ' << p.x << ' ' << p.y << ' ' << p.z;
  out << ' ' << c.pieces.size();
  for (const Obstacle& o : c.pieces) WriteObstacle(out, o);
  out << ' ' << c.flights.size();
  for (const DebrisFlight& f : c.flights) {
    out << ' ' << f.from.x << ' ' << f.from.y << ' ' << f.from.z << ' ' << f.duration << ' '
        << f.spin << ' ' << f.fromYaw << ' ' << f.restYaw << ' ' << (f.solid ? 1 : 0);
  }
}

bool ReadWorldChange(std::istream& in, WorldChange* c) {
  int kind = 0;
  size_t polyCount = 0, pieceCount = 0;
  if (!(in >> c->id >> c->round >> c->at >> kind >> c->target >> c->rect.x >> c->rect.y >>
        c->rect.z >> c->rect.w >> c->delta >> c->wallEdge >> c->u0 >> c->u1 >> c->y0 >> c->y1 >>
        polyCount) ||
      kind < 0 || kind >= kWorldChangeKindCount || polyCount > kMaxPolygon) {
    return false;
  }
  c->kind = static_cast<WorldChangeKind>(kind);
  c->polygon.resize(polyCount);
  for (glm::vec3& p : c->polygon) {
    if (!(in >> p.x >> p.y >> p.z)) return false;
  }
  if (!(in >> pieceCount) || pieceCount > kMaxPieces) return false;
  c->pieces.assign(pieceCount, Obstacle{});
  for (Obstacle& o : c->pieces) {
    if (!ReadObstacle(in, &o)) return false;
  }
  size_t flightCount = 0;
  if (!(in >> flightCount) || flightCount > kMaxPieces) return false;
  c->flights.assign(flightCount, DebrisFlight{});
  for (DebrisFlight& f : c->flights) {
    int solid = 1;
    if (!(in >> f.from.x >> f.from.y >> f.from.z >> f.duration >> f.spin >> f.fromYaw >>
          f.restYaw >> solid)) {
      return false;
    }
    f.solid = solid != 0;
  }
  return true;
}

}  // namespace tactics
