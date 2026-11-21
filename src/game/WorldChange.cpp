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
         a.fixtureRoll == b.fixtureRoll && a.dead == b.dead;
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
  if (x1 - x0 < kMinPieceSide || z1 - z0 < kMinPieceSide) return false;
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
  pieces.push_back(ClipPolygon3(v, [&](float x, float) { return x - x0; }));
  pieces.push_back(ClipPolygon3(v, [&](float x, float) { return x1 - x; }));
  std::vector<glm::vec3> mid = ClipPolygon3(v, [&](float x, float) { return x0 - x; });
  mid = ClipPolygon3(mid, [&](float x, float) { return x - x1; });
  pieces.push_back(ClipPolygon3(mid, [&](float, float z) { return z - z0; }));
  pieces.push_back(ClipPolygon3(mid, [&](float, float z) { return z1 - z; }));
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

void WriteObstacle(std::ostream& out, const Obstacle& o) {
  out << ' ' << o.bounds.min.x << ' ' << o.bounds.min.y << ' ' << o.bounds.min.z << ' '
      << o.bounds.max.x << ' ' << o.bounds.max.y << ' ' << o.bounds.max.z << ' '
      << static_cast<int>(o.kind) << ' ' << o.building << ' ' << (o.navOnly ? 1 : 0) << ' '
      << static_cast<int>(o.fixtureModel) << ' ' << o.fixtureYaw << ' ' << o.fixtureBaseY << ' '
      << o.fixtureRoll << ' ' << o.footprint.size();
  for (const glm::vec2& p : o.footprint) out << ' ' << p.x << ' ' << p.y;
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
  return true;
}

}  // namespace

bool WorldChange::operator==(const WorldChange& o) const {
  if (id != o.id || round != o.round || at != o.at || kind != o.kind || target != o.target ||
      polygon != o.polygon || rect != o.rect || delta != o.delta || wallEdge != o.wallEdge ||
      u0 != o.u0 || u1 != o.u1 || y0 != o.y0 || y1 != o.y1 || pieces.size() != o.pieces.size()) {
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
    case WorldChangeKind::WallHole: {
      if (!ValidObstacle(*scene, c.target) || scene->obstacles[c.target].dead) return false;
      if (!(c.u1 > c.u0) || !(c.y1 > c.y0)) return false;
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
      // The part of the requested hole actually on this wall piece. A hole
      // that misses it entirely does not apply.
      const float u0 = std::max(0.0f, c.u0), u1 = std::min(len, c.u1);
      const float y0 = std::max(ylo, c.y0), y1 = std::min(yhi, c.y1);
      if (u1 - u0 < kMinPieceSide || y1 - y0 < kMinPieceSide) return false;
      KillObstacle(scene, c.target);
      const auto emit = [&](float pu0, float pu1, float py0, float py1) {
        if (pu1 - pu0 < kMinPieceSide || py1 - py0 < kMinPieceSide) return;
        const glm::vec2 s0 = a + d * pu0, s1 = a + d * pu1;
        Obstacle piece = wall;
        piece.dead = false;
        piece.footprint = {s0, s1, s1 + tv, s0 + tv};
        piece.bounds.min = glm::vec3(1e30f, py0, 1e30f);
        piece.bounds.max = glm::vec3(-1e30f, py1, -1e30f);
        for (const glm::vec2& v : piece.footprint) {
          piece.bounds.min.x = std::min(piece.bounds.min.x, v.x);
          piece.bounds.max.x = std::max(piece.bounds.max.x, v.x);
          piece.bounds.min.z = std::min(piece.bounds.min.z, v.y);
          piece.bounds.max.z = std::max(piece.bounds.max.z, v.y);
        }
        scene->obstacles.push_back(std::move(piece));
      };
      emit(0.0f, u0, ylo, yhi);  // Left jamb.
      emit(u1, len, ylo, yhi);   // Right jamb.
      emit(u0, u1, ylo, y0);     // Sill.
      emit(u0, u1, y1, yhi);     // Head.
      // The surviving jambs/sill/head keep the building alive.
      if (wall.building >= 0 && wall.building < static_cast<int>(scene->buildings.size())) {
        for (const Obstacle& o : scene->obstacles) {
          if (!o.dead && o.building == wall.building) {
            scene->buildings[wall.building].dead = false;
            break;
          }
        }
      }
      FacadeOpening hole;
      hole.kind = FacadeOpening::Kind::Breach;
      hole.building = wall.building;
      hole.obstacle = c.target;
      hole.wallEdge = c.wallEdge;
      hole.u0 = c.u0;
      hole.u1 = c.u1;
      hole.y0 = c.y0;
      hole.y1 = c.y1;
      scene->facadeOpenings.push_back(hole);
      return true;
    }
    case WorldChangeKind::FloorHole:
      return ApplyFloorHole(scene, c);
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
  return true;
}

}  // namespace tactics
