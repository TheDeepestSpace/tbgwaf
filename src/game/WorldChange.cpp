#include "game/WorldChange.h"

#include <algorithm>
#include <cmath>
#include <istream>
#include <ostream>

namespace tactics {

namespace {

constexpr size_t kMaxPieces = 64;
constexpr size_t kMaxPolygon = 64;
constexpr float kParkedY = -1000.0f;

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
  // Buildings that name this prism as their base but whose own prisms
  // carry another (or no) building index.
  for (size_t b = 0; b < scene->buildings.size(); ++b) {
    const Building& building = scene->buildings[b];
    if (building.obstacle == index) {
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
      FacadeOpening hole;
      hole.kind = FacadeOpening::Kind::Breach;
      hole.building = scene->obstacles[c.target].building;
      hole.obstacle = c.target;
      hole.wallEdge = c.wallEdge;
      hole.u0 = c.u0;
      hole.u1 = c.u1;
      hole.y0 = c.y0;
      hole.y1 = c.y1;
      scene->facadeOpenings.push_back(hole);
      return true;
    }
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
