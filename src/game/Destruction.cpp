#include "game/Destruction.h"

#include <algorithm>
#include <cmath>
#include <random>

#include <glm/gtc/matrix_transform.hpp>

#include "game/Geometry.h"

namespace tactics {
namespace {

// Fracture tuning. Everything concrete is pre-cracked the same way: a seeded
// Voronoi diagram in the element's own plane (the wall plane for upright
// prisms, XZ for slabs, floors and roof massing). Cells near the blast are
// knocked out -- each one becomes a debris chunk of exactly that shape --
// and the cells further out stay standing as cracked pieces.
constexpr float kRemovedRing = 0.72f;     // Sites within this share of the radius go.
constexpr float kWindowScale = 1.3f;      // Cracks reach this far (x in-plane radius).
constexpr float kMaxWallThickness = 0.7f; // Thicker solids are not fractured (yet).
constexpr size_t kMaxPlanChanges = 96;    // Overall change budget per blast.
constexpr size_t kMaxSolidChunks = 8;     // Largest chunks: rubble obstacles.
constexpr size_t kMaxDebrisPieces = 56;   // Solid + litter chunks per blast.
constexpr size_t kMaxPiecesPerChange = 60;
constexpr float kMinKeptArea = 0.01f;     // Smaller standing slivers crumble to dust.
constexpr float kMinChunkArea = 0.03f;    // Smaller knocked-out cells are dust.
constexpr float kSolidChunkArea = 0.25f;  // Chunks at least this big can be solid rubble.
constexpr float kGravity = 9.81f;         // Debris ballistics.

// Same hand-drawn mt19937 draws as MapGenerator's Rng: fully specified by
// the standard, so a seed fractures identically on every platform.
class Rng {
 public:
  explicit Rng(uint32_t seed) : engine_(seed) {}
  int Int(int lo, int hi) { return lo + static_cast<int>(engine_() % static_cast<uint32_t>(hi - lo + 1)); }
  float Unit01() { return static_cast<float>(engine_() >> 8) / 16777216.0f; }
  float Float(float lo, float hi) { return lo + (hi - lo) * Unit01(); }

 private:
  std::mt19937 engine_;
};

uint32_t Mix(uint32_t h, uint32_t v) {
  h ^= v;
  h *= 16777619u;  // FNV-1a prime.
  return h;
}

// Distance from `point` to the prism (footprint x [min.y, max.y]).
float DistanceToPrism(const glm::vec3& point, const Obstacle& o) {
  const std::vector<glm::vec2> f = ObstacleFootprint(o);
  const glm::vec2 p(point.x, point.z);
  float dxz = 0.0f;
  if (!PointInConvexPolygon(p, f)) {
    float best = 1e30f;
    for (size_t i = 0; i < f.size(); ++i) {
      const glm::vec2 a = f[i], b = f[(i + 1) % f.size()];
      const glm::vec2 ab = b - a;
      const float len2 = glm::dot(ab, ab);
      const float t = len2 > 1e-9f ? std::clamp(glm::dot(p - a, ab) / len2, 0.0f, 1.0f) : 0.0f;
      best = std::min(best, glm::distance(p, a + ab * t));
    }
    dxz = best;
  }
  const float dy = point.y < o.bounds.min.y ? o.bounds.min.y - point.y
                   : point.y > o.bounds.max.y ? point.y - o.bounds.max.y
                                              : 0.0f;
  return std::sqrt(dxz * dxz + dy * dy);
}


using Poly = std::vector<glm::vec2>;

glm::vec2 Perp(glm::vec2 v) { return {-v.y, v.x}; }

glm::vec2 Rotate2(glm::vec2 v, float angle) {
  const float c = std::cos(angle), s = std::sin(angle);
  return {v.x * c - v.y * s, v.x * s + v.y * c};
}

// Sutherland-Hodgman step: the part of convex `poly` with dot(p, normal) <=
// offset. Keeps the subject's winding.
Poly ClipHalf(const Poly& poly, glm::vec2 normal, float offset) {
  Poly out;
  const size_t n = poly.size();
  for (size_t i = 0; i < n; ++i) {
    const glm::vec2 p = poly[i], q = poly[(i + 1) % n];
    const float dp = glm::dot(p, normal) - offset, dq = glm::dot(q, normal) - offset;
    if (dp <= 0.0f) out.push_back(p);
    if ((dp < 0.0f) != (dq < 0.0f) && std::fabs(dq - dp) > 1e-9f) {
      out.push_back(p + (q - p) * (dp / (dp - dq)));
    }
  }
  return out;
}

// Drops consecutive duplicate vertices clipping can leave behind.
void Tidy(Poly* poly) {
  Poly out;
  for (const glm::vec2& p : *poly) {
    if (out.empty() || glm::distance(out.back(), p) > 1e-4f) out.push_back(p);
  }
  while (out.size() > 1 && glm::distance(out.front(), out.back()) <= 1e-4f) out.pop_back();
  *poly = std::move(out);
}

glm::vec2 VertexMean(const Poly& poly) {
  glm::vec2 sum(0.0f);
  for (const glm::vec2& p : poly) sum += p;
  return poly.empty() ? sum : sum / static_cast<float>(poly.size());
}

void Bounds2(const Poly& poly, glm::vec2* lo, glm::vec2* hi) {
  *lo = *hi = poly.front();
  for (const glm::vec2& p : poly) {
    *lo = glm::min(*lo, p);
    *hi = glm::max(*hi, p);
  }
}

// True when `poly` is (within eps) exactly the axis-aligned rectangle of its
// own bounding box: such a wall piece is a plain prism.
bool IsBoundingRect(const Poly& poly) {
  if (poly.size() != 4) return false;
  glm::vec2 lo, hi;
  Bounds2(poly, &lo, &hi);
  constexpr float kEps = 1e-3f;
  for (const glm::vec2& p : poly) {
    if (std::fabs(p.x - lo.x) > kEps && std::fabs(p.x - hi.x) > kEps) return false;
    if (std::fabs(p.y - lo.y) > kEps && std::fabs(p.y - hi.y) > kEps) return false;
  }
  return true;
}

// The parts of convex `poly` outside the box (x0,y0)-(x1,y1), as up to four
// disjoint convex pieces.
std::vector<Poly> ClipAroundRect(const Poly& poly, float x0, float y0, float x1, float y1) {
  std::vector<Poly> pieces;
  pieces.push_back(ClipHalf(poly, {1.0f, 0.0f}, x0));    // x <= x0.
  pieces.push_back(ClipHalf(poly, {-1.0f, 0.0f}, -x1));  // x >= x1.
  Poly mid = ClipHalf(poly, {-1.0f, 0.0f}, -x0);
  mid = ClipHalf(mid, {1.0f, 0.0f}, x1);
  pieces.push_back(ClipHalf(mid, {0.0f, 1.0f}, y0));     // y <= y0 strip.
  pieces.push_back(ClipHalf(mid, {0.0f, -1.0f}, -y1));   // y >= y1 strip.
  return pieces;
}

// The blast's crack pattern. Sites are world-anchored (offsets from the bomb)
// and projected into whatever plane is being fractured, so coplanar elements
// -- the jamb, sill and head prisms CutWindows split one facade into, or a
// ceiling slab and the floor surface on top of it -- crack along the same
// lines and their chunks fit together.
struct SiteSet {
  std::vector<glm::vec3> offsets;  // Random sites, dense near the bomb.
  // A ring of always-standing sites around the blast (angle, radius scale):
  // it bounds the knocked-out region, so the hole has roughly the blast's
  // in-plane radius however the random sites fell.
  std::vector<glm::vec2> anchors;
};

SiteSet MakeSites(Rng* rng, float radius, int count, int anchors) {
  SiteSet set;
  for (int i = 0; i < count; ++i) {
    const float h = rng->Float(-1.0f, 1.0f);
    const float phi = rng->Float(0.0f, 6.2831853f);
    const float ring = std::sqrt(std::max(0.0f, 1.0f - h * h));
    const float unit = rng->Unit01();
    const float rho = radius * unit * std::sqrt(unit);  // Graded: dense at the center.
    set.offsets.push_back(rho * glm::vec3(ring * std::cos(phi), h, ring * std::sin(phi)));
  }
  for (int k = 0; k < anchors; ++k) {
    set.anchors.emplace_back(6.2831853f * (k + rng->Float(0.15f, 0.85f)) / anchors,
                             rng->Float(1.0f, 1.18f));
  }
  return set;
}

struct PlaneFracture {
  std::vector<Poly> kept;     // Standing pieces (convex), covering what is left.
  std::vector<Poly> removed;  // Knocked-out cells (convex): the debris chunks.
};

// Pre-cracks convex `solid` -- given in blast-plane coordinates, i.e.
// relative to the bomb's projection onto the plane -- into Voronoi cells of
// `projected` (the site set projected into the same coordinates) plus the
// anchor ring, and sorts the cells into standing and knocked-out. `reach` is
// the bomb's distance to the plane. Outside the crack window the solid is
// left in plain pieces. Empty when the blast knocks nothing out of it.
PlaneFracture FracturePlane(const Poly& solid, const std::vector<glm::vec2>& projected,
                            const SiteSet& set, float reach, float radius) {
  const float removedRadius = kRemovedRing * radius;
  if (solid.size() < 3 || reach >= removedRadius * 0.97f) return {};
  const float planeRadius = std::sqrt(std::max(0.01f, radius * radius - reach * reach));
  struct Site {
    glm::vec2 p;
    bool removed;
  };
  std::vector<Site> sites;
  const auto add = [&](glm::vec2 p, bool removed) {
    for (const Site& s : sites) {
      if (glm::distance(s.p, p) < 0.06f) return;  // Degenerate bisectors.
    }
    sites.push_back({p, removed});
  };
  add(glm::vec2(0.0f), true);  // The spot facing the bomb always goes.
  for (const glm::vec2& p : projected) {
    add(p, glm::dot(p, p) + reach * reach < removedRadius * removedRadius);
  }
  for (const glm::vec2& anchor : set.anchors) {
    add(planeRadius * anchor.y * glm::vec2(std::cos(anchor.x), std::sin(anchor.x)), false);
  }

  PlaneFracture out;
  const float w = planeRadius * kWindowScale;
  Poly inner = ClipHalf(solid, {1.0f, 0.0f}, w);
  inner = ClipHalf(inner, {-1.0f, 0.0f}, w);
  inner = ClipHalf(inner, {0.0f, 1.0f}, w);
  inner = ClipHalf(inner, {0.0f, -1.0f}, w);
  Tidy(&inner);
  if (inner.size() < 3) return {};
  float removedArea = 0.0f;
  for (size_t i = 0; i < sites.size(); ++i) {
    Poly cell = inner;
    for (size_t j = 0; j < sites.size() && cell.size() >= 3; ++j) {
      if (j == i) continue;
      const glm::vec2 normal = sites[j].p - sites[i].p;
      cell = ClipHalf(cell, normal, glm::dot(0.5f * (sites[i].p + sites[j].p), normal));
    }
    Tidy(&cell);
    if (cell.size() < 3) continue;
    const float area = PolygonArea(cell);
    if (area < 1e-4f) continue;
    if (sites[i].removed) {
      removedArea += area;
      out.removed.push_back(std::move(cell));
    } else {
      out.kept.push_back(std::move(cell));
    }
  }
  if (removedArea < 0.02f) return {};
  for (Poly& piece : ClipAroundRect(solid, -w, -w, w, w)) {
    Tidy(&piece);
    if (piece.size() >= 3) out.kept.push_back(std::move(piece));
  }
  return out;
}

bool Fracturable(const Obstacle& o) {
  return !o.dead && !o.navOnly && o.kind != ObstacleKind::Pier &&
         o.kind != ObstacleKind::Rubble && !IsStreetFixture(o.kind);
}

// A thin horizontal plate (ceiling or roof slab): fractured in XZ.
bool SlabLike(const Obstacle& o) {
  if (!Fracturable(o)) return false;
  if (o.bounds.max.y - o.bounds.min.y > 0.7f) return false;
  const std::vector<glm::vec2> f = ObstacleFootprint(o);
  return f.size() >= 3 && PolygonArea(f) >= 1.0f;
}

// An upright prism of wall thickness -- a wall, a window jamb / sill / head
// strip, a corner post, or a piece an earlier blast left standing -- in the
// frame it is fractured in: (u, y) with u along footprint edge `a -> a + d *
// len` and the solid extruded along `tv` through the thickness.
struct Upright {
  glm::vec2 a{0.0f}, d{1.0f, 0.0f}, tv{0.0f};
  float len = 0.0f, thickness = 0.0f;
  Poly solid;  // Silhouette in (u, y): the piece's profile or its full rect.
};

bool UprightOf(const Obstacle& o, Upright* out) {
  if (!Fracturable(o) || SlabLike(o)) return false;
  const std::vector<glm::vec2> f = ObstacleFootprint(o);
  if (f.size() != 4) return false;
  // A profiled piece is re-cut in its own wall plane (the profile's frame is
  // footprint edge 0); a plain prism along its long face.
  const bool profiled = o.profile.size() >= 3;
  const size_t e =
      profiled || glm::distance(f[0], f[1]) >= glm::distance(f[1], f[2]) ? 0 : 1;
  out->a = f[e];
  out->len = glm::distance(f[e], f[(e + 1) % 4]);
  out->tv = f[(e + 3) % 4] - f[e];
  out->thickness = glm::length(out->tv);
  if (out->len < 0.02f || out->thickness < 1e-3f || out->thickness > kMaxWallThickness) {
    return false;
  }
  if (o.bounds.max.y - o.bounds.min.y < 0.05f) return false;
  out->d = (f[(e + 1) % 4] - f[e]) / out->len;
  if (profiled) {
    out->solid = o.profile;
  } else {
    out->solid = {{0.0f, o.bounds.min.y},
                  {out->len, o.bounds.min.y},
                  {out->len, o.bounds.max.y},
                  {0.0f, o.bounds.max.y}};
  }
  return true;
}

void FitBoundsXZ(Obstacle* o) {
  o->bounds.min.x = o->bounds.max.x = o->footprint.front().x;
  o->bounds.min.z = o->bounds.max.z = o->footprint.front().y;
  for (const glm::vec2& v : o->footprint) {
    o->bounds.min.x = std::min(o->bounds.min.x, v.x);
    o->bounds.max.x = std::max(o->bounds.max.x, v.x);
    o->bounds.min.z = std::min(o->bounds.min.z, v.y);
    o->bounds.max.z = std::max(o->bounds.max.z, v.y);
  }
}

Obstacle PrismFromFootprint(const Obstacle& base, Poly footprint) {
  Obstacle piece = base;
  piece.dead = false;
  piece.profile.clear();
  piece.footprint = std::move(footprint);
  FitBoundsXZ(&piece);
  return piece;
}

// The standing wall piece with silhouette `shape` (u, y) in `frame`. Cells
// keep their silhouette as the profile even when it is a plain rect, so
// every fracture piece is treated alike downstream (a low stub left at the
// foot of a breach can be stepped over; see NavMesh::Build).
Obstacle WallPiece(const Obstacle& base, const Upright& frame, Poly shape, bool cell) {
  if (PolygonSignedArea(shape) < 0.0f) std::reverse(shape.begin(), shape.end());
  glm::vec2 lo, hi;
  Bounds2(shape, &lo, &hi);
  const glm::vec2 s0 = frame.a + frame.d * lo.x, s1 = frame.a + frame.d * hi.x;
  Obstacle piece = base;
  piece.dead = false;
  piece.profile.clear();
  piece.footprint = {s0, s1, s1 + frame.tv, s0 + frame.tv};
  FitBoundsXZ(&piece);
  piece.bounds.min.y = lo.y;
  piece.bounds.max.y = hi.y;
  if (cell || !IsBoundingRect(shape)) {
    for (const glm::vec2& p : shape) piece.profile.emplace_back(p.x - lo.x, p.y);
  }
  return piece;
}

// Highest support under (x, z) below `belowY`: the ground, or a live walk
// surface (a surviving floor or roof a chunk can land on).
float SupportHeight(const Scene& scene, glm::vec2 xz, float belowY) {
  float support = scene.ground.HeightAt(xz.x, xz.y);
  for (const WalkSurface& s : scene.walkSurfaces) {
    if (s.dead || s.climbProxy || s.vertices.size() < 3) continue;
    if (!SurfaceContainsXZ(s, xz.x, xz.y)) continue;
    const float y = SurfaceHeightAt(s, xz.x, xz.y);
    if (y <= belowY + 0.3f) support = std::max(support, y);
  }
  return support;
}

// True when a chunk resting at `center` with vertical extent [y0, y1] would
// sit inside a live obstacle.
bool RestSpotBlocked(const Scene& scene, glm::vec2 center, float y0, float y1) {
  for (const Obstacle& o : scene.obstacles) {
    if (o.dead || o.navOnly) continue;
    if (y1 <= o.bounds.min.y + 0.02f || y0 >= o.bounds.max.y - 0.02f) continue;
    if (center.x < o.bounds.min.x - 0.01f || center.x > o.bounds.max.x + 0.01f ||
        center.y < o.bounds.min.z - 0.01f || center.y > o.bounds.max.z + 0.01f) {
      continue;
    }
    if (PointInConvexPolygon(center, ObstacleFootprint(o))) return true;
  }
  return false;
}

// One knocked-out cell on its way to becoming debris.
struct Chunk {
  Poly local;            // Outline relative to its vertex mean: (along, up), or (x, z) when flat.
  glm::vec3 center{0.0f};  // The vertex mean at launch (world, mid-thickness).
  glm::vec2 along{1.0f, 0.0f};  // Wall's long axis (unit, XZ); +X for flat chunks.
  glm::vec2 outward{0.0f};      // Through the wall, away from the bomb (upright only).
  bool flat = false;     // A slab/parapet cell: launched lying down.
  float thickness = 0.2f;
  float area = 0.0f;
};

glm::mat4 YawMatrix(float yaw) {
  const float c = std::cos(yaw), s = std::sin(yaw);
  glm::mat4 m(1.0f);
  m[0] = glm::vec4(c, 0.0f, s, 0.0f);
  m[2] = glm::vec4(-s, 0.0f, c, 0.0f);
  return m;
}

}  // namespace

uint32_t ExplosionSeed(uint32_t matchSeed, int round, int bombId) {
  uint32_t h = 2166136261u;  // FNV-1a offset basis.
  h = Mix(h, matchSeed);
  h = Mix(h, static_cast<uint32_t>(round));
  h = Mix(h, static_cast<uint32_t>(bombId));
  return h;
}

glm::mat4 DebrisPose(const Obstacle& piece, const DebrisFlight& flight, float t) {
  if (!(flight.duration > 1e-3f) || t >= flight.duration || piece.footprint.empty()) {
    return glm::mat4(1.0f);
  }
  const glm::vec2 mean = VertexMean(piece.footprint);
  const glm::vec3 rest(mean.x, 0.5f * (piece.bounds.min.y + piece.bounds.max.y), mean.y);
  t = std::max(0.0f, t);
  const float s = t / flight.duration;
  const float vy0 =
      (rest.y - flight.from.y + 0.5f * kGravity * flight.duration * flight.duration) /
      flight.duration;
  const glm::vec3 pos(glm::mix(flight.from.x, rest.x, s),
                      flight.from.y + vy0 * t - 0.5f * kGravity * t * t,
                      glm::mix(flight.from.z, rest.z, s));
  float dYaw = flight.fromYaw - flight.restYaw;
  while (dYaw > 3.1415927f) dYaw -= 6.2831853f;
  while (dYaw < -3.1415927f) dYaw += 6.2831853f;
  const glm::vec3 axis(std::cos(flight.restYaw), 0.0f, std::sin(flight.restYaw));
  return glm::translate(glm::mat4(1.0f), pos) * YawMatrix(dYaw * (1.0f - s)) *
         glm::rotate(glm::mat4(1.0f), flight.spin * (s - 1.0f), axis) *
         glm::translate(glm::mat4(1.0f), -rest);
}

std::vector<WorldChange> PlanExplosion(const Scene& scene, const BombSpec& bomb) {
  std::vector<WorldChange> plan;
  if (!(bomb.radius > 0.1f)) return plan;
  Scene scratch = scene;  // Debris lands on what the cuts leave behind.
  const auto planChange = [&](WorldChange change) {
    if (plan.size() >= kMaxPlanChanges) return false;
    if (!ApplyWorldChange(&scratch, change)) return false;
    plan.push_back(std::move(change));
    return true;
  };
  const glm::vec2 bombXZ(bomb.position.x, bomb.position.z);
  // One crack pattern for everything upright and a coarser one for
  // everything horizontal (floor pieces feed the navmesh's surface routing,
  // so they stay few).
  Rng wallRng(Mix(bomb.seed, 0x9e3779b9u));
  const SiteSet wallSites = MakeSites(&wallRng, bomb.radius, 30 + wallRng.Int(0, 8), 9);
  Rng floorRng(Mix(bomb.seed, 0x85ebca6bu));
  const SiteSet floorSites = MakeSites(&floorRng, bomb.radius, 12 + floorRng.Int(0, 4), 6);
  std::vector<glm::vec2> floorProjected;
  for (const glm::vec3& o : floorSites.offsets) floorProjected.emplace_back(o.x, o.z);
  const auto toBlast = [&](const Poly& world) {
    Poly out;
    for (const glm::vec2& p : world) out.push_back(p - bombXZ);
    return out;
  };
  const auto fromBlast = [&](Poly poly) {
    for (glm::vec2& p : poly) p += bombXZ;
    return poly;
  };

  std::vector<Chunk> chunks;
  const auto flatChunk = [&](const Poly& world, float y0, float y1) {
    Chunk chunk;
    chunk.area = PolygonArea(world);
    if (chunk.area < kMinChunkArea) return;
    const glm::vec2 mean = VertexMean(world);
    for (const glm::vec2& p : world) chunk.local.push_back(p - mean);
    chunk.center = glm::vec3(mean.x, 0.5f * (y0 + y1), mean.y);
    chunk.flat = true;
    chunk.thickness = y1 - y0;
    chunks.push_back(std::move(chunk));
  };

  // 1. Street fixtures (and rubble from earlier blasts) in the blast are
  // blown away.
  const size_t baseObstacles = scene.obstacles.size();
  for (size_t i = 0; i < baseObstacles; ++i) {
    const Obstacle& o = scene.obstacles[i];
    if (o.dead || !(IsStreetFixture(o.kind) || o.kind == ObstacleKind::Rubble)) continue;
    if (DistanceToPrism(bomb.position, o) >= bomb.radius) continue;
    WorldChange remove;
    remove.kind = WorldChangeKind::ObstacleRemoved;
    remove.target = static_cast<int>(i);
    planChange(std::move(remove));
  }

  // 2. Everything upright of wall thickness -- walls, the jamb / sill / head
  // strips around windows, corner posts, pieces earlier blasts left -- is
  // pre-cracked in its own plane; the cells near the blast come out as
  // chunks, the rest stay standing as profiled pieces.
  for (size_t i = 0; i < baseObstacles; ++i) {
    const Obstacle& wall = scene.obstacles[i];
    Upright frame;
    if (!UprightOf(wall, &frame)) continue;
    if (DistanceToPrism(bomb.position, wall) >= bomb.radius) continue;
    const glm::vec2 normal = Perp(frame.d);
    const glm::vec2 midA = frame.a + 0.5f * frame.tv;
    const float side = glm::dot(bombXZ - midA, normal);
    const float reach = std::max(0.0f, std::fabs(side) - 0.5f * frame.thickness);
    // Blast-plane coordinates: origin at the bomb's projection, axes the
    // canonical along-direction (the same for every prism in this plane,
    // whichever way its own footprint runs) and world up.
    const bool flip = frame.d.x < -1e-6f || (std::fabs(frame.d.x) <= 1e-6f && frame.d.y < 0.0f);
    const float sign = flip ? -1.0f : 1.0f;
    const float bombU = glm::dot(bombXZ - frame.a, frame.d);
    Poly solid;
    for (const glm::vec2& p : frame.solid) {
      solid.emplace_back(sign * (p.x - bombU), p.y - bomb.position.y);
    }
    std::vector<glm::vec2> projected;
    for (const glm::vec3& o : wallSites.offsets) {
      projected.emplace_back(sign * glm::dot(glm::vec2(o.x, o.z), frame.d), o.y);
    }
    PlaneFracture fracture = FracturePlane(solid, projected, wallSites, reach, bomb.radius);
    if (fracture.removed.empty()) continue;
    const auto toWall = [&](Poly poly) {
      for (glm::vec2& p : poly) p = glm::vec2(bombU + sign * p.x, p.y + bomb.position.y);
      return poly;
    };
    WorldChange cut;
    cut.kind = WorldChangeKind::ObstacleReplaced;
    cut.target = static_cast<int>(i);
    const size_t cells = fracture.kept.size();
    for (size_t k = 0; k < cells; ++k) {
      Poly shape = toWall(std::move(fracture.kept[k]));
      glm::vec2 lo, hi;
      Bounds2(shape, &lo, &hi);
      if (PolygonArea(shape) < kMinKeptArea || hi.x - lo.x < 0.02f || hi.y - lo.y < 0.02f) {
        continue;  // Crumbles.
      }
      if (cut.pieces.size() >= kMaxPiecesPerChange) break;
      cut.pieces.push_back(WallPiece(wall, frame, std::move(shape), true));
    }
    if (cut.pieces.empty()) cut.kind = WorldChangeKind::ObstacleRemoved;
    if (!planChange(std::move(cut))) continue;
    // Chunks fly out the far side; a bomb inside the wall itself throws
    // them both ways.
    const bool inside = std::fabs(side) <= 0.5f * frame.thickness;
    Rng sideRng(Mix(bomb.seed, static_cast<uint32_t>(i) * 2654435761u));
    for (Poly& cell : fracture.removed) {
      const glm::vec2 outward =
          (inside ? sideRng.Int(0, 1) == 1 : side > 0.0f) ? -normal : normal;
      const Poly shape = toWall(std::move(cell));
      Chunk chunk;
      chunk.area = PolygonArea(shape);
      if (chunk.area < kMinChunkArea) continue;
      const glm::vec2 mean = VertexMean(shape);
      for (const glm::vec2& p : shape) chunk.local.push_back(p - mean);
      const glm::vec2 xz = midA + frame.d * mean.x;
      chunk.center = glm::vec3(xz.x, mean.y, xz.y);
      chunk.along = frame.d;
      chunk.outward = outward;
      chunk.thickness = frame.thickness;
      chunks.push_back(std::move(chunk));
    }
  }

  // 3. Ceiling / roof slabs: the same pre-crack in XZ. Each cut slab's level
  // is remembered so the floor surface lying on it breaks along the very
  // same cells (step 5).
  std::vector<float> levels;
  const auto levelFor = [&](float y) {
    for (const float level : levels) {
      if (std::fabs(level - y) < 0.8f) return level;
    }
    levels.push_back(y);
    return y;
  };
  const auto fractureFlat = [&](const Poly& footprint, float reach) {
    return FracturePlane(toBlast(footprint), floorProjected, floorSites, reach, bomb.radius);
  };
  for (size_t i = 0; i < baseObstacles; ++i) {
    const Obstacle& slab = scene.obstacles[i];
    if (!SlabLike(slab) || slab.building < 0) continue;
    if (DistanceToPrism(bomb.position, slab) >= bomb.radius) continue;
    const float midY = 0.5f * (slab.bounds.min.y + slab.bounds.max.y);
    if (std::fabs(bomb.position.y - midY) >= kRemovedRing * bomb.radius) continue;
    const float level = levelFor(midY);
    PlaneFracture fracture =
        fractureFlat(ObstacleFootprint(slab), std::fabs(bomb.position.y - level));
    if (fracture.removed.empty()) continue;
    WorldChange cut;
    cut.kind = WorldChangeKind::ObstacleReplaced;
    cut.target = static_cast<int>(i);
    for (Poly& piece : fracture.kept) {
      if (PolygonArea(piece) < kMinKeptArea || cut.pieces.size() >= kMaxPiecesPerChange) continue;
      cut.pieces.push_back(PrismFromFootprint(slab, fromBlast(std::move(piece))));
    }
    if (cut.pieces.empty()) cut.kind = WorldChangeKind::ObstacleRemoved;
    if (!planChange(std::move(cut))) continue;
    for (Poly& cell : fracture.removed) {
      flatChunk(fromBlast(std::move(cell)), slab.bounds.min.y, slab.bounds.max.y);
    }
  }

  // 4. Building massing in the blast -- the parapet band wrapping the roof
  // edge, roof furniture, awnings -- breaks with the structure under it,
  // along the same horizontal cells. Sloped-top details (deck rails) cannot
  // be partially cut: those go whole when the blast box covers them.
  const size_t baseMassing = scene.massing.size();
  for (size_t i = 0; i < baseMassing; ++i) {
    const MassingDetail& detail = scene.massing[i];
    if (detail.dead || detail.footprint.size() < 3 || detail.y1 <= detail.y0) continue;
    Obstacle prism;
    prism.footprint = detail.footprint;
    prism.bounds.min = glm::vec3(0.0f, detail.y0, 0.0f);
    prism.bounds.max = glm::vec3(0.0f, detail.y1, 0.0f);
    if (DistanceToPrism(bomb.position, prism) >= bomb.radius) continue;
    const float dy = bomb.position.y < detail.y0   ? detail.y0 - bomb.position.y
                     : bomb.position.y > detail.y1 ? bomb.position.y - detail.y1
                                                   : 0.0f;
    WorldChange cut;
    cut.kind = WorldChangeKind::MassingCut;
    cut.target = static_cast<int>(i);
    const float inPlane = std::sqrt(std::max(0.0f, bomb.radius * bomb.radius - dy * dy));
    const float half = std::min(inPlane * 0.8f, 4.5f);
    cut.rect = glm::vec4(bomb.position.x - half, bomb.position.z - half,
                         bomb.position.x + half, bomb.position.z + half);
    if (!detail.top.empty()) {
      if (half >= 0.3f) planChange(std::move(cut));  // Whole or not at all (the apply decides).
      continue;
    }
    PlaneFracture fracture = fractureFlat(detail.footprint, dy);
    if (fracture.removed.empty()) continue;
    for (Poly& piece : fracture.kept) {
      if (PolygonArea(piece) < kMinKeptArea || cut.pieces.size() >= kMaxPiecesPerChange) continue;
      cut.pieces.push_back(PrismFromFootprint(prism, fromBlast(std::move(piece))));
    }
    if (cut.pieces.empty()) {  // Nothing left standing: the legacy whole-detail removal.
      glm::vec2 lo, hi;
      Bounds2(detail.footprint, &lo, &hi);
      cut.rect = glm::vec4(lo.x - 0.1f, lo.y - 0.1f, hi.x + 0.1f, hi.y + 0.1f);
    }
    if (!planChange(std::move(cut))) continue;
    for (Poly& cell : fracture.removed) {
      flatChunk(fromBlast(std::move(cell)), detail.y0, detail.y1);
    }
  }

  // 5. Building floor surfaces: holes a figure can fall through, torn along
  // the cells of the slab underneath.
  const size_t baseSurfaces = scene.walkSurfaces.size();
  for (size_t i = 0; i < baseSurfaces; ++i) {
    const WalkSurface& floor = scene.walkSurfaces[i];
    if (floor.dead || floor.building < 0 || floor.stair || floor.climbProxy ||
        floor.vertices.size() < 3) {
      continue;
    }
    float yMin = floor.vertices.front().y, yMax = yMin;
    Poly outline;
    for (const glm::vec3& v : floor.vertices) {
      yMin = std::min(yMin, v.y);
      yMax = std::max(yMax, v.y);
      outline.emplace_back(v.x, v.z);
    }
    if (yMax - yMin > 0.5f) continue;  // Flat floors only.
    const float floorY = 0.5f * (yMin + yMax);
    if (std::fabs(bomb.position.y - floorY) >= kRemovedRing * bomb.radius) continue;
    float level = floorY;
    for (const float known : levels) {
      if (std::fabs(known - floorY) < 0.8f) level = known;
    }
    PlaneFracture fracture = fractureFlat(outline, std::fabs(bomb.position.y - level));
    if (fracture.removed.empty()) continue;
    WorldChange hole;
    hole.kind = WorldChangeKind::FloorHole;
    hole.target = static_cast<int>(i);
    glm::vec2 lo(1e30f), hi(-1e30f);
    for (const Poly& cell : fracture.removed) {
      for (const glm::vec2& p : cell) {
        lo = glm::min(lo, p + bombXZ);
        hi = glm::max(hi, p + bombXZ);
      }
    }
    hole.rect = glm::vec4(lo.x, lo.y, hi.x, hi.y);  // Where figures may have lost their footing.
    Obstacle patch;
    patch.bounds.min.y = patch.bounds.max.y = floorY;
    for (Poly& piece : fracture.kept) {
      if (PolygonArea(piece) < 0.05f || hole.pieces.size() >= kMaxPiecesPerChange) continue;
      hole.pieces.push_back(PrismFromFootprint(patch, fromBlast(std::move(piece))));
    }
    if (hole.pieces.empty()) {  // Nothing left: a rect hole swallowing the whole patch.
      Bounds2(outline, &lo, &hi);
      hole.rect = glm::vec4(lo.x - 0.1f, lo.y - 0.1f, hi.x + 0.1f, hi.y + 0.1f);
    }
    planChange(std::move(hole));
  }

  // 6. Debris: every knocked-out cell detaches as a chunk of exactly its own
  // shape, is thrown away from the blast and lands flat. The ballistics run
  // here -- on whatever authority is planning the blast, i.e. the server for
  // networked matches -- against the post-cut scratch scene (so chunks land
  // on surviving floors/ground), and the rest poses plus flight parameters
  // ship in one Debris change for clients to replay. The largest chunks
  // become solid rubble obstacles; smaller ones settle as litter that blocks
  // nothing; whatever finds no free spot to rest is dust.
  if (!chunks.empty()) {
    std::stable_sort(chunks.begin(), chunks.end(),
                     [](const Chunk& a, const Chunk& b) { return a.area > b.area; });
    Rng rng(Mix(bomb.seed, 0x6d2b79f5u));
    WorldChange debris;
    debris.kind = WorldChangeKind::Debris;
    debris.target = -1;
    struct Placed {
      glm::vec2 center;
      float radius;
      float y;
    };
    std::vector<Placed> placed;
    size_t solids = 0;
    for (const Chunk& chunk : chunks) {
      if (debris.pieces.size() >= kMaxDebrisPieces) break;
      const bool solid = solids < kMaxSolidChunks && chunk.area >= kSolidChunkArea;
      float chunkRadius = 0.0f;
      for (const glm::vec2& p : chunk.local) chunkRadius = std::max(chunkRadius, glm::length(p));
      const glm::vec3 radial = chunk.center - bomb.position;
      const float dist = glm::length(radial);
      const float push = std::clamp(1.0f - dist / bomb.radius, 0.0f, 1.0f);
      const glm::vec2 from(chunk.center.x, chunk.center.z);
      const float t = chunk.thickness;
      for (int attempt = 0; attempt < 5; ++attempt) {
        glm::vec2 dir;
        float speed, vy, belowY;
        if (chunk.flat) {
          // A slab cell drops, drifting a little away from the blast.
          const glm::vec2 away(radial.x, radial.z);
          dir = glm::length(away) > 1e-3f ? glm::normalize(away) : glm::vec2(1.0f, 0.0f);
          dir = Rotate2(dir, rng.Float(-1.2f, 1.2f));
          speed = rng.Float(0.1f, 0.9f) + 0.4f * attempt;
          vy = 0.0f;
          belowY = chunk.center.y - 0.5f * t - 0.35f;
        } else {
          // A wall cell is thrown through the wall, fanned out radially.
          glm::vec2 base = chunk.outward;
          if (dist > 1e-3f) base += glm::vec2(radial.x, radial.z) * (0.5f / dist);
          base = glm::length(base) > 1e-3f ? glm::normalize(base) : chunk.outward;
          dir = Rotate2(base, rng.Float(-0.45f, 0.45f));
          if (glm::dot(dir, chunk.outward) < 0.25f) dir = chunk.outward;
          speed = 1.3f + 3.2f * push + rng.Float(0.0f, 0.9f) + 0.7f * attempt;
          vy = 0.3f + 1.6f * push * std::max(0.0f, (dist > 1e-3f ? radial.y / dist : 0.0f) + 0.4f) +
               rng.Float(0.0f, 0.7f);
          belowY = chunk.center.y;
        }
        // Ballistic flight to the support under the landing spot. The
        // landing moves the support, so settle the pair with one re-solve.
        glm::vec2 land = from;
        float support = SupportHeight(scratch, land, belowY);
        float duration = 0.0f;
        for (int solve = 0; solve < 2; ++solve) {
          const float drop = std::max(0.0f, chunk.center.y - (support + 0.5f * t));
          duration = (vy + std::sqrt(vy * vy + 2.0f * kGravity * drop)) / kGravity;
          land = from + dir * (speed * duration);
          support = SupportHeight(scratch, land, belowY);
        }
        const float sigma = rng.Int(0, 1) ? 1.0f : -1.0f;
        const float fromYaw = chunk.flat ? 0.0f : std::atan2(chunk.along.y, chunk.along.x);
        const float restYaw = fromYaw + rng.Float(-0.9f, 0.9f) * (chunk.flat ? 0.5f : 1.0f);
        // A wall chunk must clear the opening (not plug the fresh breach);
        // every chunk stays on the map and comes to rest in the open, clear
        // of the chunks already down.
        if (!chunk.flat && glm::dot(land - from, chunk.outward) <
                               0.5f * chunk.thickness + chunkRadius + 0.1f) {
          continue;
        }
        const float margin = scene.mapHalfExtent - 0.4f;
        if (std::fabs(land.x) > margin || std::fabs(land.y) > margin) continue;
        if (RestSpotBlocked(scratch, land, support + 0.02f, support + t)) continue;
        bool crowded = false;
        for (const Unit& unit : scene.units) {  // Nobody gets entombed.
          crowded |= unit.alive && std::fabs(unit.position.y - support) < 2.0f &&
                     glm::distance(land, glm::vec2(unit.position.x, unit.position.z)) <
                         chunkRadius + 1.0f;
        }
        for (const Placed& other : placed) {
          crowded |= std::fabs(other.y - support) < 1.0f &&
                     glm::distance(land, other.center) < 0.8f * (chunkRadius + other.radius);
        }
        if (crowded) continue;

        // Rest pose: the cell lying flat. A wall cell tips over about its
        // along-axis (a quarter turn plus whole tumbles), a slab cell lands
        // as it left; DebrisPose plays the same motion backwards from here.
        const glm::vec2 axis(std::cos(restYaw), std::sin(restYaw));
        const glm::vec2 across = Perp(axis) * (chunk.flat ? 1.0f : sigma);
        Obstacle piece;
        piece.kind = ObstacleKind::Rubble;
        piece.building = -1;
        for (const glm::vec2& p : chunk.local) {
          piece.footprint.push_back(land + axis * p.x + across * p.y);
        }
        if (PolygonSignedArea(piece.footprint) < 0.0f) {
          std::reverse(piece.footprint.begin(), piece.footprint.end());
        }
        FitBoundsXZ(&piece);
        piece.bounds.min.y = support;
        piece.bounds.max.y = support + t;
        DebrisFlight flight;
        flight.from = chunk.center;
        flight.duration = duration;
        const float turns = duration > 0.8f ? 6.2831853f : 0.0f;
        flight.spin = sigma * (chunk.flat ? turns : 1.5707963f + turns);
        flight.fromYaw = fromYaw;
        flight.restYaw = restYaw;
        flight.solid = solid;
        debris.pieces.push_back(std::move(piece));
        debris.flights.push_back(flight);
        placed.push_back({land, chunkRadius, support});
        if (solid) ++solids;
        break;
      }
    }
    if (!debris.pieces.empty()) planChange(std::move(debris));
  }

  return plan;
}

}  // namespace tactics
