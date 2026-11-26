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
constexpr float kMaxWallThickness = 0.7f; // Thicker solids fracture in layers (3D).
// Elements cut per blast. Past it the remaining elements are left intact --
// never cut without their chunks: every knocked-out cell becomes debris.
constexpr size_t kMaxPlanCuts = 600;
constexpr size_t kMaxPiecesPerChange = 240;  // Standing pieces per cut element.
constexpr size_t kDebrisBatch = 60;       // Chunks per Debris change.
constexpr float kMinKeptArea = 0.01f;     // Smaller standing slivers break off as chunks.
constexpr float kDustArea = 0.004f;       // Chunks smaller than this are dust.
constexpr float kSolidChunkArea = 0.1f;   // Chunks at least this big are rubble obstacles.
// Drawn thickness of a walk surface (the renderer's deck/stair slab): what
// a chunk of an overpass deck, ramp or stair flight is made of.
constexpr float kSurfaceThickness = 0.45f;
constexpr float kFrameBar = 0.08f;        // Window-frame bar width / depth.
constexpr float kUnitClearance = 0.45f;   // A figure's body radius, for chunks coming to rest.
constexpr float kUnitBodyHeight = 1.9f;
constexpr float kMaxDebrisSeconds = 12.0f;
constexpr size_t kMaxDebrisKeys = 12 * 20 + 2;  // kMaxDebrisSeconds at kDebrisKeyDt, plus ends.

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
    // One draw per statement, here and below: the order of two draws in one
    // expression is up to the compiler, and every build must fracture alike.
    const float reach = rng->Float(1.0f, 1.18f);
    const float turn = rng->Float(0.15f, 0.85f);
    set.anchors.emplace_back(6.2831853f * (k + turn) / anchors, reach);
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
// left in plain pieces. `anchorTurn` rotates the anchor ring (layers of a thick
// solid crack differently). Empty when the blast knocks nothing out of it.
PlaneFracture FracturePlane(const Poly& solid, const std::vector<glm::vec2>& projected,
                            const SiteSet& set, float reach, float radius,
                            float anchorTurn = 0.0f) {
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
    const float angle = anchor.x + anchorTurn;
    add(planeRadius * anchor.y * glm::vec2(std::cos(angle), std::sin(angle)), false);
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
  return !o.dead && !o.navOnly && o.kind != ObstacleKind::Rubble && !IsStreetFixture(o.kind);
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

// Height of the plane through planar polygon `top` at (x, z).
float PlaneHeight(const std::vector<glm::vec3>& top, glm::vec2 xz) {
  glm::vec3 n(0.0f), c(0.0f);
  for (size_t i = 0; i < top.size(); ++i) {
    const glm::vec3& a = top[i];
    const glm::vec3& b = top[(i + 1) % top.size()];
    n.x += (a.y - b.y) * (a.z + b.z);
    n.y += (a.z - b.z) * (a.x + b.x);
    n.z += (a.x - b.x) * (a.y + b.y);
    c += a;
  }
  c /= static_cast<float>(top.size());
  if (std::fabs(n.y) < 1e-6f) return c.y;
  return c.y - (n.x * (xz.x - c.x) + n.z * (xz.y - c.z)) / n.y;
}

// One knocked-out cell on its way to becoming debris.
struct Chunk {
  Obstacle piece;        // The prism as it left the wall / slab (kind Rubble).
  glm::vec3 center{0.0f};  // The vertex mean at launch (world, mid-thickness).
  glm::vec2 along{1.0f, 0.0f};  // Wall's long axis (unit, XZ); +X for flat chunks.
  glm::vec2 outward{0.0f};      // Through the wall, away from the bomb (upright only).
  bool flat = false;     // A slab / layer / parapet cell: launched lying down.
  bool thrown = false;   // Flat, but blasted outward (a thick solid's cell, old rubble).
  float thickness = 0.2f;
  float area = 0.0f;
  float radius = 0.0f;   // Furthest outline vertex from the centre.
};

// Frame (local -> world) of a prism lying with its outline in the XZ plane:
// local x = world X, local y = world Z, local z = world -Y (a proper
// rotation), centred on `origin`.
glm::mat4 FlatFrame(const glm::vec3& origin) {
  glm::mat4 m(1.0f);
  m[0] = glm::vec4(1.0f, 0.0f, 0.0f, 0.0f);
  m[1] = glm::vec4(0.0f, 0.0f, 1.0f, 0.0f);
  m[2] = glm::vec4(0.0f, -1.0f, 0.0f, 0.0f);
  m[3] = glm::vec4(origin, 1.0f);
  return m;
}

// What debris flies through and comes to rest on: the scene after the cuts.
// Footprints are resolved once (the launch probes them).
class DebrisWorld {
 public:
  // Only what lies within `range` of `center` in XZ: as far as a chunk flies.
  DebrisWorld(const Scene& scene, glm::vec2 center, float range) : scene_(scene) {
    const auto near = [&](glm::vec2 lo, glm::vec2 hi) {
      return center.x >= lo.x - range && center.x <= hi.x + range && center.y >= lo.y - range &&
             center.y <= hi.y + range;
    };
    for (const Obstacle& o : scene.obstacles) {
      if (o.dead || o.navOnly) continue;
      if (!near({o.bounds.min.x, o.bounds.min.z}, {o.bounds.max.x, o.bounds.max.z})) continue;
      Solid s;
      s.footprint = ObstacleFootprint(o);
      if (s.footprint.size() < 3) continue;
      s.bounds = o.bounds;
      if (o.profile.size() >= 3 && o.footprint.size() >= 2) {
        const glm::vec2 ab = o.footprint[1] - o.footprint[0];
        const float len = glm::length(ab);
        if (len > 1e-5f) {
          s.profile = o.profile;
          s.a = o.footprint[0];
          s.d = ab / len;
        }
      }
      solids_.push_back(std::move(s));
      DebrisShape shape;
      if (DebrisPieceShape(o, &shape)) shapes_.push_back(std::move(shape));
    }
  }

  // Inside standing geometry (a wall piece's torn silhouette included).
  bool SolidAt(const glm::vec3& p) const {
    const glm::vec2 xz(p.x, p.z);
    for (const Solid& s : solids_) {
      if (p.y <= s.bounds.min.y + 0.02f || p.y >= s.bounds.max.y - 0.02f) continue;
      if (p.x < s.bounds.min.x || p.x > s.bounds.max.x || p.z < s.bounds.min.z ||
          p.z > s.bounds.max.z) {
        continue;
      }
      if (!PointInConvexPolygon(xz, s.footprint)) continue;
      if (!s.profile.empty() &&
          !PointInConvexPolygon(glm::vec2(glm::dot(xz - s.a, s.d), p.y), s.profile)) {
        continue;
      }
      return true;
    }
    return false;
  }

  // Everything chunks collide with besides each other: the standing prisms,
  // the figures (immovable posts: a chunk deflects off one, never lies on
  // it), floors / roofs / ramps and the ground.
  DebrisWorldDesc Describe() const {
    DebrisWorldDesc desc;
    desc.ground = &scene_.ground;
    desc.statics = shapes_;
    desc.extent = scene_.mapHalfExtent - 0.1f;
    for (const WalkSurface& w : scene_.walkSurfaces) {
      if (!w.dead && !w.climbProxy && w.vertices.size() >= 3) desc.decks.push_back(&w);
    }
    for (const Unit& unit : scene_.units) {
      if (!unit.alive) continue;
      DebrisShape post;
      for (int k = 0; k < 8; ++k) {
        const float a = k * 0.7853982f;
        post.outline.emplace_back(kUnitClearance * std::cos(a), kUnitClearance * std::sin(a));
      }
      post.thickness = 60.0f;
      post.pose = FlatFrame(glm::vec3(unit.position.x, unit.position.y + 29.95f, unit.position.z));
      desc.statics.push_back(std::move(post));
    }
    return desc;
  }

 private:
  struct Solid {
    AABB bounds;
    Poly footprint, profile;
    glm::vec2 a{0.0f}, d{1.0f, 0.0f};
  };
  const Scene& scene_;
  std::vector<Solid> solids_;
  std::vector<DebrisShape> shapes_;
};

}  // namespace

bool DebrisPieceShape(const Obstacle& piece, DebrisShape* out) {
  if (piece.rubble.outline.size() >= 3 && piece.rubble.thickness > 1e-4f) {
    out->outline = piece.rubble.outline;
    out->thickness = piece.rubble.thickness;
    out->pose = piece.rubble.pose;
    return true;
  }
  if (piece.profile.size() >= 3 && piece.footprint.size() == 4) {
    // A cell of a wall: the profile (u, y) in the footprint's plane.
    const glm::vec2 ab = piece.footprint[1] - piece.footprint[0];
    const glm::vec2 tv = piece.footprint[3] - piece.footprint[0];
    const float len = glm::length(ab), thickness = glm::length(tv);
    if (len < 1e-5f || thickness < 1e-4f) return false;
    const glm::vec2 d = ab / len;
    const glm::vec2 n = Perp(d);
    const glm::vec2 mean = VertexMean(piece.profile);
    out->outline.clear();
    for (const glm::vec2& p : piece.profile) out->outline.push_back(p - mean);
    if (PolygonSignedArea(out->outline) < 0.0f) std::reverse(out->outline.begin(), out->outline.end());
    out->thickness = thickness;
    const glm::vec2 xz = piece.footprint[0] + d * mean.x + n * (0.5f * thickness);
    glm::mat4 pose(1.0f);
    pose[0] = glm::vec4(d.x, 0.0f, d.y, 0.0f);
    pose[1] = glm::vec4(0.0f, 1.0f, 0.0f, 0.0f);
    pose[2] = glm::vec4(n.x, 0.0f, n.y, 0.0f);
    pose[3] = glm::vec4(xz.x, mean.y, xz.y, 1.0f);
    out->pose = pose;
    return true;
  }
  Poly fp = ObstacleFootprint(piece);
  if (fp.size() < 3 || !(piece.bounds.max.y - piece.bounds.min.y > 1e-4f)) return false;
  if (PolygonSignedArea(fp) < 0.0f) std::reverse(fp.begin(), fp.end());
  const glm::vec2 mean = VertexMean(fp);
  out->outline.clear();
  for (const glm::vec2& p : fp) out->outline.push_back(p - mean);
  out->thickness = piece.bounds.max.y - piece.bounds.min.y;
  out->pose = FlatFrame(glm::vec3(mean.x, 0.5f * (piece.bounds.min.y + piece.bounds.max.y), mean.y));
  return true;
}

uint32_t ExplosionSeed(uint32_t matchSeed, int round, int bombId) {
  uint32_t h = 2166136261u;  // FNV-1a offset basis.
  h = Mix(h, matchSeed);
  h = Mix(h, static_cast<uint32_t>(round));
  h = Mix(h, static_cast<uint32_t>(bombId));
  return h;
}

float DebrisRoundAge(const WorldChange& change, int round, bool executing, float elapsed) {
  if (!(change.at > 0.0f)) return -1.0f;
  if (change.round >= round) return std::max(0.0f, elapsed - change.at);
  // The flight outlasts its turn: physics is frozen while a turn is over, so
  // the chunk holds its pose at the turn's end and carries on with the same
  // motion once the next turn executes.
  const float whole = static_cast<float>(round - change.round - 1);
  return std::max(0.0f, constants::kRoundDuration - change.at) +
         whole * constants::kRoundDuration + (executing ? std::max(0.0f, elapsed) : 0.0f);
}

glm::mat4 DebrisPose(const DebrisFlight& flight, float t) {
  const std::vector<DebrisKey>& keys = flight.keys;
  if (keys.size() < 2) return glm::mat4(1.0f);
  const float at = std::clamp(t, 0.0f, flight.Duration()) / kDebrisKeyDt;
  const size_t i = std::min(static_cast<size_t>(at), keys.size() - 2);
  const float f = std::clamp(at - static_cast<float>(i), 0.0f, 1.0f);
  const glm::vec3 pos = glm::mix(keys[i].position, keys[i + 1].position, f);
  glm::quat a = keys[i].rotation, b = keys[i + 1].rotation;
  if (glm::dot(a, b) < 0.0f) b = -b;
  const glm::quat q = glm::normalize(glm::mix(a, b, f));
  const glm::quat turn = q * glm::inverse(keys[0].rotation);
  return glm::translate(glm::mat4(1.0f), pos) * glm::mat4_cast(turn) *
         glm::translate(glm::mat4(1.0f), -keys[0].position);
}

Obstacle DebrisRestObstacle(const Obstacle& piece, const DebrisFlight& flight) {
  Obstacle out;
  out.kind = ObstacleKind::Rubble;
  out.building = -1;
  DebrisShape shape;
  if (!DebrisPieceShape(piece, &shape)) return out;
  const glm::mat4 rest = DebrisPose(flight, flight.Duration()) * shape.pose;
  Poly xz;
  glm::vec3 lo(1e30f), hi(-1e30f);
  const float h = 0.5f * shape.thickness;
  for (const glm::vec2& p : shape.outline) {
    for (const float z : {-h, h}) {
      const glm::vec3 w(rest * glm::vec4(p.x, p.y, z, 1.0f));
      xz.emplace_back(w.x, w.z);
      lo = glm::min(lo, w);
      hi = glm::max(hi, w);
    }
  }
  out.footprint = ConvexHull2D(xz);
  if (out.footprint.size() < 3) {
    out.footprint = {{lo.x, lo.z}, {hi.x, lo.z}, {hi.x, hi.z}, {lo.x, hi.z}};
  }
  out.bounds = AABB{lo, hi};
  out.rubble.outline = shape.outline;
  out.rubble.thickness = shape.thickness;
  out.rubble.pose = rest;
  return out;
}

std::vector<WorldChange> PlanExplosion(const Scene& scene, const BombSpec& bomb) {
  std::vector<WorldChange> plan;
  if (!(bomb.radius > 0.1f)) return plan;
  Scene scratch = scene;  // Debris lands on what the cuts leave behind.
  // A cut that does not apply (or is past the budget) is not planned, and
  // its element stays whole: nothing is ever cut without its chunks.
  size_t cuts = 0;
  const auto planChange = [&](WorldChange change) {
    if (cuts >= kMaxPlanCuts) return false;
    if (!ApplyWorldChange(&scratch, change)) return false;
    plan.push_back(std::move(change));
    ++cuts;
    return true;
  };
  const glm::vec2 bombXZ(bomb.position.x, bomb.position.z);
  const float removedRadius = kRemovedRing * bomb.radius;
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
  // Common tail of every chunk: radius and the launch centre come from the
  // very shape clients will derive from the piece.
  const auto addChunk = [&](Chunk chunk) {
    DebrisShape shape;
    if (!DebrisPieceShape(chunk.piece, &shape)) return;
    chunk.thickness = shape.thickness;
    chunk.center = glm::vec3(shape.pose[3]);
    for (const glm::vec2& p : shape.outline) chunk.radius = std::max(chunk.radius, glm::length(p));
    chunks.push_back(std::move(chunk));
  };
  const auto flatChunk = [&](Poly world, float y0, float y1, bool thrown) {
    Chunk chunk;
    chunk.area = PolygonArea(world);
    if (chunk.area < kDustArea || !(y1 > y0)) return;
    if (PolygonSignedArea(world) < 0.0f) std::reverse(world.begin(), world.end());
    chunk.piece.kind = ObstacleKind::Rubble;
    chunk.piece.building = -1;
    chunk.piece.footprint = std::move(world);
    FitBoundsXZ(&chunk.piece);
    chunk.piece.bounds.min.y = y0;
    chunk.piece.bounds.max.y = y1;
    chunk.flat = true;
    chunk.thrown = thrown;
    addChunk(std::move(chunk));
  };
  // A cell in a wall plane: `shape` in (u, y) along `d` from `a`, its
  // mid-thickness line through `mid` (the point at u = 0).
  const auto wallChunk = [&](Poly shape, glm::vec2 mid, glm::vec2 d, glm::vec2 outward,
                             float thickness) {
    Chunk chunk;
    chunk.area = PolygonArea(shape);
    if (chunk.area < kDustArea) return;
    if (PolygonSignedArea(shape) < 0.0f) std::reverse(shape.begin(), shape.end());
    glm::vec2 lo, hi;
    Bounds2(shape, &lo, &hi);
    const glm::vec2 tv = Perp(d) * thickness;
    const glm::vec2 a = mid - 0.5f * tv;
    const glm::vec2 s0 = a + d * lo.x, s1 = a + d * hi.x;
    chunk.piece.kind = ObstacleKind::Rubble;
    chunk.piece.building = -1;
    chunk.piece.footprint = {s0, s1, s1 + tv, s0 + tv};
    FitBoundsXZ(&chunk.piece);
    chunk.piece.bounds.min.y = lo.y;
    chunk.piece.bounds.max.y = hi.y;
    for (const glm::vec2& p : shape) chunk.piece.profile.emplace_back(p.x - lo.x, p.y);
    chunk.along = d;
    chunk.outward = outward;
    addChunk(std::move(chunk));
  };
  // Rubble an earlier blast left, thrown again as the very body it is.
  const auto rubbleChunk = [&](const Obstacle& rubble) {
    Chunk chunk;
    chunk.piece = rubble;
    chunk.piece.dead = false;
    chunk.area = PolygonArea(rubble.rubble.outline);
    chunk.flat = true;
    chunk.thrown = true;
    addChunk(std::move(chunk));
  };

  // 1. Street fixtures in the blast are blown away; rubble an earlier blast
  // left there is thrown again.
  const size_t baseObstacles = scene.obstacles.size();
  for (size_t i = 0; i < baseObstacles; ++i) {
    const Obstacle& o = scene.obstacles[i];
    if (o.dead || !(IsStreetFixture(o.kind) || o.kind == ObstacleKind::Rubble)) continue;
    if (DistanceToPrism(bomb.position, o) >= bomb.radius) continue;
    WorldChange remove;
    remove.kind = WorldChangeKind::ObstacleRemoved;
    remove.target = static_cast<int>(i);
    const bool rubble = o.kind == ObstacleKind::Rubble;
    if (planChange(std::move(remove)) && rubble) {
      if (o.rubble.outline.size() >= 3) {
        rubbleChunk(o);
      } else {
        flatChunk(ObstacleFootprint(o), o.bounds.min.y, o.bounds.max.y, true);
      }
    }
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
    std::vector<Poly> loose;  // Knocked-out cells, plus slivers too small to stand.
    for (Poly& cell : fracture.removed) loose.push_back(toWall(std::move(cell)));
    for (Poly& cell : fracture.kept) {
      Poly shape = toWall(std::move(cell));
      glm::vec2 lo, hi;
      Bounds2(shape, &lo, &hi);
      if (PolygonArea(shape) < kMinKeptArea || hi.x - lo.x < 0.02f || hi.y - lo.y < 0.02f ||
          cut.pieces.size() >= kMaxPiecesPerChange) {
        loose.push_back(std::move(shape));
        continue;
      }
      cut.pieces.push_back(WallPiece(wall, frame, std::move(shape), true));
    }
    if (cut.pieces.empty()) cut.kind = WorldChangeKind::ObstacleRemoved;
    if (!planChange(std::move(cut))) continue;
    // Chunks fly out the far side; a bomb inside the wall itself throws
    // them both ways.
    const bool inside = std::fabs(side) <= 0.5f * frame.thickness;
    Rng sideRng(Mix(bomb.seed, static_cast<uint32_t>(i) * 2654435761u));
    for (const Poly& shape : loose) {
      const glm::vec2 outward =
          (inside ? sideRng.Int(0, 1) == 1 : side > 0.0f) ? -normal : normal;
      wallChunk(shape, midA, frame.d, outward, frame.thickness);
    }
  }

  // Horizontal cuts remember their level and outline, so whatever lies on a
  // cut slab or solid (its floor / roof surface) breaks along the very same
  // cells (step 6) and is not turned into a second set of chunks.
  struct FlatCut {
    float level;
    Poly outline;
  };
  std::vector<FlatCut> flatCuts;
  const auto findLevel = [&](float y, float fallback) {
    for (const FlatCut& cut : flatCuts) {
      if (std::fabs(cut.level - y) < 0.8f) return cut.level;
    }
    return fallback;
  };
  const auto fractureFlat = [&](const Poly& footprint, float reach) {
    return FracturePlane(toBlast(footprint), floorProjected, floorSites, reach, bomb.radius);
  };

  // 3. Ceiling / roof slabs: the same pre-crack in XZ.
  for (size_t i = 0; i < baseObstacles; ++i) {
    const Obstacle& slab = scene.obstacles[i];
    if (!SlabLike(slab)) continue;
    if (DistanceToPrism(bomb.position, slab) >= bomb.radius) continue;
    const float midY = 0.5f * (slab.bounds.min.y + slab.bounds.max.y);
    if (std::fabs(bomb.position.y - midY) >= removedRadius) continue;
    const float level = findLevel(midY, midY);
    const Poly footprint = ObstacleFootprint(slab);
    PlaneFracture fracture = fractureFlat(footprint, std::fabs(bomb.position.y - level));
    if (fracture.removed.empty()) continue;
    WorldChange cut;
    cut.kind = WorldChangeKind::ObstacleReplaced;
    cut.target = static_cast<int>(i);
    std::vector<Poly> loose;
    for (Poly& cell : fracture.removed) loose.push_back(fromBlast(std::move(cell)));
    for (Poly& piece : fracture.kept) {
      Poly world = fromBlast(std::move(piece));
      if (PolygonArea(world) < kMinKeptArea || cut.pieces.size() >= kMaxPiecesPerChange) {
        loose.push_back(std::move(world));
        continue;
      }
      cut.pieces.push_back(PrismFromFootprint(slab, std::move(world)));
    }
    if (cut.pieces.empty()) cut.kind = WorldChangeKind::ObstacleRemoved;
    const float y0 = slab.bounds.min.y, y1 = slab.bounds.max.y;
    if (!planChange(std::move(cut))) continue;
    flatCuts.push_back({level, footprint});
    for (const Poly& cell : loose) flatChunk(cell, y0, y1, false);
  }

  // 4. Thick solids -- overpass piers and their footings, solid building
  // blocks, crates -- fracture in 3D: the part within the blast's height is
  // split into layers, each layer pre-cracked in XZ with its own turn of the
  // crack pattern (the top layer with the pattern as it is, so the surface
  // on top tears along the same lines), and the cells within the blast come
  // out. What is above or below the blast's reach stays in one piece.
  const float layerHeight = std::clamp(0.3f * bomb.radius, 0.5f, 1.2f);
  for (size_t i = 0; i < baseObstacles; ++i) {
    const Obstacle& solid = scene.obstacles[i];
    Upright unused;
    if (!Fracturable(solid) || SlabLike(solid) || UprightOf(solid, &unused)) continue;
    const float y0 = solid.bounds.min.y, y1 = solid.bounds.max.y;
    if (y1 - y0 < 0.05f) continue;
    if (DistanceToPrism(bomb.position, solid) >= bomb.radius) continue;
    const Poly footprint = ObstacleFootprint(solid);
    if (footprint.size() < 3) continue;
    const float lo = std::max(y0, bomb.position.y - removedRadius);
    const float hi = std::min(y1, bomb.position.y + removedRadius);
    if (hi - lo < 0.02f) continue;
    // Layer boundaries, bottom-up: the blast's vertical reach, and the
    // layer lines anchored to the bomb's height in between.
    std::vector<float> cutsY = {y0};
    const auto addCut = [&](float y) {
      if (y - cutsY.back() >= 0.2f && y1 - y >= 0.2f) cutsY.push_back(y);
    };
    addCut(lo);
    const int k0 = static_cast<int>(std::ceil((lo - bomb.position.y) / layerHeight - 0.5f));
    for (int k = k0; bomb.position.y + (k + 0.5f) * layerHeight < hi; ++k) {
      addCut(bomb.position.y + (k + 0.5f) * layerHeight);
    }
    addCut(hi);
    cutsY.push_back(y1);

    WorldChange cut;
    cut.kind = WorldChangeKind::ObstacleReplaced;
    cut.target = static_cast<int>(i);
    struct Loose {
      Poly cell;
      float y0, y1;
    };
    std::vector<Loose> loose;
    bool topCut = false;
    float topLevel = y1;
    float wholeFrom = y0;  // Bottom of the run of uncut layers being merged.
    const auto flushWhole = [&](float upTo) {
      if (upTo - wholeFrom < 1e-3f) return;
      Obstacle piece = PrismFromFootprint(solid, footprint);
      piece.bounds.min.y = wholeFrom;
      piece.bounds.max.y = upTo;
      cut.pieces.push_back(std::move(piece));
    };
    for (size_t b = 0; b + 1 < cutsY.size(); ++b) {
      const float ya = cutsY[b], yb = cutsY[b + 1];
      const bool top = b + 2 == cutsY.size();
      PlaneFracture fracture;
      if (cut.pieces.size() + 40 < kMaxPiecesPerChange) {
        if (top) {
          topLevel = findLevel(y1, y1);
          fracture = fractureFlat(footprint, std::fabs(bomb.position.y - topLevel));
        } else {
          const float reach = bomb.position.y < ya   ? ya - bomb.position.y
                              : bomb.position.y > yb ? bomb.position.y - yb
                                                     : 0.0f;
          // Each layer turns the pattern by the golden angle.
          const float turn = 2.3999632f * static_cast<float>(b + 1);
          std::vector<glm::vec2> turned;
          for (const glm::vec2& p : floorProjected) turned.push_back(Rotate2(p, turn));
          fracture =
              FracturePlane(toBlast(footprint), turned, floorSites, reach, bomb.radius, turn);
        }
      }
      if (fracture.removed.empty()) continue;  // Stays part of the uncut run.
      flushWhole(ya);
      wholeFrom = yb;
      topCut |= top;
      for (Poly& cell : fracture.removed) loose.push_back({fromBlast(std::move(cell)), ya, yb});
      for (Poly& piece : fracture.kept) {
        Poly world = fromBlast(std::move(piece));
        if (PolygonArea(world) < kMinKeptArea) {
          loose.push_back({std::move(world), ya, yb});
          continue;
        }
        Obstacle prism = PrismFromFootprint(solid, std::move(world));
        prism.bounds.min.y = ya;
        prism.bounds.max.y = yb;
        cut.pieces.push_back(std::move(prism));
      }
    }
    if (loose.empty()) continue;
    flushWhole(y1);
    if (cut.pieces.empty()) cut.kind = WorldChangeKind::ObstacleRemoved;
    if (!planChange(std::move(cut))) continue;
    if (topCut) flatCuts.push_back({topLevel, footprint});
    for (const Loose& l : loose) flatChunk(l.cell, l.y0, l.y1, true);
  }

  // 5. Massing in the blast -- the parapet band wrapping the roof edge, roof
  // furniture, awnings, the overpass's pier caps and deck rails -- breaks
  // with the structure under it, along the same horizontal cells. A
  // sloped-top detail (deck rail) keeps its slope on every piece.
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
    PlaneFracture fracture = fractureFlat(detail.footprint, dy);
    if (fracture.removed.empty()) continue;
    WorldChange cut;
    cut.kind = WorldChangeKind::MassingCut;
    cut.target = static_cast<int>(i);
    std::vector<Poly> loose;
    for (Poly& cell : fracture.removed) loose.push_back(fromBlast(std::move(cell)));
    for (Poly& piece : fracture.kept) {
      Poly world = fromBlast(std::move(piece));
      if (PolygonArea(world) < kMinKeptArea || cut.pieces.size() >= kMaxPiecesPerChange) {
        loose.push_back(std::move(world));
        continue;
      }
      cut.pieces.push_back(PrismFromFootprint(prism, std::move(world)));
    }
    // With nothing left standing the whole detail goes (the rect form).
    glm::vec2 lo, hi;
    Bounds2(detail.footprint, &lo, &hi);
    cut.rect = glm::vec4(lo.x - 0.1f, lo.y - 0.1f, hi.x + 0.1f, hi.y + 0.1f);
    const std::vector<glm::vec3> top = detail.top;
    const float y0 = detail.y0, y1 = detail.y1, depth = detail.depth;
    if (!planChange(std::move(cut))) continue;
    for (const Poly& cell : loose) {
      if (top.size() >= 3 && depth > 0.0f) {
        const float y = PlaneHeight(top, VertexMean(cell));
        flatChunk(cell, y - depth, y, false);
      } else {
        flatChunk(cell, y0, y1, false);
      }
    }
  }

  // 6. Walk surfaces: building floors, roofs, overpass decks and ramps tear
  // along the same cells as the slab or solid under them (a figure left
  // without footing falls); a surface with nothing cut under it -- a deck, a
  // ramp, a stoop -- is its own concrete and sheds the chunks itself. A
  // stair flight hangs between its two ends: once the blast knocks a cell
  // out of it, the whole flight comes down in pieces. Terrain is not cut.
  const size_t baseSurfaces = scene.walkSurfaces.size();
  for (size_t i = 0; i < baseSurfaces; ++i) {
    const WalkSurface& floor = scene.walkSurfaces[i];
    if (floor.dead || floor.terrain || floor.vertices.size() < 3) continue;
    float yMin = floor.vertices.front().y, yMax = yMin;
    Poly outline;
    for (const glm::vec3& v : floor.vertices) {
      yMin = std::min(yMin, v.y);
      yMax = std::max(yMax, v.y);
      outline.emplace_back(v.x, v.z);
    }
    const bool flat = yMax - yMin <= 0.5f;
    const float floorY = 0.5f * (yMin + yMax);
    float reach;
    if (flat) {
      reach = std::fabs(bomb.position.y - findLevel(floorY, floorY));
    } else {
      reach = bomb.position.y < yMin   ? yMin - bomb.position.y
              : bomb.position.y > yMax ? bomb.position.y - yMax
                                       : 0.0f;
    }
    if (reach >= removedRadius) continue;
    glm::vec2 lo, hi;
    Bounds2(outline, &lo, &hi);
    if (bombXZ.x < lo.x - bomb.radius || bombXZ.x > hi.x + bomb.radius ||
        bombXZ.y < lo.y - bomb.radius || bombXZ.y > hi.y + bomb.radius) {
      continue;
    }
    PlaneFracture fracture = fractureFlat(outline, reach);
    if (fracture.removed.empty()) continue;
    // Chunks come from the surface itself unless the cut under it made them.
    const bool invisible = floor.climbProxy;
    const auto shed = [&](const Poly& cell) {
      if (invisible) return;
      const glm::vec2 mean = VertexMean(cell);
      if (flat) {
        for (const FlatCut& cut : flatCuts) {
          if (std::fabs(cut.level - floorY) < 0.8f && PointInConvexPolygon(mean, cut.outline)) {
            return;
          }
        }
      }
      const float y = SurfaceHeightAt(floor, mean.x, mean.y);
      flatChunk(cell, y - kSurfaceThickness, y, false);
    };
    std::vector<Poly> loose;
    for (Poly& cell : fracture.removed) loose.push_back(fromBlast(std::move(cell)));
    WorldChange hole;
    hole.target = static_cast<int>(i);
    if (floor.stair) {
      hole.kind = WorldChangeKind::SurfaceRemoved;
      for (Poly& piece : fracture.kept) loose.push_back(fromBlast(std::move(piece)));
    } else {
      hole.kind = WorldChangeKind::FloorHole;
      glm::vec2 holeLo(1e30f), holeHi(-1e30f);
      for (const Poly& cell : loose) {
        for (const glm::vec2& p : cell) {
          holeLo = glm::min(holeLo, p);
          holeHi = glm::max(holeHi, p);
        }
      }
      // Where figures may have lost their footing.
      hole.rect = glm::vec4(holeLo.x, holeLo.y, holeHi.x, holeHi.y);
      Obstacle patch;
      patch.bounds.min.y = patch.bounds.max.y = floorY;
      for (Poly& piece : fracture.kept) {
        Poly world = fromBlast(std::move(piece));
        if (PolygonArea(world) < 0.05f || hole.pieces.size() >= kMaxPiecesPerChange) {
          loose.push_back(std::move(world));  // Too small to stand on.
          continue;
        }
        hole.pieces.push_back(PrismFromFootprint(patch, std::move(world)));
      }
      if (hole.pieces.empty()) hole.kind = WorldChangeKind::SurfaceRemoved;  // Nothing left.
    }
    if (!planChange(std::move(hole))) continue;
    for (const Poly& cell : loose) shed(cell);
  }

  // 7. Window frames and door leaves within the blast are blown off the
  // wall: the opening dies and its bars leave as chunks. Frames further out
  // stay on whatever is left standing (see ObstacleReplaced).
  const size_t baseOpenings = scene.facadeOpenings.size();
  for (size_t i = 0; i < baseOpenings; ++i) {
    const FacadeOpening& opening = scene.facadeOpenings[i];
    if (opening.dead || opening.kind == FacadeOpening::Kind::Breach) continue;
    if (!(opening.u1 > opening.u0) || !(opening.y1 > opening.y0)) continue;
    glm::vec2 a, b;
    if (!FacadeOpeningEdge(scene, opening, &a, &b)) continue;
    const glm::vec2 d = glm::normalize(b - a);
    const glm::vec2 inward = Perp(d);  // The facade faces -inward.
    const float bombU = glm::dot(bombXZ - a, d);
    const float side = glm::dot(bombXZ - a, inward);
    const float du = bombU < opening.u0   ? opening.u0 - bombU
                     : bombU > opening.u1 ? bombU - opening.u1
                                          : 0.0f;
    const float dy = bomb.position.y < opening.y0   ? opening.y0 - bomb.position.y
                     : bomb.position.y > opening.y1 ? bomb.position.y - opening.y1
                                                    : 0.0f;
    if (du * du + dy * dy + side * side >= removedRadius * removedRadius) continue;
    WorldChange gone;
    gone.kind = WorldChangeKind::OpeningRemoved;
    gone.target = static_cast<int>(i);
    const bool realGap = opening.kind == FacadeOpening::Kind::Door && opening.building >= 0 &&
                         opening.building < static_cast<int>(scene.buildings.size()) &&
                         scene.buildings[opening.building].enterable;
    const FacadeOpening o = opening;
    if (!planChange(std::move(gone)) || realGap) continue;  // A doorway gap has no leaf.
    const glm::vec2 outward = side > 0.0f ? -inward : inward;  // Away from the bomb.
    const glm::vec2 mid = a - inward * (0.5f * kFrameBar);    // Proud of the facade.
    const auto bar = [&](float u0, float u1, float y0, float y1) {
      if (u1 - u0 < 1e-3f || y1 - y0 < 1e-3f) return;
      wallChunk({{u0, y0}, {u1, y0}, {u1, y1}, {u0, y1}}, mid, d, outward, kFrameBar);
    };
    const float w = std::min({kFrameBar, 0.25f * (o.u1 - o.u0), 0.25f * (o.y1 - o.y0)});
    if (o.kind == FacadeOpening::Kind::Door) {
      bar(o.u0, o.u1, o.y0, o.y1);  // The leaf.
    } else {
      bar(o.u0, o.u1, o.y0, o.y0 + w);
      bar(o.u0, o.u1, o.y1 - w, o.y1);
      bar(o.u0, o.u0 + w, o.y0 + w, o.y1 - w);
      bar(o.u1 - w, o.u1, o.y0 + w, o.y1 - w);
    }
  }

  // 8. Debris: every knocked-out cell detaches as a chunk of exactly its own
  // shape and is thrown by the blast. The chunks are simulated together as
  // rigid bodies (game/DebrisPhysics.h) -- on whatever authority is planning
  // the blast, i.e. the server for networked matches -- against the post-cut
  // scratch scene: they tumble, hit standing geometry, floors, the ground,
  // figures and each other, and come to rest in whatever pose the contacts
  // leave them in. Each chunk's sampled pose track ships in the Debris
  // changes for clients to replay verbatim. Settled chunks of kSolidChunkArea
  // and up are rubble obstacles (their vertical envelope blocks movement and
  // sight); smaller ones, and any that would entomb a figure, are litter.
  if (!chunks.empty()) {
    const DebrisWorld world(scratch, bombXZ, bomb.radius + 20.0f);
    Rng rng(Mix(bomb.seed, 0x6d2b79f5u));
    std::vector<DebrisBodyDesc> bodies;
    bodies.reserve(chunks.size());
    for (const Chunk& chunk : chunks) {
      DebrisBodyDesc body;
      DebrisPieceShape(chunk.piece, &body.shape);
      const glm::vec3 radial = chunk.center - bomb.position;
      const float dist = glm::length(radial);
      const float push = std::clamp(1.0f - dist / bomb.radius, 0.0f, 1.0f);
      const glm::vec2 away = glm::length(glm::vec2(radial.x, radial.z)) > 1e-3f
                                 ? glm::normalize(glm::vec2(radial.x, radial.z))
                                 : glm::vec2(1.0f, 0.0f);
      glm::vec2 dir;
      float speed, vy;
      glm::vec3 spin(0.0f);
      if (!chunk.flat) {
        // A wall cell is thrown through the wall, fanned out radially, and
        // tipped about its long axis by the shove.
        glm::vec2 base = chunk.outward;
        if (dist > 1e-3f) base += glm::vec2(radial.x, radial.z) * (0.5f / dist);
        base = glm::length(base) > 1e-3f ? glm::normalize(base) : chunk.outward;
        dir = Rotate2(base, rng.Float(-0.45f, 0.45f));
        if (glm::dot(dir, chunk.outward) < 0.25f) dir = chunk.outward;
        speed = 2.5f + 5.5f * push + rng.Float(0.0f, 1.5f);
        vy = 0.6f + 2.6f * push * std::max(0.0f, (dist > 1e-3f ? radial.y / dist : 0.0f) + 0.4f) +
             rng.Float(0.0f, 1.0f);
        const float tipSign = rng.Int(0, 1) ? 1.0f : -1.0f;
        const float tip = tipSign * (1.2f + 4.0f * push + rng.Float(0.0f, 2.0f));
        spin = glm::vec3(chunk.along.x, 0.0f, chunk.along.y) * tip +
               glm::vec3(0.0f, rng.Float(-1.5f, 1.5f), 0.0f);
      } else if (chunk.thrown) {
        // A cell of a thick solid (or old rubble) is blasted away from the
        // bomb -- or, on the face the blast hit, back out of the crater.
        dir = Rotate2(away, rng.Float(-0.5f, 0.5f));
        const glm::vec3 probe =
            chunk.center + glm::vec3(dir.x, 0.0f, dir.y) * (chunk.radius + 0.15f);
        if (world.SolidAt(probe)) dir = -dir;
        speed = 2.0f + 4.5f * push + rng.Float(0.0f, 1.2f);
        vy = 0.6f + 2.2f * push + rng.Float(0.0f, 0.9f);
        const float spinZ = rng.Float(-2.5f, 2.5f);
        const float spinY = rng.Float(-1.5f, 1.5f);
        const float spinX = rng.Float(-2.5f, 2.5f);
        spin = glm::vec3(spinX, spinY, spinZ) * (0.4f + push);
      } else {
        // A slab cell is punched down and out by the blast: a downward
        // impulse (stronger near the bomb) plus a drift away from it.
        dir = Rotate2(away, rng.Float(-1.2f, 1.2f));
        speed = 0.6f + 2.4f * push + rng.Float(0.0f, 0.8f);
        vy = -(0.8f + 3.0f * push + rng.Float(0.0f, 0.8f));
        const float spinZ = rng.Float(-0.8f, 0.8f);
        const float spinY = rng.Float(-0.8f, 0.8f);
        const float spinX = rng.Float(-0.8f, 0.8f);
        spin = glm::vec3(spinX, spinY, spinZ);
      }
      body.light = chunk.area < kSolidChunkArea;
      body.velocity = glm::vec3(dir.x * speed, vy, dir.y * speed);
      body.angularVelocity = spin;
      bodies.push_back(std::move(body));
    }
    const std::vector<DebrisTrack> tracks =
        SimulateDebris(world.Describe(), bodies, kMaxDebrisSeconds);

    WorldChange debris;
    debris.kind = WorldChangeKind::Debris;
    debris.target = -1;
    const auto flush = [&]() {
      if (debris.pieces.empty()) return;
      if (ApplyWorldChange(&scratch, debris)) plan.push_back(debris);
      debris.pieces.clear();
      debris.flights.clear();
    };
    for (size_t i = 0; i < chunks.size(); ++i) {
      DebrisFlight flight;
      flight.keys = tracks[i].keys;
      if (flight.keys.size() > kMaxDebrisKeys) flight.keys.resize(kMaxDebrisKeys);
      // Solid only if it came to rest, is big enough to matter, and does not
      // lie over a figure (no figure is ever entombed in an obstacle) nor
      // pin one where it stands: the navmesh pads every blocking footprint
      // by kAgentRadius, so a chunk whose walk-band silhouette rests within
      // that padding (plus one 0.25 routing step of clearance) of a figure
      // would swallow its start node and leave it with no legal move at all.
      bool solid = chunks[i].area >= kSolidChunkArea && tracks[i].settled;
      if (solid) {
        const Obstacle rest = DebrisRestObstacle(chunks[i].piece, flight);
        float ground = 0.0f;
        if (!scratch.ground.Empty()) {
          for (const glm::vec2& v : rest.footprint) {
            ground = std::max(ground, scratch.ground.HeightAt(v.x, v.y));
          }
        }
        const Poly band = RubbleBandFootprint(rest, ground + constants::kStepOverHeight,
                                              ground + constants::kUnitHeight);
        const Poly fat = ExpandConvexPolygon(rest.footprint, 0.2f);
        const Poly pinning =
            band.empty() ? band : ExpandConvexPolygon(band, constants::kAgentRadius + 0.25f);
        for (const Unit& unit : scratch.units) {
          if (!unit.alive) continue;
          if (rest.bounds.min.y > unit.position.y + kUnitBodyHeight ||
              rest.bounds.max.y < unit.position.y - 0.05f) {
            continue;
          }
          const glm::vec2 foot(unit.position.x, unit.position.z);
          if (PointInConvexPolygon(foot, fat) ||
              (!pinning.empty() && PointInConvexPolygon(foot, pinning))) {
            solid = false;
          }
        }
      }
      flight.solid = solid;
      debris.pieces.push_back(chunks[i].piece);
      debris.flights.push_back(std::move(flight));
      if (debris.pieces.size() >= kDebrisBatch) flush();
    }
    flush();
  }

  return plan;
}

}  // namespace tactics
