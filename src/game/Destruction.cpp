#include "game/Destruction.h"

#include <algorithm>
#include <cmath>
#include <random>

#include "game/Geometry.h"

namespace tactics {
namespace {

// Fracture tuning. The grid cell is the fracture's sampling resolution: the
// removed region is rasterized per column, then the column runs are
// simplified into a jagged outline whose edges follow the seeded Voronoi
// cell boundaries -- torn edges at random angles, not stair-steps.
constexpr float kFractureCell = 0.25f;
constexpr int kMaxHolesPerWall = 4;       // Fracture outlines per wall piece.
constexpr size_t kMaxChainPoints = 7;     // Outline vertices per chain (angle budget).
constexpr size_t kMaxPlanChanges = 48;    // Overall piece budget per blast.
constexpr size_t kMaxDebrisChunks = 10;   // Rubble chunks per blast.
constexpr float kFloorSnap = 0.35f;       // Holes this close to the wall base/top reach it.
constexpr float kGravity = 9.81f;         // Debris ballistics.
// Graded rings: sites inside kShardRing of the blast radius are removed
// (the inner ones as dust, the rest as small shards -- the split will feed
// cosmetic debris later); sites beyond stay standing as cracked chunks.
constexpr float kShardRing = 0.85f;

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

// A thin upright prism a blast can punch a hole through. Structural piers,
// unseen nav walls and street items are not walls.
bool WallLike(const Obstacle& o) {
  if (o.dead || o.navOnly) return false;
  if (o.kind == ObstacleKind::Pier || o.kind == ObstacleKind::Rubble || IsStreetFixture(o.kind)) {
    return false;
  }
  const std::vector<glm::vec2> f = ObstacleFootprint(o);
  if (f.size() != 4) return false;
  const float e0 = glm::distance(f[0], f[1]);
  const float e1 = glm::distance(f[1], f[2]);
  const float thickness = std::min(e0, e1);
  const float length = std::max(e0, e1);
  if (thickness < 1e-3f || thickness > 0.7f || length < 0.6f) return false;
  // Short strips count too: CutWindows pre-splits a facade into jambs,
  // sills and heads, and the head band under the roofline must fracture
  // with the rest of the wall or every blast leaves it floating over the
  // hole (the "wrap-around" of the breach screenshots).
  return o.bounds.max.y - o.bounds.min.y >= 0.35f;
}

// Footprint edge index of the wall's long face (0 or 1; 2/3 mirror them).
int LongEdge(const std::vector<glm::vec2>& f) {
  return glm::distance(f[0], f[1]) >= glm::distance(f[1], f[2]) ? 0 : 1;
}

// A thin horizontal plate (ceiling or roof slab) a blast can hole.
bool SlabLike(const Obstacle& o) {
  if (o.dead || o.navOnly) return false;
  if (o.kind == ObstacleKind::Pier || o.kind == ObstacleKind::Rubble || IsStreetFixture(o.kind)) {
    return false;
  }
  if (o.bounds.max.y - o.bounds.min.y > 0.7f) return false;
  const std::vector<glm::vec2> f = ObstacleFootprint(o);
  return f.size() >= 3 && PolygonArea(f) >= 1.0f;
}

// Ramer-Douglas-Peucker polyline simplification, raising the tolerance until
// the chain fits `maxPoints`. Keeps a subset of the input points, so a
// u-monotone chain stays u-monotone.
void SimplifyChain(std::vector<glm::vec2>* chain, float tolerance, size_t maxPoints) {
  while (chain->size() > 2) {
    const size_t n = chain->size();
    std::vector<char> keep(n, 0);
    keep[0] = 1;
    keep[n - 1] = 1;
    std::vector<std::pair<size_t, size_t>> stack{{0, n - 1}};
    while (!stack.empty()) {
      const auto [lo, hi] = stack.back();
      stack.pop_back();
      if (hi <= lo + 1) continue;
      const glm::vec2 a = (*chain)[lo], b = (*chain)[hi];
      const glm::vec2 ab = b - a;
      const float len = glm::length(ab);
      float worst = 0.0f;
      size_t at = lo;
      for (size_t i = lo + 1; i < hi; ++i) {
        const glm::vec2 ap = (*chain)[i] - a;
        const float dist = len > 1e-6f ? std::fabs(ab.x * ap.y - ab.y * ap.x) / len
                                       : glm::length(ap);
        if (dist > worst) {
          worst = dist;
          at = i;
        }
      }
      if (worst > tolerance && at > lo) {
        keep[at] = 1;
        stack.push_back({lo, at});
        stack.push_back({at, hi});
      }
    }
    std::vector<glm::vec2> out;
    for (size_t i = 0; i < chain->size(); ++i) {
      if (keep[i]) out.push_back((*chain)[i]);
    }
    if (out.size() <= maxPoints) {
      *chain = std::move(out);
      return;
    }
    tolerance *= 1.6f;
  }
}

// Graded Voronoi fracture of one wall piece in its own plane: ~30-40 seeded
// sites at distance r * random^2 (dense near the center), cells classified
// by ring -- core (dust) and small-shard cells are removed, the outer large
// chunks stay standing. The removed region is rasterized per grid column
// (one y run per column), then each connected run of columns becomes one
// hole outline: the runs' lower and upper envelopes simplified into short
// jagged chains, pinched together at the ends -- a u-monotone polygon in the
// wall plane whose torn edges sit at the fracture's own angles. Returned
// outlines are closed CCW (u, y) polygons.
std::vector<std::vector<glm::vec2>> FractureWall(const Obstacle& wall, const BombSpec& bomb,
                                                 uint32_t stream) {
  const std::vector<glm::vec2> f = ObstacleFootprint(wall);
  const int e = LongEdge(f);
  const glm::vec2 a = f[e], b = f[(e + 1) % 4];
  const float len = glm::distance(a, b);
  if (len < 0.3f) return {};
  const glm::vec2 d = (b - a) / len;
  const glm::vec2 n(-d.y, d.x);
  const glm::vec2 blastXZ(bomb.position.x, bomb.position.z);
  const float thickness = glm::distance(f[(e + 1) % 4], f[(e + 2) % 4]);
  // Distance to the wall's mid-thickness plane; the in-plane blast disc.
  const float dPlane = std::fabs(glm::dot(blastXZ - a, n)) - 0.5f * thickness;
  const float reach = std::max(0.0f, dPlane);
  if (reach >= bomb.radius * 0.95f) return {};
  const float r = std::sqrt(std::max(0.01f, bomb.radius * bomb.radius - reach * reach));
  const float uc = glm::dot(blastXZ - a, d);
  const float yc = bomb.position.y;
  const float ylo = wall.bounds.min.y, yhi = wall.bounds.max.y;
  if (uc + r <= 0.0f || uc - r >= len || yc + r <= ylo || yc - r >= yhi) return {};

  Rng rng(stream);
  struct Site {
    glm::vec2 p;
    bool removed;
  };
  std::vector<Site> sites;
  const int count = 30 + rng.Int(0, 10);
  sites.reserve(count);
  for (int i = 0; i < count; ++i) {
    const float angle = rng.Float(0.0f, 6.2831853f);
    const float unit = rng.Unit01();
    const float rho = r * unit * unit;  // distance = R x random^2: graded density.
    sites.push_back({glm::vec2(uc + rho * std::cos(angle), yc + rho * std::sin(angle)),
                     rho < r * kShardRing});
  }

  const auto cellRemoved = [&](float u, float y) {
    const glm::vec2 c(u, y);
    const float rho = glm::distance(c, glm::vec2(uc, yc));
    if (rho >= r) return false;
    if (rho < r * 0.35f) return true;  // The core always opens.
    float best = 1e30f;
    bool removed = false;
    for (const Site& s : sites) {
      const float dist = glm::distance(c, s.p);
      if (dist < best) {
        best = dist;
        removed = s.removed;
      }
    }
    return removed;
  };

  // Columns on the absolute fracture grid, so overlapping blasts agree.
  const int ku0 = static_cast<int>(std::floor(std::max(0.0f, uc - r) / kFractureCell));
  const int ku1 = static_cast<int>(std::ceil(std::min(len, uc + r) / kFractureCell));
  const int ky0 = static_cast<int>(std::floor(std::max(0.0f, yc - r - ylo) / kFractureCell));
  const int ky1 = static_cast<int>(std::ceil(std::min(yhi - ylo, yc + r - ylo) / kFractureCell));
  struct Run {
    int column;
    float lo, hi;
  };
  std::vector<std::vector<Run>> segments;  // Connected column runs.
  for (int ku = ku0; ku < ku1; ++ku) {
    const float u = (ku + 0.5f) * kFractureCell;
    if (u <= 0.0f || u >= len) continue;
    float runLo = 1e30f, runHi = -1e30f;
    for (int ky = ky0; ky < ky1; ++ky) {
      const float y = ylo + (ky + 0.5f) * kFractureCell;
      if (y <= ylo || y >= yhi) continue;
      if (!cellRemoved(u, y)) continue;
      runLo = std::min(runLo, ylo + ky * kFractureCell);
      runHi = std::max(runHi, ylo + (ky + 1) * kFractureCell);
    }
    if (runHi <= runLo) continue;
    if (segments.empty() || segments.back().back().column != ku - 1) segments.emplace_back();
    segments.back().push_back({ku, runLo, runHi});
  }
  // Hole budget: keep the widest outlines (removing less wall than the cells
  // suggested is fine -- the applied outline is the hole).
  while (static_cast<int>(segments.size()) > kMaxHolesPerWall) {
    size_t narrowest = 0;
    for (size_t i = 1; i < segments.size(); ++i) {
      if (segments[i].size() < segments[narrowest].size()) narrowest = i;
    }
    segments.erase(segments.begin() + narrowest);
  }

  std::vector<std::vector<glm::vec2>> outlines;
  for (const std::vector<Run>& segment : segments) {
    // Lower/upper envelopes of the column runs, pinched at the segment ends.
    const float uStart = std::max(0.0f, segment.front().column * kFractureCell);
    const float uEnd = std::min(len, (segment.back().column + 1) * kFractureCell);
    const glm::vec2 pinchL(uStart, 0.5f * (segment.front().lo + segment.front().hi));
    const glm::vec2 pinchR(uEnd, 0.5f * (segment.back().lo + segment.back().hi));
    std::vector<glm::vec2> lower{pinchL}, upper{pinchL};
    for (const Run& run : segment) {
      const float u = std::clamp((run.column + 0.5f) * kFractureCell, uStart, uEnd);
      lower.emplace_back(u, run.lo);
      upper.emplace_back(u, run.hi);
    }
    lower.push_back(pinchR);
    upper.push_back(pinchR);
    SimplifyChain(&lower, 0.5f * kFractureCell, kMaxChainPoints);
    SimplifyChain(&upper, 0.5f * kFractureCell, kMaxChainPoints);
    const auto snap = [&](std::vector<glm::vec2>* chain) {
      for (glm::vec2& p : *chain) {
        if (p.y - ylo < kFloorSnap) p.y = ylo;  // Reaches the floor: a doorway.
        if (yhi - p.y < kFloorSnap) p.y = yhi;
        p.y = std::clamp(p.y, ylo, yhi);
      }
    };
    snap(&lower);
    snap(&upper);
    // Closed CCW outline: lower chain left to right, then the upper chain's
    // interior back (the pinch endpoints are shared by both chains).
    std::vector<glm::vec2> outline = lower;
    for (size_t i = upper.size() - 1; i-- > 1;) outline.push_back(upper[i]);
    glm::vec2 lo = outline.front(), hi = outline.front();
    for (const glm::vec2& p : outline) {
      lo = glm::min(lo, p);
      hi = glm::max(hi, p);
    }
    if (outline.size() < 3 || hi.x - lo.x < 0.2f || hi.y - lo.y < 0.2f ||
        PolygonArea(outline) < 0.15f) {
      continue;
    }
    outlines.push_back(std::move(outline));
  }
  return outlines;
}

// The live wall piece of `scratch` containing `point` (XZ inside the
// footprint, y inside the bounds) whose long face runs along `along` (so a
// perpendicular wall overlapping at a corner is never hit), or -1. Later
// strips of a fractured wall land on the jamb pieces earlier strips appended.
int FindPieceAt(const Scene& scratch, const glm::vec3& point, const glm::vec2& along) {
  for (size_t i = 0; i < scratch.obstacles.size(); ++i) {
    const Obstacle& o = scratch.obstacles[i];
    if (!WallLike(o)) continue;
    if (point.y < o.bounds.min.y - 1e-3f || point.y > o.bounds.max.y + 1e-3f) continue;
    const std::vector<glm::vec2> f = ObstacleFootprint(o);
    const int e = LongEdge(f);
    const glm::vec2 dir = glm::normalize(f[(e + 1) % 4] - f[e]);
    if (std::fabs(glm::dot(dir, along)) < 0.9f) continue;
    if (PointInConvexPolygon({point.x, point.z}, f, 1e-3f)) return static_cast<int>(i);
  }
  return -1;
}

// Clips `footprint` to the part outside rect (x0,z0)-(x1,z1), as up to four
// disjoint convex pieces.
std::vector<std::vector<glm::vec2>> ClipAroundRect(const std::vector<glm::vec2>& footprint,
                                                   float x0, float z0, float x1, float z1) {
  const auto clip = [](const std::vector<glm::vec2>& poly, glm::vec2 normal, float offset) {
    std::vector<glm::vec2> out;
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
  };
  std::vector<std::vector<glm::vec2>> pieces;
  pieces.push_back(clip(footprint, {1.0f, 0.0f}, x0));   // x <= x0.
  pieces.push_back(clip(footprint, {-1.0f, 0.0f}, -x1)); // x >= x1.
  std::vector<glm::vec2> mid = clip(footprint, {-1.0f, 0.0f}, -x0);
  mid = clip(mid, {1.0f, 0.0f}, x1);
  pieces.push_back(clip(mid, {0.0f, 1.0f}, z0));   // z <= z0 strip.
  pieces.push_back(clip(mid, {0.0f, -1.0f}, -z1)); // z >= z1 strip.
  pieces.erase(std::remove_if(pieces.begin(), pieces.end(),
                              [](const std::vector<glm::vec2>& p) {
                                return p.size() < 3 || PolygonArea(p) < 0.01f;
                              }),
               pieces.end());
  return pieces;
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

glm::vec2 Rotate2(glm::vec2 v, float angle) {
  const float c = std::cos(angle), s = std::sin(angle);
  return {v.x * c - v.y * s, v.x * s + v.y * c};
}

Obstacle PrismFromFootprint(const Obstacle& base, std::vector<glm::vec2> footprint) {
  Obstacle piece = base;
  piece.dead = false;
  piece.footprint = std::move(footprint);
  piece.bounds.min.x = piece.bounds.max.x = piece.footprint.front().x;
  piece.bounds.min.z = piece.bounds.max.z = piece.footprint.front().y;
  for (const glm::vec2& v : piece.footprint) {
    piece.bounds.min.x = std::min(piece.bounds.min.x, v.x);
    piece.bounds.max.x = std::max(piece.bounds.max.x, v.x);
    piece.bounds.min.z = std::min(piece.bounds.min.z, v.y);
    piece.bounds.max.z = std::max(piece.bounds.max.z, v.y);
  }
  return piece;
}

}  // namespace

uint32_t ExplosionSeed(uint32_t matchSeed, int round, int bombId) {
  uint32_t h = 2166136261u;  // FNV-1a offset basis.
  h = Mix(h, matchSeed);
  h = Mix(h, static_cast<uint32_t>(round));
  h = Mix(h, static_cast<uint32_t>(bombId));
  return h;
}

std::vector<WorldChange> PlanExplosion(const Scene& scene, const BombSpec& bomb) {
  std::vector<WorldChange> plan;
  if (!(bomb.radius > 0.1f)) return plan;
  Scene scratch = scene;  // Chained targets: later changes hit earlier pieces.
  const auto planChange = [&](WorldChange change) {
    if (plan.size() >= kMaxPlanChanges) return false;
    if (!ApplyWorldChange(&scratch, change)) return false;
    plan.push_back(std::move(change));
    return true;
  };

  // Openings cut by this plan, remembered as debris launch sites.
  struct DebrisSource {
    glm::vec3 pos;        // A point inside the opening (world, mid-thickness).
    glm::vec2 along;      // Horizontal unit along the wall's long axis.
    glm::vec2 outward;    // Horizontal unit away from the bomb, through the wall.
    float thickness = 0.2f;
    float area = 0.5f;    // Opening area: scales the chunk count.
  };
  std::vector<DebrisSource> debrisSources;

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

  // 2. Walls: seeded graded Voronoi fracture, one jagged outline per
  // connected opening. The per-wall stream hashes the pristine obstacle
  // index, so the fracture of one wall never depends on which other walls
  // the blast also touched.
  for (size_t i = 0; i < baseObstacles; ++i) {
    const Obstacle& wall = scene.obstacles[i];
    if (!WallLike(wall)) continue;
    if (DistanceToPrism(bomb.position, wall) >= bomb.radius) continue;
    const std::vector<std::vector<glm::vec2>> outlines =
        FractureWall(wall, bomb, Mix(bomb.seed, static_cast<uint32_t>(i) * 2654435761u));
    if (outlines.empty()) continue;
    const std::vector<glm::vec2> f = ObstacleFootprint(wall);
    const int e = LongEdge(f);
    const glm::vec2 a = f[e];
    const glm::vec2 d = glm::normalize(f[(e + 1) % 4] - f[e]);
    const glm::vec2 inward = 0.5f * (f[(e + 3) % 4] - f[e]);
    for (const std::vector<glm::vec2>& outline : outlines) {
      glm::vec2 lo = outline.front(), hi = outline.front();
      for (const glm::vec2& p : outline) {
        lo = glm::min(lo, p);
        hi = glm::max(hi, p);
      }
      const glm::vec2 mid2 = a + d * (0.5f * (lo.x + hi.x)) + inward;
      const glm::vec3 mid(mid2.x, 0.5f * (lo.y + hi.y), mid2.y);
      const int target = FindPieceAt(scratch, mid, d);
      if (target < 0) continue;
      const Obstacle& piece = scratch.obstacles[target];
      const std::vector<glm::vec2> pf = ObstacleFootprint(piece);
      if (pf.size() != 4) continue;
      const int pe = LongEdge(pf);
      const glm::vec2 pa = pf[pe];
      const glm::vec2 pd = glm::normalize(pf[(pe + 1) % 4] - pf[pe]);
      WorldChange hole;
      hole.kind = WorldChangeKind::WallHole;
      hole.target = target;
      hole.wallEdge = pe;
      hole.u0 = 1e30f;
      hole.u1 = -1e30f;
      hole.y0 = 1e30f;
      hole.y1 = -1e30f;
      hole.polygon.reserve(outline.size());
      for (const glm::vec2& p : outline) {
        // Map from the pristine wall's edge frame into the target piece's.
        const glm::vec2 world = a + d * p.x;
        const float pu = glm::dot(world - pa, pd);
        hole.polygon.emplace_back(pu, p.y, 0.0f);
        hole.u0 = std::min(hole.u0, pu);
        hole.u1 = std::max(hole.u1, pu);
        hole.y0 = std::min(hole.y0, p.y);
        hole.y1 = std::max(hole.y1, p.y);
      }
      const float area = PolygonArea(outline);
      if (planChange(std::move(hole))) {
        glm::vec2 outward(-d.y, d.x);
        if (glm::dot(mid2 - glm::vec2(bomb.position.x, bomb.position.z), outward) < 0.0f) {
          outward = -outward;
        }
        debrisSources.push_back(
            {mid, d, outward, std::max(0.1f, 2.0f * glm::length(inward)), area});
      }
    }
  }

  // 3. Ceiling / roof slabs near the blast get a matching hole.
  for (size_t i = 0; i < baseObstacles; ++i) {
    const Obstacle& slab = scene.obstacles[i];
    if (!SlabLike(slab) || slab.building < 0) continue;
    const float midY = 0.5f * (slab.bounds.min.y + slab.bounds.max.y);
    const float dy = std::fabs(bomb.position.y - midY);
    if (dy >= bomb.radius * 0.8f) continue;
    if (DistanceToPrism(bomb.position, slab) >= bomb.radius) continue;
    const float inPlane = std::sqrt(std::max(0.0f, bomb.radius * bomb.radius - dy * dy));
    const float half = std::min(inPlane * 0.6f, 4.0f);
    if (half < 0.4f) continue;
    const float x0 = bomb.position.x - half, x1 = bomb.position.x + half;
    const float z0 = bomb.position.z - half, z1 = bomb.position.z + half;
    const std::vector<glm::vec2> f = ObstacleFootprint(slab);
    std::vector<std::vector<glm::vec2>> pieces = ClipAroundRect(f, x0, z0, x1, z1);
    if (PolygonArea(f) - 0.05f <= [&] {
          float sum = 0.0f;
          for (const auto& p : pieces) sum += PolygonArea(p);
          return sum;
        }()) {
      continue;  // Hole misses the slab.
    }
    WorldChange cut;
    cut.kind = pieces.empty() ? WorldChangeKind::ObstacleRemoved : WorldChangeKind::ObstacleReplaced;
    cut.target = static_cast<int>(i);
    for (std::vector<glm::vec2>& piece : pieces) {
      cut.pieces.push_back(PrismFromFootprint(slab, std::move(piece)));
    }
    planChange(std::move(cut));
  }

  // 3.5. Building massing in the blast -- the parapet band wrapping the roof
  // edge, roof furniture, awnings -- breaks with the wall below it: each
  // detail the blast sphere reaches is cut around the blast box at its own
  // height (a detail entirely inside it is removed whole).
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
    const float inPlane = std::sqrt(std::max(0.0f, bomb.radius * bomb.radius - dy * dy));
    const float half = std::min(inPlane * 0.8f, 4.5f);
    if (half < 0.3f) continue;
    WorldChange cut;
    cut.kind = WorldChangeKind::MassingCut;
    cut.target = static_cast<int>(i);
    cut.rect = glm::vec4(bomb.position.x - half, bomb.position.z - half,
                         bomb.position.x + half, bomb.position.z + half);
    if (planChange(std::move(cut))) {
      glm::vec2 lo = detail.footprint.front(), hi = lo;
      for (const glm::vec2& p : detail.footprint) {
        lo = glm::min(lo, p);
        hi = glm::max(hi, p);
      }
      const glm::vec2 center = glm::clamp(glm::vec2(bomb.position.x, bomb.position.z), lo, hi);
      glm::vec2 outward = center - glm::vec2(bomb.position.x, bomb.position.z);
      const float outLen = glm::length(outward);
      outward = outLen > 1e-4f ? outward / outLen : glm::vec2(1.0f, 0.0f);
      const glm::vec2 edge = hi - lo;
      const glm::vec2 along =
          edge.x >= edge.y ? glm::vec2(1.0f, 0.0f) : glm::vec2(0.0f, 1.0f);
      debrisSources.push_back({glm::vec3(center.x, 0.5f * (detail.y0 + detail.y1), center.y),
                               along, outward, 0.2f, 0.6f});
    }
  }

  // 4. Building floor surfaces: a hole a figure can fall through.
  const size_t baseSurfaces = scene.walkSurfaces.size();
  for (size_t i = 0; i < baseSurfaces; ++i) {
    const WalkSurface& floor = scene.walkSurfaces[i];
    if (floor.dead || floor.building < 0 || floor.stair || floor.climbProxy ||
        floor.vertices.size() < 3) {
      continue;
    }
    float yMin = floor.vertices.front().y, yMax = yMin;
    for (const glm::vec3& v : floor.vertices) {
      yMin = std::min(yMin, v.y);
      yMax = std::max(yMax, v.y);
    }
    if (yMax - yMin > 0.5f) continue;  // Flat floors only.
    const float dy = std::fabs(bomb.position.y - 0.5f * (yMin + yMax));
    if (dy >= bomb.radius * 0.8f) continue;
    const float inPlane = std::sqrt(std::max(0.0f, bomb.radius * bomb.radius - dy * dy));
    const float half = std::min(inPlane * 0.6f, 4.0f);
    if (half < 0.4f) continue;
    WorldChange hole;
    hole.kind = WorldChangeKind::FloorHole;
    hole.target = static_cast<int>(i);
    hole.rect = glm::vec4(bomb.position.x - half, bomb.position.z - half,
                          bomb.position.x + half, bomb.position.z + half);
    planChange(std::move(hole));  // Validation (hole actually overlaps) is the apply's.
  }

  // 5. Debris: chunks knocked out of the openings detach, fly away from the
  // blast and land as rubble obstacles. The ballistics run here -- on
  // whatever authority is planning the blast, i.e. the server for networked
  // matches -- against the post-cut scratch scene (so chunks land on
  // surviving floors/ground), and the resulting rest poses plus flight
  // parameters ship in one Debris change for clients to replay.
  if (!debrisSources.empty()) {
    Rng rng(Mix(bomb.seed, 0x6d2b79f5u));
    WorldChange debris;
    debris.kind = WorldChangeKind::Debris;
    debris.target = -1;
    std::vector<glm::vec2> placed;  // Rest centers, to spread chunks out.
    for (const DebrisSource& src : debrisSources) {
      const int chunks = std::clamp(1 + static_cast<int>(src.area / 2.0f), 1, 3);
      for (int chunk = 0; chunk < chunks; ++chunk) {
        if (debris.pieces.size() >= kMaxDebrisChunks) break;
        for (int attempt = 0; attempt < 4; ++attempt) {
          const float w = rng.Float(0.3f, 0.65f);   // Along the wall at launch.
          const float h = rng.Float(0.25f, 0.5f);   // Vertical at launch, flat at rest.
          const float t = std::clamp(src.thickness, 0.08f, 0.35f);
          const glm::vec3 spawn =
              src.pos + glm::vec3(src.along.x, 0.0f, src.along.y) * rng.Float(-0.6f, 0.6f) +
              glm::vec3(0.0f, rng.Float(-0.4f, 0.4f), 0.0f);
          const glm::vec2 dir = Rotate2(src.outward, rng.Float(-0.7f, 0.7f));
          const float speed = rng.Float(1.6f, 3.8f);
          const float vy = rng.Float(0.3f, 1.8f);
          // Ballistic flight to the support under the landing spot. The
          // landing moves the support, so settle the pair with one re-solve.
          glm::vec2 land(spawn.x, spawn.z);
          float support = SupportHeight(scratch, land, spawn.y);
          float duration = 0.0f;
          for (int solve = 0; solve < 2; ++solve) {
            const float drop = std::max(0.0f, spawn.y - (support + 0.5f * t));
            duration = (vy + std::sqrt(vy * vy + 2.0f * kGravity * drop)) / kGravity;
            land = glm::vec2(spawn.x, spawn.z) + dir * (speed * duration);
            support = SupportHeight(scratch, land, spawn.y);
          }
          // The chunk must clear the opening (not plug the fresh breach),
          // stay on the map, and come to rest in the open.
          if (glm::dot(land - glm::vec2(src.pos.x, src.pos.z), src.outward) < 0.45f) continue;
          const float margin = scene.mapHalfExtent - 0.4f;
          if (std::fabs(land.x) > margin || std::fabs(land.y) > margin) continue;
          if (RestSpotBlocked(scratch, land, support + 0.02f, support + t)) continue;
          bool crowded = false;
          for (const glm::vec2& other : placed) crowded |= glm::distance(land, other) < 0.5f;
          if (crowded) continue;

          const float restYaw =
              std::atan2(src.along.y, src.along.x) + rng.Float(-0.7f, 0.7f);
          const glm::vec2 axisW = Rotate2(glm::vec2(1.0f, 0.0f), restYaw) * (0.5f * w);
          const glm::vec2 perp = glm::vec2(-axisW.y, axisW.x) * (h / w);  // 0.5h, perpendicular.
          Obstacle piece;
          piece.kind = ObstacleKind::Rubble;
          piece.building = -1;
          piece.footprint = {land - axisW - perp, land + axisW - perp, land + axisW + perp,
                             land - axisW + perp};
          piece.bounds.min = glm::vec3(1e30f, support, 1e30f);
          piece.bounds.max = glm::vec3(-1e30f, support + t, -1e30f);
          for (const glm::vec2& v : piece.footprint) {
            piece.bounds.min.x = std::min(piece.bounds.min.x, v.x);
            piece.bounds.max.x = std::max(piece.bounds.max.x, v.x);
            piece.bounds.min.z = std::min(piece.bounds.min.z, v.y);
            piece.bounds.max.z = std::max(piece.bounds.max.z, v.y);
          }
          DebrisFlight flight;
          flight.from = spawn;
          flight.duration = duration;
          flight.spin = (rng.Int(0, 1) ? 1.0f : -1.0f) * 6.2831853f *
                        (duration > 0.8f ? 2.0f : 1.0f);
          flight.fromYaw = std::atan2(src.along.y, src.along.x);
          debris.pieces.push_back(std::move(piece));
          debris.flights.push_back(flight);
          placed.push_back(land);
          break;
        }
      }
      if (debris.pieces.size() >= kMaxDebrisChunks) break;
    }
    if (!debris.pieces.empty()) planChange(std::move(debris));
  }

  return plan;
}

}  // namespace tactics
