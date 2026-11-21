#include "game/Destruction.h"

#include <algorithm>
#include <cmath>
#include <random>

#include "game/Geometry.h"

namespace tactics {
namespace {

// Fracture tuning. The grid cell is the fracture's spatial resolution: hole
// outlines are unions of cells, so they stay grid-aligned (every wall piece
// remains an upright prism) while the seeded Voronoi sites give each blast
// its own silhouette.
constexpr float kFractureCell = 0.25f;
constexpr int kMaxStripsPerWall = 6;      // Rect holes per wall piece.
constexpr size_t kMaxPlanChanges = 48;    // Overall piece budget per blast.
constexpr float kFloorSnap = 0.35f;       // Holes this close to the wall base/top reach it.
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
  if (o.kind == ObstacleKind::Pier || IsStreetFixture(o.kind)) return false;
  const std::vector<glm::vec2> f = ObstacleFootprint(o);
  if (f.size() != 4) return false;
  const float e0 = glm::distance(f[0], f[1]);
  const float e1 = glm::distance(f[1], f[2]);
  const float thickness = std::min(e0, e1);
  const float length = std::max(e0, e1);
  if (thickness < 1e-3f || thickness > 0.7f || length < 1.0f) return false;
  return o.bounds.max.y - o.bounds.min.y >= 0.8f;
}

// Footprint edge index of the wall's long face (0 or 1; 2/3 mirror them).
int LongEdge(const std::vector<glm::vec2>& f) {
  return glm::distance(f[0], f[1]) >= glm::distance(f[1], f[2]) ? 0 : 1;
}

// A thin horizontal plate (ceiling or roof slab) a blast can hole.
bool SlabLike(const Obstacle& o) {
  if (o.dead || o.navOnly) return false;
  if (o.kind == ObstacleKind::Pier || IsStreetFixture(o.kind)) return false;
  if (o.bounds.max.y - o.bounds.min.y > 0.7f) return false;
  const std::vector<glm::vec2> f = ObstacleFootprint(o);
  return f.size() >= 3 && PolygonArea(f) >= 1.0f;
}

struct Strip {
  float u0, u1, y0, y1;
};

// Graded Voronoi fracture of one wall piece in its own plane: ~30-40 seeded
// sites at distance r * random^2 (dense near the center), cells classified
// by ring -- core (dust) and small-shard cells are removed, the outer large
// chunks stay standing. The removed region is rasterized on the fracture
// grid and merged into vertical strips (one rect per column run, adjacent
// columns with the same run joined), so every hole is an axis-aligned rect
// in the wall plane and every leftover piece an upright prism.
std::vector<Strip> FractureWall(const Obstacle& wall, const BombSpec& bomb, uint32_t stream) {
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
  std::vector<Strip> strips;
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
    const float cu0 = ku * kFractureCell, cu1 = (ku + 1) * kFractureCell;
    if (!strips.empty() && std::fabs(strips.back().u1 - cu0) < 1e-3f &&
        std::fabs(strips.back().y0 - runLo) <= kFractureCell + 1e-3f &&
        std::fabs(strips.back().y1 - runHi) <= kFractureCell + 1e-3f) {
      strips.back().u1 = cu1;
      strips.back().y0 = std::min(strips.back().y0, runLo);
      strips.back().y1 = std::max(strips.back().y1, runHi);
    } else {
      strips.push_back({cu0, cu1, runLo, runHi});
    }
  }
  // Piece budget: join adjacent strips (removing slightly more wall) until
  // the count fits.
  while (static_cast<int>(strips.size()) > kMaxStripsPerWall) {
    size_t best = 0;
    float bestGap = 1e30f;
    for (size_t i = 0; i + 1 < strips.size(); ++i) {
      const float gap = strips[i + 1].u0 - strips[i].u1;
      if (gap < bestGap) {
        bestGap = gap;
        best = i;
      }
    }
    strips[best].u1 = strips[best + 1].u1;
    strips[best].y0 = std::min(strips[best].y0, strips[best + 1].y0);
    strips[best].y1 = std::max(strips[best].y1, strips[best + 1].y1);
    strips.erase(strips.begin() + best + 1);
  }
  for (Strip& s : strips) {
    s.u0 = std::max(0.0f, s.u0);
    s.u1 = std::min(len, s.u1);
    if (s.y0 - ylo < kFloorSnap) s.y0 = ylo;  // Reaches the floor: a doorway.
    if (yhi - s.y1 < kFloorSnap) s.y1 = yhi;
    s.y0 = std::max(ylo, s.y0);
    s.y1 = std::min(yhi, s.y1);
  }
  strips.erase(std::remove_if(strips.begin(), strips.end(),
                              [](const Strip& s) {
                                return s.u1 - s.u0 < 0.05f || s.y1 - s.y0 < 0.05f;
                              }),
               strips.end());
  return strips;
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

  // 1. Street fixtures in the blast are blown away.
  const size_t baseObstacles = scene.obstacles.size();
  for (size_t i = 0; i < baseObstacles; ++i) {
    const Obstacle& o = scene.obstacles[i];
    if (o.dead || !IsStreetFixture(o.kind)) continue;
    if (DistanceToPrism(bomb.position, o) >= bomb.radius) continue;
    WorldChange remove;
    remove.kind = WorldChangeKind::ObstacleRemoved;
    remove.target = static_cast<int>(i);
    planChange(std::move(remove));
  }

  // 2. Walls: seeded graded Voronoi fracture, one rect hole per strip. The
  // per-wall stream hashes the pristine obstacle index, so the fracture of
  // one wall never depends on which other walls the blast also touched.
  for (size_t i = 0; i < baseObstacles; ++i) {
    const Obstacle& wall = scene.obstacles[i];
    if (!WallLike(wall)) continue;
    if (DistanceToPrism(bomb.position, wall) >= bomb.radius) continue;
    const std::vector<Strip> strips =
        FractureWall(wall, bomb, Mix(bomb.seed, static_cast<uint32_t>(i) * 2654435761u));
    if (strips.empty()) continue;
    const std::vector<glm::vec2> f = ObstacleFootprint(wall);
    const int e = LongEdge(f);
    const glm::vec2 a = f[e];
    const glm::vec2 d = glm::normalize(f[(e + 1) % 4] - f[e]);
    const glm::vec2 inward = 0.5f * (f[(e + 3) % 4] - f[e]);
    for (const Strip& strip : strips) {
      const glm::vec2 lo2 = a + d * strip.u0 + inward;
      const glm::vec2 hi2 = a + d * strip.u1 + inward;
      const glm::vec2 mid2 = 0.5f * (lo2 + hi2);
      const glm::vec3 mid(mid2.x, 0.5f * (strip.y0 + strip.y1), mid2.y);
      const int target = FindPieceAt(scratch, mid, d);
      if (target < 0) continue;
      const Obstacle& piece = scratch.obstacles[target];
      const std::vector<glm::vec2> pf = ObstacleFootprint(piece);
      if (pf.size() != 4) continue;
      const int pe = LongEdge(pf);
      const glm::vec2 pa = pf[pe];
      const glm::vec2 pd = glm::normalize(pf[(pe + 1) % 4] - pf[pe]);
      const float pu0 = glm::dot(lo2 - inward - pa, pd);
      const float pu1 = glm::dot(hi2 - inward - pa, pd);
      WorldChange hole;
      hole.kind = WorldChangeKind::WallHole;
      hole.target = target;
      hole.wallEdge = pe;
      hole.u0 = std::min(pu0, pu1);
      hole.u1 = std::max(pu0, pu1);
      hole.y0 = strip.y0;
      hole.y1 = strip.y1;
      planChange(std::move(hole));
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

  return plan;
}

}  // namespace tactics
