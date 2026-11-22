#include "game/DebrisPhysics.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <unordered_set>

#include <glm/gtc/matrix_transform.hpp>

#include "game/Geometry.h"

namespace tactics {
namespace {

constexpr float kDt = 1.0f / 120.0f;
constexpr int kStepsPerKey = 6;  // kDt * 6 == kDebrisKeyDt.
constexpr int kIterations = 16;
constexpr float kGravity = 9.81f;
constexpr float kMargin = 0.05f;         // Contacts are made this far before touching.
constexpr float kSlop = 0.004f;          // Penetration left uncorrected.
constexpr float kMaxPush = 1.5f;         // m/s: cap on the velocity added to separate.
constexpr float kFriction = 0.7f;
constexpr float kRestitution = 0.18f;
constexpr float kRestitutionSpeed = 1.2f;  // Slower impacts do not bounce.
constexpr float kDensity = 1000.0f;
constexpr float kMaxSpeed = 30.0f;
constexpr float kMaxSpin = 25.0f;
constexpr float kSleepLinear = 0.10f;    // m/s
constexpr float kSleepAngular = 0.35f;   // rad/s
constexpr float kSleepTime = 0.45f;
constexpr float kStillTime = 0.8f;       // ... or this long without going anywhere.
constexpr float kStillDistance = 0.004f;
constexpr float kStillDot = 0.99998f;    // cos(half the angle): ~0.006 rad.      // Seconds quiet before an island freezes.
constexpr float kWakeSpeed = 0.5f;       // An impact this hard wakes a sleeper.
constexpr int kSettleAfter = 360;          // Step (3 s) after which piles are damped ever harder.
constexpr float kSettleRamp = 180.0f;
constexpr int kLaunchGrace = 48;          // Steps launch-time contacts are ignored for.
constexpr float kDeckBelowReach = 0.1f;  // A body this far under a deck's top is below it.

struct Face {
  std::vector<int> idx;  // Counter-clockwise seen from outside.
  glm::vec3 n{0.0f};     // Outward, local frame.
};

// Convex prism geometry in the body's local frame (about its centre of mass).
struct Hull {
  std::vector<glm::vec3> v;
  std::vector<Face> faces;
  std::vector<std::pair<int, int>> reps;  // One edge per distinct direction.
};

Hull MakeHull(const std::vector<glm::vec2>& outline, float halfT, const glm::mat4& pose) {
  Hull h;
  const int n = static_cast<int>(outline.size());
  const glm::mat3 rot(pose);
  const auto place = [&](const glm::vec3& p) { return glm::vec3(pose * glm::vec4(p, 1.0f)); };
  for (int i = 0; i < n; ++i) h.v.push_back(place(glm::vec3(outline[i], -halfT)));
  for (int i = 0; i < n; ++i) h.v.push_back(place(glm::vec3(outline[i], halfT)));
  Face bottom, top;
  bottom.n = rot * glm::vec3(0.0f, 0.0f, -1.0f);
  top.n = rot * glm::vec3(0.0f, 0.0f, 1.0f);
  for (int i = n - 1; i >= 0; --i) bottom.idx.push_back(i);
  for (int i = 0; i < n; ++i) top.idx.push_back(n + i);
  h.faces.push_back(bottom);
  h.faces.push_back(top);
  for (int i = 0; i < n; ++i) {
    const int j = (i + 1) % n;
    Face side;
    side.idx = {i, j, n + j, n + i};
    const glm::vec2 d = outline[j] - outline[i];
    const float len = glm::length(d);
    if (len < 1e-6f) continue;
    side.n = rot * glm::vec3(d.y / len, -d.x / len, 0.0f);
    h.faces.push_back(side);
    // Edge i -> j stands for its direction unless an earlier one is parallel.
    bool parallel = false;
    for (const auto& rep : h.reps) {
      const glm::vec3 e0 = h.v[rep.second] - h.v[rep.first];
      const glm::vec3 e1 = h.v[j] - h.v[i];
      if (glm::length(glm::cross(e0, e1)) < 1e-4f * glm::length(e0) * glm::length(e1)) {
        parallel = true;
        break;
      }
    }
    if (!parallel) h.reps.emplace_back(i, j);
  }
  h.reps.emplace_back(0, n);  // The through-thickness direction.
  return h;
}

struct Body {
  Hull hull;
  bool isDynamic = false;
  bool light = false;         // Litter: moves nothing heavier than itself.
  bool asleep = false;
  float quiet = 0.0f;         // Seconds below the sleep thresholds.
  float still = 0.0f;         // Seconds within a hair of refX / refQ (jitter is not motion).
  glm::vec3 refX{0.0f};
  glm::quat refQ{1.0f, 0.0f, 0.0f, 0.0f};
  bool touching = false;      // In contact this step (only those may sleep).

  glm::vec3 x{0.0f};
  glm::quat q{1.0f, 0.0f, 0.0f, 0.0f};
  glm::vec3 v{0.0f}, w{0.0f};
  float invMass = 0.0f;
  glm::mat3 invInertiaLocal{0.0f};
  glm::mat3 invInertia{0.0f};  // World frame.
  glm::mat3 rot{1.0f};

  std::vector<glm::vec3> wv;   // World vertices.
  std::vector<glm::vec3> wn;   // World face normals.
  glm::vec3 lo{0.0f}, hi{0.0f};
  glm::vec3 center{0.0f};

  std::vector<DebrisKey> keys;
  bool justSlept = false;      // Fell asleep this step (its last key is due).
  bool settled = true;
};

void UpdateWorld(Body* b) {
  b->rot = glm::mat3_cast(b->q);
  b->wv.resize(b->hull.v.size());
  b->lo = glm::vec3(1e30f);
  b->hi = glm::vec3(-1e30f);
  for (size_t i = 0; i < b->hull.v.size(); ++i) {
    b->wv[i] = b->x + b->rot * b->hull.v[i];
    b->lo = glm::min(b->lo, b->wv[i]);
    b->hi = glm::max(b->hi, b->wv[i]);
  }
  b->wn.resize(b->hull.faces.size());
  for (size_t i = 0; i < b->hull.faces.size(); ++i) b->wn[i] = b->rot * b->hull.faces[i].n;
  b->center = 0.5f * (b->lo + b->hi);
  b->invInertia = b->rot * b->invInertiaLocal * glm::transpose(b->rot);
}

// ---- Narrowphase ----------------------------------------------------------

struct ContactPoint {
  glm::vec3 p{0.0f};
  float depth = 0.0f;  // > 0 penetrating, < 0 a gap (speculative).
};

struct Manifold {
  glm::vec3 n{0.0f, 1.0f, 0.0f};  // From A to B.
  std::vector<ContactPoint> pts;
};

// Largest separation along A's face normals (B measured against them).
float FaceSeparation(const Body& A, const Body& B, int* face) {
  float best = -1e30f;
  *face = 0;
  for (size_t f = 0; f < A.hull.faces.size(); ++f) {
    const glm::vec3& n = A.wn[f];
    const glm::vec3& p0 = A.wv[A.hull.faces[f].idx[0]];
    float lowest = 1e30f;
    for (const glm::vec3& p : B.wv) lowest = std::min(lowest, glm::dot(n, p));
    const float sep = lowest - glm::dot(n, p0);
    if (sep > best) {
      best = sep;
      *face = static_cast<int>(f);
      if (best > kMargin) return best;
    }
  }
  return best;
}

std::vector<glm::vec3> ClipToPlane(const std::vector<glm::vec3>& poly, const glm::vec3& n,
                                   const glm::vec3& p0) {
  std::vector<glm::vec3> out;
  const size_t count = poly.size();
  for (size_t i = 0; i < count; ++i) {
    const glm::vec3& s = poly[i];
    const glm::vec3& e = poly[(i + 1) % count];
    const float ds = glm::dot(n, s - p0), de = glm::dot(n, e - p0);
    if (ds >= 0.0f) out.push_back(s);
    if ((ds >= 0.0f) != (de >= 0.0f)) {
      const float t = ds / (ds - de);
      out.push_back(s + (e - s) * t);
    }
  }
  return out;
}

// At most four points that span the patch: deepest, farthest from it, then
// the two extremes either side of that line.
void ReduceContacts(std::vector<ContactPoint>* pts, const glm::vec3& n) {
  if (pts->size() <= 4) return;
  std::vector<ContactPoint>& v = *pts;
  size_t i0 = 0;
  for (size_t i = 1; i < v.size(); ++i) {
    if (v[i].depth > v[i0].depth) i0 = i;
  }
  size_t i1 = i0 == 0 ? 1 : 0;
  for (size_t i = 0; i < v.size(); ++i) {
    if (i != i0 && glm::distance(v[i].p, v[i0].p) > glm::distance(v[i1].p, v[i0].p)) i1 = i;
  }
  size_t i2 = i0, i3 = i0;
  float maxArea = 0.0f, minArea = 0.0f;
  for (size_t i = 0; i < v.size(); ++i) {
    if (i == i0 || i == i1) continue;
    const float area = glm::dot(n, glm::cross(v[i1].p - v[i0].p, v[i].p - v[i0].p));
    if (area > maxArea) { maxArea = area; i2 = i; }
    if (area < minArea) { minArea = area; i3 = i; }
  }
  std::vector<ContactPoint> keep{v[i0], v[i1]};
  if (i2 != i0) keep.push_back(v[i2]);
  if (i3 != i0) keep.push_back(v[i3]);
  v = std::move(keep);
}

// SAT on two convex prisms. False when they are separated by more than the
// contact margin; otherwise `m` holds the contacts (normal A -> B).
bool Collide(const Body& A, const Body& B, Manifold* m) {
  int faceA = 0, faceB = 0;
  const float sepA = FaceSeparation(A, B, &faceA);
  if (sepA > kMargin) return false;
  const float sepB = FaceSeparation(B, A, &faceB);
  if (sepB > kMargin) return false;

  // Edge-edge axes.
  float edgeSep = -1e30f;
  glm::vec3 edgeN(0.0f), edgeDirA(0.0f), edgeDirB(0.0f);
  const glm::vec3 toB = B.center - A.center;
  for (const auto& ra : A.hull.reps) {
    const glm::vec3 da = A.wv[ra.second] - A.wv[ra.first];
    for (const auto& rb : B.hull.reps) {
      const glm::vec3 db = B.wv[rb.second] - B.wv[rb.first];
      glm::vec3 axis = glm::cross(da, db);
      const float len = glm::length(axis);
      if (len < 1e-4f * glm::length(da) * glm::length(db)) continue;
      axis /= len;
      if (glm::dot(axis, toB) < 0.0f) axis = -axis;
      float maxA = -1e30f, minB = 1e30f;
      for (const glm::vec3& p : A.wv) maxA = std::max(maxA, glm::dot(axis, p));
      for (const glm::vec3& p : B.wv) minB = std::min(minB, glm::dot(axis, p));
      const float sep = minB - maxA;
      if (sep > kMargin) return false;
      if (sep > edgeSep) {
        edgeSep = sep;
        edgeN = axis;
        edgeDirA = da;
        edgeDirB = db;
      }
    }
  }

  const float faceSep = std::max(sepA, sepB);
  m->pts.clear();
  if (edgeSep > faceSep + 0.005f) {
    // Edge against edge: the support vertices pin both lines.
    glm::vec3 pa = A.wv[0], pb = B.wv[0];
    float bestA = -1e30f, bestB = 1e30f;
    for (const glm::vec3& p : A.wv) {
      if (glm::dot(edgeN, p) > bestA) { bestA = glm::dot(edgeN, p); pa = p; }
    }
    for (const glm::vec3& p : B.wv) {
      if (glm::dot(edgeN, p) < bestB) { bestB = glm::dot(edgeN, p); pb = p; }
    }
    // Closest points of the lines pa + s da and pb + t db.
    const glm::vec3 r = pa - pb;
    const float a = glm::dot(edgeDirA, edgeDirA), b = glm::dot(edgeDirA, edgeDirB);
    const float c = glm::dot(edgeDirB, edgeDirB);
    const float d = glm::dot(edgeDirA, r), e = glm::dot(edgeDirB, r);
    const float den = a * c - b * b;
    if (std::fabs(den) < 1e-9f) return true;  // Cannot happen (parallels are skipped).
    const float s = (b * e - c * d) / den;
    const float t = (a * e - b * d) / den;
    ContactPoint cp;
    cp.p = 0.5f * ((pa + edgeDirA * s) + (pb + edgeDirB * t));
    cp.depth = -edgeSep;
    m->n = edgeN;
    m->pts.push_back(cp);
    return true;
  }

  // Face contact: clip the incident face against the reference face's sides.
  const bool refIsA = sepA + 0.0005f >= sepB;
  const Body& R = refIsA ? A : B;
  const Body& I = refIsA ? B : A;
  const int refFace = refIsA ? faceA : faceB;
  const glm::vec3 nR = R.wn[refFace];
  int incFace = 0;
  float mostOpposed = 1e30f;
  for (size_t f = 0; f < I.hull.faces.size(); ++f) {
    const float d = glm::dot(I.wn[f], nR);
    if (d < mostOpposed) { mostOpposed = d; incFace = static_cast<int>(f); }
  }
  std::vector<glm::vec3> poly;
  for (const int i : I.hull.faces[incFace].idx) poly.push_back(I.wv[i]);
  const std::vector<int>& ref = R.hull.faces[refFace].idx;
  for (size_t k = 0; k < ref.size() && !poly.empty(); ++k) {
    const glm::vec3& p0 = R.wv[ref[k]];
    const glm::vec3& p1 = R.wv[ref[(k + 1) % ref.size()]];
    const glm::vec3 e = p1 - p0;
    if (glm::length(e) < 1e-7f) continue;
    const glm::vec3 inward = glm::normalize(glm::cross(nR, e));
    poly = ClipToPlane(poly, inward, p0);
  }
  const glm::vec3& refPoint = R.wv[ref[0]];
  for (const glm::vec3& p : poly) {
    const float dist = glm::dot(nR, p - refPoint);
    if (dist > kMargin) continue;
    ContactPoint cp;
    cp.p = p - nR * (dist * 0.5f);
    cp.depth = -dist;
    m->pts.push_back(cp);
  }
  m->n = refIsA ? nR : -nR;
  ReduceContacts(&m->pts, m->n);
  return !m->pts.empty();
}

// ---- Contacts and the solver ----------------------------------------------

struct Contact {
  int a = 0, b = 0;  // Body indices (0 is the world).
  glm::vec3 p{0.0f}, n{0.0f, 1.0f, 0.0f};
  float depth = 0.0f;
  glm::vec3 ra{0.0f}, rb{0.0f}, t1{0.0f}, t2{0.0f};
  float massN = 0.0f, massT1 = 0.0f, massT2 = 0.0f;
  float targetV = 0.0f;
  bool moveA = false, moveB = false;  // Which side the impulse moves.
  float ln = 0.0f, lt1 = 0.0f, lt2 = 0.0f;  // Accumulated impulses (cold start each step).
};

uint64_t PairKey(int a, int b) {
  if (a > b) std::swap(a, b);
  return (static_cast<uint64_t>(a) << 32) | static_cast<uint32_t>(b);
}

glm::vec3 Perpendicular(const glm::vec3& n) {
  const glm::vec3 axis = std::fabs(n.x) < 0.6f ? glm::vec3(1, 0, 0) : glm::vec3(0, 1, 0);
  return glm::normalize(glm::cross(n, axis));
}

// Area moments of a counter-clockwise polygon about its own centroid.
struct PolygonMoments {
  float area = 0.0f;
  glm::vec2 centroid{0.0f};
  float ix2 = 0.0f, iy2 = 0.0f, ixy = 0.0f;  // About the centroid.
};

PolygonMoments MomentsOf(const std::vector<glm::vec2>& poly) {
  PolygonMoments out;
  const size_t n = poly.size();
  float a2 = 0.0f;
  glm::vec2 c(0.0f);
  for (size_t i = 0; i < n; ++i) {
    const glm::vec2 p = poly[i], q = poly[(i + 1) % n];
    const float cr = p.x * q.y - q.x * p.y;
    a2 += cr;
    c += (p + q) * cr;
  }
  if (std::fabs(a2) < 1e-12f) return out;
  out.area = 0.5f * a2;
  out.centroid = c / (3.0f * a2);
  float ix = 0.0f, iy = 0.0f, ixy = 0.0f;
  for (size_t i = 0; i < n; ++i) {
    const glm::vec2 p = poly[i] - out.centroid, q = poly[(i + 1) % n] - out.centroid;
    const float cr = p.x * q.y - q.x * p.y;
    ix += (p.x * p.x + p.x * q.x + q.x * q.x) * cr;
    iy += (p.y * p.y + p.y * q.y + q.y * q.y) * cr;
    ixy += (p.x * q.y + 2.0f * p.x * p.y + 2.0f * q.x * q.y + q.x * p.y) * cr;
  }
  out.ix2 = ix / 12.0f;
  out.iy2 = iy / 12.0f;
  out.ixy = ixy / 24.0f;
  return out;
}

}  // namespace

glm::vec2 DebrisCentroid(const std::vector<glm::vec2>& outline) {
  if (outline.size() < 3) return glm::vec2(0.0f);
  const PolygonMoments m = MomentsOf(outline);
  return m.area > 0.0f ? m.centroid : glm::vec2(0.0f);
}

namespace {

glm::vec3 GroundNormal(const HeightField& g, float x, float z) {
  constexpr float h = 0.25f;
  const float dx = g.HeightAt(x + h, z) - g.HeightAt(x - h, z);
  const float dz = g.HeightAt(x, z + h) - g.HeightAt(x, z - h);
  return glm::normalize(glm::vec3(-dx / (2.0f * h), 1.0f, -dz / (2.0f * h)));
}

struct Deck {
  const WalkSurface* surface = nullptr;
  std::vector<glm::vec2> outline;
  glm::vec2 lo{0.0f}, hi{0.0f};
  float minY = 0.0f, maxY = 0.0f;
  glm::vec3 normal{0.0f, 1.0f, 0.0f};
};

}  // namespace

std::vector<DebrisTrack> SimulateDebris(const DebrisWorldDesc& world,
                                        const std::vector<DebrisBodyDesc>& descs,
                                        float maxSeconds) {
  const int dynamicCount = static_cast<int>(descs.size());
  std::vector<DebrisTrack> tracks(descs.size());
  if (dynamicCount == 0) return tracks;

  // Bodies: [0] the world (ground, decks, map edge), then the chunks, then
  // the standing prisms.
  std::vector<Body> bodies(1 + dynamicCount + world.statics.size());
  for (int i = 0; i < dynamicCount; ++i) {
    const DebrisBodyDesc& d = descs[i];
    Body& b = bodies[1 + i];
    b.isDynamic = true;
    const PolygonMoments m = MomentsOf(d.shape.outline);
    std::vector<glm::vec2> outline = d.shape.outline;
    for (glm::vec2& p : outline) p -= m.centroid;
    const float t = d.shape.thickness;
    b.hull = MakeHull(outline, 0.5f * t, glm::mat4(1.0f));
    b.x = glm::vec3(d.shape.pose * glm::vec4(m.centroid.x, m.centroid.y, 0.0f, 1.0f));
    b.q = glm::normalize(glm::quat_cast(glm::mat3(d.shape.pose)));
    b.v = d.velocity;
    b.w = d.angularVelocity;
    b.light = d.light;
    // Real masses span orders of magnitude (a roof slab against a window
    // bar), which makes piles stiff and jumpy. The mass is compressed
    // (volume^0.7) with the inertia scaled along, so a heavy chunk still
    // outweighs a light one without crushing it.
    const float volume = std::max(1e-6f, m.area * t);
    const float mass = kDensity * std::pow(volume, 0.7f);
    const float scale = mass / (kDensity * volume);
    const float slab = m.area * t * t * t / 12.0f;
    glm::mat3 inertia(0.0f);  // Column-major: inertia[col][row].
    inertia[0][0] = scale * kDensity * (t * m.iy2 + slab);
    inertia[1][1] = scale * kDensity * (t * m.ix2 + slab);
    inertia[2][2] = scale * kDensity * t * (m.ix2 + m.iy2);
    inertia[0][1] = inertia[1][0] = -scale * kDensity * t * m.ixy;
    b.invMass = 1.0f / mass;
    b.invInertiaLocal = glm::inverse(inertia);
    UpdateWorld(&b);
    b.keys.push_back({b.x, b.q});
    b.refX = b.x;
    b.refQ = b.q;
  }
  for (size_t i = 0; i < world.statics.size(); ++i) {
    Body& b = bodies[1 + dynamicCount + i];
    const DebrisShape& s = world.statics[i];
    b.hull = MakeHull(s.outline, 0.5f * s.thickness, s.pose);
    UpdateWorld(&b);
  }

  // Decks near the chunks.
  std::vector<Deck> decks;
  {
    glm::vec3 lo(1e30f), hi(-1e30f);
    for (int i = 0; i < dynamicCount; ++i) {
      lo = glm::min(lo, bodies[1 + i].lo);
      hi = glm::max(hi, bodies[1 + i].hi);
    }
    const float reach = 25.0f;
    for (const WalkSurface* w : world.decks) {
      if (!w || w->dead || w->climbProxy || w->vertices.size() < 3) continue;
      Deck deck;
      deck.surface = w;
      deck.lo = deck.hi = glm::vec2(w->vertices.front().x, w->vertices.front().z);
      deck.minY = deck.maxY = w->vertices.front().y;
      glm::vec3 nrm(0.0f);
      for (size_t k = 0; k < w->vertices.size(); ++k) {
        const glm::vec3& p = w->vertices[k];
        const glm::vec3& q = w->vertices[(k + 1) % w->vertices.size()];
        deck.outline.emplace_back(p.x, p.z);
        deck.lo = glm::min(deck.lo, deck.outline.back());
        deck.hi = glm::max(deck.hi, deck.outline.back());
        deck.minY = std::min(deck.minY, p.y);
        deck.maxY = std::max(deck.maxY, p.y);
        nrm += glm::vec3((p.y - q.y) * (p.z + q.z), (p.z - q.z) * (p.x + q.x),
                         (p.x - q.x) * (p.y + q.y));
      }
      if (deck.hi.x < lo.x - reach || deck.lo.x > hi.x + reach || deck.hi.y < lo.z - reach ||
          deck.lo.y > hi.z + reach) {
        continue;
      }
      if (glm::length(nrm) > 1e-6f) deck.normal = glm::normalize(nrm);
      if (deck.normal.y < 0.0f) deck.normal = -deck.normal;
      if (deck.normal.y < 0.2f) continue;  // Not something to lie on.
      decks.push_back(std::move(deck));
    }
  }

  // Standing prisms in a coarse grid (cells of kCell metres).
  constexpr float kCell = 4.0f;
  std::unordered_map<int64_t, std::vector<int>> grid;
  const auto cellKey = [](int cx, int cz) {
    return (static_cast<int64_t>(cx) << 32) ^ static_cast<int64_t>(static_cast<uint32_t>(cz));
  };
  for (size_t i = 0; i < world.statics.size(); ++i) {
    const Body& b = bodies[1 + dynamicCount + i];
    const int x0 = static_cast<int>(std::floor(b.lo.x / kCell));
    const int x1 = static_cast<int>(std::floor(b.hi.x / kCell));
    const int z0 = static_cast<int>(std::floor(b.lo.z / kCell));
    const int z1 = static_cast<int>(std::floor(b.hi.z / kCell));
    if ((x1 - x0 + 1) * (z1 - z0 + 1) > 400) continue;  // Absurdly big: ignore.
    for (int cx = x0; cx <= x1; ++cx) {
      for (int cz = z0; cz <= z1; ++cz) grid[cellKey(cx, cz)].push_back(1 + dynamicCount + static_cast<int>(i));
    }
  }

  std::unordered_set<uint64_t> ignored;  // Pairs that began overlapping, until they part.
  std::vector<int> order(dynamicCount);
  for (int i = 0; i < dynamicCount; ++i) order[i] = 1 + i;

  const int maxSteps = static_cast<int>(maxSeconds / kDt);
  int step = 0;
  bool anyAwake = true;
  std::vector<Contact> contacts;

  const auto pushKey = [&](Body& b) {
    DebrisKey k{b.x, b.q};
    if (!b.keys.empty() && glm::dot(b.keys.back().rotation, k.rotation) < 0.0f) {
      k.rotation = -k.rotation;  // Keep the track in one hemisphere.
    }
    b.keys.push_back(k);
  };
  const auto fillTo = [&](Body& b, size_t count) {
    while (b.keys.size() < count) b.keys.push_back(b.keys.back());
  };

  for (; step < maxSteps && anyAwake; ++step) {
    contacts.clear();
    for (int i = 1; i <= dynamicCount; ++i) {
      bodies[i].touching = false;
      bodies[i].justSlept = false;
    }
    const auto live = [&](const Body& b) { return b.isDynamic && !b.asleep; };

    // Wakes a sleeper, and the sleepers touching it.
    const std::function<void(int)> wake = [&](int idx) {
      std::vector<int> work{idx};
      while (!work.empty()) {
        const int w = work.back();
        work.pop_back();
        Body& body = bodies[w];
        if (!body.asleep) continue;
        body.asleep = false;
        body.quiet = body.still = 0.0f;
        body.refX = body.x;
        body.refQ = body.q;
        fillTo(body, static_cast<size_t>(step / kStepsPerKey));
        for (int j = 1; j <= dynamicCount; ++j) {
          const Body& other = bodies[j];
          if (!other.asleep || j == w) continue;
          if (other.lo.x > body.hi.x + kMargin || other.hi.x < body.lo.x - kMargin ||
              other.lo.y > body.hi.y + kMargin || other.hi.y < body.lo.y - kMargin ||
              other.lo.z > body.hi.z + kMargin || other.hi.z < body.lo.z - kMargin) {
            continue;
          }
          work.push_back(j);
        }
      }
    };

    // --- Contact generation.
    const auto addManifold = [&](int ia, int ib, const Manifold& m) {
      for (const ContactPoint& cp : m.pts) {
        Contact c;
        c.a = ia;
        c.b = ib;
        c.p = cp.p;
        c.n = m.n;
        c.depth = cp.depth;
        contacts.push_back(c);
      }
    };
    if (step == kLaunchGrace) ignored.clear();
    // Chunk against chunk: sweep along x.
    std::sort(order.begin(), order.end(), [&](int a, int b) {
      if (bodies[a].lo.x != bodies[b].lo.x) return bodies[a].lo.x < bodies[b].lo.x;
      return a < b;
    });
    Manifold manifold;
    const auto considerPair = [&](int ia, int ib) {
      Body& A = bodies[ia];
      Body& B = bodies[ib];
      if (!live(A) && !live(B)) return;
      const uint64_t key = PairKey(ia, ib);
      const bool hit = Collide(A, B, &manifold);
      if (!hit) {
        ignored.erase(key);
        return;
      }
      // Whatever touches (or overlaps) at launch -- the neighbouring cells,
      // the rim of the hole a cell is dropping out of -- lets go of it
      // first: the pair collides again once it has parted, or after a moment.
      if (step == 0) ignored.insert(key);
      if (!ignored.empty() && ignored.count(key)) return;
      // A hard hit wakes a sleeper (and whatever sleeps against it).
      for (const int idx : {ia, ib}) {
        const Body& sleeper = bodies[idx];
        const Body& other = bodies[idx == ia ? ib : ia];
        if (sleeper.isDynamic && sleeper.asleep && live(other) &&
            glm::length(other.v) + glm::length(other.w) * 0.3f > kWakeSpeed) {
          wake(idx);
        }
      }
      addManifold(ia, ib, manifold);
      A.touching = true;
      B.touching = true;
    };
    for (size_t i = 0; i < order.size(); ++i) {
      const int ia = order[i];
      for (size_t j = i + 1; j < order.size(); ++j) {
        const int ib = order[j];
        if (bodies[ib].lo.x > bodies[ia].hi.x + kMargin) break;
        if (bodies[ib].lo.y > bodies[ia].hi.y + kMargin || bodies[ib].hi.y < bodies[ia].lo.y - kMargin ||
            bodies[ib].lo.z > bodies[ia].hi.z + kMargin || bodies[ib].hi.z < bodies[ia].lo.z - kMargin) {
          continue;
        }
        considerPair(std::min(ia, ib), std::max(ia, ib));
      }
    }
    // Chunk against standing prisms.
    for (int ia = 1; ia <= dynamicCount; ++ia) {
      Body& A = bodies[ia];
      if (!live(A)) continue;
      const int x0 = static_cast<int>(std::floor((A.lo.x - kMargin) / kCell));
      const int x1 = static_cast<int>(std::floor((A.hi.x + kMargin) / kCell));
      const int z0 = static_cast<int>(std::floor((A.lo.z - kMargin) / kCell));
      const int z1 = static_cast<int>(std::floor((A.hi.z + kMargin) / kCell));
      std::vector<int> near;
      for (int cx = x0; cx <= x1; ++cx) {
        for (int cz = z0; cz <= z1; ++cz) {
          const auto it = grid.find(cellKey(cx, cz));
          if (it == grid.end()) continue;
          for (const int is : it->second) {
            near.push_back(is);
          }
        }
      }
      std::sort(near.begin(), near.end());
      near.erase(std::unique(near.begin(), near.end()), near.end());
      for (const int is : near) {
        const Body& S = bodies[is];
        if (S.lo.x > A.hi.x + kMargin || S.hi.x < A.lo.x - kMargin || S.lo.y > A.hi.y + kMargin ||
            S.hi.y < A.lo.y - kMargin || S.lo.z > A.hi.z + kMargin || S.hi.z < A.lo.z - kMargin) {
          continue;
        }
        considerPair(ia, is);
      }
    }
    // Chunk vertices against the ground, decks and the map edge.
    for (int ia = 1; ia <= dynamicCount; ++ia) {
      Body& A = bodies[ia];
      if (!live(A)) continue;
      const auto addVertexContact = [&](const glm::vec3& p, const glm::vec3& n, float depth) {
        Contact c;
        c.a = 0;
        c.b = ia;
        c.p = p;
        c.n = n;
        c.depth = depth;
        contacts.push_back(c);
        A.touching = true;
      };
      for (const glm::vec3& p : A.wv) {
        if (world.ground && !world.ground->Empty()) {
          const float h = world.ground->HeightAt(p.x, p.z);
          if (p.y - h < kMargin) {
            addVertexContact(p, GroundNormal(*world.ground, p.x, p.z), h - p.y);
          }
        } else if (p.y < kMargin) {
          addVertexContact(p, glm::vec3(0.0f, 1.0f, 0.0f), -p.y);
        }
        if (p.x > world.extent - kMargin) addVertexContact(p, glm::vec3(-1, 0, 0), p.x - world.extent);
        if (p.x < -world.extent + kMargin) addVertexContact(p, glm::vec3(1, 0, 0), -world.extent - p.x);
        if (p.z > world.extent - kMargin) addVertexContact(p, glm::vec3(0, 0, -1), p.z - world.extent);
        if (p.z < -world.extent + kMargin) addVertexContact(p, glm::vec3(0, 0, 1), -world.extent - p.z);
        for (const Deck& deck : decks) {
          if (p.x < deck.lo.x || p.x > deck.hi.x || p.z < deck.lo.y || p.z > deck.hi.y) continue;
          if (p.y > deck.maxY + kMargin || p.y < deck.minY - kDeckThickness) continue;
          if (!PointStrictlyInConvexPolygon(glm::vec2(p.x, p.z), deck.outline, 0.01f)) continue;
          const float top = SurfaceHeightAt(*deck.surface, p.x, p.z);
          if (A.x.y < top - kDeckBelowReach) continue;  // The body is under the deck.
          const float above = (p.y - top) * deck.normal.y;
          if (above > kMargin || above < -kDeckThickness) continue;
          addVertexContact(p, deck.normal, -above);
        }
      }
    }

    // --- Prepare.
    for (Contact& c : contacts) {
      Body& A = bodies[c.a];
      Body& B = bodies[c.b];
      // Litter bounces off everything but pushes nothing heavier around.
      c.moveA = live(A) && !(B.isDynamic && A.light != B.light && !A.light);
      c.moveB = live(B) && !(A.isDynamic && A.light != B.light && !B.light);
      const bool liveA = c.moveA, liveB = c.moveB;
      const float imA = liveA ? A.invMass : 0.0f, imB = liveB ? B.invMass : 0.0f;
      c.ra = c.p - A.x;
      c.rb = c.p - B.x;
      c.t1 = Perpendicular(c.n);
      c.t2 = glm::cross(c.n, c.t1);
      const auto effective = [&](const glm::vec3& dir) {
        float k = imA + imB;
        if (liveA) {
          const glm::vec3 ca = glm::cross(c.ra, dir);
          k += glm::dot(ca, A.invInertia * ca);
        }
        if (liveB) {
          const glm::vec3 cb = glm::cross(c.rb, dir);
          k += glm::dot(cb, B.invInertia * cb);
        }
        return k > 1e-12f ? 1.0f / k : 0.0f;
      };
      c.massN = effective(c.n);
      c.massT1 = effective(c.t1);
      c.massT2 = effective(c.t2);
      const glm::vec3 va = liveA ? A.v + glm::cross(A.w, c.ra) : glm::vec3(0.0f);
      const glm::vec3 vb = liveB ? B.v + glm::cross(B.w, c.rb) : glm::vec3(0.0f);
      const float vn = glm::dot(vb - va, c.n);
      // Penetration is removed by moving bodies apart (below), never by
      // adding velocity: pushing with velocity is what makes resting piles
      // creep and jitter. A gap may close at gap / dt.
      c.targetV = c.depth > 0.0f ? 0.0f : c.depth / kDt;
      if (vn < -kRestitutionSpeed) c.targetV = std::max(c.targetV, -kRestitution * vn);
    }

    // Gravity.
    for (int i = 1; i <= dynamicCount; ++i) {
      if (live(bodies[i])) bodies[i].v.y -= kGravity * kDt;
    }

    // --- Solve.
    for (int iter = 0; iter < kIterations; ++iter) {
      for (Contact& c : contacts) {
        Body& A = bodies[c.a];
        Body& B = bodies[c.b];
        const bool liveA = c.moveA, liveB = c.moveB;
        const float imA = liveA ? A.invMass : 0.0f, imB = liveB ? B.invMass : 0.0f;
        const auto relative = [&]() {
          const glm::vec3 va = liveA ? A.v + glm::cross(A.w, c.ra) : glm::vec3(0.0f);
          const glm::vec3 vb = liveB ? B.v + glm::cross(B.w, c.rb) : glm::vec3(0.0f);
          return vb - va;
        };
        const auto apply = [&](const glm::vec3& impulse) {
          if (liveA) {
            A.v -= impulse * imA;
            A.w -= A.invInertia * glm::cross(c.ra, impulse);
          }
          if (liveB) {
            B.v += impulse * imB;
            B.w += B.invInertia * glm::cross(c.rb, impulse);
          }
        };
        // Friction, bounded by the normal impulse of the last iteration.
        const float limit = kFriction * c.ln;
        {
          const float dv = glm::dot(relative(), c.t1);
          const float old = c.lt1;
          c.lt1 = std::clamp(old - dv * c.massT1, -limit, limit);
          apply(c.t1 * (c.lt1 - old));
        }
        {
          const float dv = glm::dot(relative(), c.t2);
          const float old = c.lt2;
          c.lt2 = std::clamp(old - dv * c.massT2, -limit, limit);
          apply(c.t2 * (c.lt2 - old));
        }
        // Normal: relative separating speed must reach targetV.
        {
          const float vn = glm::dot(relative(), c.n);
          const float old = c.ln;
          c.ln = std::max(0.0f, old + (c.targetV - vn) * c.massN);
          apply(c.n * (c.ln - old));
        }
      }
    }

    // Penetration: translate the bodies apart a bit each step (a few Gauss-
    // Seidel passes), leaving their velocities alone.
    std::vector<glm::vec3> shift(bodies.size(), glm::vec3(0.0f));
    if (!contacts.empty()) {
      for (int pass = 0; pass < 3; ++pass) {
        for (const Contact& c : contacts) {
          const Body& A = bodies[c.a];
          const Body& B = bodies[c.b];
          const float imA = c.moveA ? A.invMass : 0.0f, imB = c.moveB ? B.invMass : 0.0f;
          if (imA + imB <= 0.0f) continue;
          const float now = c.depth - glm::dot(c.n, shift[c.b] - shift[c.a]);
          if (now <= kSlop) continue;
          const float amount = std::min(0.5f * (now - kSlop), kMaxPush * kDt);
          shift[c.a] -= c.n * (amount * imA / (imA + imB));
          shift[c.b] += c.n * (amount * imB / (imA + imB));
        }
      }
    }

    // --- Integrate, damp, sleep, record.
    for (int i = 1; i <= dynamicCount; ++i) {
      Body& b = bodies[i];
      if (!live(b)) continue;
      float damp = b.touching ? 0.985f : 0.9995f;
      float drag = 0.9998f;
      if (step > kSettleAfter) {
        // Out of patience: whatever is still shuffling in a pile is brought
        // to a halt, more firmly the longer it takes, rather than left to
        // jitter or to plough on forever (a collapse that has not settled in
        // a few seconds is moving through honey by the time it gets here).
        const float ramp = std::min(1.0f, static_cast<float>(step - kSettleAfter) / kSettleRamp);
        drag *= 1.0f - 0.25f * ramp;
        damp *= 1.0f - 0.25f * ramp;
      }
      b.v *= drag;
      b.w *= damp;
      const float speed = glm::length(b.v);
      if (speed > kMaxSpeed) b.v *= kMaxSpeed / speed;
      const float turn = glm::length(b.w);
      if (turn > kMaxSpin) b.w *= kMaxSpin / turn;
      b.x += b.v * kDt + shift[i];
      const glm::vec3 spin = b.w * kDt;
      const float angle = glm::length(spin);
      if (angle > 1e-9f) b.q = glm::normalize(glm::angleAxis(angle, spin / angle) * b.q);
      UpdateWorld(&b);
      if (b.touching && glm::length(b.v) < kSleepLinear && glm::length(b.w) < kSleepAngular) {
        b.quiet += kDt;
      } else {
        b.quiet = 0.0f;
      }
      // A pile can hum with solver noise (velocities that never settle) while
      // going nowhere: count how long the pose has stayed put as well.
      if (glm::distance(b.x, b.refX) > kStillDistance ||
          std::fabs(glm::dot(b.q, b.refQ)) < kStillDot) {
        b.refX = b.x;
        b.refQ = b.q;
        b.still = 0.0f;
      } else {
        b.still += kDt;
      }
      if (b.x.y < -60.0f) {  // Fell out of the world.
        b.asleep = true;
        b.justSlept = true;
        b.settled = false;
        b.v = b.w = glm::vec3(0.0f);
      }
    }
    // Sleep: a body that has been still for a moment while resting on
    // something is frozen at its exact pose (it still holds up whatever lies
    // on it, and wakes if something hits it hard).
    for (int i = 1; i <= dynamicCount; ++i) {
      Body& b = bodies[i];
      if (b.asleep || !b.touching) continue;
      if (b.quiet < kSleepTime && b.still < kStillTime) continue;
      b.asleep = true;
      b.justSlept = true;
      b.v = b.w = glm::vec3(0.0f);
    }
    anyAwake = false;
    const int nextStep = step + 1;
    const bool due = nextStep % kStepsPerKey == 0;
    for (int i = 1; i <= dynamicCount; ++i) {
      Body& b = bodies[i];
      if (!b.asleep) anyAwake = true;
      if (b.asleep && !b.justSlept) continue;
      if (due) fillTo(b, static_cast<size_t>(nextStep / kStepsPerKey));
      if (due || b.justSlept) pushKey(b);
    }
  }

  // Close every track on its exact rest pose.
  for (int i = 0; i < dynamicCount; ++i) {
    Body& b = bodies[1 + i];
    if (!b.asleep) {
      b.settled = false;
      pushKey(b);
    }
    // A body that sat still while its island was still awake holds the same
    // pose for the rest of the sim: drop that tail, the track ends where it
    // stopped moving.
    while (b.keys.size() > 2) {
      const DebrisKey& last = b.keys[b.keys.size() - 1];
      const DebrisKey& prev = b.keys[b.keys.size() - 2];
      if (glm::distance(last.position, prev.position) > 3e-4f ||
          std::fabs(glm::dot(last.rotation, prev.rotation)) < 1.0f - 1e-8f) {
        break;
      }
      b.keys.pop_back();
    }
    if (b.keys.size() < 2) b.keys.push_back(b.keys.back());
    tracks[i].keys = std::move(b.keys);
    tracks[i].settled = b.settled;
  }
  return tracks;
}

}  // namespace tactics
