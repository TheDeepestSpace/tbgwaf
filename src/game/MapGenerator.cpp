#include "game/MapGenerator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "game/Geometry.h"

namespace tactics {
namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kSidewalkHeight = 0.12f;  // Curb: purely visual, well under a step.

// std::mt19937 output is fully specified by the standard, but the
// <random> distributions are not, so draw values by hand to keep a seed's
// output identical across standard libraries.
class Rng {
 public:
  explicit Rng(uint32_t seed) : engine_(seed) {}
  int Int(int lo, int hi) { return lo + static_cast<int>(engine_() % static_cast<uint32_t>(hi - lo + 1)); }
  float Unit01() { return static_cast<float>(engine_() >> 8) / 16777216.0f; }
  float Float(float lo, float hi) { return lo + (hi - lo) * Unit01(); }
  bool Chance(float p) { return Unit01() < p; }

 private:
  std::mt19937 engine_;
};

void AddBuilding(Scene* scene, float x0, float z0, float x1, float z1, float height) {
  scene->obstacles.push_back(
      Obstacle{AABB{glm::vec3(x0, 0.0f, z0), glm::vec3(x1, height, z1)}, /*climbable=*/false});
}

// Splits [a, b] into `count` buildings. Junction i (between building i and
// i+1) is a gap of `gapWidth` if gaps[i], else the footprints touch.
// `startGap`/`endGap` shorten the run to leave an opening at that end.
// Calls emit(lo, hi) per building.
template <typename Emit>
void SplitRun(float a, float b, const std::vector<bool>& gaps, float gapWidth, bool startGap,
              bool endGap, Emit emit) {
  if (startGap) a += gapWidth;
  if (endGap) b -= gapWidth;
  const int count = static_cast<int>(gaps.size()) + 1;
  int gapCount = 0;
  for (bool g : gaps) gapCount += g ? 1 : 0;
  const float buildable = (b - a) - gapCount * gapWidth;
  const float each = buildable / static_cast<float>(count);
  float cursor = a;
  for (int i = 0; i < count; ++i) {
    emit(cursor, cursor + each);
    cursor += each;
    if (i < count - 1 && gaps[i]) cursor += gapWidth;
  }
}

// One axis of the street grid: `n` blocks separated by n+1 straight streets
// that fill exactly `total`. Rim streets keep the nominal width; interior
// streets vary. Block sizes vary too, but never drop below the floor.
struct Span { float start, size; };
std::vector<Span> LayoutAxis(int n, float total, const MapGeneratorConfig& c, Rng* rng) {
  std::vector<float> streets(n + 1, c.streetWidth);
  float streetSum = 2.0f * c.streetWidth;
  for (int i = 1; i < n; ++i) {
    streets[i] = c.streetWidth * rng->Float(c.streetWidthMin, c.streetWidthMax);
    streetSum += streets[i];
  }
  const float blockSum = total - streetSum;
  const float extra = std::max(0.0f, blockSum - n * c.minBlockSize);
  std::vector<float> weights(n);
  float weightSum = 0.0f;
  for (int i = 0; i < n; ++i) weightSum += weights[i] = rng->Float(0.5f, 1.5f);
  std::vector<Span> spans;
  float cursor = -total * 0.5f;
  for (int i = 0; i < n; ++i) {
    cursor += streets[i];
    const float size = extra > 0.0f ? c.minBlockSize + extra * weights[i] / weightSum
                                    : blockSum / static_cast<float>(n);
    spans.push_back({cursor, size});
    cursor += size;
  }
  return spans;
}

struct Layout {
  std::vector<Span> xs, zs;
};

Layout LayoutCity(const MapGeneratorConfig& c, Rng* rng) {
  const float total = 2.0f * UrbanMapHalfExtent(c);
  Layout layout;
  layout.xs = LayoutAxis(c.blocksX, total, c, rng);
  layout.zs = LayoutAxis(c.blocksZ, total, c, rng);
  return layout;
}

// A lot is one or more grid cells joined into a single city block (the
// streets between them are built over): a rectangle of 1-3 cells, an
// L of three cells around a 2x2 corner, or an empty open area.
struct Lot {
  int cx0, cx1, cz0, cz1;      // Inclusive cell ranges (the bounding box).
  bool empty = false;          // Open area: sidewalks only, no buildings.
  bool notch = false;          // L-shape: the 2x2 box minus one corner cell.
  bool missingEast = false, missingNorth = false;  // Which corner cell is missing.
};

std::vector<Lot> AssignLots(const MapGeneratorConfig& c, Rng* rng) {
  const int nx = c.blocksX, nz = c.blocksZ;
  std::vector<int> owner(static_cast<size_t>(nx) * nz, -1);
  auto at = [&](int x, int z) -> int& { return owner[x * nz + z]; };
  // The middle east-west street stays open so the spawn rows keep a clear run.
  const int midRow = nz >= 2 ? nz / 2 : -1;
  auto crossesMid = [&](int z0, int z1) { return midRow >= 0 && z0 < midRow && z1 >= midRow; };
  std::vector<Lot> lots;
  auto free = [&](int x0, int x1, int z0, int z1) {
    if (x0 < 0 || z0 < 0 || x1 >= nx || z1 >= nz || crossesMid(z0, z1)) return false;
    for (int x = x0; x <= x1; ++x)
      for (int z = z0; z <= z1; ++z)
        if (at(x, z) >= 0) return false;
    return true;
  };
  auto claim = [&](Lot lot) {
    const int id = static_cast<int>(lots.size());
    for (int x = lot.cx0; x <= lot.cx1; ++x)
      for (int z = lot.cz0; z <= lot.cz1; ++z) {
        const bool missing = lot.notch && x == (lot.missingEast ? lot.cx1 : lot.cx0) &&
                             z == (lot.missingNorth ? lot.cz0 : lot.cz1);
        if (!missing) at(x, z) = id;
      }
    lots.push_back(lot);
  };
  for (int x = 0; x < nx; ++x) {
    for (int z = 0; z < nz; ++z) {
      if (at(x, z) >= 0) continue;
      const float roll = rng->Unit01();
      const bool alongX = rng->Chance(0.5f);
      const int len = rng->Chance(0.3f) ? 3 : 2;
      const int corner = rng->Int(0, 3);
      Lot lot{x, x, z, z};
      if (roll < 0.10f) {
        lot.empty = true;
      } else if (roll < 0.30f) {  // Two or three cells in a row, plus the roads between.
        const int ex = alongX ? x + len - 1 : x, ez = alongX ? z : z + len - 1;
        if (free(x, ex, z, ez)) { lot.cx1 = ex; lot.cz1 = ez; }
      } else if (roll < 0.42f) {  // Three cells at an angle.
        // The 2x2 box anchored at (x, z) with one corner cell dropped; only
        // corners other than the anchor cell itself, which we know is free.
        if (free(x, x + 1, z, z + 1)) {
          lot.cx1 = x + 1; lot.cz1 = z + 1; lot.notch = true;
          lot.missingEast = corner & 1;
          lot.missingNorth = corner & 2;
          if (!lot.missingEast && lot.missingNorth) lot.missingEast = true;  // Keep the anchor.
        }
      }
      claim(lot);
    }
  }
  return lots;
}

float PointSegmentDistance(glm::vec2 p, glm::vec2 a, glm::vec2 b) {
  const glm::vec2 ab = b - a;
  const float denom = glm::dot(ab, ab);
  const float t = denom > 1e-8f ? std::clamp(glm::dot(p - a, ab) / denom, 0.0f, 1.0f) : 0.0f;
  return glm::distance(p, a + ab * t);
}

UrbanBlock MakeUrbanBlock(std::vector<glm::vec2> vertices) {
  if (PolygonSignedArea(vertices) < 0.0f) std::reverse(vertices.begin(), vertices.end());
  UrbanBlock block;
  block.vertices = std::move(vertices);
  block.x0 = block.z0 = std::numeric_limits<float>::infinity();
  block.x1 = block.z1 = -std::numeric_limits<float>::infinity();
  for (const glm::vec2& p : block.vertices) {
    block.x0 = std::min(block.x0, p.x);
    block.x1 = std::max(block.x1, p.x);
    block.z0 = std::min(block.z0, p.y);
    block.z1 = std::max(block.z1, p.y);
  }
  return block;
}

// --- Hierarchical oblique city (issue #92). ---

constexpr float kGradeY = 0.025f;        // Pavement lift over the base plane (avoids z-fighting).
constexpr float kRoadSampleStep = 3.0f;  // Centerline sampling; short enough for smooth ramps.
// How far an elevated deck end that runs off-map continues past the map
// bounds, so the edge shows a deck passing through rather than a cut stub.
constexpr float kDeckOverhang = 12.0f;

// Presence and layout draws (issue #130) use their own stream so forcing
// either never reshuffles the rest of a seed's layout; both are drawn
// unconditionally for the same reason.
OverpassChoice ResolveOverpass(uint32_t seed, const MapGeneratorConfig& c) {
  Rng rng(seed ^ 0x0ebe7a55u);
  const bool drawnPresence = rng.Chance(c.overpassChance);
  const auto drawnLayout = static_cast<OverpassLayout>(1 + rng.Int(0, 3));
  OverpassChoice choice;
  choice.elevated = c.elevatedHighway == OverpassMode::Auto ? drawnPresence
                                                            : c.elevatedHighway == OverpassMode::On;
  choice.layout =
      c.overpassLayout == OverpassLayout::Auto ? drawnLayout : c.overpassLayout;
  return choice;
}

glm::vec2 Rotate(glm::vec2 v, float radians) {
  const float c = std::cos(radians), s = std::sin(radians);
  return {c * v.x - s * v.y, s * v.x + c * v.y};
}

// Parameter range of origin + t*dir inside the [-half, half]^2 square.
void ClipLineToSquare(glm::vec2 origin, glm::vec2 dir, float half, float* tMin, float* tMax) {
  *tMin = -std::numeric_limits<float>::infinity();
  *tMax = std::numeric_limits<float>::infinity();
  for (int axis = 0; axis < 2; ++axis) {
    const float o = axis ? origin.y : origin.x;
    const float d = axis ? dir.y : dir.x;
    if (std::fabs(d) < 1e-6f) continue;
    const float t0 = (-half - o) / d, t1 = (half - o) / d;
    *tMin = std::max(*tMin, std::min(t0, t1));
    *tMax = std::min(*tMax, std::max(t0, t1));
  }
}

// The through-highway and its branching avenue as straight centerlines, so
// the street grid can carve convex blocks against them exactly.
struct ArterySpec {
  glm::vec2 origin{0.0f}, dir{1.0f, 0.0f}, normal{0.0f, 1.0f};
  float sMin = 0.0f, sMax = 0.0f;  // Boundary-to-boundary params.
  // Sampled range: equals [sMin, sMax] at grade; an elevated end that runs
  // off-map extends kDeckOverhang past its boundary.
  float s0 = 0.0f, s1 = 0.0f;
  float rise0 = 0.0f, rise1 = 0.0f, fall0 = 0.0f, fall1 = 0.0f;  // Overpass profile params.
};

struct BranchSpec {
  glm::vec2 start{0.0f};       // On the map boundary.
  glm::vec2 dir{1.0f, 0.0f};   // Unit, start -> merge.
  float length = 0.0f;
  float width = 0.0f;
  float mergeS = 0.0f;         // Artery parameter of the merge point.
  float riseStart = 0.0f;      // Branch parameter where the on-ramp starts climbing.
  bool offMap = false;         // Stays at deck height and runs off-map; no ramp.
  float u0 = 0.0f;             // First sampled param; -kDeckOverhang when off-map.
  int side = 1;                // Sign of dot(artery normal, branch - artery origin).
};

struct HighwayPlan {
  ArterySpec artery;
  bool elevated = false;
  OverpassLayout layout = OverpassLayout::RampUpRampDown;
  bool hasBranch = false;
  BranchSpec branch;
};

// Where the branch meets the artery, as a fraction of the artery's on-map
// length (mapped from one unit draw so layouts never shift the rng stream).
// On an elevated artery the merge must land where the deck is at full
// height, so the branch genuinely ramps between grade and deck.
float MergeFraction(const HighwayPlan& plan, float t) {
  if (!plan.elevated) return 0.30f + 0.14f * t;
  switch (plan.layout) {
    case OverpassLayout::EnterRampUp: return 0.62f + 0.12f * t;   // Deck from ~0.52 on.
    case OverpassLayout::EnterRampDown: return 0.26f + 0.12f * t; // Deck until ~0.48.
    default: return 0.46f + 0.08f * t;                            // Mid-span deck.
  }
}

HighwayPlan BuildHighwayPlan(uint32_t seed, const MapGeneratorConfig& c) {
  Rng rng(seed ^ 0xa17e2d31u);
  const float half = UrbanMapHalfExtent(c);
  HighwayPlan plan;
  const OverpassChoice overpass = ResolveOverpass(seed, c);
  plan.elevated = overpass.elevated;
  plan.layout = overpass.layout;
  ArterySpec& artery = plan.artery;
  const float angle = glm::radians(rng.Float(11.0f, 21.0f)) * (rng.Chance(0.5f) ? 1.0f : -1.0f);
  artery.dir = glm::vec2(std::cos(angle), std::sin(angle));
  artery.normal = glm::vec2(-artery.dir.y, artery.dir.x);
  artery.origin = artery.normal * (half * rng.Float(-0.12f, 0.12f));
  ClipLineToSquare(artery.origin, artery.dir, half, &artery.sMin, &artery.sMax);
  const float length = artery.sMax - artery.sMin;
  artery.s0 = artery.sMin;
  artery.s1 = artery.sMax;
  // Deck profile per layout. A ramp a layout never takes is parked far
  // outside the sampled range, so the shared smoothstep profile formula
  // (rise minus fall) applies unchanged to every layout.
  const float far = 100.0f * length;
  artery.rise0 = artery.sMin + 0.10f * length;
  artery.rise1 = artery.sMin + 0.36f * length;
  artery.fall0 = artery.sMin + 0.64f * length;
  artery.fall1 = artery.sMin + 0.90f * length;
  if (plan.elevated) {
    switch (plan.layout) {
      case OverpassLayout::Auto:  // Resolved above; both ramps on-map.
      case OverpassLayout::RampUpRampDown:
        break;
      case OverpassLayout::Through:
        artery.s0 = artery.sMin - kDeckOverhang;
        artery.s1 = artery.sMax + kDeckOverhang;
        artery.rise0 = -far;
        artery.rise1 = -far + 1.0f;
        artery.fall0 = far;
        artery.fall1 = far + 1.0f;
        break;
      case OverpassLayout::EnterRampUp:
        artery.s1 = artery.sMax + kDeckOverhang;
        artery.rise0 = artery.sMin + 0.22f * length;
        artery.rise1 = artery.sMin + 0.52f * length;
        artery.fall0 = far;
        artery.fall1 = far + 1.0f;
        break;
      case OverpassLayout::EnterRampDown:
        artery.s0 = artery.sMin - kDeckOverhang;
        artery.rise0 = -far;
        artery.rise1 = -far + 1.0f;
        artery.fall0 = artery.sMin + 0.48f * length;
        artery.fall1 = artery.sMin + 0.78f * length;
        break;
    }
  }

  plan.hasBranch = c.arteryCount >= 2 || plan.elevated;
  // Branch draws happen unconditionally so toggling the branch or the deck
  // never reshuffles the rest of a seed's layout.
  const float mergeFraction = MergeFraction(plan, rng.Unit01());
  const int preferredTurn = rng.Chance(0.5f) ? 1 : -1;
  const float turn = glm::radians(rng.Float(30.0f, 44.0f));
  if (plan.hasBranch) {
    BranchSpec& branch = plan.branch;
    branch.width = c.arteryWidth * 0.72f;
    branch.mergeS = artery.sMin + mergeFraction * length;
    const glm::vec2 merge = artery.origin + artery.dir * branch.mergeS;
    for (const int turnSign : {preferredTurn, -preferredTurn}) {
      branch.dir = Rotate(artery.dir, static_cast<float>(turnSign) * turn);
      float tMin = 0.0f, tMax = 0.0f;
      ClipLineToSquare(merge, -branch.dir, half, &tMin, &tMax);
      branch.length = tMax;  // Distance back from the merge to the boundary.
      if (branch.length > half * 0.75f) break;
    }
    branch.start = merge - branch.dir * branch.length;
    branch.side = glm::dot(artery.normal, branch.start - artery.origin) > 0.0f ? 1 : -1;
    // Ramp shape and off-map ending draw from their own stream so they never
    // reshuffle the rest of a seed's layout. The off-map branch never ramps
    // down to grade, so it needs the artery to offer a way up (not Through).
    Rng branchRng(seed ^ 0x5b2a9c47u);
    const bool drawnOffMap = branchRng.Chance(c.branchOffMapChance);
    const float rampFraction = branchRng.Float(0.30f, 0.75f);
    const float rampMax = branchRng.Float(30.0f, 70.0f);
    const bool canOffMap = plan.elevated && plan.layout != OverpassLayout::Through;
    branch.offMap = canOffMap && (c.branchEnd == BranchEnd::Auto ? drawnOffMap
                                                                 : c.branchEnd == BranchEnd::OffMap);
    branch.riseStart = branch.length - std::min(branch.length * rampFraction, rampMax);
    if (branch.offMap) {
      branch.riseStart = 0.0f;
      branch.u0 = -kDeckOverhang;
    }
  }
  return plan;
}

float SmoothStep01(float t) {
  t = std::clamp(t, 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

// Overpass profile: the deck sits at highwayElevation between its rise and
// fall ramps and at grade outside them; layouts park a ramp they never take
// far outside the sampled range (through/off-map ends stay at deck height).
float ArteryElevationAt(const HighwayPlan& plan, const MapGeneratorConfig& c, float s) {
  if (!plan.elevated) return kGradeY;
  const ArterySpec& a = plan.artery;
  const float profile = SmoothStep01((s - a.rise0) / (a.rise1 - a.rise0)) -
                        SmoothStep01((s - a.fall0) / (a.fall1 - a.fall0));
  return kGradeY + (c.highwayElevation - kGradeY) * profile;
}

float BranchElevationAt(const HighwayPlan& plan, const MapGeneratorConfig& c, float u) {
  if (!plan.elevated) return kGradeY;
  const float top = ArteryElevationAt(plan, c, plan.branch.mergeS);
  if (plan.branch.offMap) return top;
  const float run = std::max(1.0f, plan.branch.length - plan.branch.riseStart);
  return kGradeY + (top - kGradeY) * SmoothStep01((u - plan.branch.riseStart) / run);
}

std::vector<float> SampleParams(float s0, float s1) {
  const int segments = std::max(1, static_cast<int>(std::ceil((s1 - s0) / kRoadSampleStep)));
  std::vector<float> params(segments + 1);
  for (int i = 0; i <= segments; ++i) {
    params[i] = s0 + (s1 - s0) * static_cast<float>(i) / static_cast<float>(segments);
  }
  return params;
}

std::vector<UrbanRoad> BuildUrbanRoads(uint32_t seed, const MapGeneratorConfig& c) {
  const HighwayPlan plan = BuildHighwayPlan(seed, c);
  std::vector<UrbanRoad> roads;
  UrbanRoad artery;
  artery.width = c.arteryWidth;
  artery.artery = true;
  artery.elevated = plan.elevated;
  for (const float s : SampleParams(plan.artery.s0, plan.artery.s1)) {
    const glm::vec2 p = plan.artery.origin + plan.artery.dir * s;
    artery.centerline.emplace_back(p.x, ArteryElevationAt(plan, c, s), p.y);
  }
  roads.push_back(std::move(artery));
  if (plan.hasBranch) {
    UrbanRoad branch;
    branch.width = plan.branch.width;
    branch.artery = true;
    branch.elevated = plan.elevated;
    for (const float u : SampleParams(plan.branch.u0, plan.branch.length)) {
      const glm::vec2 p = plan.branch.start + plan.branch.dir * u;
      branch.centerline.emplace_back(p.x, BranchElevationAt(plan, c, u), p.y);
    }
    roads.push_back(std::move(branch));
  }
  return roads;
}

// Sutherland-Hodgman against one half-plane: keeps {p : dot(n, p) >= dMin}.
std::vector<glm::vec2> KeepSide(const std::vector<glm::vec2>& polygon, glm::vec2 n, float dMin) {
  std::vector<glm::vec2> out;
  for (size_t i = 0; i < polygon.size(); ++i) {
    const glm::vec2 a = polygon[i];
    const glm::vec2 b = polygon[(i + 1) % polygon.size()];
    const float da = glm::dot(n, a) - dMin;
    const float db = glm::dot(n, b) - dMin;
    if (da >= 0.0f) out.push_back(a);
    if ((da >= 0.0f) != (db >= 0.0f) && std::fabs(da - db) > 1e-7f) {
      out.push_back(a + (b - a) * (da / (da - db)));
    }
  }
  return out;
}

// Carves a street band of `width` around the line origin + t*dir out of
// every region, keeping the convex pieces on both sides.
void CutStreet(std::vector<std::vector<glm::vec2>>* regions, glm::vec2 origin, glm::vec2 dir,
               float width) {
  const glm::vec2 n(-dir.y, dir.x);
  const float d = glm::dot(n, origin);
  std::vector<std::vector<glm::vec2>> next;
  for (const auto& region : *regions) {
    for (const auto& piece : {KeepSide(region, n, d + width * 0.5f),
                              KeepSide(region, -n, -(d - width * 0.5f))}) {
      if (piece.size() >= 3 && PolygonArea(piece) > 1.0f) next.push_back(piece);
    }
  }
  *regions = std::move(next);
}

// Convex polygon blocks: the rim rectangle is split by the artery corridor,
// the branch corridor, avenues roughly parallel to the artery and oblique
// cross streets. Every surviving region is a block whose shape follows
// whatever angles the streets around it happen to make.
std::vector<UrbanBlock> BuildCityBlocks(uint32_t seed, const MapGeneratorConfig& c) {
  const HighwayPlan plan = BuildHighwayPlan(seed, c);
  const ArterySpec& artery = plan.artery;
  Rng rng(seed ^ 0xb10c5eedu);
  const float half = UrbanMapHalfExtent(c);
  const float rim = std::max(c.streetWidth, c.localStreetWidth);
  const float lo = -half + rim, hi = half - rim;
  const std::vector<glm::vec2> rimRect = {{lo, lo}, {hi, lo}, {hi, hi}, {lo, hi}};
  const float band = c.arteryWidth * 0.5f + c.localStreetWidth * 0.45f;
  const float arteryD = glm::dot(artery.normal, artery.origin);

  std::vector<UrbanBlock> blocks;
  for (const int side : {1, -1}) {
    const glm::vec2 n = artery.normal * static_cast<float>(side);
    std::vector<std::vector<glm::vec2>> regions;
    std::vector<glm::vec2> base = KeepSide(rimRect, n, static_cast<float>(side) * arteryD + band);
    if (base.size() >= 3) regions.push_back(std::move(base));
    if (plan.hasBranch && plan.branch.side == side) {
      CutStreet(&regions, plan.branch.start, plan.branch.dir,
                plan.branch.width + c.localStreetWidth * 0.8f);
    }
    // Avenues roughly parallel to the artery.
    float offset = band + c.blockSize * rng.Float(0.8f, 1.1f);
    while (offset < half * 1.5f) {
      const glm::vec2 dir = Rotate(artery.dir, rng.Float(-0.09f, 0.09f));
      const float width = c.localStreetWidth * rng.Float(0.85f, 1.25f);
      CutStreet(&regions, artery.origin + artery.normal * (static_cast<float>(side) * offset),
                dir, width);
      offset += c.blockSize * rng.Float(0.85f, 1.25f) + width;
    }
    // Oblique cross streets. Each side draws its own, so junctions don't
    // line up across the artery and the block shapes differ everywhere.
    float s = artery.sMin + c.blockSize * rng.Float(0.55f, 0.9f);
    while (s < artery.sMax - c.blockSize * 0.4f) {
      const glm::vec2 dir =
          Rotate(artery.normal, rng.Float(-c.localStreetSkew, c.localStreetSkew));
      const float width = c.localStreetWidth * rng.Float(0.85f, 1.25f);
      CutStreet(&regions, artery.origin + artery.dir * s, dir, width);
      s += c.blockSize * rng.Float(0.9f, 1.35f) + width;
    }
    for (auto& region : regions) {
      if (PolygonArea(region) < c.minPolygonArea) continue;
      blocks.push_back(MakeUrbanBlock(std::move(region)));
    }
  }
  return blocks;
}

// Ribbon edge; an end vertex sitting exactly on the map boundary slides
// along the road direction onto it so the oblique road is cut off flush
// with the edge. Ends past the boundary (an elevated layout's off-map
// overhang) keep their square cut beyond the map instead.
std::vector<glm::vec3> RibbonSide(const UrbanRoad& road, bool left, float half) {
  std::vector<glm::vec3> side;
  side.reserve(road.centerline.size());
  for (size_t i = 0; i < road.centerline.size(); ++i) {
    const glm::vec3& prev = road.centerline[i == 0 ? i : i - 1];
    const glm::vec3& next = road.centerline[i + 1 < road.centerline.size() ? i + 1 : i];
    glm::vec2 tangent(next.x - prev.x, next.z - prev.z);
    if (glm::length(tangent) < 1e-5f) tangent = glm::vec2(1.0f, 0.0f);
    tangent = glm::normalize(tangent);
    const glm::vec2 normal(-tangent.y, tangent.x);
    const float sign = left ? 1.0f : -1.0f;
    const glm::vec3& p = road.centerline[i];
    glm::vec2 q(p.x + sign * normal.x * road.width * 0.5f, p.z + sign * normal.y * road.width * 0.5f);
    const bool first = i == 0, last = i + 1 == road.centerline.size();
    const bool onEdge = std::fabs(std::max(std::fabs(p.x), std::fabs(p.z)) - half) < 1e-2f;
    if ((first || last) && onEdge) {
      float tMin = 0.0f, tMax = 0.0f;
      ClipLineToSquare(q, tangent, half, &tMin, &tMax);
      q += tangent * (first ? tMin : tMax);
    }
    side.emplace_back(q.x, p.y, q.y);
  }
  return side;
}

void AddPolygonBuilding(Scene* scene, std::vector<glm::vec2> footprint, float height) {
  if (footprint.size() < 3 || PolygonArea(footprint) < 1.0f) return;
  if (PolygonSignedArea(footprint) < 0.0f) std::reverse(footprint.begin(), footprint.end());
  AABB bounds;
  bounds.min = glm::vec3(std::numeric_limits<float>::infinity(), 0.0f,
                         std::numeric_limits<float>::infinity());
  bounds.max = glm::vec3(-std::numeric_limits<float>::infinity(), height,
                         -std::numeric_limits<float>::infinity());
  for (const glm::vec2& p : footprint) {
    bounds.min.x = std::min(bounds.min.x, p.x);
    bounds.max.x = std::max(bounds.max.x, p.x);
    bounds.min.z = std::min(bounds.min.z, p.y);
    bounds.max.z = std::max(bounds.max.z, p.y);
  }
  scene->obstacles.push_back(Obstacle{bounds, false, std::move(footprint)});
}

bool PolygonClearOfRoads(const std::vector<glm::vec2>& polygon,
                         const std::vector<UrbanRoad>& roads, float extra) {
  for (const UrbanRoad& road : roads) {
    const float clearance = road.width * 0.5f + extra;
    for (size_t i = 0; i + 1 < road.centerline.size(); ++i) {
      const glm::vec2 a(road.centerline[i].x, road.centerline[i].z);
      const glm::vec2 b(road.centerline[i + 1].x, road.centerline[i + 1].z);
      if (PointInConvexPolygon(a, polygon) || PointInConvexPolygon(b, polygon)) return false;
      for (size_t edge = 0; edge < polygon.size(); ++edge) {
        if (PointSegmentDistance(polygon[edge], a, b) < clearance) return false;
        const glm::vec2 midpoint = (polygon[edge] + polygon[(edge + 1) % polygon.size()]) * 0.5f;
        if (PointSegmentDistance(midpoint, a, b) < clearance) return false;
      }
    }
  }
  return true;
}

// Frontage-following building rows: each block edge gets one or more
// buildings whose outer walls lie on the (possibly angled) frontage, with
// wall-to-wall junctions, seeded alley gaps and at least one opening into
// the courtyard. Acute wedge blocks whose inner ring collapses become one
// real angled building; blocks whose first inset collapses stay open.
void EmitBlockBuildings(Scene* scene, const UrbanBlock& block, size_t blockIndex,
                        const std::vector<UrbanRoad>& arteries, const MapGeneratorConfig& c,
                        Rng* rng) {
  RoadSurface sidewalk;
  for (const glm::vec2& p : block.vertices) {
    sidewalk.vertices.emplace_back(p.x, kSidewalkHeight, p.y);
  }
  scene->sidewalkSurfaces.push_back(std::move(sidewalk));
  const bool park = rng->Chance(0.10f);  // Pocket park/open lot.
  const float cluster = rng->Float(c.minBuildingHeight, c.maxBuildingHeight);
  if (park) return;
  std::vector<glm::vec2> outer = InsetConvexPolygon(block.vertices, c.sidewalkWidth);
  if (outer.size() < 3 || PolygonArea(outer) < c.minPolygonArea ||
      !PolygonClearOfRoads(outer, arteries, c.sidewalkWidth)) {
    return;
  }
  std::vector<glm::vec2> inner = InsetConvexPolygon(outer, c.buildingDepth);
  const std::vector<glm::vec2> courtyard = InsetConvexPolygon(outer, c.buildingDepth + 1.2f);
  // A deep inset can collapse an edge into a self-crossing "bowtie" while
  // keeping the vertex count; a usable ring needs every inner edge to still
  // run alongside its frontage and the courtyard to stay convex.
  auto ringUsable = [&]() {
    if (inner.size() != outer.size() || courtyard.size() < 3 ||
        PolygonArea(courtyard) < c.minPolygonArea * 0.5f) {
      return false;
    }
    for (size_t e = 0; e < outer.size(); ++e) {
      const size_t next = (e + 1) % outer.size();
      if (glm::dot(outer[next] - outer[e], inner[next] - inner[e]) <= 0.0f) return false;
      const glm::vec2 a = inner[e] - inner[(e + outer.size() - 1) % outer.size()];
      const glm::vec2 b = inner[next] - inner[e];
      if (a.x * b.y - a.y * b.x <= 0.0f) return false;
    }
    return true;
  };
  if (!ringUsable()) {
    // Too shallow for a courtyard ring (opposite frontages would overlap):
    // a row of slab buildings along the block's longest frontage instead.
    // An acute wedge block keeps its sharp corner in the end slab.
    glm::vec2 axis(1.0f, 0.0f);
    float longest = 0.0f;
    for (size_t e = 0; e < outer.size(); ++e) {
      const glm::vec2 d = outer[(e + 1) % outer.size()] - outer[e];
      const float length = glm::length(d);
      if (length > longest) {
        longest = length;
        axis = d / length;
      }
    }
    float lo = std::numeric_limits<float>::infinity(), hi = -lo;
    for (const glm::vec2& p : outer) {
      lo = std::min(lo, glm::dot(axis, p));
      hi = std::max(hi, glm::dot(axis, p));
    }
    const int count = std::clamp(static_cast<int>(std::lround((hi - lo) / 14.0f)), 1, 4);
    std::vector<std::vector<glm::vec2>> slabs = {outer};
    for (int i = 1; i < count; ++i) {
      CutStreet(&slabs, axis * (lo + (hi - lo) * static_cast<float>(i) / count),
                glm::vec2(-axis.y, axis.x), c.gapWidth);
    }
    for (auto& slab : slabs) {
      if (PolygonArea(slab) < c.minPolygonArea * 0.35f) continue;
      AddPolygonBuilding(scene, std::move(slab),
                         std::clamp(cluster + rng->Float(-1.2f, 1.2f), c.minBuildingHeight,
                                    c.maxBuildingHeight));
    }
    return;
  }

  // Plan every junction first so the block is guaranteed an opening.
  const size_t edges = outer.size();
  std::vector<bool> cornerGap(edges);
  std::vector<std::vector<bool>> innerGap(edges);
  std::vector<int> pieces(edges);
  bool anyGap = false;
  for (size_t e = 0; e < edges; ++e) {
    const float length = glm::distance(outer[e], outer[(e + 1) % edges]);
    const int count = std::clamp(static_cast<int>(std::lround(length / 12.0f)), 1, 5);
    pieces[e] = length < c.gapWidth + 2.5f ? 0 : count;
    innerGap[e].assign(static_cast<size_t>(std::max(0, count - 1)), false);
    for (size_t j = 0; j < innerGap[e].size(); ++j) {
      innerGap[e][j] = rng->Chance(c.gapProbability);
      anyGap |= innerGap[e][j];
    }
    cornerGap[e] = rng->Chance(c.gapProbability * 0.6f);
    anyGap |= cornerGap[e];
  }
  if (!anyGap) cornerGap[blockIndex % edges] = true;

  for (size_t e = 0; e < edges; ++e) {
    if (!pieces[e]) continue;
    const size_t next = (e + 1) % edges;
    const float length = glm::distance(outer[e], outer[next]);
    const int count = pieces[e];
    std::vector<float> bounds(count + 1);
    for (int i = 0; i <= count; ++i) {
      bounds[i] = static_cast<float>(i) / static_cast<float>(count);
    }
    for (int i = 1; i < count; ++i) {
      bounds[i] += rng->Float(-0.12f, 0.12f) / static_cast<float>(count);
    }
    const float halfGapT = c.gapWidth * 0.5f / length;
    // Party walls drop perpendicularly from the frontage onto the inner ring
    // (instead of converging on the block centroid); an occasional wall gets
    // a lateral skew, shared across the junction so alleys keep parallel
    // walls. Corner pieces still clamp onto the inner ring's miter.
    const glm::vec2 innerDelta = inner[next] - inner[e];
    const float innerLen = glm::length(innerDelta);
    const glm::vec2 innerDir = innerDelta / innerLen;
    std::vector<float> wallSkew(count + 1, 0.0f);
    for (int i = 1; i < count; ++i) {
      if (rng->Chance(0.2f)) wallSkew[i] = rng->Float(-0.5f, 0.5f) * c.buildingDepth;
    }
    auto innerU = [&](float t, float skew) {
      const glm::vec2 foot = glm::mix(outer[e], outer[next], t);
      return std::clamp(glm::dot(foot - inner[e], innerDir) + skew, 0.0f, innerLen);
    };
    for (int i = 0; i < count; ++i) {
      float ta = bounds[i], tb = bounds[i + 1];
      if (i == 0 && cornerGap[e]) ta += halfGapT;
      if (i == count - 1 && cornerGap[next]) tb -= halfGapT;
      if (i > 0 && innerGap[e][i - 1]) ta += halfGapT;
      if (i < count - 1 && innerGap[e][i]) tb -= halfGapT;
      if (tb - ta < 0.04f) continue;
      // A hair's width of separation at the corners (invisible at render
      // scale) keeps adjacent quads' radial sides from running exactly
      // along one another, which exact-collinearity tests can't classify.
      ta = std::max(ta, 0.0005f);
      tb = std::min(tb, 0.9995f);
      float ua = innerU(ta, wallSkew[i]), ub = innerU(tb, wallSkew[i + 1]);
      if (ub < ua) ua = ub = (ua + ub) * 0.5f;
      std::vector<glm::vec2> footprint = {
          glm::mix(outer[e], outer[next], ta), glm::mix(outer[e], outer[next], tb),
          inner[e] + innerDir * ub, inner[e] + innerDir * ua};
      // A back wall pinched to a point becomes a (still convex) triangle.
      if (ub - ua < 0.05f) footprint.pop_back();
      if (PolygonArea(footprint) < c.minPolygonArea * 0.18f) continue;
      AddPolygonBuilding(scene, std::move(footprint),
                         std::clamp(cluster + rng->Float(-1.2f, 1.2f), c.minBuildingHeight,
                                    c.maxBuildingHeight));
    }
  }
}

Scene GenerateArterialUrbanMap(uint32_t seed, const MapGeneratorConfig& c) {
  Rng rng(seed);
  Scene scene;
  scene.mapHalfExtent = UrbanMapHalfExtent(c);
  const HighwayPlan plan = BuildHighwayPlan(seed, c);
  const std::vector<UrbanRoad> roads = BuildUrbanRoads(seed, c);

  // Pavement and deck layers come from the same mitered ribbons. At-grade
  // stretches stay ordinary ground roads; everything above grade becomes an
  // explicit WalkSurface chain so the deck never erases the ground beneath.
  struct Chain {
    int first = -1, last = -1;
    std::vector<float> loS, hiS;  // Centerline param range per chain surface.
  };
  std::vector<Chain> chains(roads.size());
  for (size_t roadIndex = 0; roadIndex < roads.size(); ++roadIndex) {
    const UrbanRoad& road = roads[roadIndex];
    const auto left = RibbonSide(road, true, scene.mapHalfExtent);
    const auto right = RibbonSide(road, false, scene.mapHalfExtent);
    const std::vector<float> params = roadIndex == 0
                                          ? SampleParams(plan.artery.s0, plan.artery.s1)
                                          : SampleParams(plan.branch.u0, plan.branch.length);
    Chain& chain = chains[roadIndex];
    for (size_t i = 0; i + 1 < road.centerline.size(); ++i) {
      const bool deck =
          plan.elevated &&
          std::max(road.centerline[i].y, road.centerline[i + 1].y) > kGradeY + 0.05f;
      if (!deck) {
        scene.roads.push_back(RoadSurface{{left[i], right[i], right[i + 1], left[i + 1]}});
        continue;
      }
      WalkSurface surface;
      surface.vertices = {left[i], right[i], right[i + 1], left[i + 1]};
      const int index = static_cast<int>(scene.walkSurfaces.size());
      if (chain.first < 0) chain.first = index;
      if (chain.last >= 0) {
        surface.neighbors.push_back(chain.last);
        scene.walkSurfaces[chain.last].neighbors.push_back(index);
      }
      chain.last = index;
      chain.loS.push_back(params[i]);
      chain.hiS.push_back(params[i + 1]);
      scene.walkSurfaces.push_back(std::move(surface));
    }
  }
  if (plan.elevated && chains[0].first >= 0) {
    // A deck chain end is a legal ground transition only where the profile
    // actually returns to grade on-map (an off-map elevated end never does);
    // the connecting ramp's foot always starts at grade.
    if (ArteryElevationAt(plan, c, plan.artery.s0) < kGradeY + 0.05f) {
      scene.walkSurfaces[chains[0].first].connectsToGround = true;
    }
    if (ArteryElevationAt(plan, c, plan.artery.s1) < kGradeY + 0.05f) {
      scene.walkSurfaces[chains[0].last].connectsToGround = true;
    }
    if (roads.size() > 1 && chains[1].first >= 0) {
      if (!plan.branch.offMap) scene.walkSurfaces[chains[1].first].connectsToGround = true;
      int deckAtMerge = -1;
      for (size_t i = 0; i < chains[0].loS.size(); ++i) {
        if (plan.branch.mergeS >= chains[0].loS[i] - 1e-3f &&
            plan.branch.mergeS <= chains[0].hiS[i] + 1e-3f) {
          deckAtMerge = chains[0].first + static_cast<int>(i);
        }
      }
      if (deckAtMerge >= 0) {
        // The on-ramp's top merges onto the deck span holding its merge point.
        scene.walkSurfaces[chains[1].last].neighbors.push_back(deckAtMerge);
        scene.walkSurfaces[deckAtMerge].neighbors.push_back(chains[1].last);
      }
    }
  }

  // Bridge stands: paired pier columns under the high spans, leaving broad
  // navigable ground between bents. None near the on-ramp's merge, where the
  // ramp slab sweeps below deck level.
  if (plan.elevated) {
    // A column may only stand where it stays below every slab crossing it
    // (and never on at-grade pavement, whose slab sits at ground level).
    auto columnFits = [&](glm::vec2 foot, float top) {
      const float sA = glm::dot(foot - plan.artery.origin, plan.artery.dir);
      const float dA = std::fabs(glm::dot(foot - plan.artery.origin, plan.artery.normal));
      if (dA < c.arteryWidth * 0.5f + 0.8f &&
          top > ArteryElevationAt(plan, c, sA) - c.highwayThickness - 0.049f) {
        return false;
      }
      if (plan.hasBranch) {
        const glm::vec2 rel = foot - plan.branch.start;
        const float u = glm::dot(rel, plan.branch.dir);
        const glm::vec2 bn(-plan.branch.dir.y, plan.branch.dir.x);
        if (u > -0.5f && u < plan.branch.length + 0.5f &&
            std::fabs(glm::dot(rel, bn)) < plan.branch.width * 0.5f + 0.8f &&
            top > BranchElevationAt(plan, c, u) - c.highwayThickness - 0.049f) {
          return false;
        }
      }
      return true;
    };
    auto addBent = [&](glm::vec2 center, glm::vec2 across, float halfSpan, float deckY) {
      for (const float lat : {1.0f, -1.0f}) {
        const glm::vec2 foot = center + across * (lat * halfSpan);
        const float top = deckY - c.highwayThickness - 0.05f;
        // A column stands fully on the map even where the deck runs past it.
        if (std::max(std::fabs(foot.x), std::fabs(foot.y)) > scene.mapHalfExtent - 0.75f) continue;
        if (!columnFits(foot, top)) continue;
        scene.obstacles.push_back(
            Obstacle{AABB{glm::vec3(foot.x - 0.7f, 0.0f, foot.y - 0.7f),
                          glm::vec3(foot.x + 0.7f, top, foot.y + 0.7f)},
                     false});
      }
    };
    // Bents march outward from the map-spanning corridor's midpoint and
    // continue down any ramps while there is still head clearance below the
    // slab. Always on-map, even when the deck itself runs past the boundary.
    constexpr float kMinClearance = 2.2f;
    const float pierLo = plan.artery.sMin + 1.5f;
    const float pierHi = plan.artery.sMax - 1.5f;
    const float mid = 0.5f * (pierLo + pierHi);
    for (float s = mid - std::floor((mid - pierLo) / c.supportSpacing) * c.supportSpacing;
         s <= pierHi; s += c.supportSpacing) {
      const float deckY = ArteryElevationAt(plan, c, s);
      if (deckY < kMinClearance) continue;
      if (plan.hasBranch && std::fabs(s - plan.branch.mergeS) < c.supportSpacing * 0.6f) continue;
      addBent(plan.artery.origin + plan.artery.dir * s, plan.artery.normal,
              c.arteryWidth * 0.5f - 1.1f, deckY);
    }
    if (plan.hasBranch) {
      for (float u = std::max(0.0f, plan.branch.riseStart); u < plan.branch.length - 8.0f;
           u += c.supportSpacing * 0.75f) {
        const float deckY = BranchElevationAt(plan, c, u);
        if (deckY < kMinClearance) continue;
        addBent(plan.branch.start + plan.branch.dir * u,
                glm::vec2(-plan.branch.dir.y, plan.branch.dir.x),
                plan.branch.width * 0.5f - 0.9f, deckY);
      }
    }
  }

  const std::vector<UrbanBlock> blocks = BuildCityBlocks(seed, c);
  for (size_t blockIndex = 0; blockIndex < blocks.size(); ++blockIndex) {
    EmitBlockBuildings(&scene, blocks[blockIndex], blockIndex, roads, c, &rng);
  }

  // Landmark towers reuse actual polygon footprints rather than reverting to
  // bounding boxes.
  const int towers = std::min<int>(c.towerCount, static_cast<int>(scene.obstacles.size()));
  std::vector<size_t> candidates;
  for (size_t i = 0; i < scene.obstacles.size(); ++i) {
    if (!scene.obstacles[i].footprint.empty()) candidates.push_back(i);
  }
  for (int i = 0; i < towers && !candidates.empty(); ++i) {
    const size_t pick = static_cast<size_t>(rng.Int(0, static_cast<int>(candidates.size()) - 1));
    scene.obstacles[candidates[pick]].bounds.max.y = rng.Float(c.towerMinHeight, c.towerMaxHeight);
    candidates.erase(candidates.begin() + pick);
  }

  // Squads spawn at ground level on the artery's ends (beneath the deck
  // where an elevated layout runs off-map), facing each other down it.
  const float spawnInset = std::max(c.streetWidth, c.localStreetWidth) * 0.8f;
  const float yawBlue = std::atan2(plan.artery.dir.y, plan.artery.dir.x);
  for (int i = 0; i < 6; ++i) {
    const bool blueTeam = i < 3;
    const float s = blueTeam ? plan.artery.sMin + spawnInset : plan.artery.sMax - spawnInset;
    const float lane = static_cast<float>(i % 3 - 1) * 4.0f;
    const glm::vec2 p = plan.artery.origin + plan.artery.dir * s + plan.artery.normal * lane;
    Unit unit;
    unit.id = i;
    unit.team = blueTeam ? Team::Blue : Team::Red;
    unit.position = glm::vec3(p.x, 0.0f, p.y);
    unit.facingYaw = blueTeam ? yawBlue : yawBlue + kPi;
    scene.units.push_back(unit);
  }
  return scene;
}

}  // namespace

std::vector<UrbanBlock> UrbanBlocks(uint32_t seed, const MapGeneratorConfig& c) {
  if (c.arteryCount > 0) return BuildCityBlocks(seed, c);
  Rng rng(seed);
  const Layout layout = LayoutCity(c, &rng);
  std::vector<UrbanBlock> blocks;
  for (const Span& x : layout.xs) {
    for (const Span& z : layout.zs) {
      blocks.push_back({x.start, x.start + x.size, z.start, z.start + z.size});
    }
  }
  return blocks;
}

std::vector<UrbanRoad> UrbanRoads(uint32_t seed, const MapGeneratorConfig& c) {
  if (c.arteryCount <= 0) return {};
  return BuildUrbanRoads(seed, c);
}

OverpassChoice UrbanOverpass(uint32_t seed, const MapGeneratorConfig& c) {
  if (c.arteryCount <= 0) return OverpassChoice{};
  return ResolveOverpass(seed, c);
}

std::vector<UrbanLot> UrbanLots(uint32_t seed, const MapGeneratorConfig& c) {
  if (c.arteryCount > 0) {
    std::vector<UrbanLot> out;
    for (const UrbanBlock& block : BuildCityBlocks(seed, c)) {
      out.push_back({block.x0, block.x1, block.z0, block.z1, false, false});
    }
    return out;
  }
  Rng rng(seed);
  const Layout layout = LayoutCity(c, &rng);
  // Mirror GenerateUrbanMap's draws: one height per cell before the lots.
  for (int i = 0; i < c.blocksX * c.blocksZ; ++i) rng.Float(-1.6f, 1.6f);
  std::vector<UrbanLot> out;
  for (const Lot& lot : AssignLots(c, &rng)) {
    out.push_back({layout.xs[lot.cx0].start, layout.xs[lot.cx1].start + layout.xs[lot.cx1].size,
                   layout.zs[lot.cz0].start, layout.zs[lot.cz1].start + layout.zs[lot.cz1].size,
                   lot.empty, lot.notch});
  }
  return out;
}

float UrbanMapHalfExtent(const MapGeneratorConfig& c) {
  const float width = c.blocksX * c.blockSize + (c.blocksX + 1) * c.streetWidth;
  const float depth = c.blocksZ * c.blockSize + (c.blocksZ + 1) * c.streetWidth;
  return std::max(width, depth) * 0.5f;
}

Scene GenerateUrbanMap(uint32_t seed, const MapGeneratorConfig& c) {
  if (c.arteryCount > 0) return GenerateArterialUrbanMap(seed, c);
  Rng rng(seed);
  Scene scene;
  scene.mapHalfExtent = UrbanMapHalfExtent(c);
  const Layout layout = LayoutCity(c, &rng);

  // Per-block base height: a random walk over the grid that blends the
  // already-assigned west/south neighbors with a medium default, so nearby
  // blocks form broad hills and valleys spanning most of the height range.
  const float mediumHeight = 0.5f * (c.minBuildingHeight + c.maxBuildingHeight) - 0.5f;
  std::vector<float> baseHeight(static_cast<size_t>(c.blocksX) * c.blocksZ);
  auto baseAt = [&](int bx, int bz) -> float& { return baseHeight[bx * c.blocksZ + bz]; };
  for (int bx = 0; bx < c.blocksX; ++bx) {
    for (int bz = 0; bz < c.blocksZ; ++bz) {
      float sum = 0.0f;
      int n = 0;
      if (bx > 0) { sum += baseAt(bx - 1, bz); ++n; }
      if (bz > 0) { sum += baseAt(bx, bz - 1); ++n; }
      const float neighbors = n ? sum / static_cast<float>(n) : mediumHeight;
      baseAt(bx, bz) = std::clamp(0.88f * neighbors + 0.12f * mediumHeight + rng.Float(-3.4f, 3.4f),
                                  c.minBuildingHeight, c.maxBuildingHeight);
    }
  }

  for (const Lot& lot : AssignLots(c, &rng)) {
    const Span& sx0 = layout.xs[lot.cx0];
    const Span& sx1 = layout.xs[lot.cx1];
    const Span& sz0 = layout.zs[lot.cz0];
    const Span& sz1 = layout.zs[lot.cz1];
    const float sw = c.sidewalkWidth, d = c.buildingDepth;
    // Everything below is built in a canonical frame (u, v) where an L's
    // notch is the north-east corner, then mirrored back into world (x, z).
    const bool flipU = lot.notch && !lot.missingEast;
    const bool flipV = lot.notch && !lot.missingNorth;
    auto toU = [&](float x) { return flipU ? -x : x; };
    auto toV = [&](float z) { return flipV ? -z : z; };
    const float xa = toU(sx0.start), xb = toU(sx1.start + sx1.size);
    const float za = toV(sz0.start), zb = toV(sz1.start + sz1.size);
    const float U0 = std::min(xa, xb), U1 = std::max(xa, xb);
    const float V0 = std::min(za, zb), V1 = std::max(za, zb);
    float nu = U1, nv = V0;  // Notch edges (a rectangle has none).
    if (lot.notch) {
      const float worldX = lot.missingEast ? sx0.start + sx0.size : layout.xs[lot.cx0 + 1].start;
      const float worldZ = lot.missingNorth ? layout.zs[lot.cz0 + 1].start : sz0.start + sz0.size;
      nu = toU(worldX);
      nv = toV(worldZ);
    }
    auto box = [&](float u0, float v0, float u1, float v1, float h, bool building) {
      const float xl = flipU ? -u1 : u0, xr = flipU ? -u0 : u1;
      const float zl = flipV ? -v1 : v0, zr = flipV ? -v0 : v1;
      if (building) AddBuilding(&scene, xl, zl, xr, zr, h);
      else scene.sidewalks.push_back(AABB{glm::vec3(xl, 0.0f, zl), glm::vec3(xr, kSidewalkHeight, zr)});
    };
    auto walk = [&](float u0, float v0, float u1, float v1) { box(u0, v0, u1, v1, 0.0f, false); };

    // Outer edge of the building ring (behind the sidewalk setback).
    const float iu0 = U0 + sw, iu1 = U1 - sw, iv0 = V0 + sw, iv1 = V1 - sw;
    const float inu = lot.notch ? nu - sw : iu1, inv = lot.notch ? nv + sw : iv0;

    // The whole block sits at curb height, buildings included.
    if (!lot.notch) {
      walk(U0, V0, U1, V1);
    } else {
      walk(U0, V0, nu, V1);
      walk(nu, nv, U1, V1);
    }
    if (lot.empty) continue;

    // Building runs in ring order. Horizontal runs own the corners; vertical
    // runs sit between and may open a gap at either end.
    struct Run { bool alongU; float a, b, c0, c1; };
    std::vector<Run> runs;
    if (!lot.notch) {
      runs = {{true, iu0, iu1, iv0, iv0 + d}, {true, iu0, iu1, iv1 - d, iv1},
              {false, iv0 + d, iv1 - d, iu0, iu0 + d}, {false, iv0 + d, iv1 - d, iu1 - d, iu1}};
    } else {
      runs = {{true, iu0, inu, iv0, iv0 + d}, {true, inu - d, iu1, inv, inv + d},
              {true, iu0, iu1, iv1 - d, iv1},
              {false, iv0 + d, inv, inu - d, inu}, {false, inv + d, iv1 - d, iu1 - d, iu1},
              {false, iv0 + d, iv1 - d, iu0, iu0 + d}};
    }
    const float base = baseAt(lot.cx0, lot.cz0);
    auto heightHere = [&] {
      return std::clamp(base + rng.Float(-0.6f, 0.6f), c.minBuildingHeight, c.maxBuildingHeight);
    };

    // Decide every junction up front so the lot is guaranteed a mix of
    // wall-to-wall and gapped runs.
    struct Side {
      std::vector<bool> inner;  // Junctions between this run's buildings.
      bool startGap = false, endGap = false;
    };
    std::vector<Side> sides(runs.size());
    struct Ref { size_t side; int index; };  // index -1 = start, -2 = end, >=0 inner.
    std::vector<Ref> refs;
    for (size_t s = 0; s < runs.size(); ++s) {
      const float len = runs[s].b - runs[s].a;
      const int lo = std::max(1, static_cast<int>(std::lround(len / 10.0f)));
      const int hi = std::max(lo, static_cast<int>(std::lround(len / 7.0f)));
      const int count = rng.Int(lo, hi);
      sides[s].inner.assign(count - 1, false);
      for (int i = 0; i < count - 1; ++i) refs.push_back({s, i});
      if (!runs[s].alongU) {
        refs.push_back({s, -1});
        refs.push_back({s, -2});
      }
    }
    auto set = [&](const Ref& r, bool v) {
      if (r.index >= 0) sides[r.side].inner[r.index] = v;
      else if (r.index == -1) sides[r.side].startGap = v;
      else sides[r.side].endGap = v;
    };
    std::vector<bool> isGap(refs.size());
    for (size_t i = 0; i < refs.size(); ++i) isGap[i] = rng.Chance(c.gapProbability);
    if (std::find(isGap.begin(), isGap.end(), true) == isGap.end()) {
      isGap[static_cast<size_t>(rng.Int(0, static_cast<int>(refs.size()) - 1))] = true;
    }
    if (std::find(isGap.begin(), isGap.end(), false) == isGap.end()) {
      isGap[static_cast<size_t>(rng.Int(0, static_cast<int>(refs.size()) - 1))] = false;
    }
    for (size_t i = 0; i < refs.size(); ++i) set(refs[i], isGap[i]);

    for (size_t s = 0; s < runs.size(); ++s) {
      const Run& r = runs[s];
      SplitRun(r.a, r.b, sides[s].inner, c.gapWidth, sides[s].startGap, sides[s].endGap,
               [&](float lo, float hi) {
                 if (r.alongU) box(lo, r.c0, hi, r.c1, heightHere(), true);
                 else box(r.c0, lo, r.c1, hi, heightHere(), true);
               });
    }
  }

  // A couple of towers: pick distinct buildings and stretch them well above
  // the medium-height fabric.
  const int towers = std::min<int>(c.towerCount, static_cast<int>(scene.obstacles.size()));
  std::vector<size_t> picked;
  while (static_cast<int>(picked.size()) < towers) {
    const size_t i = static_cast<size_t>(rng.Int(0, static_cast<int>(scene.obstacles.size()) - 1));
    if (std::find(picked.begin(), picked.end(), i) != picked.end()) continue;
    picked.push_back(i);
    scene.obstacles[i].bounds.max.y = rng.Float(c.towerMinHeight, c.towerMaxHeight);
  }

  // Squads line up on the west (Blue) and east (Red) rim streets, centered
  // on the middle east-west street so the z == midZ row has clear sightlines.
  const float spawnX = scene.mapHalfExtent - c.streetWidth * 0.5f;
  float midZ = 0.0f;
  if (c.blocksZ >= 2) {
    const Span& before = layout.zs[c.blocksZ / 2 - 1];
    midZ = 0.5f * (before.start + before.size + layout.zs[c.blocksZ / 2].start);
  }
  const float rows[3] = {midZ - 6.0f, midZ, midZ + 6.0f};
  for (int i = 0; i < 3; ++i) {
    Unit blue;
    blue.id = i;
    blue.team = Team::Blue;
    blue.position = glm::vec3(-spawnX, 0.0f, rows[i]);
    blue.facingYaw = 0.0f;
    blue.weapon = DefaultWeaponForUnit(blue.id);
    scene.units.push_back(blue);
  }
  for (int i = 0; i < 3; ++i) {
    Unit red;
    red.id = 3 + i;
    red.team = Team::Red;
    red.position = glm::vec3(spawnX, 0.0f, rows[i]);
    red.facingYaw = kPi;
    red.weapon = DefaultWeaponForUnit(red.id);
    scene.units.push_back(red);
  }
  return scene;
}

namespace {

// Deterministic integer hash (SplitMix-style avalanche), so the terrain is
// identical across platforms/standard libraries, like the Rng above.
uint32_t HashU32(uint32_t x) {
  x ^= x >> 16;
  x *= 0x7feb352du;
  x ^= x >> 15;
  x *= 0x846ca68bu;
  x ^= x >> 16;
  return x;
}

// Lattice value in [0, 1) at integer noise coordinates.
float LatticeValue(uint32_t seed, int ix, int iz) {
  const uint32_t h = HashU32(seed ^ HashU32(static_cast<uint32_t>(ix) * 0x9e3779b9u) ^
                             HashU32(static_cast<uint32_t>(iz) * 0x85ebca6bu));
  return static_cast<float>(h >> 8) / 16777216.0f;
}

// Smoothstep-interpolated value noise in [0, 1), lattice spacing 1.
float ValueNoise(uint32_t seed, float x, float z) {
  const float fx = std::floor(x), fz = std::floor(z);
  const int ix = static_cast<int>(fx), iz = static_cast<int>(fz);
  float tx = x - fx, tz = z - fz;
  tx = tx * tx * (3.0f - 2.0f * tx);
  tz = tz * tz * (3.0f - 2.0f * tz);
  const float v00 = LatticeValue(seed, ix, iz), v10 = LatticeValue(seed, ix + 1, iz);
  const float v01 = LatticeValue(seed, ix, iz + 1), v11 = LatticeValue(seed, ix + 1, iz + 1);
  const float v0 = v00 + (v10 - v00) * tx;
  const float v1 = v01 + (v11 - v01) * tx;
  return v0 + (v1 - v0) * tz;
}

}  // namespace

Scene GenerateHillyMap(uint32_t seed, const HillyMapConfig& c) {
  Scene scene;
  scene.mapHalfExtent = c.halfExtent;

  // Two octaves of value noise, both non-negative so the whole field sits at
  // or above y = 0 (valleys bottom out at the map's base plane).
  HeightField& ground = scene.ground;
  ground.step = c.cellSize;
  ground.minX = ground.minZ = -c.halfExtent;
  ground.nx = ground.nz = static_cast<int>(std::round(2.0f * c.halfExtent / c.cellSize)) + 1;
  ground.heights.resize(static_cast<size_t>(ground.nx) * ground.nz);
  const float baseFreq = 1.0f / c.hillWavelength;
  for (int iz = 0; iz < ground.nz; ++iz) {
    for (int ix = 0; ix < ground.nx; ++ix) {
      const float x = ground.minX + ix * ground.step;
      const float z = ground.minZ + iz * ground.step;
      const float coarse = ValueNoise(seed, x * baseFreq, z * baseFreq);
      const float fine = ValueNoise(seed ^ 0x51ed270bu, x * baseFreq * 2.7f, z * baseFreq * 2.7f);
      ground.heights[static_cast<size_t>(iz) * ground.nx + ix] =
          c.hillAmplitude * (coarse + 0.3f * fine);
    }
  }

  // Impassable rocks embedded in the slopes, kept out of the spawn strips.
  // Each sinks below the lowest nearby terrain and tops out above the
  // highest, so no slope exposes a floating base or a walk-over lip.
  Rng rng(seed);
  const float rockFieldMax = c.halfExtent - c.spawnMargin;
  for (int i = 0; i < c.rockCount; ++i) {
    const float hx = rng.Float(c.rockMinExtent, c.rockMaxExtent);
    const float hz = rng.Float(c.rockMinExtent, c.rockMaxExtent);
    const float cx = rng.Float(-rockFieldMax + hx, rockFieldMax - hx);
    const float cz = rng.Float(-c.halfExtent + hx + 2.0f, c.halfExtent - hz - 2.0f);
    float lo = std::numeric_limits<float>::infinity();
    float hi = -std::numeric_limits<float>::infinity();
    for (const float x : {cx - hx, cx, cx + hx}) {
      for (const float z : {cz - hz, cz, cz + hz}) {
        const float h = ground.HeightAt(x, z);
        lo = std::min(lo, h);
        hi = std::max(hi, h);
      }
    }
    scene.obstacles.push_back(Obstacle{
        AABB{glm::vec3(cx - hx, lo - 1.0f, cz - hz), glm::vec3(cx + hx, hi + c.rockHeight, cz + hz)},
        /*climbable=*/false});
  }

  // Same 3v3 spawn rows as the urban generator, standing on the terrain.
  const float spawnX = c.halfExtent - 3.0f;
  const float rows[3] = {-6.0f, 0.0f, 6.0f};
  for (int i = 0; i < 3; ++i) {
    Unit blue;
    blue.id = i;
    blue.team = Team::Blue;
    blue.position = glm::vec3(-spawnX, ground.HeightAt(-spawnX, rows[i]), rows[i]);
    blue.facingYaw = 0.0f;
    scene.units.push_back(blue);
  }
  for (int i = 0; i < 3; ++i) {
    Unit red;
    red.id = 3 + i;
    red.team = Team::Red;
    red.position = glm::vec3(spawnX, ground.HeightAt(spawnX, rows[i]), rows[i]);
    red.facingYaw = kPi;
    scene.units.push_back(red);
  }
  return scene;
}

}  // namespace tactics
