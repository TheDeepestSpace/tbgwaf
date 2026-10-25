#include "game/MapGenerator.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <random>
#include <vector>

#include "game/Geometry.h"
#include "game/NavMesh.h"

namespace tactics {
namespace {

// Rng sub-streams. Every pass draws from its own `Rng(seed ^ constant)` so
// adding or forcing one never reshuffles another (issue #146). New passes
// pick a fresh constant and add a row here.
//
//   constant      pass             draws
//   (seed)        parcels, towers  park/height/gap/junction draws per block, then tower picks
//   0x0ebe7a55u   overpass         presence + layout (ResolveOverpass)
//   0xa17e2d31u   highway plan     artery placement and profile
//   0x5b2a9c47u   highway branch   branch end, ramp fraction/length
//   0xb10c5eedu   street graph     avenues and cross streets
//   0x6c57f00du   furniture        feature presence, then curb-side item stations/kinds
//   0x51674ea1u   traffic control  light-vs-sign draws per junction, street-name corners
constexpr uint32_t kStreetStream = 0xb10c5eedu;
constexpr uint32_t kFurnitureStream = 0x6c57f00du;
constexpr uint32_t kTrafficStream = 0x51674ea1u;

constexpr float kPi = 3.14159265358979323846f;
constexpr float kSidewalkHeight = 0.12f;  // Curb: purely visual, well under a step.

// Every urban scene emitter places its output relative to this sampled base.
// The urban map remains flat for now; keeping the sampler at the emission
// boundary lets terrain generation replace it without revisiting each feature.
using GroundY = std::function<float(float, float)>;

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

void AddBuilding(Scene* scene, float x0, float z0, float x1, float z1, float height,
                 const GroundY& groundY) {
  const float baseY = groundY(0.5f * (x0 + x1), 0.5f * (z0 + z1));
  scene->obstacles.push_back(Obstacle{
      AABB{glm::vec3(x0, baseY, z0), glm::vec3(x1, baseY + height, z1)},
      /*climbable=*/false});
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
// bounds. Zero: the deck is cut off flush with the map edge.
constexpr float kDeckOverhang = 0.0f;

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
  float riseEnd = 0.0f;        // Branch parameter where the on-ramp reaches deck height.
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
    // The ramp tops out before the branch slab crosses the deck's edge, so
    // the whole junction sits at deck height and the branch only starts its
    // descent once clear of the artery (no wedge cutting under the deck).
    const float sinTurn =
        std::max(0.2f, std::fabs(artery.dir.x * branch.dir.y - artery.dir.y * branch.dir.x));
    const float junctionClear = (c.arteryWidth * 0.5f + branch.width * 0.5f + 2.0f) / sinTurn;
    branch.riseEnd = std::max(branch.length * 0.5f, branch.length - junctionClear);
    branch.riseStart = branch.riseEnd - std::min(branch.riseEnd * rampFraction, rampMax);
    if (branch.offMap) {
      branch.riseStart = 0.0f;
      branch.riseEnd = 0.0f;
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
  const float run = std::max(1.0f, plan.branch.riseEnd - plan.branch.riseStart);
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

// A street carved out of one side's blocks: a band of `width` around the
// line origin + t*dir. Recorded in the order drawn so the block pass replays
// exactly the cuts the street pass chose.
struct StreetCut {
  glm::vec2 origin, dir;
  float width;
  int side;
  UrbanStreetClass cls;
};

// Every street of the city: the highway roads plus the local cuts.
struct StreetGraph {
  std::vector<UrbanRoad> roads;
  std::vector<StreetCut> cuts;
};

// Everything the passes share. Built front to back: HighwayPlan ->
// StreetGraph -> Blocks; Parcels and Scene emission consume it.
struct CityPlan {
  HighwayPlan highway;
  StreetGraph streets;
  std::vector<UrbanBlock> blocks;
};

float RimWidth(const MapGeneratorConfig& c) { return std::max(c.streetWidth, c.localStreetWidth); }
float ArteryBand(const MapGeneratorConfig& c) {
  return c.arteryWidth * 0.5f + c.localStreetWidth * 0.45f;
}

// Street pass: the branch corridor, avenues roughly parallel to the artery
// and oblique cross streets for each side. Each side draws its own, so
// junctions don't line up across the artery and block shapes differ.
StreetGraph BuildStreetGraph(uint32_t seed, const MapGeneratorConfig& c, const HighwayPlan& plan) {
  const ArterySpec& artery = plan.artery;
  Rng rng(seed ^ kStreetStream);
  const float half = UrbanMapHalfExtent(c);
  const float band = ArteryBand(c);
  StreetGraph graph;
  graph.roads = BuildUrbanRoads(seed, c);
  for (const int side : {1, -1}) {
    if (plan.hasBranch && plan.branch.side == side) {
      graph.cuts.push_back({plan.branch.start, plan.branch.dir,
                            plan.branch.width + c.localStreetWidth * 0.8f, side,
                            UrbanStreetClass::Highway});
    }
    float offset = band + c.blockSize * rng.Float(0.8f, 1.1f);
    while (offset < half * 1.5f) {
      const glm::vec2 dir = Rotate(artery.dir, rng.Float(-0.09f, 0.09f));
      const float width = c.localStreetWidth * rng.Float(0.85f, 1.25f);
      graph.cuts.push_back(
          {artery.origin + artery.normal * (static_cast<float>(side) * offset), dir, width, side,
           UrbanStreetClass::Avenue});
      offset += c.blockSize * rng.Float(0.85f, 1.25f) + width;
    }
    float s = artery.sMin + c.blockSize * rng.Float(0.55f, 0.9f);
    while (s < artery.sMax - c.blockSize * 0.4f) {
      const glm::vec2 dir =
          Rotate(artery.normal, rng.Float(-c.localStreetSkew, c.localStreetSkew));
      const float width = c.localStreetWidth * rng.Float(0.85f, 1.25f);
      graph.cuts.push_back({artery.origin + artery.dir * s, dir, width, side,
                            UrbanStreetClass::Cross});
      s += c.blockSize * rng.Float(0.9f, 1.35f) + width;
    }
  }
  return graph;
}

// Block pass. Convex polygon blocks: the rim rectangle is split by the
// artery corridor and every street cut. Each surviving region is a block
// whose shape follows whatever angles the streets around it happen to make.
std::vector<UrbanBlock> BuildBlocks(const MapGeneratorConfig& c, const HighwayPlan& plan,
                                    const StreetGraph& graph) {
  const ArterySpec& artery = plan.artery;
  const float half = UrbanMapHalfExtent(c);
  const float rim = RimWidth(c);
  const float lo = -half + rim, hi = half - rim;
  const std::vector<glm::vec2> rimRect = {{lo, lo}, {hi, lo}, {hi, hi}, {lo, hi}};
  const float arteryD = glm::dot(artery.normal, artery.origin);

  std::vector<UrbanBlock> blocks;
  for (const int side : {1, -1}) {
    const glm::vec2 n = artery.normal * static_cast<float>(side);
    std::vector<std::vector<glm::vec2>> regions;
    std::vector<glm::vec2> base =
        KeepSide(rimRect, n, static_cast<float>(side) * arteryD + ArteryBand(c));
    if (base.size() >= 3) regions.push_back(std::move(base));
    for (const StreetCut& cut : graph.cuts) {
      if (cut.side == side) CutStreet(&regions, cut.origin, cut.dir, cut.width);
    }
    for (auto& region : regions) {
      if (PolygonArea(region) < c.minPolygonArea) continue;
      blocks.push_back(MakeUrbanBlock(std::move(region)));
    }
  }
  return blocks;
}

CityPlan BuildCityPlan(uint32_t seed, const MapGeneratorConfig& c) {
  CityPlan plan;
  plan.highway = BuildHighwayPlan(seed, c);
  plan.streets = BuildStreetGraph(seed, c, plan.highway);
  plan.blocks = BuildBlocks(c, plan.highway, plan.streets);
  return plan;
}

std::vector<UrbanBlock> BuildCityBlocks(uint32_t seed, const MapGeneratorConfig& c) {
  const HighwayPlan highway = BuildHighwayPlan(seed, c);
  return BuildBlocks(c, highway, BuildStreetGraph(seed, c, highway));
}

// Ribbon edge; an end vertex sitting exactly on the map boundary slides
// along the road direction onto it so the oblique road is cut off flush
// with the edge, elevated off-map deck ends included.
std::vector<glm::vec3> RibbonSide(const UrbanRoad& road, bool left, float half,
                                  const GroundY& groundY) {
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
    side.emplace_back(q.x, groundY(q.x, q.y) + p.y, q.y);
  }
  return side;
}

float DistanceToPolygonEdges(glm::vec2 p, const std::vector<glm::vec2>& poly);
float RimWidth(const MapGeneratorConfig& c);

// A buildable lot of a block (issue #151, roadmap 3.1): the footprint plus
// where and what it fronts. Becomes exactly one Building (and its Obstacle
// prism(s)); see UrbanParcel for the public mirror.
struct Parcel {
  std::vector<glm::vec2> footprint;  // Counter-clockwise.
  int frontageEdge = 0;
  UrbanStreetClass street = UrbanStreetClass::Cross;
  float depth = 0.0f;
  bool corner = false;
  int block = -1;
  // Per-parcel draw from the layout rng (the same stream position the old
  // per-building height jitter used, so layouts never shift): picks the
  // BuildingKind from the weighted table.
  float kindRoll = 0.0f;
};

// Street class of the street a block edge `a -> b` fronts (`inward` points
// into the block). Probes just outside the building line and takes the
// street band it is nearest to: the artery and branch roads, the local cuts
// of this side of the artery, or the rim street when the probe leaves the map.
UrbanStreetClass FrontageClass(const CityPlan& city, glm::vec2 a, glm::vec2 b, glm::vec2 inward,
                               const MapGeneratorConfig& c) {
  const float half = UrbanMapHalfExtent(c);
  const float rim = RimWidth(c);
  const glm::vec2 probe = (a + b) * 0.5f - inward * (c.sidewalkWidth + 0.5f);
  const glm::vec2 edgeDir = glm::normalize(b - a);
  constexpr float kParallel = 0.8f;  // |cos| between the edge and the street it fronts.
  float best = std::numeric_limits<float>::infinity();
  UrbanStreetClass cls = UrbanStreetClass::Cross;
  // Rim streets: axis-aligned bands just inside the map square.
  if (std::fabs(edgeDir.y) > kParallel) {
    best = std::fabs(std::fabs(probe.x) - (half - rim * 0.5f)) - rim * 0.5f;
  } else if (std::fabs(edgeDir.x) > kParallel) {
    best = std::fabs(std::fabs(probe.y) - (half - rim * 0.5f)) - rim * 0.5f;
  }
  for (const UrbanRoad& road : city.streets.roads) {
    for (size_t i = 0; i + 1 < road.centerline.size(); ++i) {
      const glm::vec2 p(road.centerline[i].x, road.centerline[i].z);
      const glm::vec2 q(road.centerline[i + 1].x, road.centerline[i + 1].z);
      if (glm::length(q - p) < 1e-4f ||
          std::fabs(glm::dot(edgeDir, glm::normalize(q - p))) < kParallel) {
        continue;
      }
      const float d = PointSegmentDistance(probe, p, q) - road.width * 0.5f;
      if (d < best) {
        best = d;
        cls = UrbanStreetClass::Highway;
      }
    }
  }
  const ArterySpec& artery = city.highway.artery;
  const int side = glm::dot(probe - artery.origin, artery.normal) >= 0.0f ? 1 : -1;
  for (const StreetCut& cut : city.streets.cuts) {
    if (cut.cls == UrbanStreetClass::Highway || cut.side != side) continue;
    if (std::fabs(glm::dot(edgeDir, cut.dir)) < kParallel) continue;
    const glm::vec2 n(-cut.dir.y, cut.dir.x);
    const float d = std::fabs(glm::dot(probe - cut.origin, n)) - cut.width * 0.5f;
    if (d < best) {
      best = d;
      cls = cut.cls;
    }
  }
  return cls;
}

// Winds a parcel counter-clockwise (keeping `frontageEdge` on the same
// edge) and fills in `depth` as the footprint's extent behind the frontage.
void FinishParcel(Parcel* parcel) {
  std::vector<glm::vec2>& fp = parcel->footprint;
  if (PolygonSignedArea(fp) < 0.0f) {
    std::reverse(fp.begin(), fp.end());
    parcel->frontageEdge = static_cast<int>(fp.size()) - 2 - parcel->frontageEdge;
  }
  const glm::vec2 a = fp[parcel->frontageEdge];
  const glm::vec2 b = fp[(parcel->frontageEdge + 1) % fp.size()];
  const glm::vec2 inward = glm::normalize(glm::vec2(-(b - a).y, (b - a).x));  // Left of a->b.
  float depth = 0.0f;
  for (const glm::vec2& v : fp) depth = std::max(depth, glm::dot(v - a, inward));
  parcel->depth = depth;
}

UrbanParcel ToPublicParcel(const Parcel& parcel) {
  return UrbanParcel{parcel.footprint, parcel.frontageEdge, parcel.street, parcel.depth,
                     parcel.corner, parcel.block};
}

// Smaller footprints than this never become buildings.
constexpr float kMinBuildingArea = 1.0f;

// Appends a prism over `footprint` from the ground (plus `baseLift`) up
// `height`, tagged with its Building index; returns the obstacle index.
int AddPolygonBuilding(Scene* scene, std::vector<glm::vec2> footprint, float baseLift,
                       float height, int building, const GroundY& groundY) {
  if (footprint.size() < 3 || PolygonArea(footprint) < kMinBuildingArea) return -1;
  if (PolygonSignedArea(footprint) < 0.0f) std::reverse(footprint.begin(), footprint.end());
  AABB bounds;
  bounds.min = glm::vec3(std::numeric_limits<float>::infinity(), 0.0f,
                         std::numeric_limits<float>::infinity());
  bounds.max = glm::vec3(-std::numeric_limits<float>::infinity(), height,
                         -std::numeric_limits<float>::infinity());
  glm::vec2 center(0.0f);
  for (const glm::vec2& p : footprint) {
    center += p;
    bounds.min.x = std::min(bounds.min.x, p.x);
    bounds.max.x = std::max(bounds.max.x, p.x);
    bounds.min.z = std::min(bounds.min.z, p.y);
    bounds.max.z = std::max(bounds.max.z, p.y);
  }
  center /= static_cast<float>(footprint.size());
  const float baseY = groundY(center.x, center.y) + baseLift;
  bounds.min.y = baseY;
  bounds.max.y = baseY + height;
  scene->obstacles.push_back(Obstacle{bounds, false, std::move(footprint), ObstacleKind::Generic, building});
  return static_cast<int>(scene->obstacles.size()) - 1;
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
// parcels whose outer walls lie on the (possibly angled) frontage, with
// wall-to-wall junctions, seeded alley gaps and at least one opening into
// the courtyard. Acute wedge blocks whose inner ring collapses become one
// real angled parcel; blocks whose first inset collapses stay open. Appends
// to `out`; `characterRoll` receives the block's neighbourhood draw.
void PlanBlockParcels(const CityPlan& city, size_t blockIndex, const MapGeneratorConfig& c,
                      Rng* rng, std::vector<Parcel>* out, float* characterRoll) {
  const UrbanBlock& block = city.blocks[blockIndex];
  const bool park = rng->Chance(0.10f);  // Pocket park/open lot.
  *characterRoll = rng->Unit01();
  if (park) return;
  std::vector<glm::vec2> outer = InsetConvexPolygon(block.vertices, c.sidewalkWidth);
  if (outer.size() < 3 || PolygonArea(outer) < c.minPolygonArea ||
      !PolygonClearOfRoads(outer, city.streets.roads, c.sidewalkWidth)) {
    return;
  }
  glm::vec2 blockCentroid(0.0f);
  for (const glm::vec2& p : outer) blockCentroid += p;
  blockCentroid /= static_cast<float>(outer.size());
  auto inwardOf = [&](glm::vec2 a, glm::vec2 b) {
    glm::vec2 inward = glm::normalize(glm::vec2(-(b - a).y, (b - a).x));
    if (glm::dot(inward, blockCentroid - a) < 0.0f) inward = -inward;
    return inward;
  };
  auto emit = [&](Parcel parcel) {
    if (parcel.footprint.size() < 3 || PolygonArea(parcel.footprint) < kMinBuildingArea) return;
    parcel.block = static_cast<int>(blockIndex);
    parcel.kindRoll = rng->Unit01();
    FinishParcel(&parcel);
    out->push_back(std::move(parcel));
  };
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
    // a row of slab parcels along the block's longest frontage instead.
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
      // The slab fronts the longest of its edges that lie on the building
      // line; it is a corner piece when it keeps one of the block's corners.
      Parcel parcel;
      float best = -1.0f;
      bool bestOnLine = false;
      for (size_t i = 0; i < slab.size(); ++i) {
        const glm::vec2 a = slab[i], b = slab[(i + 1) % slab.size()];
        const float length = glm::length(b - a);
        const bool onLine = DistanceToPolygonEdges(a, outer) < 1e-3f &&
                            DistanceToPolygonEdges(b, outer) < 1e-3f &&
                            DistanceToPolygonEdges((a + b) * 0.5f, outer) < 1e-3f;
        if (onLine && !bestOnLine) best = -1.0f;  // Edges on the line beat any chord.
        if ((onLine || !bestOnLine) && length > best) {
          best = length;
          bestOnLine = onLine;
          parcel.frontageEdge = static_cast<int>(i);
        }
      }
      for (const glm::vec2& v : slab) {
        for (const glm::vec2& o : outer) parcel.corner |= glm::distance(v, o) < 1e-3f;
      }
      const glm::vec2 fa = slab[parcel.frontageEdge];
      const glm::vec2 fb = slab[(parcel.frontageEdge + 1) % slab.size()];
      parcel.street = FrontageClass(city, fa, fb, inwardOf(fa, fb), c);
      parcel.footprint = std::move(slab);
      emit(std::move(parcel));
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
    const UrbanStreetClass street =
        FrontageClass(city, outer[e], outer[next], inwardOf(outer[e], outer[next]), c);
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
      Parcel parcel;
      parcel.footprint = {glm::mix(outer[e], outer[next], ta), glm::mix(outer[e], outer[next], tb),
                          inner[e] + innerDir * ub, inner[e] + innerDir * ua};
      // A back wall pinched to a point becomes a (still convex) triangle.
      if (ub - ua < 0.05f) parcel.footprint.pop_back();
      if (PolygonArea(parcel.footprint) < c.minPolygonArea * 0.18f) continue;
      parcel.frontageEdge = 0;
      parcel.street = street;
      parcel.corner = (i == 0 && !cornerGap[e]) || (i == count - 1 && !cornerGap[next]);
      emit(std::move(parcel));
    }
  }
}

// Parcel pass over every block, in block order (the layout rng's draws
// here are the only ones the building fabric makes, so UrbanParcels can
// replay them). `characterRolls` gets one draw per block.
std::vector<Parcel> PlanParcels(const CityPlan& city, const MapGeneratorConfig& c, Rng* rng,
                                std::vector<float>* characterRolls) {
  std::vector<Parcel> parcels;
  characterRolls->assign(city.blocks.size(), 0.0f);
  for (size_t blockIndex = 0; blockIndex < city.blocks.size(); ++blockIndex) {
    PlanBlockParcels(city, blockIndex, c, rng, &parcels, &(*characterRolls)[blockIndex]);
  }
  return parcels;
}

// --- Building archetypes (issue #151, roadmap 3.2) -------------------------

constexpr uint32_t kArchetypeStream = 0xa5c47e75u;  // Floors, variants, setbacks.
constexpr uint32_t kMassingStream = 0x3a55196eu;    // Parapets, roof furniture, awnings.

// Street-class-weighted kind table, Residential/MixedUse/Office/Commercial/
// Garage (Civic is reserved for the landmark towers). Artery frontage leans
// office/mixed-use, back streets residential/warehouse.
const float kKindWeights[3][kBuildingKindCount] = {
    /* Highway */ {0.08f, 0.34f, 0.40f, 0.10f, 0.0f, 0.08f},
    /* Avenue  */ {0.26f, 0.30f, 0.24f, 0.12f, 0.0f, 0.08f},
    /* Cross   */ {0.42f, 0.15f, 0.09f, 0.20f, 0.0f, 0.14f},
};
// Per-character multipliers on the table above (Mixed/Downtown/Residential/
// Industrial), so a block's buildings agree with their neighbours.
const float kCharacterBias[4][kBuildingKindCount] = {
    /* Mixed       */ {1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f},
    /* Downtown    */ {0.5f, 1.6f, 3.0f, 0.7f, 0.0f, 1.2f},
    /* Residential */ {2.5f, 1.3f, 0.3f, 0.5f, 0.0f, 0.6f},
    /* Industrial  */ {0.4f, 0.5f, 0.6f, 2.5f, 0.0f, 2.0f},
};

int CharacterIndex(BlockCharacter character) {
  switch (character) {
    case BlockCharacter::Downtown: return 1;
    case BlockCharacter::Residential: return 2;
    case BlockCharacter::Industrial: return 3;
    default: return 0;
  }
}

// A block's character from its roll: blocks with artery frontage are mostly
// downtown, the rest mostly residential, unless the config forces one.
BlockCharacter DrawCharacter(float roll, bool highwayFrontage, const MapGeneratorConfig& c) {
  if (c.character != BlockCharacter::Auto) return c.character;
  const BlockCharacter order[4] = {BlockCharacter::Downtown, BlockCharacter::Mixed,
                                   BlockCharacter::Residential, BlockCharacter::Industrial};
  const float downtownWeights[4] = {0.45f, 0.30f, 0.10f, 0.15f};
  const float outskirtWeights[4] = {0.10f, 0.30f, 0.40f, 0.20f};
  const float* weights = highwayFrontage ? downtownWeights : outskirtWeights;
  float acc = 0.0f;
  for (int i = 0; i < 4; ++i) {
    acc += weights[i];
    if (roll < acc) return order[i];
  }
  return order[3];
}

BuildingKind DrawKind(float roll, UrbanStreetClass street, BlockCharacter character) {
  const float* base = kKindWeights[static_cast<int>(street)];
  const float* bias = kCharacterBias[CharacterIndex(character)];
  float total = 0.0f;
  for (int k = 0; k < kBuildingKindCount; ++k) total += base[k] * bias[k];
  float acc = 0.0f;
  for (int k = 0; k < kBuildingKindCount; ++k) {
    acc += base[k] * bias[k] / total;
    if (roll < acc) return static_cast<BuildingKind>(k);
  }
  return BuildingKind::Residential;
}

// Floor count/height per kind; the building's height follows from them
// (replacing the old cluster +-1.2 draw), kept within the config's ordinary
// height range by trimming/adding floors.
void DrawFloors(Building* b, const MapGeneratorConfig& c, Rng* rng) {
  switch (b->kind) {
    case BuildingKind::Residential:
      b->groundFloorHeight = b->floorHeight = 3.0f;
      b->floorCount = rng->Int(2, 3);
      break;
    case BuildingKind::MixedUse:
      b->groundFloorHeight = 4.0f;  // Shopfront floor.
      b->floorHeight = 3.0f;
      b->floorCount = rng->Int(2, 3);
      break;
    case BuildingKind::Office:
      b->groundFloorHeight = b->floorHeight = 3.5f;
      b->floorCount = rng->Int(2, 3);
      break;
    case BuildingKind::Commercial:
      b->groundFloorHeight = b->floorHeight = rng->Float(4.5f, 5.5f);  // One tall hall.
      b->floorCount = 1;
      break;
    case BuildingKind::Garage:
      b->groundFloorHeight = b->floorHeight = 2.8f;  // Parking decks.
      b->floorCount = rng->Int(2, 3);
      break;
    case BuildingKind::Civic:
      b->groundFloorHeight = b->floorHeight = 3.5f;
      b->floorCount = 4;
      break;
  }
  while (b->floorCount > 1 && b->Height() > c.maxBuildingHeight + 1e-3f) --b->floorCount;
  while (b->Height() < c.minBuildingHeight - 1e-3f) ++b->floorCount;
}

// Kind, floors, facade variant and setback per parcel, then the landmark
// towers (Civic, from the layout rng exactly where the old tower pick drew).
std::vector<Building> PlanBuildings(const std::vector<Parcel>& parcels,
                                    const std::vector<float>& characterRolls, uint32_t seed,
                                    const MapGeneratorConfig& c, Rng* layoutRng) {
  Rng rng(seed ^ kArchetypeStream);
  std::vector<bool> highwayFrontage(characterRolls.size(), false);
  for (const Parcel& parcel : parcels) {
    if (parcel.street == UrbanStreetClass::Highway) highwayFrontage[parcel.block] = true;
  }
  std::vector<Building> buildings;
  buildings.reserve(parcels.size());
  for (size_t i = 0; i < parcels.size(); ++i) {
    const Parcel& parcel = parcels[i];
    const BlockCharacter character =
        DrawCharacter(characterRolls[parcel.block], highwayFrontage[parcel.block], c);
    Building b;
    b.kind = DrawKind(parcel.kindRoll, parcel.street, character);
    b.parcel = static_cast<int>(i);
    b.block = parcel.block;
    DrawFloors(&b, c, &rng);
    b.style = static_cast<int>(b.kind) * 2 + (rng.Chance(0.5f) ? 1 : 0);
    // Offices step their top floor back when there is room for it.
    if (b.kind == BuildingKind::Office && b.floorCount >= 3 && rng.Chance(0.6f)) {
      b.upperObstacle = -2;  // Request; EmitBuildings resolves it (or drops it).
    }
    buildings.push_back(b);
  }

  // Landmark towers: the same two picks the old generator made.
  const int towers = std::min<int>(c.towerCount, static_cast<int>(buildings.size()));
  std::vector<size_t> candidates(buildings.size());
  for (size_t i = 0; i < candidates.size(); ++i) candidates[i] = i;
  for (int i = 0; i < towers && !candidates.empty(); ++i) {
    const size_t pick =
        static_cast<size_t>(layoutRng->Int(0, static_cast<int>(candidates.size()) - 1));
    Building& b = buildings[candidates[pick]];
    const float height = layoutRng->Float(c.towerMinHeight, c.towerMaxHeight);
    b.kind = BuildingKind::Civic;
    b.style = static_cast<int>(b.kind) * 2 + (b.style & 1);
    b.floorHeight = 3.5f;
    b.floorCount = std::max(2, static_cast<int>(std::lround(height / b.floorHeight)));
    b.groundFloorHeight = height - static_cast<float>(b.floorCount - 1) * b.floorHeight;
    b.upperObstacle = -1;
    candidates.erase(candidates.begin() + pick);
  }
  return buildings;
}

// True when two deck spans overlap in plan at matching height, i.e. a figure
// standing on either could step straight onto the other. The height check
// runs at a representative overlap point so a span crossing beneath an
// elevated one never counts as touching it.
bool SpansTouchAtSameHeight(const WalkSurface& a, const WalkSurface& b) {
  std::vector<glm::vec2> fa, fb;
  fa.reserve(a.vertices.size());
  fb.reserve(b.vertices.size());
  for (const glm::vec3& v : a.vertices) fa.emplace_back(v.x, v.z);
  for (const glm::vec3& v : b.vertices) fb.emplace_back(v.x, v.z);
  glm::vec2 probe(0.0f);
  int contained = 0;
  for (const glm::vec2& v : fa) {
    if (PointInConvexPolygon(v, fb)) {
      probe += v;
      ++contained;
    }
  }
  for (const glm::vec2& v : fb) {
    if (PointInConvexPolygon(v, fa)) {
      probe += v;
      ++contained;
    }
  }
  if (contained) {
    probe /= static_cast<float>(contained);
  } else {
    // Quads can also cross edge-through-edge with no corner contained.
    bool crosses = false;
    for (size_t i = 0; i < fa.size() && !crosses; ++i) {
      crosses = SegmentEntersConvexPolygon(fa[i], fa[(i + 1) % fa.size()], fb);
    }
    if (!crosses) return false;
    const glm::vec3 ca = SurfaceCenter(a), cb = SurfaceCenter(b);
    probe = glm::vec2((ca.x + cb.x) * 0.5f, (ca.z + cb.z) * 0.5f);
  }
  return std::fabs(SurfaceHeightAt(a, probe.x, probe.y) -
                   SurfaceHeightAt(b, probe.x, probe.y)) < 0.6f;
}

// --- Ziplines (issue #166) -------------------------------------------------

// Distance from `p` to a convex polygon of either winding (0 inside).
float DistanceToPolygon(glm::vec2 p, const std::vector<glm::vec2>& poly) {
  bool allLeft = true, allRight = true;
  float best = std::numeric_limits<float>::infinity();
  for (size_t i = 0; i < poly.size(); ++i) {
    const glm::vec2 a = poly[i], b = poly[(i + 1) % poly.size()];
    const glm::vec2 e = b - a, r = p - a;
    const float cross = e.x * r.y - e.y * r.x;
    allLeft &= cross >= 0.0f;
    allRight &= cross <= 0.0f;
    best = std::min(best, PointSegmentDistance(p, a, b));
  }
  return allLeft || allRight ? 0.0f : best;
}

// Closest approach of two XZ segments (0 if they cross).
float SegmentSegmentDistance(glm::vec2 a0, glm::vec2 a1, glm::vec2 b0, glm::vec2 b1) {
  auto side = [](glm::vec2 p, glm::vec2 q, glm::vec2 r) {
    return (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
  };
  if (side(a0, a1, b0) * side(a0, a1, b1) < 0.0f && side(b0, b1, a0) * side(b0, b1, a1) < 0.0f) {
    return 0.0f;
  }
  return std::min({PointSegmentDistance(a0, b0, b1), PointSegmentDistance(a1, b0, b1),
                   PointSegmentDistance(b0, a0, a1), PointSegmentDistance(b1, a0, a1)});
}

// Distance from `p` to the nearest edge of a polygon (any winding).
float DistanceToPolygonEdges(glm::vec2 p, const std::vector<glm::vec2>& poly) {
  float best = std::numeric_limits<float>::infinity();
  for (size_t i = 0; i < poly.size(); ++i) {
    best = std::min(best, PointSegmentDistance(p, poly[i], poly[(i + 1) % poly.size()]));
  }
  return best;
}

// Draws each block's ziplines: a short post on the street and a post on the
// roof of one of the block's buildings, so the line is how a figure gets up
// onto (and back down from) that roof. The roof gets a flat walkable slab
// (a WalkSurface joined to nothing, so the line is its only access). The
// street end must be ordinary walkable ground, and the cable (post top to
// post top) must clear every obstacle, other ziplines, and the spawn points.
// Uses its own RNG stream (seed-derived), so adding or removing ziplines
// never changes the rest of a seed's map (apart from the roof slabs).
void PlaceZiplines(Scene* scene, const std::vector<UrbanBlock>& blocks, uint32_t seed,
                   const MapGeneratorConfig& c) {
  if (c.maxZiplinesPerBlock <= 0) return;
  Rng rng(seed ^ 0x21b1e5a7u);
  constexpr float kClearance = 0.7f;     // Street anchor stays this far from any obstacle.
  constexpr float kCableClearance = 0.5f;  // Cable passes this far (XZ) / above (Y) obstacles.
  constexpr float kRoofAnchorInset = 0.3f;  // Roof anchor's distance in from the footprint edge.
  // The street anchor stands on the sidewalk strip: mid-strip, never nearer
  // the block edge (the road) than kSidewalkMin nor farther than kSidewalkMax.
  constexpr float kSidewalkMid = 0.75f;
  constexpr float kSidewalkMin = 0.3f;
  constexpr float kSidewalkMax = 1.2f;
  constexpr float kMinRoofHeight = 2.5f;
  constexpr float kLineSpacing = 2.5f;  // Min gap between two ziplines (and from a spawn).
  constexpr int kAttempts = 60;

  std::vector<std::vector<glm::vec2>> footprints;
  std::vector<glm::vec4> footprintBounds;  // minX, minZ, maxX, maxZ.
  for (const Obstacle& o : scene->obstacles) {
    footprints.push_back(ObstacleFootprint(o));
    footprintBounds.emplace_back(o.bounds.min.x, o.bounds.min.z, o.bounds.max.x, o.bounds.max.z);
  }
  // The street anchor must be ordinary walkable ground in the real (padded) navmesh.
  NavMesh nav;
  nav.Build(scene->obstacles, scene->mapHalfExtent, constants::kAgentRadius);
  const float post = constants::kZiplinePostHeight;
  // Existing decks and ramps (the roof slabs added below come after): the
  // cable and street anchor must stay clear of them.
  const size_t deckCount = scene->walkSurfaces.size();

  // The street anchor keeps clear of every obstacle; the cable (post tops)
  // stays out of every obstacle's footprint+margin unless it is above it.
  auto clear = [&](glm::vec3 roof, glm::vec3 ground, size_t owner) {
    if (!nav.IsWalkable(ground.x, ground.z)) return false;
    const glm::vec2 g(ground.x, ground.z);
    for (size_t i = 0; i < deckCount; ++i) {
      const WalkSurface& deck = scene->walkSurfaces[i];
      std::vector<glm::vec2> deckXZ;
      for (const glm::vec3& v : deck.vertices) deckXZ.emplace_back(v.x, v.z);
      if (DistanceToPolygon(g, deckXZ) < kClearance) return false;
      const glm::vec3 t0 = roof + glm::vec3(0.0f, post, 0.0f);
      const glm::vec3 t1 = ground + glm::vec3(0.0f, post, 0.0f);
      for (int s = 0; s <= 40; ++s) {
        const glm::vec3 p = glm::mix(t0, t1, static_cast<float>(s) / 40.0f);
        const glm::vec2 pxz(p.x, p.z);
        if (DistanceToPolygon(pxz, deckXZ) >= kCableClearance) continue;
        const float deckY = SurfaceHeightAt(deck, p.x, p.z);
        // Fine only if it passes beneath the slab's underside.
        if (p.y > deckY - c.highwayThickness - 0.2f) return false;
      }
    }
    for (size_t i = 0; i < footprints.size(); ++i) {
      const glm::vec4& bb = footprintBounds[i];
      if (g.x < bb.x - kClearance || g.x > bb.z + kClearance || g.y < bb.y - kClearance ||
          g.y > bb.w + kClearance) {
        continue;
      }
      if (DistanceToPolygon(g, footprints[i]) < kClearance) return false;
    }
    const glm::vec3 top0 = roof + glm::vec3(0.0f, post, 0.0f);
    const glm::vec3 top1 = ground + glm::vec3(0.0f, post, 0.0f);
    const int steps = std::max(2, static_cast<int>(glm::distance(top0, top1) / 0.25f));
    for (size_t i = 0; i < footprints.size(); ++i) {
      const glm::vec4& bb = footprintBounds[i];
      const float roofTop = scene->obstacles[i].bounds.max.y;
      // The cable leaves its own building at the roof edge, so it only has
      // to clear the roof itself; every other obstacle also gets a margin.
      const float margin = i == owner ? 0.0f : kCableClearance;
      for (int s = 0; s <= steps; ++s) {
        const glm::vec3 p = glm::mix(top0, top1, static_cast<float>(s) / steps);
        if (p.x < bb.x - margin || p.x > bb.z + margin || p.z < bb.y - margin ||
            p.z > bb.w + margin) {
          continue;
        }
        if (DistanceToPolygon(glm::vec2(p.x, p.z), footprints[i]) <= margin &&
            p.y < roofTop + 0.3f) {
          return false;
        }
      }
    }
    return true;
  };

  std::vector<char> hasLine(scene->obstacles.size(), 0);
  for (const UrbanBlock& block : blocks) {
    std::vector<glm::vec2> poly = block.vertices;
    if (poly.empty()) {
      poly = {{block.x0, block.z0}, {block.x1, block.z0}, {block.x1, block.z1}, {block.x0, block.z1}};
    }
    if (poly.size() < 3) continue;
    // The block's roofs: tall enough obstacles whose footprint centre is in the block.
    std::vector<size_t> roofs;
    for (size_t i = 0; i < footprints.size(); ++i) {
      glm::vec2 centre(0.0f);
      for (const glm::vec2& v : footprints[i]) centre += v;
      centre /= static_cast<float>(footprints[i].size());
      // A base prism carrying a set-back top floor has no free roof.
      const int building = scene->obstacles[i].building;
      const bool covered = building >= 0 &&
                           scene->buildings[building].upperObstacle >= 0 &&
                           scene->buildings[building].obstacle == static_cast<int>(i);
      if (!covered && scene->obstacles[i].bounds.max.y >= kMinRoofHeight &&
          PointInConvexPolygon(centre, poly)) {
        roofs.push_back(i);
      }
    }
    if (roofs.empty()) continue;

    const int want = rng.Int(0, c.maxZiplinesPerBlock);
    int placed = 0;
    for (int attempt = 0; attempt < kAttempts * want && placed < want; ++attempt) {
      const size_t building = roofs[static_cast<size_t>(rng.Int(0, static_cast<int>(roofs.size()) - 1))];
      const float edgePick = rng.Unit01();
      const float bearing = rng.Float(-1.0f, 1.0f) * 1.0472f;  // +-60 deg off the edge's outward normal.
      if (hasLine[building]) continue;

      // Roof anchor: right at a footprint edge, the cable heading out over it.
      const std::vector<glm::vec2>& fp = footprints[building];
      const size_t edge = std::min(fp.size() - 1, static_cast<size_t>(edgePick * fp.size()));
      const glm::vec2 e0 = fp[edge], e1 = fp[(edge + 1) % fp.size()];
      const float edgeLen = glm::length(e1 - e0);
      if (edgeLen < 2.0f) continue;
      const glm::vec2 dir = (e1 - e0) / edgeLen;
      glm::vec2 centre(0.0f);
      for (const glm::vec2& v : fp) centre += v;
      centre /= static_cast<float>(fp.size());
      glm::vec2 outward(dir.y, -dir.x);
      if (glm::dot(outward, e0 - centre) < 0.0f) outward = -outward;
      const float along = rng.Float(0.3f, edgeLen - 0.3f);
      const glm::vec2 anchor = e0 + dir * along - outward * kRoofAnchorInset;
      if (!PointInConvexPolygon(anchor, fp) ||
          DistanceToPolygonEdges(anchor, fp) < kRoofAnchorInset - 0.05f) {
        continue;
      }
      const float roofY = scene->obstacles[building].bounds.max.y + 0.05f;

      const float cs = std::cos(bearing), sn = std::sin(bearing);
      const glm::vec2 heading(outward.x * cs - outward.y * sn, outward.x * sn + outward.y * cs);
      // Street anchor: on the block's sidewalk strip, where the ray from the
      // roof anchor leaves the block (pulled back to mid-sidewalk).
      float exitT = std::numeric_limits<float>::infinity();
      for (size_t i = 0; i < poly.size(); ++i) {
        const glm::vec2 p0 = poly[i], p1 = poly[(i + 1) % poly.size()];
        const glm::vec2 seg = p1 - p0;
        const float denom = heading.x * seg.y - heading.y * seg.x;
        if (std::fabs(denom) < 1e-5f) continue;
        const glm::vec2 rel = p0 - anchor;
        const float t = (rel.x * seg.y - rel.y * seg.x) / denom;
        const float u = (rel.x * heading.y - rel.y * heading.x) / denom;
        if (t > 0.0f && u >= 0.0f && u <= 1.0f) exitT = std::min(exitT, t);
      }
      if (!std::isfinite(exitT)) continue;
      const glm::vec2 street2 = anchor + heading * (exitT - kSidewalkMid);
      const float edgeDist = DistanceToPolygonEdges(street2, poly);
      if (!PointInConvexPolygon(street2, poly) || edgeDist < kSidewalkMin ||
          edgeDist > kSidewalkMax) {
        continue;
      }
      if (std::fabs(street2.x) >= scene->mapHalfExtent - 1.0f ||
          std::fabs(street2.y) >= scene->mapHalfExtent - 1.0f) {
        continue;
      }
      const glm::vec3 roofFoot(anchor.x, roofY, anchor.y);
      const glm::vec3 streetFoot(street2.x, scene->ground.HeightAt(street2.x, street2.y), street2.y);
      if (!clear(roofFoot, streetFoot, building)) continue;
      const float actual = glm::distance(roofFoot, streetFoot);
      if (actual < c.ziplineMinLength || actual > c.ziplineMaxLength) continue;

      bool ok = true;
      for (const Zipline& other : scene->ziplines) {
        ok &= SegmentSegmentDistance(anchor, street2, glm::vec2(other.a.x, other.a.z),
                                     glm::vec2(other.b.x, other.b.z)) >= kLineSpacing;
      }
      for (const Unit& unit : scene->units) {
        ok &= PointSegmentDistance(glm::vec2(unit.position.x, unit.position.z), anchor, street2) >=
              kLineSpacing;
      }
      if (!ok) continue;

      // Roof slab: the footprint, wound counter-clockwise seen from above,
      // at the anchor's height (flush with the roof top).
      WalkSurface slab;
      for (const glm::vec2& v : fp) slab.vertices.emplace_back(v.x, roofY, v.y);
      const glm::vec3 n = glm::cross(slab.vertices[1] - slab.vertices[0],
                                     slab.vertices[2] - slab.vertices[0]);
      if (n.y < 0.0f) std::reverse(slab.vertices.begin(), slab.vertices.end());
      scene->walkSurfaces.push_back(std::move(slab));
      hasLine[building] = 1;

      // The street end is `a` (a figure that walked up to the line starts there).
      scene->ziplines.push_back(Zipline{streetFoot, roofFoot});
      ++placed;
    }
  }
}

// Scene emission, one emitter per feature family. Obstacles are appended in
// emitter order (piers, then buildings), which the tower pick depends on.

// Road pavement, elevated deck chains and their ground/branch links.
void EmitHighwaySurfaces(Scene* scene, const CityPlan& city, const MapGeneratorConfig& c,
                         const GroundY& groundY) {
  const HighwayPlan& plan = city.highway;
  const std::vector<UrbanRoad>& roads = city.streets.roads;
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
    const auto left = RibbonSide(road, true, scene->mapHalfExtent, groundY);
    const auto right = RibbonSide(road, false, scene->mapHalfExtent, groundY);
    const std::vector<float> params = roadIndex == 0
                                          ? SampleParams(plan.artery.s0, plan.artery.s1)
                                          : SampleParams(plan.branch.u0, plan.branch.length);
    Chain& chain = chains[roadIndex];
    for (size_t i = 0; i + 1 < road.centerline.size(); ++i) {
      const bool deck =
          plan.elevated &&
          std::max(road.centerline[i].y, road.centerline[i + 1].y) > kGradeY + 0.05f;
      if (!deck) {
        scene->roads.push_back(RoadSurface{{left[i], right[i], right[i + 1], left[i + 1]}});
        continue;
      }
      WalkSurface surface;
      surface.vertices = {left[i], right[i], right[i + 1], left[i + 1]};
      const int index = static_cast<int>(scene->walkSurfaces.size());
      if (chain.first < 0) chain.first = index;
      if (chain.last >= 0) {
        surface.neighbors.push_back(chain.last);
        scene->walkSurfaces[chain.last].neighbors.push_back(index);
      }
      chain.last = index;
      chain.loS.push_back(params[i]);
      chain.hiS.push_back(params[i + 1]);
      scene->walkSurfaces.push_back(std::move(surface));
    }
  }
  if (plan.elevated && chains[0].first >= 0) {
    // A deck chain end is a legal ground transition only where the profile
    // actually returns to grade on-map (an off-map elevated end never does);
    // the connecting ramp's foot always starts at grade.
    if (ArteryElevationAt(plan, c, plan.artery.s0) < kGradeY + 0.05f) {
      scene->walkSurfaces[chains[0].first].connectsToGround = true;
    }
    if (ArteryElevationAt(plan, c, plan.artery.s1) < kGradeY + 0.05f) {
      scene->walkSurfaces[chains[0].last].connectsToGround = true;
    }
    if (roads.size() > 1 && chains[1].first >= 0) {
      if (!plan.branch.offMap) scene->walkSurfaces[chains[1].first].connectsToGround = true;
      // The on-ramp rides level with the deck across the whole junction
      // overlap (its profile only starts descending once clear of the
      // artery), so every overlapping same-height deck/branch span pair is a
      // legal crossing. A single link at the merge span would funnel every
      // deck<->branch route through that one waypoint, pricing branch cells
      // right next to a mover out of its move budget (issue #135).
      bool linked = false;
      for (int bi = chains[1].first; bi <= chains[1].last; ++bi) {
        for (int di = chains[0].first; di <= chains[0].last; ++di) {
          if (!SpansTouchAtSameHeight(scene->walkSurfaces[di], scene->walkSurfaces[bi])) continue;
          scene->walkSurfaces[bi].neighbors.push_back(di);
          scene->walkSurfaces[di].neighbors.push_back(bi);
          linked = true;
        }
      }
      if (!linked) {
        // Degenerate junction (no detectable overlap): fall back to linking
        // the on-ramp's top to the deck span holding its merge point.
        int deckAtMerge = -1;
        for (size_t i = 0; i < chains[0].loS.size(); ++i) {
          if (plan.branch.mergeS >= chains[0].loS[i] - 1e-3f &&
              plan.branch.mergeS <= chains[0].hiS[i] + 1e-3f) {
            deckAtMerge = chains[0].first + static_cast<int>(i);
          }
        }
        if (deckAtMerge >= 0) {
          scene->walkSurfaces[chains[1].last].neighbors.push_back(deckAtMerge);
          scene->walkSurfaces[deckAtMerge].neighbors.push_back(chains[1].last);
        }
      }
    }
  }
}

// Asphalt for every local street (avenues and cross streets): the cut's
// band clipped to the map and to its side of the artery corridor. Neighbouring
// bands overlap coplanar at junctions, so the crossing is paved by both.
void EmitStreetSurfaces(Scene* scene, const CityPlan& city, const MapGeneratorConfig& c,
                        const GroundY& groundY) {
  const ArterySpec& artery = city.highway.artery;
  const float arteryD = glm::dot(artery.normal, artery.origin);
  for (const StreetCut& cut : city.streets.cuts) {
    if (cut.cls == UrbanStreetClass::Highway) continue;  // A highway road already.
    float t0, t1;
    ClipLineToSquare(cut.origin, cut.dir, scene->mapHalfExtent, &t0, &t1);
    if (!(t1 > t0)) continue;
    const glm::vec2 n(-cut.dir.y, cut.dir.x);
    const float hw = cut.width * 0.5f;
    const float half = scene->mapHalfExtent;
    const float d = glm::dot(n, cut.origin);
    // Clip an infinite street band out of the map square. Starting with a
    // centerline clipped to the square would leave only half the pavement at
    // an oblique boundary crossing: one corner lies outside the map while its
    // opposite corner still stops short of the edge.
    std::vector<glm::vec2> band = {{-half, half}, {half, half}, {half, -half}, {-half, -half}};
    band = KeepSide(band, n, d - hw);
    band = KeepSide(band, -n, -(d + hw));
    band = KeepSide(band, artery.normal * static_cast<float>(cut.side),
                    static_cast<float>(cut.side) * arteryD + c.arteryWidth * 0.5f - 0.2f);
    if (band.size() < 3 || PolygonArea(band) < 1.0f) continue;
    RoadSurface road;
    for (const glm::vec2& p : band) road.vertices.emplace_back(p.x, groundY(p.x, p.y) + kGradeY, p.y);
    scene->roads.push_back(std::move(road));
  }
}

// Bridge piers under the elevated spans.
void EmitPiers(Scene* scene, const CityPlan& city, const MapGeneratorConfig& c,
               const GroundY& groundY) {
  const HighwayPlan& plan = city.highway;
  // Bridge stands: paired pier columns under the high spans, leaving broad
  // navigable ground between bents. None near the on-ramp's merge, where the
  // two slabs overlap.
  if (plan.elevated) {
    // A column may only stand where it stays below every slab crossing it
    // (and never on at-grade pavement, whose slab sits at ground level).
    auto columnFits = [&](glm::vec2 foot, float relativeTop) {
      const float sA = glm::dot(foot - plan.artery.origin, plan.artery.dir);
      const float dA = std::fabs(glm::dot(foot - plan.artery.origin, plan.artery.normal));
      if (dA < c.arteryWidth * 0.5f + 0.8f &&
          relativeTop > ArteryElevationAt(plan, c, sA) - c.highwayThickness - 0.049f) {
        return false;
      }
      if (plan.hasBranch) {
        const glm::vec2 rel = foot - plan.branch.start;
        const float u = glm::dot(rel, plan.branch.dir);
        const glm::vec2 bn(-plan.branch.dir.y, plan.branch.dir.x);
        if (u > -0.5f && u < plan.branch.length + 0.5f &&
            std::fabs(glm::dot(rel, bn)) < plan.branch.width * 0.5f + 0.8f &&
            relativeTop > BranchElevationAt(plan, c, u) - c.highwayThickness - 0.049f) {
          return false;
        }
      }
      return true;
    };
    auto addBent = [&](glm::vec2 center, glm::vec2 across, float halfSpan, float deckY) {
      for (const float lat : {1.0f, -1.0f}) {
        const glm::vec2 foot = center + across * (lat * halfSpan);
        const float height = deckY - c.highwayThickness - 0.05f;
        // A column stands fully on the map even where the deck runs past it.
        if (std::max(std::fabs(foot.x), std::fabs(foot.y)) > scene->mapHalfExtent - 0.75f) continue;
        if (!columnFits(foot, height)) continue;
        const float baseY = groundY(foot.x, foot.y);
        scene->obstacles.push_back(
            Obstacle{AABB{glm::vec3(foot.x - 0.7f, baseY, foot.y - 0.7f),
                          glm::vec3(foot.x + 0.7f, baseY + height, foot.y + 0.7f)},
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
}

// Sidewalk rings, then one Building per parcel: its base prism over the
// whole footprint and, for a set-back office, the top floor as a second,
// inset prism standing on it (both tagged with the Building index).
void EmitBuildings(Scene* scene, const CityPlan& city, const std::vector<Parcel>& parcels,
                   std::vector<Building> buildings, const MapGeneratorConfig& c,
                   const GroundY& groundY) {
  for (const UrbanBlock& block : city.blocks) {
    // Sidewalk ring only: the parcel interior stays at grade. Each block
    // edge owns the convex strip of the block within `sidewalkWidth` of it;
    // strips overlap coplanar at the corners.
    const std::vector<glm::vec2>& outline = block.vertices;
    glm::vec2 centroid(0.0f);
    for (const glm::vec2& p : outline) centroid += p;
    centroid /= static_cast<float>(outline.size());
    for (size_t e = 0; e < outline.size(); ++e) {
      const glm::vec2 a = outline[e], b = outline[(e + 1) % outline.size()];
      if (glm::length(b - a) < 1e-4f) continue;
      glm::vec2 inward = glm::normalize(glm::vec2(-(b - a).y, (b - a).x));
      if (glm::dot(inward, centroid - a) < 0.0f) inward = -inward;
      const std::vector<glm::vec2> strip =
          KeepSide(outline, -inward, -(glm::dot(inward, a) + c.sidewalkWidth));
      if (strip.size() < 3 || PolygonArea(strip) < 0.5f) continue;
      RoadSurface sidewalk;
      for (const glm::vec2& p : strip) {
        sidewalk.vertices.emplace_back(p.x, groundY(p.x, p.y) + kSidewalkHeight, p.y);
      }
      scene->sidewalkSurfaces.push_back(std::move(sidewalk));
    }
  }
  constexpr float kSetbackInset = 1.2f;
  for (size_t i = 0; i < parcels.size(); ++i) {
    Building& b = buildings[i];
    const int index = static_cast<int>(scene->buildings.size());
    const bool setback = b.upperObstacle == -2;
    std::vector<glm::vec2> upper;
    if (setback) {
      upper = InsetConvexPolygon(parcels[i].footprint, kSetbackInset);
      if (upper.size() < 3 || PolygonArea(upper) < 6.0f ||
          PolygonArea(upper) < 0.4f * PolygonArea(parcels[i].footprint)) {
        upper.clear();
      }
    }
    b.upperObstacle = -1;
    const float height = b.Height();
    const float baseHeight = upper.empty() ? height : height - b.floorHeight;
    b.obstacle = AddPolygonBuilding(scene, parcels[i].footprint, 0.0f, baseHeight, index, groundY);
    if (!upper.empty()) {
      b.upperObstacle =
          AddPolygonBuilding(scene, std::move(upper), baseHeight, b.floorHeight, index, groundY);
    }
    scene->buildings.push_back(b);
  }
}

// --- Massing details (issue #151, roadmap 3.3) ------------------------------

// Parapets, roof furniture and awnings (sight-blocking, not collision). Every
// roof gets them, zipline roofs included: a parapet is skipped at random
// (the furniture stays), and furniture keeps clear of zipline landings.
// Awnings hang over the sidewalk well above head height. Own rng stream, so
// the buildings never move when the dressing changes.
void EmitMassing(Scene* scene, const std::vector<Parcel>& parcels, uint32_t seed) {
  Rng rng(seed ^ kMassingStream);
  constexpr float kParapetThickness = 0.25f;
  constexpr float kAwningDepth = 1.1f;  // Over the 1.5 sidewalk, short of the curb.
  constexpr float kAwningThickness = 0.15f;
  struct Fixture { float along, across, height; };
  const Fixture kTank{1.2f, 1.2f, 1.5f}, kAc{0.9f, 0.6f, 0.6f}, kBulkhead{2.2f, 1.6f, 1.3f};

  for (size_t bi = 0; bi < scene->buildings.size(); ++bi) {
    const Building& b = scene->buildings[bi];
    const Parcel& parcel = parcels[b.parcel];
    const Obstacle& roofPrism =
        scene->obstacles[b.upperObstacle >= 0 ? b.upperObstacle : b.obstacle];
    const std::vector<glm::vec2>& roof = roofPrism.footprint;
    const float roofTop = roofPrism.bounds.max.y;
    const glm::vec2 fa = parcel.footprint[parcel.frontageEdge];
    const glm::vec2 fb = parcel.footprint[(parcel.frontageEdge + 1) % parcel.footprint.size()];
    const glm::vec2 along = glm::normalize(fb - fa);
    const glm::vec2 outward(along.y, -along.x);  // Right of a CCW edge: the street side.

    std::vector<glm::vec2> landings;  // Zipline feet on this roof.
    for (const Zipline& line : scene->ziplines) {
      for (const glm::vec3& foot : {line.a, line.b}) {
        if (std::fabs(foot.y - roofTop) < 0.2f &&
            PointInConvexPolygon(glm::vec2(foot.x, foot.z), roof)) {
          landings.emplace_back(foot.x, foot.z);
        }
      }
    }
    constexpr float kLandingClearance = 1.5f;
    const bool wrapAround = rng.Float(0.0f, 1.0f) < 0.75f;
    auto push = [&](MassingDetail::Kind kind, std::vector<glm::vec2> footprint, float y0,
                    float y1) {
      if (PolygonSignedArea(footprint) < 0.0f) std::reverse(footprint.begin(), footprint.end());
      // Parapets and furniture also stop figures walking on the roof slab.
      if (kind != MassingDetail::Kind::Awning) {
        for (WalkSurface& slab : scene->walkSurfaces) {
          if (slab.vertices.empty() || !slab.neighbors.empty() || slab.connectsToGround ||
              std::fabs(slab.vertices[0].y - y0) > 0.1f) {
            continue;
          }
          glm::vec2 mid(0.0f);
          for (const glm::vec2& v : footprint) mid += v / static_cast<float>(footprint.size());
          if (SurfaceContainsXZ(slab, mid.x, mid.y)) {
            slab.blockers.push_back(footprint);
            break;
          }
        }
      }
      scene->massing.push_back(
          MassingDetail{kind, std::move(footprint), y0, y1, static_cast<int>(bi)});
    };

    {
      // Parapet: a low wall along every roof edge (most roofs).
      float parapet = 0.6f;
      switch (b.kind) {
        case BuildingKind::Residential: parapet = 0.6f; break;
        case BuildingKind::MixedUse: parapet = 0.7f; break;
        case BuildingKind::Office: parapet = 0.9f; break;
        case BuildingKind::Commercial: parapet = 0.5f; break;
        case BuildingKind::Civic: parapet = 1.0f; break;
        case BuildingKind::Garage: parapet = 1.1f; break;  // Deck guard wall.
      }
      // Exact miters from the inset outline; a roof too small or too sharp
      // for a valid inset (collapsed or flipped edges) gets no parapet.
      const std::vector<glm::vec2> coping = InsetConvexPolygon(roof, kParapetThickness);
      bool copingUsable = coping.size() == roof.size() && PolygonSignedArea(coping) > 0.1f;
      for (size_t e = 0; copingUsable && e < roof.size(); ++e) {
        const size_t next = (e + 1) % roof.size();
        copingUsable = glm::dot(roof[next] - roof[e], coping[next] - coping[e]) > 0.0f &&
                       PointInConvexPolygon(coping[e], roof);
      }
      for (size_t e = 0; wrapAround && copingUsable && e < roof.size(); ++e) {
        const size_t next = (e + 1) % roof.size();
        if (glm::distance(roof[e], roof[next]) < 2.0f * kParapetThickness) continue;
        // Leave a gap where a zipline leaves the roof.
        bool landingEdge = false;
        for (const glm::vec2& landing : landings) {
          landingEdge |= PointSegmentDistance(landing, roof[e], roof[next]) < 0.6f;
        }
        if (landingEdge) continue;
        push(MassingDetail::Kind::Parapet, {roof[e], roof[next], coping[next], coping[e]},
             roofTop, roofTop + parapet);
      }

      // Roof furniture: water tanks, AC units, stair bulkheads, placed inside
      // the roof clear of the parapet and of one another.
      int want = 0;
      std::vector<const Fixture*> menu;
      switch (b.kind) {
        case BuildingKind::Residential: want = rng.Int(1, 2); menu = {&kTank, &kAc}; break;
        case BuildingKind::MixedUse: want = rng.Int(1, 2); menu = {&kTank, &kAc, &kBulkhead}; break;
        case BuildingKind::Office: want = rng.Int(1, 3); menu = {&kAc, &kBulkhead}; break;
        case BuildingKind::Commercial: want = rng.Int(2, 4); menu = {&kAc}; break;
        case BuildingKind::Civic: want = 1; menu = {&kBulkhead}; break;
        case BuildingKind::Garage: want = rng.Int(0, 1); menu = {&kBulkhead}; break;
      }
      std::vector<std::pair<glm::vec2, float>> placed;  // Centre, half-diagonal.
      for (int n = 0; n < want; ++n) {
        const Fixture& f = *menu[static_cast<size_t>(rng.Int(0, static_cast<int>(menu.size()) - 1))];
        const float halfDiag = 0.5f * std::hypot(f.along, f.across);
        const float margin = kParapetThickness + 0.4f;  // Clear of the parapet.
        glm::vec2 lo(std::numeric_limits<float>::infinity()), hi(-lo);
        for (const glm::vec2& v : roof) {
          lo = glm::min(lo, v);
          hi = glm::max(hi, v);
        }
        if (PolygonArea(roof) < 4.0f * (halfDiag + margin) * (halfDiag + margin)) break;
        for (int attempt = 0; attempt < 12; ++attempt) {
          const glm::vec2 centre(rng.Float(lo.x, hi.x), rng.Float(lo.y, hi.y));
          // Every corner on the roof and clear of its edge (checked against
          // the roof itself: a deep inset of a sharp roof isn't trustworthy).
          const glm::vec2 u = along * (0.5f * f.along), v = outward * (0.5f * f.across);
          const glm::vec2 corners[4] = {centre - u - v, centre + u - v, centre + u + v,
                                        centre - u + v};
          bool fits = true;
          for (const glm::vec2& corner : corners) {
            fits &= PointInConvexPolygon(corner, roof) &&
                    DistanceToPolygonEdges(corner, roof) >= margin;
          }
          for (const glm::vec2& landing : landings) {
            fits &= glm::distance(centre, landing) >= halfDiag + kLandingClearance;
          }
          if (!fits) continue;
          bool overlaps = false;
          for (const auto& [other, r] : placed) {
            overlaps |= glm::distance(centre, other) < r + halfDiag + 0.3f;
          }
          if (overlaps) continue;
          push(MassingDetail::Kind::RoofFixture, {corners[0], corners[1], corners[2], corners[3]},
               roofTop, roofTop + f.height);
          placed.emplace_back(centre, halfDiag);
          break;
        }
      }
    }

    // Awning/canopy over the shopfront of commercial ground floors.
    if (b.kind == BuildingKind::MixedUse || b.kind == BuildingKind::Commercial) {
      const float frontage = glm::distance(fa, fb);
      if (frontage > 2.0f) {
        const float trim = 0.4f;
        const glm::vec2 a = fa + along * trim, d = fb - along * trim;
        const float base = scene->obstacles[b.obstacle].bounds.min.y;
        const float y0 = base + std::min(b.groundFloorHeight - 0.9f, 3.4f);
        push(MassingDetail::Kind::Awning,
             {a, d, d + outward * kAwningDepth, a + outward * kAwningDepth}, y0,
             y0 + kAwningThickness);
      }
    }
  }
}

// Squads at the artery's ends.
void EmitSpawns(Scene* scene, const CityPlan& city, const MapGeneratorConfig& c,
                const GroundY& groundY) {
  const HighwayPlan& plan = city.highway;
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
    unit.position = glm::vec3(p.x, groundY(p.x, p.y), p.y);
    unit.facingYaw = blueTeam ? yawBlue : yawBlue + kPi;
    scene->units.push_back(unit);
  }
}

// Presence of the furniture layer (issue #150): unconditional unless the
// feature is forced Off; the seed never decides (Auto behaves like On).
bool FurnitureEnabled(const MapGeneratorConfig& c) {
  return c.features.furniture.mode != FeatureMode::Off;
}

// Curb-side furniture items (issue #150, roadmap 2.4). Footprints stay small
// enough that the sidewalk keeps 2 * kAgentRadius of clear width behind every
// item: sidewalkWidth 1.5 - curb inset 0.1 - depth <= 0.36 leaves >= 1.0.
struct FurnitureSpec {
  float sx, sz, height;
};
constexpr FurnitureSpec kFurnitureSpecs[] = {
    {0.36f, 0.36f, 0.95f},  // Trash can.
    {0.14f, 0.14f, 0.75f},  // Bollard.
    {0.26f, 0.26f, 0.70f},  // Fire hydrant.
    {0.12f, 0.12f, 2.40f},  // Street sign (post; the plate is below eye level anyway).
    {0.16f, 0.16f, 5.40f},  // Lamp post (the rendered arm continues above/over the street).
    {0.34f, 0.26f, 1.05f},  // Mailbox.
};
constexpr int kFurnitureKinds = static_cast<int>(sizeof(kFurnitureSpecs) / sizeof(FurnitureSpec));

// Street furniture, first batch (issue #150, roadmap 2.4): small obstacles
// hugging the sidewalk's curb edge. Items only stand where a building fronts
// the sidewalk (so alleys, block openings and park blocks stay clear), keep
// 2 * kAgentRadius of clear sidewalk behind them, and stay off the spawn
// rows. Axis-aligned boxes with no polygon footprint, so the tower pick and
// the building-height expectations never see them. Posts are thinner than
// the agent radius, so they block LOS only trivially.
void EmitStreetFurniture(Scene* scene, const CityPlan& city, const MapGeneratorConfig& c,
                         uint32_t seed, const GroundY& groundY) {
  Rng rng(seed ^ kFurnitureStream);
  rng.Chance(0.5f);  // Legacy presence draw, kept so the item layout stays stable.
  if (!FurnitureEnabled(c)) return;

  // Copies, not pointers: appending furniture reallocates scene->obstacles.
  std::vector<std::vector<glm::vec2>> buildings;
  for (const Obstacle& o : scene->obstacles) {
    if (!o.footprint.empty()) buildings.push_back(o.footprint);
  }

  constexpr float kCornerMargin = 2.2f;  // Corners stay free for the signage pass.
  constexpr float kEdgeInset = 0.10f;    // Outer face sits just inside the curb line.
  constexpr float kItemSpacing = 1.6f;
  constexpr float kSpawnClearance = 3.0f;
  const float clearBehind = 2.0f * constants::kAgentRadius;

  std::vector<glm::vec2> placed;
  for (const UrbanBlock& block : city.blocks) {
    const std::vector<glm::vec2>& poly = block.vertices;
    glm::vec2 centroid(0.0f);
    for (const glm::vec2& p : poly) centroid += p;
    centroid /= static_cast<float>(poly.size());
    for (size_t e = 0; e < poly.size(); ++e) {
      const glm::vec2 a = poly[e], b = poly[(e + 1) % poly.size()];
      const float length = glm::distance(a, b);
      if (length < 2.0f * kCornerMargin + 1.0f) continue;
      const glm::vec2 dir = (b - a) / length;
      glm::vec2 inward(-dir.y, dir.x);
      if (glm::dot(inward, centroid - a) < 0.0f) inward = -inward;
      // Seeded stations along the curb. Every draw happens whether or not the
      // station passes its checks, so a rejected item never reshuffles the
      // rest of the seed's furniture.
      float along = kCornerMargin + rng.Float(0.0f, 4.0f);
      while (along < length - kCornerMargin) {
        const glm::vec2 curb = a + dir * along;
        const int kind = rng.Int(0, kFurnitureKinds - 1);
        const bool skip = rng.Chance(0.35f);  // Density: some stretches stay bare.
        along += rng.Float(4.5f, 9.0f);
        if (skip) continue;
        const FurnitureSpec& spec = kFurnitureSpecs[kind];
        const float halfOut = 0.5f * std::max(spec.sx, spec.sz);
        const glm::vec2 center = curb + inward * (kEdgeInset + halfOut);
        if (std::max(std::fabs(center.x), std::fabs(center.y)) > scene->mapHalfExtent - 0.6f) {
          continue;
        }
        // Only in front of a building, with the clear-sidewalk gap behind:
        // alleys, courtyard openings and park blocks keep their full width.
        const glm::vec2 probe = curb + inward * (c.sidewalkWidth + 0.5f * c.buildingDepth);
        bool fronted = false;
        bool clear = true;
        for (const std::vector<glm::vec2>& footprint : buildings) {
          fronted |= PointInConvexPolygon(probe, footprint);
          clear &= DistanceToPolygon(center, footprint) >= halfOut + clearBehind + 0.05f;
        }
        for (const Unit& unit : scene->units) {
          clear &= glm::distance(glm::vec2(unit.position.x, unit.position.z), center) >=
                   kSpawnClearance;
        }
        for (const glm::vec2& q : placed) clear &= glm::distance(q, center) >= kItemSpacing;
        if (!fronted || !clear) continue;
        const float baseY = groundY(center.x, center.y);
        scene->obstacles.push_back(Obstacle{
            AABB{glm::vec3(center.x - spec.sx * 0.5f, baseY, center.y - spec.sz * 0.5f),
                 glm::vec3(center.x + spec.sx * 0.5f, baseY + kSidewalkHeight + spec.height,
                           center.y + spec.sz * 0.5f)},
            /*climbable=*/false,
            /*footprint=*/{},
            ObstacleKind::StreetFurniture});
        if (kind == 4) {
          // The thin obstacle is the physical pole. Its visible arm is a
          // fixture aimed out from the curb so the lamp hangs over the road.
          const glm::vec2 outward = -inward;
          const float yaw = std::atan2(outward.y, outward.x);
          scene->props.push_back(StreetProp{StreetPropKind::LampPost,
                                            glm::vec3(center.x, baseY + kSidewalkHeight, center.y),
                                            yaw, yaw});
        }
        placed.push_back(center);
      }
    }
  }
}

// Centerline param range of a local street cut: clipped to the map square
// and to its side of the artery corridor (the same clips the street-surface
// emitter makes), so junction tests run against the street's real extent.
bool CutParamRange(const StreetCut& cut, const ArterySpec& artery, const MapGeneratorConfig& c,
                   float half, float* t0, float* t1) {
  ClipLineToSquare(cut.origin, cut.dir, half, t0, t1);
  const glm::vec2 n = artery.normal * static_cast<float>(cut.side);
  const float dMin = static_cast<float>(cut.side) * glm::dot(artery.normal, artery.origin) +
                     c.arteryWidth * 0.5f - 0.2f;
  const float originD = glm::dot(n, cut.origin);
  const float dirD = glm::dot(n, cut.dir);
  if (std::fabs(dirD) < 1e-6f) {
    if (originD < dMin) return false;
  } else {
    const float tCross = (dMin - originD) / dirD;
    if (dirD > 0.0f) {
      *t0 = std::max(*t0, tCross);
    } else {
      *t1 = std::min(*t1, tCross);
    }
  }
  return *t1 > *t0;
}

// Traffic lights and signage (issue #150, roadmap 2.6), all visual-only
// Scene::props: traffic lights on the corners of four-arm local crossings,
// a per-junction draw of lights vs. stop/yield signs where a street tees
// into the artery corridor, and street-name signs on block corners.
void EmitTrafficControl(Scene* scene, const CityPlan& city, const MapGeneratorConfig& c,
                        uint32_t seed, const GroundY& groundY) {
  if (!FurnitureEnabled(c)) return;
  Rng rng(seed ^ kTrafficStream);
  const ArterySpec& artery = city.highway.artery;
  const float half = scene->mapHalfExtent;

  // Validity checks consume no rng, so a rejected corner (off-map, inside a
  // building, crowding another prop) never reshuffles later draws.
  std::vector<glm::vec2> placed;
  auto propY = [&](glm::vec2 p) {
    for (const UrbanBlock& block : city.blocks) {
      if (PointInConvexPolygon(p, block.vertices)) return groundY(p.x, p.y) + kSidewalkHeight;
    }
    return groundY(p.x, p.y) + kGradeY;
  };
  // Signals and signs stand on the sidewalk corner, not where the street
  // geometry estimates it: snap to the nearest block corner, nudged inward.
  auto snapToCorner = [&](glm::vec2 p, glm::vec2* out) {
    float best = 3.5f;
    bool found = false;
    for (const UrbanBlock& block : city.blocks) {
      glm::vec2 centroid(0.0f);
      for (const glm::vec2& v : block.vertices) centroid += v;
      centroid /= static_cast<float>(block.vertices.size());
      for (const glm::vec2& v : block.vertices) {
        const float d = glm::distance(p, v);
        if (d >= best) continue;
        best = d;
        *out = v + glm::normalize(centroid - v) * 0.5f;
        found = true;
      }
    }
    return found;
  };
  auto add = [&](StreetPropKind kind, glm::vec2 p, float yaw, float secondaryYaw) {
    if (kind != StreetPropKind::StreetNameSign && !snapToCorner(p, &p)) return;
    if (std::max(std::fabs(p.x), std::fabs(p.y)) > half - 0.5f) return;
    for (const Obstacle& o : scene->obstacles) {
      if (p.x < o.bounds.min.x - 0.4f || p.x > o.bounds.max.x + 0.4f ||
          p.y < o.bounds.min.z - 0.4f || p.y > o.bounds.max.z + 0.4f) {
        continue;
      }
      if (DistanceToPolygon(p, ObstacleFootprint(o)) < 0.4f) return;
    }
    for (const glm::vec2& q : placed) {
      if (glm::distance(p, q) < 1.0f) return;
    }
    const float y = propY(p);
    scene->props.push_back(StreetProp{kind, glm::vec3(p.x, y, p.y), yaw, secondaryYaw});
    // The pole is solid: a thin obstacle hidden inside the rendered pole
    // (heights match BuildStreetPropParts) so units can't walk through it.
    const float post = 0.06f;
    const float height = kind == StreetPropKind::TrafficLight   ? 3.1f
                         : kind == StreetPropKind::StreetNameSign ? 2.5f
                                                                  : 2.2f;
    scene->obstacles.push_back(Obstacle{
        AABB{glm::vec3(p.x - post * 0.5f, y, p.y - post * 0.5f),
             glm::vec3(p.x + post * 0.5f, y + height, p.y + post * 0.5f)},
        /*climbable=*/false,
        /*footprint=*/{},
        ObstacleKind::StreetFurniture});
    placed.push_back(p);
  };
  const std::vector<StreetCut>& cuts = city.streets.cuts;
  std::vector<float> lo(cuts.size()), hi(cuts.size());
  std::vector<char> valid(cuts.size());
  for (size_t i = 0; i < cuts.size(); ++i) {
    valid[i] = CutParamRange(cuts[i], artery, c, half, &lo[i], &hi[i]) ? 1 : 0;
  }

  // Four-arm crossings between two streets of one side (the wide branch
  // corridor counts as a street). A smaller street crossing a larger one
  // (avenue or branch corridor) always gets a signal; two streets of the same
  // class get one 20% of the time. Either way only one street is signalled:
  // a head on the sidewalk corner facing each approach of the smaller street.
  constexpr float kInteriorMargin = 1.5f;
  constexpr float kCornerSetback = 0.55f;  // Post stands just off the pavement corner.
  for (size_t i = 0; i < cuts.size(); ++i) {
    for (size_t j = i + 1; j < cuts.size(); ++j) {
      // Drawn before any rejection so a skipped crossing never reshuffles
      // the rest.
      const bool minorLight = rng.Chance(0.2f);
      if (!valid[i] || !valid[j] || cuts[i].side != cuts[j].side) continue;
      const auto rank = [](UrbanStreetClass cls) {
        return cls == UrbanStreetClass::Cross ? 0 : 1;
      };
      const int ri = rank(cuts[i].cls), rj = rank(cuts[j].cls);
      if (ri == rj && !minorLight) continue;
      // The street whose approaches carry the heads: the smaller one.
      const bool signalI = ri <= rj;
      // An elevated seed's branch corridor carries a ramp overhead: no posts
      // under it.
      if (city.highway.elevated && (cuts[i].cls == UrbanStreetClass::Highway ||
                                    cuts[j].cls == UrbanStreetClass::Highway)) {
        continue;
      }
      const glm::vec2 d1 = cuts[i].dir, d2 = cuts[j].dir;
      const float denom = d1.x * d2.y - d1.y * d2.x;
      if (std::fabs(denom) < 1e-4f) continue;
      const glm::vec2 rel = cuts[j].origin - cuts[i].origin;
      const float ti = (rel.x * d2.y - rel.y * d2.x) / denom;
      const float tj = (rel.x * d1.y - rel.y * d1.x) / denom;
      if (ti < lo[i] + kInteriorMargin || ti > hi[i] - kInteriorMargin) continue;
      if (tj < lo[j] + kInteriorMargin || tj > hi[j] - kInteriorMargin) continue;
      const glm::vec2 junction = cuts[i].origin + d1 * ti;
      for (const float s1 : {1.0f, -1.0f}) {
        for (const float s2 : {1.0f, -1.0f}) {
          const glm::vec2 corner = junction + d1 * (s1 * (cuts[j].width * 0.5f + kCornerSetback)) +
                                   d2 * (s2 * (cuts[i].width * 0.5f + kCornerSetback));
          // Opposite corners serve the two approaches of the signalled
          // street; face down that street toward approaching traffic.
          if ((s1 * s2 > 0.0f) != signalI) continue;
          const glm::vec2 facing = signalI ? -s1 * d1 : -s2 * d2;
          const float yaw = std::atan2(facing.y, facing.x);
          add(StreetPropKind::TrafficLight, corner, yaw, yaw);
        }
      }
    }
  }

  // T junctions where a local street meets the artery corridor: per-junction
  // draw of a pair of traffic lights vs. stop/yield signs facing the traffic
  // that arrives at the artery.
  const float edgeD = c.arteryWidth * 0.5f - 0.2f;
  for (size_t i = 0; i < cuts.size(); ++i) {
    // Draws come first, unconditionally, so a skipped junction never
    // reshuffles the ones after it.
    const bool light = rng.Chance(0.55f);
    const bool stop = rng.Chance(0.5f);
    if (!valid[i] || cuts[i].cls == UrbanStreetClass::Highway) continue;
    auto arteryDistance = [&](float t) {
      return std::fabs(glm::dot(artery.normal, cuts[i].origin + cuts[i].dir * t - artery.origin));
    };
    const float dLo = arteryDistance(lo[i]);
    const float dHi = arteryDistance(hi[i]);
    if (std::min(dLo, dHi) > edgeD + 1.0f) continue;  // Never reaches the artery.
    const float tEnd = dLo < dHi ? lo[i] : hi[i];
    const glm::vec2 toward = dLo < dHi ? -cuts[i].dir : cuts[i].dir;  // Toward the artery.
    const glm::vec2 end = cuts[i].origin + cuts[i].dir * tEnd;
    // No junction props where the artery runs overhead as a deck.
    const float arteryS = glm::dot(end - artery.origin, artery.dir);
    if (ArteryElevationAt(city.highway, c, arteryS) > kGradeY + 0.05f) continue;
    const glm::vec2 n(-cuts[i].dir.y, cuts[i].dir.x);
    for (const float s : {1.0f, -1.0f}) {
      const glm::vec2 corner =
          end - toward * 0.6f + n * (s * (cuts[i].width * 0.5f + kCornerSetback));
      if (light) {
        const float yaw = std::atan2(-toward.y, -toward.x);
        add(StreetPropKind::TrafficLight, corner, yaw, yaw);
      } else {
        const float yaw = std::atan2(-toward.y, -toward.x);
        add(stop ? StreetPropKind::StopSign : StreetPropKind::YieldSign, corner, yaw, yaw);
      }
    }
  }

  // Street-name signs on block corners (seeded draw per corner).
  for (const UrbanBlock& block : city.blocks) {
    const std::vector<glm::vec2>& poly = block.vertices;
    glm::vec2 centroid(0.0f);
    for (const glm::vec2& p : poly) centroid += p;
    centroid /= static_cast<float>(poly.size());
    for (size_t v = 0; v < poly.size(); ++v) {
      if (!rng.Chance(0.4f)) continue;
      const glm::vec2 inward = glm::normalize(centroid - poly[v]);
      const glm::vec2 nextEdge = glm::normalize(poly[(v + 1) % poly.size()] - poly[v]);
      const glm::vec2 previousEdge =
          glm::normalize(poly[v] - poly[(v + poly.size() - 1) % poly.size()]);
      add(StreetPropKind::StreetNameSign, poly[v] + inward * 0.5f,
          std::atan2(nextEdge.y, nextEdge.x), std::atan2(previousEdge.y, previousEdge.x));
    }
  }
}

Scene GenerateArterialUrbanMap(uint32_t seed, const MapGeneratorConfig& c,
                               const GroundY& groundY) {
  Rng rng(seed);
  Scene scene;
  scene.mapHalfExtent = UrbanMapHalfExtent(c);
  const CityPlan city = BuildCityPlan(seed, c);
  EmitHighwaySurfaces(&scene, city, c, groundY);
  EmitStreetSurfaces(&scene, city, c, groundY);
  EmitPiers(&scene, city, c, groundY);
  std::vector<float> characterRolls;
  const std::vector<Parcel> parcels = PlanParcels(city, c, &rng, &characterRolls);
  EmitBuildings(&scene, city, parcels, PlanBuildings(parcels, characterRolls, seed, c, &rng), c,
                groundY);
  EmitSpawns(&scene, city, c, groundY);
  // Furniture before ziplines: anchors and cables then keep clear of it.
  EmitStreetFurniture(&scene, city, c, seed, groundY);
  EmitTrafficControl(&scene, city, c, seed, groundY);
  PlaceZiplines(&scene, city.blocks, seed, c);
  EmitMassing(&scene, parcels, seed);
  return scene;
}

}  // namespace

FeatureToggle* UrbanFeatureByName(UrbanFeatures* f, const std::string& name) {
  if (name == "lanes") return &f->lanes;
  if (name == "curbs") return &f->curbs;
  if (name == "furniture") return &f->furniture;
  if (name == "terrain") return &f->terrain;
  if (name == "interiors") return &f->interiors;
  if (name == "rail") return &f->rail;
  return nullptr;
}

bool ParseBlockCharacter(const std::string& text, BlockCharacter* character) {
  if (text == "auto") *character = BlockCharacter::Auto;
  else if (text == "mixed") *character = BlockCharacter::Mixed;
  else if (text == "downtown") *character = BlockCharacter::Downtown;
  else if (text == "residential") *character = BlockCharacter::Residential;
  else if (text == "industrial") *character = BlockCharacter::Industrial;
  else return false;
  return true;
}

std::vector<UrbanParcel> UrbanParcels(uint32_t seed, const MapGeneratorConfig& c) {
  if (c.arteryCount <= 0) return {};
  Rng rng(seed);  // Mirrors GenerateArterialUrbanMap: the parcel pass draws first.
  const CityPlan city = BuildCityPlan(seed, c);
  std::vector<float> characterRolls;
  std::vector<UrbanParcel> out;
  for (const Parcel& parcel : PlanParcels(city, c, &rng, &characterRolls)) {
    out.push_back(ToPublicParcel(parcel));
  }
  return out;
}

bool ParseFeatureMode(const std::string& text, FeatureMode* mode) {
  if (text == "auto") *mode = FeatureMode::Auto;
  else if (text == "off") *mode = FeatureMode::Off;
  else if (text == "on") *mode = FeatureMode::On;
  else return false;
  return true;
}

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

std::vector<UrbanStreet> UrbanStreets(uint32_t seed, const MapGeneratorConfig& c) {
  std::vector<UrbanStreet> out;
  if (c.arteryCount <= 0) return out;
  const HighwayPlan plan = BuildHighwayPlan(seed, c);
  const StreetGraph graph = BuildStreetGraph(seed, c, plan);
  for (const UrbanRoad& road : graph.roads) {
    for (size_t i = 0; i + 1 < road.centerline.size(); ++i) {
      out.push_back({road.centerline[i], road.centerline[i + 1], road.width,
                     UrbanStreetClass::Highway, road.elevated});
    }
  }
  const float half = UrbanMapHalfExtent(c);
  for (const StreetCut& cut : graph.cuts) {
    if (cut.cls == UrbanStreetClass::Highway) continue;  // Already a road above.
    float t0, t1;
    ClipLineToSquare(cut.origin, cut.dir, half, &t0, &t1);
    if (!(t1 > t0)) continue;
    const glm::vec2 a = cut.origin + cut.dir * t0, b = cut.origin + cut.dir * t1;
    out.push_back({glm::vec3(a.x, 0.0f, a.y), glm::vec3(b.x, 0.0f, b.y), cut.width, cut.cls, false});
  }
  return out;
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
  const GroundY groundY = [](float, float) { return 0.0f; };
  if (c.arteryCount > 0) return GenerateArterialUrbanMap(seed, c, groundY);
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
      if (building) {
        AddBuilding(&scene, xl, zl, xr, zr, h, groundY);
      } else {
        const float baseY = groundY(0.5f * (xl + xr), 0.5f * (zl + zr));
        scene.sidewalks.push_back(
            AABB{glm::vec3(xl, baseY, zl), glm::vec3(xr, baseY + kSidewalkHeight, zr)});
      }
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
    AABB& bounds = scene.obstacles[i].bounds;
    bounds.max.y = bounds.min.y + rng.Float(c.towerMinHeight, c.towerMaxHeight);
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
    blue.position = glm::vec3(-spawnX, groundY(-spawnX, rows[i]), rows[i]);
    blue.facingYaw = 0.0f;
    blue.weapon = DefaultWeaponForUnit(blue.id);
    scene.units.push_back(blue);
  }
  for (int i = 0; i < 3; ++i) {
    Unit red;
    red.id = 3 + i;
    red.team = Team::Red;
    red.position = glm::vec3(spawnX, groundY(spawnX, rows[i]), rows[i]);
    red.facingYaw = kPi;
    red.weapon = DefaultWeaponForUnit(red.id);
    scene.units.push_back(red);
  }
  PlaceZiplines(&scene, UrbanBlocks(seed, c), seed, c);
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
