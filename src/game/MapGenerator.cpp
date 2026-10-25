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
//   0xa2233e18u   terrain presence on/off draw
//   0x7e22ac31u   terrain levels   level-step lines, raised sides, rises
//   0x1eae1007u   terrain accesses avenue ramp-throughs and stair placement
//   0x21b1e5a7u   ziplines         roof/anchor selection
constexpr uint32_t kStreetStream = 0xb10c5eedu;

constexpr float kPi = 3.14159265358979323846f;
constexpr float kSidewalkHeight = 0.12f;  // Curb: purely visual, well under a step.

// Every urban scene emitter places its output relative to this sampled base.
// Sampling at emission keeps buildings, pavement and spawns on the field.
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

// Whether this (seed, config) generates terrain levels. Presence has its
// own stream so forcing terrain never reshuffles the rest of a layout.
bool TerrainEnabled(uint32_t seed, const MapGeneratorConfig& c) {
  if (c.arteryCount <= 0 || c.terrainAmplitude <= 0.0f) return false;
  const FeatureToggle& toggle = c.features.terrain;
  if (toggle.mode == FeatureMode::Off) return false;
  if (toggle.mode == FeatureMode::On) return true;
  Rng presence(seed ^ 0xa2233e18u);
  return presence.Chance(toggle.chance);
}

// Presence and layout draws (issue #130) use their own stream so forcing
// either never reshuffles the rest of a seed's layout; both are drawn
// unconditionally for the same reason.
OverpassChoice ResolveOverpass(uint32_t seed, const MapGeneratorConfig& c) {
  Rng rng(seed ^ 0x0ebe7a55u);
  // Terrain levels invite flyovers: a terrain-enabled seed draws presence
  // with a boosted chance. Still one draw, so terrain on/off never shifts
  // the layout draw that follows.
  const float chance = TerrainEnabled(seed, c) ? 1.0f - (1.0f - c.overpassChance) * 0.5f
                                               : c.overpassChance;
  const bool drawnPresence = rng.Chance(chance);
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

// --- Map-spanning terrain levels (issue #155 redo). ---
//
// A terrain-enabled seed raises whole portions of the map behind retaining
// walls. Each level step is a straight line crossing the entire map,
// roughly perpendicular to the artery, laid along its own boundary street:
// the wall slides parallel to that street and blocks never straddle it.
// Steps stack where raised sides overlap, nesting tiers at different
// heights. The artery and branch either ramp through a gap in a wall they
// meet at grade -- a main road cutting through the level -- or, when
// elevated there, fly over the wall and settle onto the level beyond.

constexpr uint32_t kTerrainStream = 0x7e22ac31u;
constexpr float kRampGrade = 0.30f;    // Max rise/run of a road/walk ramp.
constexpr float kStairGrade = 0.80f;   // Steeper flight reserved for stairs.
constexpr float kWallHalfWidth = 1.1f;
constexpr float kStepBlend = 0.25f;    // Cliff feather of a level step.
constexpr float kWallSidewalk = 4.2f;  // Walkable strip between a wall and its street's pavement.

// A ramped opening through one wall (an avenue crossing or a through-wall
// stair): inside the across-band around `foot + t * along`, the wall's step
// becomes a linear climb over `length` instead of a sharp cliff.
struct TerrainChannel {
  glm::vec2 foot{0.0f}, along{0.0f, 1.0f};  // `along` points at the raised side.
  float width = 0.0f, length = 1.0f;
  bool stair = false;  // Narrow stair channel (gets its own side walls).
};

struct TerrainWallSpec {
  glm::vec2 origin{0.0f}, dir{1.0f, 0.0f};  // Boundary street centerline.
  glm::vec2 raisedN{0.0f, 1.0f};            // Unit normal toward the raised side.
  float stepD = 0.0f;                       // Step line: dot(raisedN, p) >= stepD is raised.
  float rise = 0.0f;
  float cutWidth = 0.0f, paveWidth = 0.0f;  // Boundary street carve/pavement widths.
  float arteryS = 0.0f;                     // Artery param of the street crossing.
  bool arteryRamp = false;                  // Artery ramps through (vs deck flying over).
  bool branchRamp = false;                  // Same for the branch, when it crosses.
  float branchU = -1000.0f;                 // Branch param of the crossing, if any.
  std::vector<TerrainChannel> channels;     // Filled by the terrain emitter.
  std::vector<std::pair<float, float>> gaps;  // Wall-line param intervals left open.
};

struct TerrainPlan {
  bool enabled = false;
  float amplitude = 0.0f, wavelength = 1.0f;
  float deckCruise = 0.0f;  // Absolute deck height at full overpass profile.
  std::vector<TerrainWallSpec> walls;
};

// Level height from the steps alone (sharp cliff edges), before noise,
// block pads, road ramps and access channels.
float TierHeight(const TerrainPlan& terrain, glm::vec2 p) {
  float y = 0.0f;
  for (const TerrainWallSpec& w : terrain.walls) {
    const float d = glm::dot(w.raisedN, p) - w.stepD;
    y += w.rise * SmoothStep01((d + kStepBlend) / kStepBlend);
  }
  return y;
}

// Ground grade under a highway road at point `p`: a wall crossed at grade
// becomes a kRampGrade ramp climbing into its raised side along the road,
// while a wall the deck flies over keeps its sharp step (the wall itself
// continues beneath the deck).
float HighwayGradeAt(const TerrainPlan& terrain, glm::vec2 p, glm::vec2 roadDir, bool branchRoad) {
  float y = 0.0f;
  for (const TerrainWallSpec& w : terrain.walls) {
    const float d = glm::dot(w.raisedN, p) - w.stepD;
    if (branchRoad ? w.branchRamp : w.arteryRamp) {
      const float along = std::max(0.25f, std::fabs(glm::dot(w.raisedN, roadDir)));
      y += w.rise * std::clamp(d / (along * (w.rise / kRampGrade)), 0.0f, 1.0f);
    } else {
      y += w.rise * SmoothStep01((d + kStepBlend) / kStepBlend);
    }
  }
  return y;
}

TerrainPlan BuildTerrainPlan(uint32_t seed, const MapGeneratorConfig& c, const HighwayPlan& plan) {
  TerrainPlan terrain;
  if (!TerrainEnabled(seed, c)) return terrain;
  terrain.enabled = true;
  terrain.amplitude = c.terrainAmplitude;
  terrain.wavelength = std::max(32.0f, c.blockSize * 2.5f);
  terrain.deckCruise = c.highwayElevation;
  Rng rng(seed ^ kTerrainStream);
  const ArterySpec& artery = plan.artery;
  const float length = artery.sMax - artery.sMin;
  const int drawn = 1 + rng.Int(0, 2);
  const int wanted = c.terrainLevelCount < 0 ? drawn : c.terrainLevelCount;

  const auto profile01 = [&](float s) {
    return SmoothStep01((s - artery.rise0) / (artery.rise1 - artery.rise0)) -
           SmoothStep01((s - artery.fall0) / (artery.fall1 - artery.fall0));
  };
  const auto profileWithin = [&](float sA, float sB, float lo, float hi) {
    for (int i = 0; i <= 8; ++i) {
      const float p = profile01(glm::mix(sA, sB, i / 8.0f));
      if (p < lo || p > hi) return false;
    }
    return true;
  };

  for (int i = 0; i < wanted; ++i) {
    bool placed = false;
    for (int attempt = 0; attempt < 16 && !placed; ++attempt) {
      TerrainWallSpec w;
      const float sCross = artery.sMin + length * rng.Float(0.20f, 0.80f);
      w.dir = Rotate(artery.normal, rng.Float(-0.18f, 0.18f));
      w.origin = artery.origin + artery.dir * sCross;
      const float side = rng.Chance(0.5f) ? 1.0f : -1.0f;
      w.raisedN = glm::vec2(-w.dir.y, w.dir.x) * side;
      w.rise = constants::kUnitHeight * rng.Float(1.0f, i == 0 ? 5.0f : 2.5f);
      w.paveWidth = c.localStreetWidth * 1.15f;
      w.cutWidth = w.paveWidth + 2.0f * kWallSidewalk;
      w.stepD = glm::dot(w.raisedN, w.origin) + w.cutWidth * 0.5f;
      w.arteryS = sCross;

      // Where the step line meets the artery, and how far its road ramp
      // would run (toward the raised side) along it.
      const float alongA = glm::dot(w.raisedN, artery.dir);
      const float sStep = (w.stepD - glm::dot(w.raisedN, artery.origin)) / alongA;
      const float runA = w.rise / kRampGrade / std::fabs(alongA);
      const float sA = std::min(sStep, sStep + runA * (alongA > 0.0f ? 1.0f : -1.0f));
      const float sB = std::max(sStep, sStep + runA * (alongA > 0.0f ? 1.0f : -1.0f));

      // The artery must meet the wall honestly: fully at grade (it ramps
      // through a gap) or fully on the deck (it flies over).
      const bool flying = plan.elevated && profileWithin(sStep - 10.0f, sStep + 10.0f, 0.98f, 1.0f);
      w.arteryRamp = !plan.elevated || profileWithin(sA - 9.0f, sB + 9.0f, 0.0f, 0.02f);
      if (w.arteryRamp) {
        if (sA - 12.0f < artery.sMin + 4.0f || sB + 12.0f > artery.sMax - 4.0f) continue;
      } else if (flying) {
        w.arteryRamp = false;
        if (sStep - 8.0f < artery.sMin + 4.0f || sStep + 8.0f > artery.sMax - 4.0f) continue;
      } else {
        continue;  // Crossing lands on a deck ramp: no honest meeting.
      }

      // Keep level steps apart along the artery: between two ramped walls
      // the corridor needs a flat window at the local tier, so ground can
      // step on and off the road between the climbs.
      bool separated = true;
      for (const TerrainWallSpec& other : terrain.walls) {
        const float needed = 14.0f + (w.arteryRamp ? runA : 2.0f) +
                             (other.arteryRamp ? other.rise / kRampGrade * 1.1f : 2.0f);
        if (std::fabs(sCross - other.arteryS) < std::max(30.0f, needed)) separated = false;
      }
      if (!separated) continue;

      if (plan.hasBranch) {
        const BranchSpec& branch = plan.branch;
        const float denom = glm::dot(w.raisedN, branch.dir);
        // A step line nearly parallel to the branch must keep clear of its
        // corridor instead of slicing along it.
        if (std::fabs(denom) < 0.25f) {
          bool nearBranch = false;
          for (float f : {0.25f, 0.5f, 0.75f}) {
            const glm::vec2 q = branch.start + branch.dir * (branch.length * f);
            if (std::fabs(glm::dot(w.raisedN, q) - w.stepD) < branch.width * 0.5f + 6.0f) {
              nearBranch = true;
            }
          }
          if (nearBranch) continue;
        } else {
          const float u = (w.stepD - glm::dot(w.raisedN, branch.start)) / denom;
          if (u > -8.0f && u < branch.length + 8.0f) {
            const float runB = w.rise / kRampGrade / std::fabs(denom);
            const float uA = std::min(u, u + runB * (denom > 0.0f ? 1.0f : -1.0f));
            const float uB = std::max(u, u + runB * (denom > 0.0f ? 1.0f : -1.0f));
            // At-grade zone of the branch: everything up to its on-ramp (the
            // whole branch when the artery stays at grade).
            const float gradeEnd = plan.elevated ? branch.riseStart : branch.length;
            if (!plan.elevated || (!branch.offMap && uB + 8.0f < gradeEnd - 2.0f && uA > 4.0f)) {
              w.branchRamp = true;
              w.branchU = u;
              if (uA - 6.0f < 2.0f || uB + 6.0f > gradeEnd - 2.0f) continue;
              // Two climbs on the branch need a flat window between them.
              bool apart = true;
              for (const TerrainWallSpec& other : terrain.walls) {
                if (!other.branchRamp) continue;
                if (std::fabs(u - other.branchU) <
                    runB + other.rise / kRampGrade * 1.1f + 14.0f) {
                  apart = false;
                }
              }
              if (!apart) continue;
            } else if (branch.offMap || u - 8.0f > branch.riseEnd + 2.0f) {
              w.branchRamp = false;  // The branch deck flies over this wall.
            } else {
              continue;  // Crossing lands on the branch's on-ramp.
            }
          }
        }
      }
      terrain.walls.push_back(std::move(w));
      placed = true;
    }
    if (!placed) break;
  }

  // Cruise height of an elevated deck: clear the highest tier it crosses at
  // full profile by the configured elevation.
  if (plan.elevated) {
    float maxTier = 0.0f;
    const float s0 = std::max(artery.s0, artery.rise1 - 2.0f);
    const float s1 = std::min(artery.s1, artery.fall0 + 2.0f);
    for (float s = s0; s <= s1; s += 2.0f) {
      const glm::vec2 p = artery.origin + artery.dir * s;
      for (const float lat : {-0.6f, 0.0f, 0.6f}) {
        maxTier = std::max(maxTier,
                           TierHeight(terrain, p + artery.normal * (lat * c.arteryWidth)));
      }
    }
    terrain.deckCruise = maxTier + c.highwayElevation;
  }
  return terrain;
}

// Overpass profile: between its rise and fall ramps the deck cruises flat
// at deckCruise (clear of every tier beneath); outside them it follows the
// local road grade, settling onto whatever level the artery ends in.
// Heights are relative lifts over the ground directly below (RibbonSide
// adds the terrain back), so at-grade stretches drape over the grade the
// ground keeps for them.
float ArteryElevationAt(const HighwayPlan& plan, const TerrainPlan& terrain,
                        const MapGeneratorConfig& c, float s) {
  if (!plan.elevated) return kGradeY;
  const ArterySpec& a = plan.artery;
  const float profile = SmoothStep01((s - a.rise0) / (a.rise1 - a.rise0)) -
                        SmoothStep01((s - a.fall0) / (a.fall1 - a.fall0));
  const float cruise = terrain.enabled ? terrain.deckCruise : c.highwayElevation;
  const float grade = HighwayGradeAt(terrain, a.origin + a.dir * s, a.dir, false);
  return kGradeY + (cruise - grade - kGradeY) * profile;
}

float BranchElevationAt(const HighwayPlan& plan, const TerrainPlan& terrain,
                        const MapGeneratorConfig& c, float u) {
  if (!plan.elevated) return kGradeY;
  // The on-ramp tops out at the deck's world height over the merge point
  // (grades there agree: level steps keep clear of the junction).
  const glm::vec2 merge = plan.artery.origin + plan.artery.dir * plan.branch.mergeS;
  const float topWorld = ArteryElevationAt(plan, terrain, c, plan.branch.mergeS) +
                         HighwayGradeAt(terrain, merge, plan.artery.dir, false);
  const glm::vec2 p = plan.branch.start + plan.branch.dir * u;
  const float grade = HighwayGradeAt(terrain, p, plan.branch.dir, true);
  if (plan.branch.offMap) return topWorld - grade;
  const float run = std::max(1.0f, plan.branch.riseEnd - plan.branch.riseStart);
  return kGradeY +
         (topWorld - grade - kGradeY) * SmoothStep01((u - plan.branch.riseStart) / run);
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
  const TerrainPlan terrain = BuildTerrainPlan(seed, c, plan);
  std::vector<UrbanRoad> roads;
  UrbanRoad artery;
  artery.width = c.arteryWidth;
  artery.artery = true;
  artery.elevated = plan.elevated;
  for (const float s : SampleParams(plan.artery.s0, plan.artery.s1)) {
    const glm::vec2 p = plan.artery.origin + plan.artery.dir * s;
    artery.centerline.emplace_back(p.x, ArteryElevationAt(plan, terrain, c, s), p.y);
  }
  roads.push_back(std::move(artery));
  if (plan.hasBranch) {
    UrbanRoad branch;
    branch.width = plan.branch.width;
    branch.artery = true;
    branch.elevated = plan.elevated;
    for (const float u : SampleParams(plan.branch.u0, plan.branch.length)) {
      const glm::vec2 p = plan.branch.start + plan.branch.dir * u;
      branch.centerline.emplace_back(p.x, BranchElevationAt(plan, terrain, c, u), p.y);
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
// exactly the cuts the street pass chose. Terrain boundary streets carve
// wider than they pave: the margins hold the retaining wall and a wall-side
// walk, and both artery sides get the same line (one entry per side).
struct StreetCut {
  glm::vec2 origin, dir;
  float width;
  int side;
  UrbanStreetClass cls;
  float pave = 0.0f;  // Pavement width; 0 means the full carve width.
  bool terrainBoundary = false;
};

// Every street of the city: the highway roads plus the local cuts.
struct StreetGraph {
  std::vector<UrbanRoad> roads;
  std::vector<StreetCut> cuts;
};

// Everything the passes share. Built front to back: HighwayPlan ->
// TerrainPlan -> StreetGraph -> Blocks; Parcels and Scene emission consume
// it. The terrain emitter later fills the plan's channels/gaps in place.
struct CityPlan {
  HighwayPlan highway;
  TerrainPlan terrain;
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
StreetGraph BuildStreetGraph(uint32_t seed, const MapGeneratorConfig& c, const HighwayPlan& plan,
                             const TerrainPlan& terrain) {
  const ArterySpec& artery = plan.artery;
  Rng rng(seed ^ kStreetStream);
  const float half = UrbanMapHalfExtent(c);
  const float band = ArteryBand(c);
  StreetGraph graph;
  graph.roads = BuildUrbanRoads(seed, c);
  // Terrain boundary streets first: one full map-spanning line per level
  // step, carved on both artery sides so no block ever straddles a wall.
  for (const TerrainWallSpec& w : terrain.walls) {
    for (const int side : {1, -1}) {
      graph.cuts.push_back({w.origin, w.dir, w.cutWidth, side, UrbanStreetClass::Cross,
                            w.paveWidth, /*terrainBoundary=*/true});
    }
  }
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
  plan.terrain = BuildTerrainPlan(seed, c, plan.highway);
  plan.streets = BuildStreetGraph(seed, c, plan.highway, plan.terrain);
  plan.blocks = BuildBlocks(c, plan.highway, plan.streets);
  return plan;
}

std::vector<UrbanBlock> BuildCityBlocks(uint32_t seed, const MapGeneratorConfig& c) {
  const HighwayPlan highway = BuildHighwayPlan(seed, c);
  const TerrainPlan terrain = BuildTerrainPlan(seed, c, highway);
  return BuildBlocks(c, highway, BuildStreetGraph(seed, c, highway, terrain));
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

// A buildable lot: footprint plus height, before it becomes an Obstacle.
struct Parcel {
  std::vector<glm::vec2> footprint;
  float height;
};

// Parcel pass output for one block.
struct BlockParcels {
  std::vector<glm::vec2> sidewalk;  // The block outline, paved.
  std::vector<Parcel> parcels;
};

void AddPolygonBuilding(Scene* scene, std::vector<glm::vec2> footprint, float height,
                        const GroundY& groundY) {
  if (footprint.size() < 3 || PolygonArea(footprint) < 1.0f) return;
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
  const float baseY = groundY(center.x, center.y);
  bounds.min.y = baseY;
  bounds.max.y = baseY + height;
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
BlockParcels PlanBlockParcels(const UrbanBlock& block, size_t blockIndex,
                              const std::vector<UrbanRoad>& arteries, const MapGeneratorConfig& c,
                              Rng* rng) {
  BlockParcels out;
  out.sidewalk = block.vertices;
  const bool park = rng->Chance(0.10f);  // Pocket park/open lot.
  const float cluster = rng->Float(c.minBuildingHeight, c.maxBuildingHeight);
  if (park) return out;
  std::vector<glm::vec2> outer = InsetConvexPolygon(block.vertices, c.sidewalkWidth);
  if (outer.size() < 3 || PolygonArea(outer) < c.minPolygonArea ||
      !PolygonClearOfRoads(outer, arteries, c.sidewalkWidth)) {
    return out;
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
      out.parcels.push_back({std::move(slab),
                         std::clamp(cluster + rng->Float(-1.2f, 1.2f), c.minBuildingHeight,
                                    c.maxBuildingHeight)});
    }
    return out;
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
      out.parcels.push_back({std::move(footprint),
                         std::clamp(cluster + rng->Float(-1.2f, 1.2f), c.minBuildingHeight,
                                    c.maxBuildingHeight)});
    }
  }
  return out;
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
                   const MapGeneratorConfig& c, size_t roofCount = std::numeric_limits<size_t>::max()) {
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
    if (!scene->ground.Empty()) {
      for (int s = 0; s <= steps; ++s) {
        const glm::vec3 p = glm::mix(top0, top1, static_cast<float>(s) / steps);
        if (p.y < scene->ground.MeshHeightAt(p.x, p.z) + kCableClearance) return false;
      }
    }
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
    for (size_t i = 0; i < std::min(roofCount, footprints.size()); ++i) {
      glm::vec2 centre(0.0f);
      for (const glm::vec2& v : footprints[i]) centre += v;
      centre /= static_cast<float>(footprints[i].size());
      if (scene->obstacles[i].bounds.max.y >= kMinRoofHeight && PointInConvexPolygon(centre, poly)) {
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

float ValueNoise(uint32_t seed, float x, float z);

float InsideDistance(glm::vec2 p, const std::vector<glm::vec2>& polygon) {
  float distance = std::numeric_limits<float>::infinity();
  for (size_t i = 0; i < polygon.size(); ++i) {
    const glm::vec2 a = polygon[i], edge = polygon[(i + 1) % polygon.size()] - a;
    const float length = glm::length(edge);
    if (length > 1e-4f) {
      distance = std::min(distance, (edge.x * (p.y - a.y) - edge.y * (p.x - a.x)) / length);
    }
  }
  return distance;
}

glm::vec2 PolygonCenter(const std::vector<glm::vec2>& polygon) {
  glm::vec2 center(0.0f);
  for (glm::vec2 p : polygon) center += p;
  return center / static_cast<float>(polygon.size());
}

std::vector<glm::vec2> AccessFootprint(const TerrainAccess& access, float margin = 0.0f) {
  const glm::vec2 a(access.bottom.x, access.bottom.z), b(access.top.x, access.top.z);
  const glm::vec2 along = glm::normalize(b - a);
  const glm::vec2 across(-along.y, along.x);
  const glm::vec2 w = across * (access.width * 0.5f + margin);
  return {a - along * margin - w, b + along * margin - w,
          b + along * margin + w, a - along * margin + w};
}

bool PolygonsOverlap(const std::vector<glm::vec2>& a, const std::vector<glm::vec2>& b) {
  auto intersection = a;
  for (size_t i = 0; i < b.size(); ++i) {
    const glm::vec2 edge = b[(i + 1) % b.size()] - b[i];
    const glm::vec2 n(-edge.y, edge.x);
    intersection = KeepSide(intersection, n, glm::dot(n, b[i]));
  }
  return intersection.size() >= 3 && PolygonArea(intersection) > 1e-4f;
}

// Merge blocked param intervals and return the free runs of [lo, hi].
std::vector<std::pair<float, float>> FreeIntervals(float lo, float hi,
                                                   std::vector<std::pair<float, float>> blocked) {
  std::sort(blocked.begin(), blocked.end());
  std::vector<std::pair<float, float>> free;
  float cursor = lo;
  for (const auto& b : blocked) {
    if (b.second <= cursor) continue;
    if (b.first > cursor) free.emplace_back(cursor, std::min(b.first, hi));
    cursor = std::max(cursor, b.second);
    if (cursor >= hi) break;
  }
  if (cursor < hi) free.emplace_back(cursor, hi);
  return free;
}

// Fills the ground heightfield and Scene::terrainWalls, and completes the
// terrain plan in place: avenue ramp-throughs, stair channels and the wall
// gaps they cut. Levels come from the plan's map-spanning steps; the ground
// under the highway corridors keeps the road's own grade (ramping through a
// wall met at grade, stepping sharply under a deck that flies over).
void BuildUrbanTerrain(Scene* scene, CityPlan* cityPlan, uint32_t seed,
                       const MapGeneratorConfig& c) {
  CityPlan& city = *cityPlan;
  TerrainPlan& terrain = city.terrain;
  if (!terrain.enabled) return;
  const HighwayPlan& plan = city.highway;
  const ArterySpec& artery = plan.artery;
  const float half = scene->mapHalfExtent;
  // Accesses draw from their own stream: the level steps never reshuffle.
  Rng rng(seed ^ 0x1eae1007u);

  // Rolling noise rides on the tiers; it fades out near the highway roads,
  // whose ground must keep an exact, steady grade.
  const auto noise = [&](glm::vec2 p) {
    float y = terrain.amplitude * 0.35f *
              ValueNoise(seed ^ kTerrainStream, p.x / terrain.wavelength, p.y / terrain.wavelength);
    float clearance = std::numeric_limits<float>::infinity();
    for (const UrbanRoad& road : city.streets.roads) {
      for (size_t i = 1; i < road.centerline.size(); ++i) {
        const auto& a = road.centerline[i - 1];
        const auto& b = road.centerline[i];
        const float d = PointSegmentDistance(p, {a.x, a.z}, {b.x, b.z});
        clearance = std::min(clearance, d - road.width * 0.5f);
      }
    }
    return y * SmoothStep01((clearance - 2.0f) / 6.0f);
  };
  std::vector<float> padHeights;
  for (const UrbanBlock& block : city.blocks) {
    padHeights.push_back(noise(PolygonCenter(block.vertices)));
  }

  for (TerrainWallSpec& w : terrain.walls) {
    TerrainWall wall;
    wall.origin = w.origin;
    wall.dir = w.dir;
    wall.raisedNormal = w.raisedN;
    wall.rise = w.rise;
    const glm::vec2 lineOrigin =
        w.origin + w.raisedN * (w.stepD - glm::dot(w.raisedN, w.origin));
    const auto lineAt = [&](float t) { return lineOrigin + w.dir * t; };
    float tMin = 0.0f, tMax = 0.0f;
    ClipLineToSquare(lineOrigin, w.dir, half, &tMin, &tMax);

    // Param interval of the step line inside a band |dot(n, p) - d0| < hw;
    // an inverted pair means the line never crosses it.
    const auto bandInterval = [&](glm::vec2 n, float d0, float hw) {
      const float slope = glm::dot(n, w.dir);
      if (std::fabs(slope) < 0.15f) return std::make_pair(1.0f, -1.0f);
      const float tc = (d0 - glm::dot(n, lineOrigin)) / slope;
      const float ht = hw / std::fabs(slope);
      return std::make_pair(tc - ht, tc + ht);
    };

    // Stretches of the wall line where nothing may be placed: the rim
    // streets, the highway corridors and every street crossing.
    std::vector<std::pair<float, float>> blocked;
    const float rim = RimWidth(c) + 2.0f;
    blocked.emplace_back(tMin - 1.0f, tMin + rim);
    blocked.emplace_back(tMax - rim, tMax + 1.0f);
    const auto arteryBand = bandInterval(artery.normal, glm::dot(artery.normal, artery.origin),
                                         c.arteryWidth * 0.5f + 6.0f);
    blocked.push_back(arteryBand);
    const glm::vec2 branchN(-plan.branch.dir.y, plan.branch.dir.x);
    if (plan.hasBranch) {
      const auto band = bandInterval(branchN, glm::dot(branchN, plan.branch.start),
                                     plan.branch.width * 0.5f + c.localStreetWidth * 0.4f + 5.0f);
      if (band.first < band.second) {
        const glm::vec2 q = lineAt(0.5f * (band.first + band.second));
        const float u = glm::dot(q - plan.branch.start, plan.branch.dir);
        if (u > -8.0f && u < plan.branch.length + 8.0f) blocked.push_back(band);
      }
    }
    for (const StreetCut& cut : city.streets.cuts) {
      const glm::vec2 n(-cut.dir.y, cut.dir.x);
      const auto band = bandInterval(n, glm::dot(n, cut.origin), cut.width * 0.5f + 3.0f);
      if (band.first < band.second) blocked.push_back(band);
    }

    // Wall gaps where a road ramps through at grade.
    if (w.arteryRamp) {
      w.gaps.push_back(bandInterval(artery.normal, glm::dot(artery.normal, artery.origin),
                                    c.arteryWidth * 0.5f + 2.7f));
    }
    if (plan.hasBranch && w.branchRamp) {
      w.gaps.push_back(bandInterval(branchN, glm::dot(branchN, plan.branch.start),
                                    plan.branch.width * 0.5f + c.localStreetWidth * 0.4f + 1.1f));
    }

    // One avenue per artery side may ramp through this wall: a wide gap
    // whose street climbs the level at road grade.
    for (const int side : {1, -1}) {
      const bool wanted = rng.Chance(0.6f);
      if (!wanted) continue;
      for (const StreetCut& cut : city.streets.cuts) {
        if (cut.side != side || cut.cls == UrbanStreetClass::Highway || cut.terrainBoundary) {
          continue;
        }
        const float denom = glm::dot(w.raisedN, cut.dir);
        if (std::fabs(denom) < 0.55f) continue;  // Too oblique for a straight climb.
        const float dLine = glm::dot(w.raisedN, cut.origin) - w.stepD;
        const glm::vec2 q = cut.origin + cut.dir * (-dLine / denom);
        const float run = w.rise / kRampGrade / std::fabs(denom);
        const glm::vec2 along = cut.dir * (denom > 0.0f ? 1.0f : -1.0f);
        bool fits = true;
        for (const glm::vec2 e : {q - along * 6.0f, q, q + along * run, q + along * (run + 6.0f)}) {
          if (std::max(std::fabs(e.x), std::fabs(e.y)) > half - RimWidth(c) - 2.0f) fits = false;
          // The whole climb stays on this cut's side of the artery: the
          // avenue only exists as a street there.
          if (static_cast<float>(side) * glm::dot(e - artery.origin, artery.normal) <
              ArteryBand(c) + 1.0f) {
            fits = false;
          }
          if (plan.hasBranch) {
            const float u = glm::dot(e - plan.branch.start, plan.branch.dir);
            if (u > -4.0f && u < plan.branch.length + 4.0f &&
                std::fabs(glm::dot(e - plan.branch.start, branchN)) <
                    plan.branch.width * 0.5f + c.localStreetWidth * 0.4f + 4.0f) {
              fits = false;
            }
          }
          for (const TerrainWallSpec& other : terrain.walls) {
            if (&other == &w) continue;
            if (std::fabs(glm::dot(other.raisedN, e) - other.stepD) < 6.0f) fits = false;
          }
        }
        if (!fits) continue;
        w.channels.push_back(TerrainChannel{q, along, cut.width, run, /*stair=*/false});
        const glm::vec2 n(-cut.dir.y, cut.dir.x);
        const auto gap = bandInterval(n, glm::dot(n, cut.origin), cut.width * 0.5f + 0.6f);
        w.gaps.push_back(gap);
        blocked.emplace_back(gap.first - 4.0f, gap.second + 4.0f);
        break;
      }
    }

    // Sections of the wall line between the splitters that fragment the
    // raised level: the highway corridors, ramped avenues (their flank
    // walls seal the climb's sides) and the other walls' boundary streets.
    // Each gets its own ground climb below, so no fragment strands.
    std::vector<std::pair<float, float>> splitters;
    splitters.emplace_back(tMin - 1.0f, tMin + rim);
    splitters.emplace_back(tMax - rim, tMax + 1.0f);
    splitters.emplace_back(arteryBand.first - 2.0f, arteryBand.second + 2.0f);
    if (plan.hasBranch) {
      const auto band = bandInterval(branchN, glm::dot(branchN, plan.branch.start),
                                     plan.branch.width * 0.5f + c.localStreetWidth * 0.4f + 6.0f);
      if (band.first < band.second) {
        const glm::vec2 q = lineAt(0.5f * (band.first + band.second));
        const float u = glm::dot(q - plan.branch.start, plan.branch.dir);
        if (u > -8.0f && u < plan.branch.length + 8.0f) splitters.push_back(band);
      }
    }
    for (const auto& gap : w.gaps) splitters.emplace_back(gap.first - 2.0f, gap.second + 2.0f);
    for (const TerrainWallSpec& other : terrain.walls) {
      if (&other == &w) continue;
      const glm::vec2 n(-other.dir.y, other.dir.x);
      const auto band = bandInterval(n, glm::dot(n, other.origin), other.cutWidth * 0.5f + 3.0f);
      if (band.first < band.second) splitters.push_back(band);
    }
    const auto sections = FreeIntervals(tMin, tMax, splitters);

    // Stairs on the free stretches between streets: flights attached along
    // the wall face (in the wall-side walk) and walled through-wall channels
    // landing on the level. At least one climb per artery side.
    const float stairRun = w.rise / kStairGrade;
    const float flightMid = -(kWallHalfWidth * 2.0f + 0.9f);  // Strip center beside the wall box.
    // A point an access rests on must be usable ground: on the map, outside
    // the highway corridors, clear of every other wall's step and of every
    // channel (whose climb and flank walls own that ground).
    const auto usableSpot = [&](glm::vec2 e) {
      if (std::max(std::fabs(e.x), std::fabs(e.y)) > half - RimWidth(c) - 1.5f) return false;
      if (std::fabs(glm::dot(e - artery.origin, artery.normal)) < c.arteryWidth * 0.5f + 3.0f) {
        return false;
      }
      if (plan.hasBranch) {
        const float u = glm::dot(e - plan.branch.start, plan.branch.dir);
        if (u > -4.0f && u < plan.branch.length + 4.0f &&
            std::fabs(glm::dot(e - plan.branch.start, branchN)) <
                plan.branch.width * 0.5f + c.localStreetWidth * 0.4f + 2.0f) {
          return false;
        }
      }
      for (const TerrainWallSpec& other : terrain.walls) {
        if (&other != &w &&
            std::fabs(glm::dot(other.raisedN, e) - other.stepD) < 3.5f) {
          return false;
        }
        for (const TerrainChannel& channel : other.channels) {
          const glm::vec2 rel = e - channel.foot;
          const float a = glm::dot(rel, channel.along);
          const float b = std::fabs(rel.x * channel.along.y - rel.y * channel.along.x);
          if (b < channel.width * 0.5f + 3.0f && a > -4.0f && a < channel.length + 4.0f) {
            return false;
          }
        }
      }
      return true;
    };
    // Stretches of the line already taken by an access, so two never collide.
    std::vector<std::pair<float, float>> used;
    const auto free = [&](float lo, float hi) {
      for (const auto& u : used) {
        if (lo < u.second && hi > u.first) return false;
      }
      return true;
    };
    const auto placeAlong = [&](float tFoot) {
      if (!free(tFoot - 2.0f, tFoot + stairRun + 4.0f)) return false;
      const glm::vec2 foot = lineAt(tFoot) + w.raisedN * flightMid;
      const glm::vec2 top = lineAt(tFoot + stairRun + 1.1f) + w.raisedN * 1.8f;
      if (!usableSpot(foot) || !usableSpot(top)) return false;
      used.emplace_back(tFoot - 2.0f, tFoot + stairRun + 4.0f);
      TerrainAccess access;
      access.alongWall = true;
      access.width = 1.6f;
      access.bottom = glm::vec3(foot.x, 0.0f, foot.y);
      access.top = glm::vec3(top.x, 0.0f, top.y);
      wall.accesses.push_back(access);
      return true;
    };
    std::vector<float> groundSpots;  // Line params of the through-wall stairs.
    const auto placeThrough = [&](float tMid) {
      if (!free(tMid - 6.0f, tMid + 6.0f)) return false;
      const float setback = 2.0f;
      const glm::vec2 foot = lineAt(tMid) - w.raisedN * setback;
      const glm::vec2 top = lineAt(tMid) + w.raisedN * stairRun;
      // The foot, the landing and a margin past it must be usable ground
      // (a landing inside a block clears its own parcels later).
      for (const glm::vec2 e : {foot, top, lineAt(tMid) + w.raisedN * (stairRun + 2.5f)}) {
        if (!usableSpot(e)) return false;
      }
      used.emplace_back(tMid - 6.0f, tMid + 6.0f);
      groundSpots.push_back(tMid);
      TerrainChannel channel{foot, w.raisedN, 4.0f, setback + stairRun, /*stair=*/true};
      w.channels.push_back(channel);
      const float gapHalf = channel.width * 0.5f + kWallHalfWidth * 2.0f + 0.4f;
      w.gaps.emplace_back(tMid - gapHalf, tMid + gapHalf);
      TerrainAccess access;
      access.width = channel.width;
      access.bottom = glm::vec3(foot.x, 0.0f, foot.y);
      access.top = glm::vec3(top.x, 0.0f, top.y);
      wall.accesses.push_back(access);
      return true;
    };
    // Fragment guarantee: every section (long enough to hold one) gets a
    // ground climb. Ground paths cannot route over the along-wall flights
    // (explicit walk surfaces), so each fragment needs a climb carried by
    // the ground itself. A section position blocked by a street crossing
    // is still acceptable here: that street dead-ends at the wall, so a
    // stair may sit at its mouth.
    for (const auto& section : sections) {
      if (section.second - section.first < 9.0f) continue;
      bool has = false;
      for (const float t : groundSpots) {
        if (t > section.first && t < section.second) has = true;
      }
      for (const float at : {0.5f, 0.3f, 0.7f, 0.15f, 0.85f}) {
        if (has) break;
        has = placeThrough(glm::mix(section.first + 3.0f, section.second - 3.0f, at));
      }
      // Last resort: at least reach this fragment by an along-wall flight.
      for (const float at : {0.2f, 0.5f}) {
        if (has || section.second - section.first < stairRun + 8.0f) break;
        has = placeAlong(glm::mix(section.first + 2.0f, section.second - stairRun - 5.0f, at));
      }
    }

    const float tArtery = 0.5f * (arteryBand.first + arteryBand.second);
    int placedCount[2] = {0, 0};
    int groundCount[2] = {0, 0};  // Through-wall stairs: ground routes over the wall.
    const auto freeRuns = FreeIntervals(tMin, tMax, blocked);
    for (const auto& freeRun : freeRuns) {
      const float length = freeRun.second - freeRun.first;
      const int sideIndex = 0.5f * (freeRun.first + freeRun.second) < tArtery ? 0 : 1;
      const bool wanted = rng.Chance(placedCount[sideIndex] ? 0.55f : 0.9f);
      const bool preferAlong = rng.Chance(0.5f);
      if (!wanted || placedCount[sideIndex] >= 2) continue;
      bool done = false;
      for (const bool along : {preferAlong, !preferAlong}) {
        if (done) break;
        if (along && length >= stairRun + 7.0f) {
          done =
              placeAlong(freeRun.first + rng.Float(2.0f, std::max(2.1f, length - stairRun - 4.5f)));
        } else if (!along && length >= 10.0f) {
          done = placeThrough(freeRun.first + rng.Float(4.0f, length - 4.0f));
          if (done) ++groundCount[sideIndex];
        }
      }
      if (done) ++placedCount[sideIndex];
    }
    scene->terrainWalls.push_back(std::move(wall));
  }

  // The assembled ground: noise, level block pads, the level steps (with
  // their channels' climbs), and the highway corridors' exact road grade.
  const float bandA = c.arteryWidth * 0.5f + 1.6f;
  const float bandB = plan.hasBranch ? plan.branch.width * 0.5f + 1.6f : 0.0f;
  const glm::vec2 branchN(-plan.branch.dir.y, plan.branch.dir.x);
  const auto groundAt = [&](glm::vec2 p) {
    const float sA = glm::dot(p - artery.origin, artery.dir);
    if (std::fabs(glm::dot(p - artery.origin, artery.normal)) < bandA &&
        sA > artery.sMin - 2.0f && sA < artery.sMax + 2.0f) {
      return HighwayGradeAt(terrain, p, artery.dir, false);
    }
    if (plan.hasBranch) {
      const float u = glm::dot(p - plan.branch.start, plan.branch.dir);
      if (u > -1.0f && u < plan.branch.length + 1.0f &&
          std::fabs(glm::dot(p - plan.branch.start, branchN)) < bandB) {
        return HighwayGradeAt(terrain, p, plan.branch.dir, true);
      }
    }
    float y = noise(p);
    for (size_t i = 0; i < city.blocks.size(); ++i) {
      const float d = InsideDistance(p, city.blocks[i].vertices);
      if (d > -2.0f) y = glm::mix(y, padHeights[i], SmoothStep01((d + 2.0f) / 2.0f));
    }
    for (const TerrainWallSpec& w : terrain.walls) {
      const float d = glm::dot(w.raisedN, p) - w.stepD;
      float factor = SmoothStep01((d + kStepBlend) / kStepBlend);
      for (const TerrainChannel& channel : w.channels) {
        const glm::vec2 rel = p - channel.foot;
        if (std::fabs(rel.x * channel.along.y - rel.y * channel.along.x) <=
            channel.width * 0.5f) {
          factor = std::clamp(glm::dot(rel, channel.along) / channel.length, 0.0f, 1.0f);
          break;
        }
      }
      y += w.rise * factor;
    }
    return y;
  };

  HeightField& ground = scene->ground;
  ground.minX = ground.minZ = -half;
  ground.nx = ground.nz = static_cast<int>(std::ceil(2.0f * half)) + 1;
  ground.step = 2.0f * half / (ground.nx - 1);
  ground.heights.resize(static_cast<size_t>(ground.nx) * ground.nz);
  for (int iz = 0; iz < ground.nz; ++iz) {
    for (int ix = 0; ix < ground.nx; ++ix) {
      ground.heights[static_cast<size_t>(iz) * ground.nx + ix] =
          groundAt({ground.minX + ix * ground.step, ground.minZ + iz * ground.step});
    }
  }
  for (TerrainWall& wall : scene->terrainWalls) {
    for (TerrainAccess& access : wall.accesses) {
      access.bottom.y = ground.HeightAt(access.bottom.x, access.bottom.z);
      access.top.y = ground.HeightAt(access.top.x, access.top.z);
    }
  }
}

// Retaining walls along every level step (gapped where roads ramp through
// and at through-wall stairs), flanks beside the ramp cuts, and the
// along-wall stair flights as explicit WalkSurfaces.
void EmitTerrainWalls(Scene* scene, const CityPlan& city, const MapGeneratorConfig& c) {
  const TerrainPlan& terrain = city.terrain;
  if (!terrain.enabled) return;
  const HighwayPlan& plan = city.highway;
  const ArterySpec& artery = plan.artery;
  const float half = scene->mapHalfExtent;
  const auto groundY = [&](glm::vec2 p) { return scene->ground.HeightAt(p.x, p.y); };

  // Short wall boxes between a and b: each stands on the lower ground
  // (sampled at lowOff from the piece) and tops out flush with the upper
  // ground (sampled at highOff), so wall tops follow the terrain.
  const auto wallRun = [&](glm::vec2 a, glm::vec2 b, glm::vec2 lowOff, glm::vec2 highOff,
                           float halfWidth) {
    const float length = glm::distance(a, b);
    if (length < 0.3f) return;
    const glm::vec2 dir = (b - a) / length;
    const glm::vec2 n(-dir.y, dir.x);
    const int pieces = std::max(1, static_cast<int>(std::ceil(length / 4.0f)));
    for (int i = 0; i < pieces; ++i) {
      const glm::vec2 p0 = a + dir * (length * i / pieces);
      const glm::vec2 p1 = a + dir * (length * (i + 1) / pieces);
      const glm::vec2 mid = 0.5f * (p0 + p1);
      const float base =
          std::min({groundY(p0 + lowOff), groundY(mid + lowOff), groundY(p1 + lowOff)}) - 0.15f;
      const float top = groundY(mid + highOff);
      if (top - base < 0.45f) continue;
      AddPolygonBuilding(scene,
                         {p0 - n * halfWidth, p1 - n * halfWidth, p1 + n * halfWidth,
                          p0 + n * halfWidth},
                         top - base, [base](float, float) { return base; });
    }
  };

  for (const TerrainWallSpec& w : terrain.walls) {
    const glm::vec2 lineOrigin =
        w.origin + w.raisedN * (w.stepD - glm::dot(w.raisedN, w.origin));
    float tMin = 0.0f, tMax = 0.0f;
    ClipLineToSquare(lineOrigin, w.dir, half, &tMin, &tMax);
    auto gaps = w.gaps;
    std::sort(gaps.begin(), gaps.end());
    // The box hugs the lower side of the step so the level's own sidewalk
    // ring stays walkable behind the wall top.
    const glm::vec2 wallOff = -w.raisedN * kWallHalfWidth;
    const auto emit = [&](float t0, float t1) {
      if (t1 - t0 < 0.1f) return;
      wallRun(lineOrigin + w.dir * t0 + wallOff, lineOrigin + w.dir * t1 + wallOff,
              -w.raisedN * 1.6f, w.raisedN * 2.2f, kWallHalfWidth);
    };
    float from = tMin - 0.6f;
    for (const auto& gap : gaps) {
      emit(from, gap.first);
      from = std::max(from, gap.second);
    }
    emit(from, tMax + 0.6f);

    // Channel flanks: stair channels get full side walls; a ramped avenue
    // walls its cut edges wherever the climb leaves a drop beside it. A
    // flank reaches from inside the cut edge to the building line so it
    // never strands a sliver of walkable sidewalk behind itself.
    for (const TerrainChannel& channel : w.channels) {
      const glm::vec2 across(-channel.along.y, channel.along.x);
      const float inner = channel.width * 0.5f + (channel.stair ? 0.0f : -0.1f);
      const float outer = inner + (channel.stair ? kWallHalfWidth * 2.0f : c.sidewalkWidth + 0.3f);
      for (const float side : {-1.0f, 1.0f}) {
        const glm::vec2 off = across * (side * 0.5f * (inner + outer));
        const glm::vec2 a = channel.foot + off - channel.along * (channel.stair ? 0.3f : 0.5f);
        const glm::vec2 b = channel.foot + off + channel.along * (channel.length + 1.0f);
        wallRun(a, b, -across * (side * (0.5f * (inner + outer) - inner + 0.7f)),
                across * (side * 2.2f), 0.5f * (outer - inner));
      }
    }

    // Flanks along a highway ramping through: the cut's sides are cliffs
    // between the road grade and the level beside it. They too span from
    // the graded corridor's edge to the adjacent block's building line.
    const auto highwayFlanks = [&](glm::vec2 origin, glm::vec2 roadDir, float corridorHalf,
                                   float frontage) {
      const glm::vec2 roadN(-roadDir.y, roadDir.x);
      const float along = glm::dot(w.raisedN, roadDir);
      const float sStep = (w.stepD - glm::dot(w.raisedN, origin)) / along;
      const float dirSign = along > 0.0f ? 1.0f : -1.0f;
      const float run = w.rise / kRampGrade / std::fabs(along);
      const float inner = corridorHalf - 0.1f;
      const float outer = frontage + c.sidewalkWidth + 0.1f;
      for (const float side : {-1.0f, 1.0f}) {
        const glm::vec2 off = roadN * (side * 0.5f * (inner + outer));
        const glm::vec2 a = origin + roadDir * (sStep - dirSign * 1.0f) + off;
        const glm::vec2 b = origin + roadDir * (sStep + dirSign * (run + 1.0f)) + off;
        wallRun(a, b, -roadN * (side * (0.5f * (outer - inner) + 0.7f)),
                roadN * (side * (0.5f * (outer - inner) + 0.5f)), 0.5f * (outer - inner));
      }
    };
    if (w.arteryRamp) {
      highwayFlanks(artery.origin, artery.dir, c.arteryWidth * 0.5f + 1.6f, ArteryBand(c));
    }
    if (plan.hasBranch && w.branchRamp) {
      highwayFlanks(plan.branch.start, plan.branch.dir, plan.branch.width * 0.5f + 1.6f,
                    plan.branch.width * 0.5f + c.localStreetWidth * 0.4f);
    }
  }

  // Along-wall stair flights: a sloped slab hugging the wall face, then a
  // flat turn over the wall top onto the level. The turn's far edge sits a
  // hair lower so its ground connection lands on the level side.
  for (const TerrainWall& wall : scene->terrainWalls) {
    const float run = wall.rise / kStairGrade;
    for (const TerrainAccess& access : wall.accesses) {
      if (!access.alongWall) continue;
      const glm::vec2 foot(access.bottom.x, access.bottom.z);
      const glm::vec2 landing(access.top.x, access.top.z);
      const glm::vec2 along = wall.dir * (glm::dot(landing - foot, wall.dir) > 0.0f ? 1.0f : -1.0f);
      const glm::vec2 flightTop = foot + along * run;
      const glm::vec2 acrossHalf = wall.raisedNormal * (access.width * 0.5f);
      // The top sits above curb height: the turn crosses the level's own
      // sidewalk ring.
      const float yBottom = access.bottom.y + 0.03f;
      const float yTop = access.top.y + 0.16f;
      WalkSurface flight;
      flight.vertices = {glm::vec3(foot.x - acrossHalf.x, yBottom, foot.y - acrossHalf.y),
                         glm::vec3(flightTop.x - acrossHalf.x, yTop, flightTop.y - acrossHalf.y),
                         glm::vec3(flightTop.x + acrossHalf.x, yTop, flightTop.y + acrossHalf.y),
                         glm::vec3(foot.x + acrossHalf.x, yBottom, foot.y + acrossHalf.y)};
      flight.connectsToGround = true;
      const int flightIndex = static_cast<int>(scene->walkSurfaces.size());
      flight.neighbors.push_back(flightIndex + 1);
      scene->walkSurfaces.push_back(std::move(flight));
      // Turn slab: from the flight's strip, across the wall top, onto the
      // level. Spans the wall box below it; no gap in the wall is needed.
      const glm::vec2 inner = -acrossHalf;  // The flight's street-side edge.
      const glm::vec2 outer = wall.raisedNormal * (1.8f + kWallHalfWidth * 2.0f + 0.9f);
      WalkSurface turn;
      const glm::vec2 t0 = flightTop + inner, t1 = flightTop + along * 2.2f + inner;
      const glm::vec2 t2 = flightTop + along * 2.2f + outer, t3 = flightTop + outer;
      turn.vertices = {glm::vec3(t0.x, yTop, t0.y), glm::vec3(t1.x, yTop, t1.y),
                       glm::vec3(t2.x, access.top.y + 0.15f, t2.y),
                       glm::vec3(t3.x, access.top.y + 0.15f, t3.y)};
      turn.connectsToGround = true;
      turn.neighbors.push_back(flightIndex);
      scene->walkSurfaces.push_back(std::move(turn));
    }
  }
}

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
    if (ArteryElevationAt(plan, city.terrain, c, plan.artery.s0) < kGradeY + 0.05f) {
      scene->walkSurfaces[chains[0].first].connectsToGround = true;
    }
    if (ArteryElevationAt(plan, city.terrain, c, plan.artery.s1) < kGradeY + 0.05f) {
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
    // Terrain boundary streets carve wider than they pave; the margins hold
    // the retaining wall and its wall-side walk.
    const float hw = (cut.pave > 0.0f ? cut.pave : cut.width) * 0.5f;
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
          relativeTop > ArteryElevationAt(plan, city.terrain, c, sA) - c.highwayThickness - 0.049f) {
        return false;
      }
      if (plan.hasBranch) {
        const glm::vec2 rel = foot - plan.branch.start;
        const float u = glm::dot(rel, plan.branch.dir);
        const glm::vec2 bn(-plan.branch.dir.y, plan.branch.dir.x);
        if (u > -0.5f && u < plan.branch.length + 0.5f &&
            std::fabs(glm::dot(rel, bn)) < plan.branch.width * 0.5f + 0.8f &&
            relativeTop > BranchElevationAt(plan, city.terrain, c, u) - c.highwayThickness -
                              0.049f) {
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
        // Never inside a retaining wall where the deck crosses a level step.
        bool nearWall = false;
        for (const TerrainWallSpec& w : city.terrain.walls) {
          if (std::fabs(glm::dot(w.raisedN, foot) - w.stepD) < 3.4f) nearWall = true;
        }
        if (nearWall) continue;
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
      const float deckY = ArteryElevationAt(plan, city.terrain, c, s);
      if (deckY < kMinClearance) continue;
      if (plan.hasBranch && std::fabs(s - plan.branch.mergeS) < c.supportSpacing * 0.6f) continue;
      addBent(plan.artery.origin + plan.artery.dir * s, plan.artery.normal,
              c.arteryWidth * 0.5f - 1.1f, deckY);
    }
    if (plan.hasBranch) {
      for (float u = std::max(0.0f, plan.branch.riseStart); u < plan.branch.length - 8.0f;
           u += c.supportSpacing * 0.75f) {
        const float deckY = BranchElevationAt(plan, city.terrain, c, u);
        if (deckY < kMinClearance) continue;
        addBent(plan.branch.start + plan.branch.dir * u,
                glm::vec2(-plan.branch.dir.y, plan.branch.dir.x),
                plan.branch.width * 0.5f - 0.9f, deckY);
      }
    }
  }
}

// Sidewalks and buildings from the parcel pass, then landmark towers.
void EmitBuildings(Scene* scene, const CityPlan& city, const MapGeneratorConfig& c, Rng* rng,
                   const GroundY& groundY) {
  std::vector<BlockParcels> blocks;
  for (size_t blockIndex = 0; blockIndex < city.blocks.size(); ++blockIndex) {
    blocks.push_back(
        PlanBlockParcels(city.blocks[blockIndex], blockIndex, city.streets.roads, c, rng));
  }
  // Parcels make way for the stairs climbing a terrain level: any parcel
  // overlapping an access (plus clearance) is dropped.
  for (const TerrainWall& wall : scene->terrainWalls) {
    for (const TerrainAccess& access : wall.accesses) {
      const auto clearance = AccessFootprint(access, 1.8f);
      for (BlockParcels& block : blocks) {
        auto& parcels = block.parcels;
        parcels.erase(std::remove_if(parcels.begin(), parcels.end(),
                                     [&](const Parcel& parcel) {
                                       return PolygonsOverlap(parcel.footprint, clearance);
                                     }),
                      parcels.end());
      }
    }
  }
  // A building needs a level foundation. Parcels keep the sidewalk inset
  // from their frontage, except that a sliver block's mitered inset can
  // leave a corner nearly on the block edge, where terrain ground already
  // steps toward a level wall or ramp channel. Such a parcel is dropped
  // rather than built on a slope. Flat cities are untouched.
  if (!scene->ground.Empty()) {
    for (BlockParcels& block : blocks) {
      auto& parcels = block.parcels;
      parcels.erase(std::remove_if(parcels.begin(), parcels.end(),
                                   [&](const Parcel& parcel) {
                                     const glm::vec2 center = PolygonCenter(parcel.footprint);
                                     const float base = groundY(center.x, center.y);
                                     for (const glm::vec2& p : parcel.footprint) {
                                       if (std::fabs(groundY(p.x, p.y) - base) > 0.02f) return true;
                                     }
                                     return false;
                                   }),
                    parcels.end());
    }
  }
  for (BlockParcels& block : blocks) {
    // Sidewalk ring only: the parcel interior stays at grade. Each block
    // edge owns the convex strip of the block within `sidewalkWidth` of it;
    // strips overlap coplanar at the corners.
    const std::vector<glm::vec2>& outline = block.sidewalk;
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
    for (Parcel& parcel : block.parcels) {
      AddPolygonBuilding(scene, std::move(parcel.footprint), parcel.height, groundY);
    }
  }

  // Landmark towers reuse actual polygon footprints rather than reverting to
  // bounding boxes.
  const int towers = std::min<int>(c.towerCount, static_cast<int>(scene->obstacles.size()));
  std::vector<size_t> candidates;
  for (size_t i = 0; i < scene->obstacles.size(); ++i) {
    if (!scene->obstacles[i].footprint.empty()) candidates.push_back(i);
  }
  for (int i = 0; i < towers && !candidates.empty(); ++i) {
    const size_t pick = static_cast<size_t>(rng->Int(0, static_cast<int>(candidates.size()) - 1));
    AABB& bounds = scene->obstacles[candidates[pick]].bounds;
    bounds.max.y = bounds.min.y + rng->Float(c.towerMinHeight, c.towerMaxHeight);
    candidates.erase(candidates.begin() + pick);
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

Scene GenerateArterialUrbanMap(uint32_t seed, const MapGeneratorConfig& c) {
  Rng rng(seed);
  Scene scene;
  scene.mapHalfExtent = UrbanMapHalfExtent(c);
  CityPlan city = BuildCityPlan(seed, c);
  BuildUrbanTerrain(&scene, &city, seed, c);
  const GroundY groundY = [&](float x, float z) { return scene.ground.HeightAt(x, z); };
  EmitHighwaySurfaces(&scene, city, c, groundY);
  EmitStreetSurfaces(&scene, city, c, groundY);
  EmitPiers(&scene, city, c, groundY);
  EmitBuildings(&scene, city, c, &rng, groundY);
  EmitSpawns(&scene, city, c, groundY);
  const size_t roofCount = scene.obstacles.size();
  EmitTerrainWalls(&scene, city, c);
  PlaceZiplines(&scene, city.blocks, seed, c, roofCount);
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
  auto roads = BuildUrbanRoads(seed, c);
  if (c.features.terrain.mode != FeatureMode::Off) {
    Scene terrain;
    terrain.mapHalfExtent = UrbanMapHalfExtent(c);
    CityPlan city = BuildCityPlan(seed, c);
    BuildUrbanTerrain(&terrain, &city, seed, c);
    for (auto& road : roads) {
      for (auto& p : road.centerline) p.y += terrain.ground.HeightAt(p.x, p.z);
    }
  }
  return roads;
}

std::vector<UrbanStreet> UrbanStreets(uint32_t seed, const MapGeneratorConfig& c) {
  std::vector<UrbanStreet> out;
  if (c.arteryCount <= 0) return out;
  const HighwayPlan plan = BuildHighwayPlan(seed, c);
  const StreetGraph graph =
      BuildStreetGraph(seed, c, plan, BuildTerrainPlan(seed, c, plan));
  for (const UrbanRoad& road : UrbanRoads(seed, c)) {
    for (size_t i = 0; i + 1 < road.centerline.size(); ++i) {
      out.push_back({road.centerline[i], road.centerline[i + 1], road.width,
                     UrbanStreetClass::Highway, road.elevated});
    }
  }
  const float half = UrbanMapHalfExtent(c);
  Scene terrain;
  terrain.mapHalfExtent = half;
  if (c.features.terrain.mode != FeatureMode::Off) {
    CityPlan city = BuildCityPlan(seed, c);
    BuildUrbanTerrain(&terrain, &city, seed, c);
  }
  for (const StreetCut& cut : graph.cuts) {
    if (cut.cls == UrbanStreetClass::Highway) continue;  // Already a road above.
    if (cut.terrainBoundary && cut.side < 0) continue;   // One entry per boundary line.
    float t0, t1;
    ClipLineToSquare(cut.origin, cut.dir, half, &t0, &t1);
    if (!(t1 > t0)) continue;
    const glm::vec2 a = cut.origin + cut.dir * t0, b = cut.origin + cut.dir * t1;
    out.push_back({glm::vec3(a.x, terrain.ground.HeightAt(a.x, a.y), a.y),
                   glm::vec3(b.x, terrain.ground.HeightAt(b.x, b.y), b.y),
                   cut.pave > 0.0f ? cut.pave : cut.width, cut.cls, false});
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
