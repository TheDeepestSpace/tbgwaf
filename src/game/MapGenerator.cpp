#include "game/MapGenerator.h"

#include <algorithm>
#include <array>
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
//   0x3c9d51e7u   highway profile  deck layout against the levels, ramp spots
//   0x6a09e667u   ramp channels   which avenue ramps through each level
//   0x1eae1007u   terrain accesses stair flights
//   0x21b1e5a7u   ziplines         roof/anchor selection
//   0xa5c47e75u   archetypes       floors, facade variants, setbacks
//   0x3a55196eu   massing          parapets, roof furniture, awnings
//   0x51f0a3c9u   facades          door spots, stoops, rear platforms, window grids
//   0x6c57f00du   furniture        feature presence, then curb-side item stations/kinds
//   0x51674ea1u   traffic control  light-vs-sign draws per junction, street-name corners
//   0x5c71123eu   skyline         height-field waves, per-building floor jitter
//   0x3a27e41bu   extra arteries  how many avenues are widened into arteries, and which
//   0x4c4f4e47u   long blocks      apartment row vs. street-divided block subdivisions
//   0x7d3a94c5u   pier form        T-bent vs twin-column bent draw (ResolvePierForm)
//   0x74ee270bu   trees            visual-only placement, variant, scale, yaw, tint
constexpr uint32_t kStreetStream = 0xb10c5eedu;
constexpr uint32_t kPierFormStream = 0x7d3a94c5u;
constexpr uint32_t kRampChannelStream = 0x6a09e667u;
constexpr uint32_t kFurnitureStream = 0x6c57f00du;
constexpr uint32_t kTrafficStream = 0x51674ea1u;
constexpr uint32_t kLongBlockStream = 0x4c4f4e47u;
constexpr uint32_t kSkylineStream = 0x5c71123eu;
constexpr uint32_t kExtraArteryStream = 0x3a27e41bu;

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

struct BlockAxis {
  glm::vec2 direction{1.0f, 0.0f};
  float lo = 0.0f, hi = 0.0f;
  float depth = 0.0f;
};

// Measures a convex block along the direction of its longest boundary.
// Missing-street blocks are broad rectangles rather than thin wedges, so
// the fill check below keeps acute/sliver polygons out of this treatment.
BlockAxis LongBlockAxis(const std::vector<glm::vec2>& polygon) {
  BlockAxis best;
  float bestSpan = -1.0f;
  for (size_t edge = 0; edge < polygon.size(); ++edge) {
    const glm::vec2 delta = polygon[(edge + 1) % polygon.size()] - polygon[edge];
    const float edgeLength = glm::length(delta);
    if (edgeLength < 1.0e-4f) continue;
    const glm::vec2 axis = delta / edgeLength;
    const glm::vec2 across(-axis.y, axis.x);
    float lo = std::numeric_limits<float>::infinity(), hi = -lo;
    float crossLo = std::numeric_limits<float>::infinity(), crossHi = -crossLo;
    for (const glm::vec2& point : polygon) {
      const float along = glm::dot(axis, point);
      const float cross = glm::dot(across, point);
      lo = std::min(lo, along);
      hi = std::max(hi, along);
      crossLo = std::min(crossLo, cross);
      crossHi = std::max(crossHi, cross);
    }
    if (hi - lo > bestSpan) {
      bestSpan = hi - lo;
      best = {axis, lo, hi, crossHi - crossLo};
    }
  }
  return best;
}

bool IsLongBlock(const UrbanBlock& block, const MapGeneratorConfig& c, BlockAxis* shape) {
  *shape = LongBlockAxis(block.vertices);
  const float length = shape->hi - shape->lo;
  if (length < c.blockSize * 1.65f || shape->depth < c.minBlockSize * 0.55f ||
      length < shape->depth * 1.6f) {
    return false;
  }
  return PolygonArea(block.vertices) >= length * shape->depth * 0.6f;
}

glm::vec2 Rotate(glm::vec2 v, float radians);

// Shortest slab side: five residential windows (1.1 wide, 1.0 gap) across
// with the facade end margins either side.
constexpr float kMinSlabSpan = 5.0f * 1.1f + 4.0f * 1.0f + 2.0f * 0.7f;

// Freestanding apartment slabs for a long block's sidewalk-inset polygon:
// angled across the short dimension, well inside the block. Empty unless at
// least three keep a decent size, so only blocks that really fit a row get one.
std::vector<std::vector<glm::vec2>> ApartmentRowSlabs(const std::vector<glm::vec2>& outer,
                                                      const MapGeneratorConfig& c,
                                                      size_t blockIndex) {
  const BlockAxis shape = LongBlockAxis(outer);
  const float inset = std::min(c.gapWidth * 0.65f, shape.depth * 0.1f);
  std::vector<glm::vec2> buildable = InsetConvexPolygon(outer, inset);
  // Very short corner edges can invert when inset a second time. The first
  // sidewalk inset is still a valid fallback and the smaller rectangles
  // below provide their own edge clearance.
  bool buildableValid = buildable.size() == outer.size() && PolygonSignedArea(buildable) > 0.0f;
  for (size_t i = 0; buildableValid && i < buildable.size(); ++i) {
    const glm::vec2 a =
        buildable[i] - buildable[(i + buildable.size() - 1) % buildable.size()];
    const glm::vec2 b = buildable[(i + 1) % buildable.size()] - buildable[i];
    buildableValid = a.x * b.y - a.y * b.x > 0.0f;
  }
  if (!buildableValid) buildable = outer;
  const glm::vec2 rowAxis = shape.direction;
  const glm::vec2 crossAxis(-rowAxis.y, rowAxis.x);
  float rowLo = std::numeric_limits<float>::infinity(), rowHi = -rowLo;
  float crossLo = std::numeric_limits<float>::infinity(), crossHi = -crossLo;
  for (const glm::vec2& point : buildable) {
    rowLo = std::min(rowLo, glm::dot(rowAxis, point));
    rowHi = std::max(rowHi, glm::dot(rowAxis, point));
    crossLo = std::min(crossLo, glm::dot(crossAxis, point));
    crossHi = std::max(crossHi, glm::dot(crossAxis, point));
  }
  const float length = rowHi - rowLo;
  const float buildableDepth = crossHi - crossLo;
  const float targetPitch = std::max(24.0f, c.blockSize * 0.95f);
  const int count = std::clamp(static_cast<int>(std::floor(length / targetPitch)), 2, 6);
  const float pitch = length / static_cast<float>(count);

  const float lean = glm::radians((blockIndex & 1u) ? 18.0f : -18.0f);
  const glm::vec2 longAxis = Rotate(crossAxis, lean);
  const glm::vec2 shortAxis(-longAxis.y, longAxis.x);
  const float slabWidth = std::min(std::max(c.buildingDepth * 1.2f, kMinSlabSpan), pitch * 0.5f);
  // Leaning slabs stay a clear alley apart along the row.
  const float slabLength = std::min(
      buildableDepth * 0.8f, (pitch - slabWidth * 0.95f - c.gapWidth * 2.0f) / 0.32f);

  // A convex block can taper near an end. Put every centre halfway across
  // its local cross-section and shorten only the slab at that position as
  // needed to keep all four corners inside the inset buildable area.
  auto crossSection = [&](float along, float* lo, float* hi) {
    *lo = std::numeric_limits<float>::infinity();
    *hi = -*lo;
    for (size_t edge = 0; edge < buildable.size(); ++edge) {
      const glm::vec2 a = buildable[edge];
      const glm::vec2 b = buildable[(edge + 1) % buildable.size()];
      const float ta = glm::dot(rowAxis, a), tb = glm::dot(rowAxis, b);
      if (std::fabs(ta - along) < 1.0e-4f) {
        const float across = glm::dot(crossAxis, a);
        *lo = std::min(*lo, across);
        *hi = std::max(*hi, across);
      }
      if (std::fabs(tb - ta) < 1.0e-5f ||
          along < std::min(ta, tb) - 1.0e-4f ||
          along > std::max(ta, tb) + 1.0e-4f) {
        continue;
      }
      const float u = std::clamp((along - ta) / (tb - ta), 0.0f, 1.0f);
      const float across = glm::dot(crossAxis, a + (b - a) * u);
      *lo = std::min(*lo, across);
      *hi = std::max(*hi, across);
    }
    return std::isfinite(*lo) && *hi > *lo;
  };
  auto rectangle = [&](glm::vec2 center, float scale) {
    const glm::vec2 u = longAxis * (slabLength * scale * 0.5f);
    const glm::vec2 v = shortAxis * (slabWidth * 0.5f);  // Width never shrinks.
    return std::vector<glm::vec2>{center - u - v, center + u - v,
                                  center + u + v, center - u + v};
  };
  auto fits = [&](glm::vec2 center, float scale) {
    for (const glm::vec2& corner : rectangle(center, scale)) {
      if (!PointInConvexPolygon(corner, buildable)) return false;
    }
    return true;
  };

  std::vector<glm::vec2> centers;
  std::vector<float> scales;
  for (int i = 0; i < count; ++i) {
    const float along = rowLo + pitch * (static_cast<float>(i) + 0.5f);
    float localCrossLo = 0.0f, localCrossHi = 0.0f;
    if (!crossSection(along, &localCrossLo, &localCrossHi)) continue;
    const glm::vec2 center =
        rowAxis * along + crossAxis * ((localCrossLo + localCrossHi) * 0.5f);
    centers.push_back(center);
    float scale = 1.0f;
    if (!fits(center, scale)) {
      float lo = 0.0f, hi = 1.0f;
      for (int step = 0; step < 12; ++step) {
        const float mid = (lo + hi) * 0.5f;
        if (fits(center, mid)) lo = mid;
        else hi = mid;
      }
      scale = lo;
    }
    scales.push_back(scale);
  }
  std::vector<std::vector<glm::vec2>> slabs;
  // Decent: most of the full length survives and the slab stays elongated.
  auto decentSlab = [&](float scale) {
    return scale >= 0.7f && slabLength * scale >= slabWidth * 1.3f;
  };
  int decent = 0;
  for (const float scale : scales) decent += decentSlab(scale) ? 1 : 0;
  if (decent < 3) return slabs;
  for (size_t i = 0; i < centers.size(); ++i) {
    if (decentSlab(scales[i])) slabs.push_back(rectangle(centers[i], scales[i]));
  }
  return slabs;
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
  if (c.arteryCount <= 0) return false;
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

// Bent column form (issue #223): one draw per seed from its own stream, so
// forcing a form (or toggling the overpass) never reshuffles anything else.
PierForm ResolvePierForm(uint32_t seed, const MapGeneratorConfig& c) {
  if (c.pierForm != PierForm::Auto) return c.pierForm;
  Rng rng(seed ^ kPierFormStream);
  return rng.Chance(0.5f) ? PierForm::Twin : PierForm::TBent;
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
  // Branch draws made before the terrain is known (same stream order as
  // ever, so a flat city keeps its layout): merge fraction and turn.
  float mergeUnit = 0.0f, turn = 0.0f;
  int preferredTurn = 1;
};

// The overpass deck between its rise and fall ramps, as the legacy layouts
// place them on a flat city. A ramp a layout never takes is parked far
// outside the sampled range, so the shared smoothstep profile formula
// (rise minus fall) applies unchanged to every layout.
void ApplyLegacyLayout(ArterySpec* artery, OverpassLayout layout) {
  ArterySpec& a = *artery;
  const float length = a.sMax - a.sMin;
  const float far = 100.0f * length;
  a.s0 = a.sMin;
  a.s1 = a.sMax;
  a.rise0 = a.sMin + 0.10f * length;
  a.rise1 = a.sMin + 0.36f * length;
  a.fall0 = a.sMin + 0.64f * length;
  a.fall1 = a.sMin + 0.90f * length;
  switch (layout) {
    case OverpassLayout::Auto:  // Resolved by the caller; both ramps on-map.
    case OverpassLayout::RampUpRampDown:
      break;
    case OverpassLayout::Through:
      a.s0 = a.sMin - kDeckOverhang;
      a.s1 = a.sMax + kDeckOverhang;
      a.rise0 = -far;
      a.rise1 = -far + 1.0f;
      a.fall0 = far;
      a.fall1 = far + 1.0f;
      break;
    case OverpassLayout::EnterRampUp:
      a.s1 = a.sMax + kDeckOverhang;
      a.rise0 = a.sMin + 0.22f * length;
      a.rise1 = a.sMin + 0.52f * length;
      a.fall0 = far;
      a.fall1 = far + 1.0f;
      break;
    case OverpassLayout::EnterRampDown:
      a.s0 = a.sMin - kDeckOverhang;
      a.rise0 = -far;
      a.rise1 = -far + 1.0f;
      a.fall0 = a.sMin + 0.48f * length;
      a.fall1 = a.sMin + 0.78f * length;
      break;
  }
}

// First half of the highway plan: the artery line, the overpass draw and
// the branch draws. The deck profile and the branch itself are settled by
// FinishHighwayPlan once the terrain levels are known.
HighwayPlan BeginHighwayPlan(uint32_t seed, const MapGeneratorConfig& c) {
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
  ApplyLegacyLayout(&artery, plan.elevated ? plan.layout : OverpassLayout::RampUpRampDown);
  plan.hasBranch = c.arteryCount >= 2 || plan.elevated;
  // Branch draws happen unconditionally so toggling the branch or the deck
  // never reshuffles the rest of a seed's layout.
  plan.mergeUnit = rng.Unit01();
  plan.preferredTurn = rng.Chance(0.5f) ? 1 : -1;
  plan.turn = glm::radians(rng.Float(30.0f, 44.0f));
  return plan;
}

float SmoothStep01(float t) {
  t = std::clamp(t, 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

// --- Map-spanning terrain levels (issue #155). ---
//
// Terrain comes first. A terrain-enabled seed raises whole portions of the
// map behind sheer cliffs: each level step is a straight line crossing
// the entire map, roughly perpendicular to the artery. A narrow strip on
// its low side (a stair strip and a walkway) is carved
// between the blocks, so no block straddles a wall and no street is owed
// to it; the local streets are drawn independently and may run into a
// wall or ramp through it. Steps stack where raised sides overlap,
// nesting tiers at different heights. The tiers are sharp: no rolling
// noise. The highway profile is planned against the walls
// (FinishHighwayPlan), and stairs climb each wall along its face.

constexpr uint32_t kTerrainStream = 0x7e22ac31u;
constexpr uint32_t kHighwayProfileStream = 0x3c9d51e7u;
constexpr float kRampGrade = 0.30f;   // Max rise/run of a road ramping through a wall.
constexpr float kDeckGrade = 0.22f;   // Rise/run of an overpass ramp.
constexpr float kStairGrade = 0.80f;  // Along-wall stair flight.
constexpr float kStepBlend = 0.02f;   // A level step is sheer: no grid sample lands on its feather.
constexpr float kFlightWidth = 2.4f;  // Stair flight hugging the cliff on its low side.
constexpr float kLanding = 2.4f;      // Flat landing at the flight's head, at the level's height.
constexpr float kFlightMargin = 1.5f; // Clear cliff line past a stair's ends.
// A level edge is a block boundary like a street: a band this wide, centered
// on the step line, is carved from the blocks on both sides, so sidewalk
// rings and buildings keep a street's setback from the cliff. The low half
// holds the stairs.
constexpr float kLevelBand = 6.4f;
constexpr float kGroundStep = 0.5f;   // Heightfield spacing: sharp steps, narrow flights.
constexpr float kDeckClearMargin = 8.0f;  // Deck at full height this far either side of a wall it flies over.
constexpr float kDeckUnderClearance = 2.2f;  // Headroom a deck keeps over any tier beneath it.
constexpr float kFlatMargin = 9.0f;       // Flat corridor either side of a wall ramped through at grade.

// A ramped opening through one wall where an avenue crosses it: inside the
// across-band around `foot + t * along`, the wall's step becomes a linear
// climb over `length` instead of a sharp cliff.
struct TerrainChannel {
  glm::vec2 foot{0.0f}, along{0.0f, 1.0f};  // `along` points at the raised side.
  float width = 0.0f, length = 1.0f;
};

// A stair attached along the cliff face, on the low side: a straight flight
// at kStairGrade, then a flat landing (kLanding) at the level's height.
// Movement follows these planar grades in the heightfield; the renderer
// draws the same shape as one extruded wedge.
struct TerrainFlight {
  float tFoot = 0.0f;  // Wall-line param of the foot.
  float dir = 1.0f;    // Climbs toward increasing (+1) or decreasing (-1) t.
  float run = 1.0f;    // Flight length along the wall (rise / kStairGrade).
};

struct TerrainWallSpec {
  glm::vec2 origin{0.0f}, dir{1.0f, 0.0f};  // The step line; also the centerline of the level band.
  glm::vec2 raisedN{0.0f, 1.0f};            // Unit normal toward the raised side.
  glm::vec2 lineOrigin{0.0f};               // A point on the step line (== origin).
  float stepD = 0.0f;                       // Step line: dot(raisedN, p) >= stepD is raised.
  float rise = 0.0f;
  float cutWidth = 0.0f;                    // Band carved from the blocks around the step line.
  float arteryS = 0.0f;                     // Artery param of the street crossing.
  bool arteryRamp = false;                  // Artery ramps through (vs deck flying over).
  bool branchRamp = false;                  // Same for the branch, when it crosses.
  float branchU = -1000.0f;                 // Branch param of the crossing, if any.
  std::vector<TerrainChannel> channels;     // Filled by the terrain emitter.
  std::vector<TerrainFlight> flights;       // Likewise.
  std::vector<std::pair<float, float>> gaps;  // Wall-line param intervals left open.

  // Where the step line meets a road through `origin` along `roadDir`:
  // the road param of the crossing and the run of its at-grade ramp.
  float CrossingParam(glm::vec2 roadOrigin, glm::vec2 roadDir) const {
    return (stepD - glm::dot(raisedN, roadOrigin)) / glm::dot(raisedN, roadDir);
  }
  float RampRun(glm::vec2 roadDir) const {
    return rise / kRampGrade / std::max(0.25f, std::fabs(glm::dot(raisedN, roadDir)));
  }
  // The corridor a road ramping through this wall climbs in: the band of
  // `halfWidth` about its centerline (through `onLine`, a point on the step
  // line, along `along` toward the raised side) between the step line and
  // the ramp's top, the parallel line rise/kRampGrade beyond it. A
  // parallelogram whose short sides run along the step, so an oblique
  // crossing leaves no sliver of the level inside the road.
  std::vector<glm::vec2> RampZone(glm::vec2 onLine, glm::vec2 along, float halfWidth) const {
    const float a = std::max(0.25f, glm::dot(raisedN, along));
    const glm::vec2 across(-along.y, along.x);
    const float lateral = glm::dot(raisedN, across);  // Step distance gained per unit across.
    const float run = rise / kRampGrade / a;
    const glm::vec2 l = onLine - across * halfWidth + along * (lateral * halfWidth / a);
    const glm::vec2 r = onLine + across * halfWidth - along * (lateral * halfWidth / a);
    return {l, r, r + along * run, l + along * run};
  }
  // Height factor of this step at p: 0 on the low side, 1 on the raised
  // side, with the channels' and flights' climbs in between.
  float Factor(glm::vec2 p) const {
    const float d = glm::dot(raisedN, p) - stepD;
    for (const TerrainChannel& channel : channels) {
      const glm::vec2 rel = p - channel.foot;
      if (std::fabs(rel.x * channel.along.y - rel.y * channel.along.x) <= channel.width * 0.5f) {
        return std::clamp(d / (rise / kRampGrade), 0.0f, 1.0f);  // One plane from the step line.
      }
    }
    if (d < 0.0f && d >= -kFlightWidth) {
      const float t = glm::dot(p - lineOrigin, dir);
      for (const TerrainFlight& flight : flights) {
        const float along = (t - flight.tFoot) * flight.dir;
        if (along < -1e-4f || along > flight.run + kLanding + 1e-4f) continue;
        return std::clamp(along / flight.run, 0.0f, 1.0f);  // The landing holds 1.
      }
    }
    return SmoothStep01((d + kStepBlend) / kStepBlend);
  }
};

struct TerrainPlan {
  bool enabled = false;
  float deckCruise = 0.0f;  // Absolute deck height at full overpass profile.
  std::vector<TerrainWallSpec> walls;
};

// Level height from the steps alone (sharp cliff edges), before road ramps,
// channels and flights.
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
      y += w.rise * std::clamp(d / (w.RampRun(roadDir) * std::fabs(glm::dot(w.raisedN, roadDir))),
                               0.0f, 1.0f);
    } else {
      y += w.rise * SmoothStep01((d + kStepBlend) / kStepBlend);
    }
  }
  return y;
}

// The level steps, placed against the artery line alone: 1-3 per seed
// (terrainLevelCount pins the count), each crossing the artery at a spot
// that leaves room on the map for an at-grade ramp through it and a flat
// stretch between neighbouring climbs.
TerrainPlan BuildTerrainPlan(uint32_t seed, const MapGeneratorConfig& c, const ArterySpec& artery) {
  TerrainPlan terrain;
  if (!TerrainEnabled(seed, c)) return terrain;
  Rng rng(seed ^ kTerrainStream);
  const float length = artery.sMax - artery.sMin;
  const int drawn = 1 + rng.Int(0, 2);
  const int wanted = c.terrainLevelCount < 0 ? drawn : c.terrainLevelCount;
  for (int i = 0; i < wanted; ++i) {
    bool placed = false;
    for (int attempt = 0; attempt < 24 && !placed; ++attempt) {
      TerrainWallSpec w;
      const float sCross = artery.sMin + length * rng.Float(0.20f, 0.80f);
      w.dir = Rotate(artery.normal, rng.Float(-0.18f, 0.18f));
      w.origin = artery.origin + artery.dir * sCross;
      const float side = rng.Chance(0.5f) ? 1.0f : -1.0f;
      w.raisedN = glm::vec2(-w.dir.y, w.dir.x) * side;
      w.rise = constants::kUnitHeight * rng.Float(1.0f, i == 0 ? 5.0f : 2.5f);
      // The level band is carved around the step line like a street.
      w.cutWidth = kLevelBand;
      w.stepD = glm::dot(w.raisedN, w.origin);
      w.lineOrigin = w.origin;
      w.arteryS = sCross;
      // The artery's at-grade climb through this wall, with a flat window
      // either side, must fit on the map.
      const float sStep = w.CrossingParam(artery.origin, artery.dir);
      const float runA = w.RampRun(artery.dir);
      const float sA = std::min(sStep, sStep + runA * (glm::dot(w.raisedN, artery.dir) > 0.0f ? 1.0f : -1.0f));
      const float sB = std::max(sStep, sStep + runA * (glm::dot(w.raisedN, artery.dir) > 0.0f ? 1.0f : -1.0f));
      if (sA - 12.0f < artery.sMin + 4.0f || sB + 12.0f > artery.sMax - 4.0f) continue;
      // Keep level steps apart along the artery: between two climbs the
      // corridor needs a flat window at the local tier.
      bool separated = true;
      for (const TerrainWallSpec& other : terrain.walls) {
        const float needed = 14.0f + runA + other.RampRun(artery.dir) * 1.1f;
        if (std::fabs(sCross - other.arteryS) < std::max(30.0f, needed)) separated = false;
      }
      if (!separated) continue;
      terrain.walls.push_back(std::move(w));
      placed = true;
    }
    if (!placed) break;
  }
  terrain.enabled = !terrain.walls.empty();
  return terrain;
}

// Second half of the highway plan, against the terrain levels. The deck
// profile is chosen among every layout the walls allow: the artery stays
// at grade and ramps through every wall (a main road cutting through the
// levels), or it climbs onto a deck before a contiguous run of walls,
// flies over them clear of the highest tier beneath and settles back onto
// the level beyond -- with either ramp optionally replaced by an elevated
// map edge, up to one tall deck above every level. One seeded draw picks
// among the feasible layouts. The branch then merges onto the deck (or the
// at-grade artery) and ramps through the walls it crosses at grade.
void FinishHighwayPlan(HighwayPlan* planOut, uint32_t seed, const MapGeneratorConfig& c,
                       TerrainPlan* terrainOut) {
  HighwayPlan& plan = *planOut;
  TerrainPlan& terrain = *terrainOut;
  ArterySpec& artery = plan.artery;
  const float half = UrbanMapHalfExtent(c);
  const float length = artery.sMax - artery.sMin;
  const float far = 100.0f * length;
  Rng rng(seed ^ kHighwayProfileStream);
  const float riseUnit = rng.Unit01(), fallUnit = rng.Unit01();
  const auto tierAt = [&](float s) {
    return TierHeight(terrain, artery.origin + artery.dir * s);
  };

  // Wall crossings along the artery, sorted, with each wall's at-grade
  // climb zone [lo, hi].
  struct Crossing {
    size_t wall;
    float sStep, lo, hi;
  };
  std::vector<Crossing> crossings;
  for (size_t i = 0; i < terrain.walls.size(); ++i) {
    const TerrainWallSpec& w = terrain.walls[i];
    const float sStep = w.CrossingParam(artery.origin, artery.dir);
    const float run = w.RampRun(artery.dir);
    const float sign = glm::dot(w.raisedN, artery.dir) > 0.0f ? 1.0f : -1.0f;
    crossings.push_back({i, sStep, std::min(sStep, sStep + sign * run),
                         std::max(sStep, sStep + sign * run)});
  }
  std::sort(crossings.begin(), crossings.end(),
            [](const Crossing& a, const Crossing& b) { return a.sStep < b.sStep; });
  const int k = static_cast<int>(crossings.size());
  for (TerrainWallSpec& w : terrain.walls) w.arteryRamp = true;
  terrain.deckCruise = c.highwayElevation;

  // Windows between the climb zones that may host a deck ramp: window i
  // lies between wall i-1 and wall i (0 = before the first, k = after the
  // last). A ramp's window edge keeps the flat margin past a wall ramped
  // through at grade, or the deck-clearance margin past a wall flown over.
  struct Option {
    int a, b;  // Rise ramp in window a (-1: elevated map edge), fall in window b (k+1: elevated edge).
    float loA, hiA, loB, hiB, lenA, lenB, cruise;
  };
  std::vector<Option> options;
  if (plan.elevated && k > 0) {
    const float minRamp = 24.0f;
    // Walls a..b-1 are flown over (a = -1 / b = k+1: from/to an elevated
    // map edge), so at least one real wall lies between the ramps.
    for (int a = -1; a < k; ++a) {
      for (int b = std::max(a + 1, 1); b <= k + 1; ++b) {
        Option o{a, b, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        // Deck span bounds (conservative: the whole ramp windows).
        const float spanLo = a < 0 ? artery.sMin : (a == 0 ? artery.sMin + 3.0f : crossings[a - 1].hi + kFlatMargin);
        const float spanHi = b > k ? artery.sMax : (b == k ? artery.sMax - 3.0f : crossings[b].lo - kFlatMargin);
        float maxTier = 0.0f;
        for (float s = spanLo; s <= spanHi; s += 2.0f) maxTier = std::max(maxTier, tierAt(s));
        o.cruise = maxTier + c.highwayElevation;
        bool fits = true;
        if (a >= 0) {
          o.loA = spanLo;
          o.hiA = crossings[a].sStep - kDeckClearMargin;
          o.lenA = std::max(minRamp, (o.cruise - tierAt(o.loA + 1.0f)) / kDeckGrade);
          if (o.hiA - o.loA < o.lenA + 2.0f) fits = false;
        }
        if (b <= k) {
          o.loB = crossings[b - 1].sStep + kDeckClearMargin;
          o.hiB = spanHi;
          o.lenB = std::max(minRamp, (o.cruise - tierAt(o.hiB - 1.0f)) / kDeckGrade);
          if (o.hiB - o.loB < o.lenB + 2.0f) fits = false;
        }
        if (fits) options.push_back(o);
      }
    }
  }
  // Lays the artery profile for one layout option.
  const auto applyOption = [&](const Option& pick) {
    artery.s0 = artery.sMin;
    artery.s1 = artery.sMax;
    if (pick.a >= 0) {
      artery.rise0 = pick.loA + riseUnit * (pick.hiA - pick.loA - pick.lenA);
      artery.rise1 = artery.rise0 + pick.lenA;
    } else {
      artery.s0 = artery.sMin - kDeckOverhang;
      artery.rise0 = -far;
      artery.rise1 = -far + 1.0f;
    }
    if (pick.b <= k) {
      artery.fall1 = pick.hiB - fallUnit * (pick.hiB - pick.loB - pick.lenB);
      artery.fall0 = artery.fall1 - pick.lenB;
    } else {
      artery.s1 = artery.sMax + kDeckOverhang;
      artery.fall0 = far;
      artery.fall1 = far + 1.0f;
    }
    plan.layout = pick.a < 0 ? (pick.b > k ? OverpassLayout::Through : OverpassLayout::EnterRampDown)
                             : (pick.b > k ? OverpassLayout::EnterRampUp
                                           : OverpassLayout::RampUpRampDown);
    for (TerrainWallSpec& w : terrain.walls) w.arteryRamp = true;
    for (int i = std::max(pick.a, 0); i < pick.b && i < k; ++i) {
      terrain.walls[crossings[i].wall].arteryRamp = false;
    }
    terrain.deckCruise = pick.cruise;
  };

  // The branch: where it merges and how it turns. On a deck the merge must
  // land where the deck is at full height, so the branch genuinely ramps
  // between grade and deck. Its crossings of the walls must be clean --
  // ramped through on its at-grade stretch, with room either side, or
  // flown over on its junction stretch -- so the merge fraction and turn
  // are searched around their draws until every crossing works.
  BranchSpec& branch = plan.branch;
  branch.width = c.arteryWidth * 0.72f;
  Rng branchRng(seed ^ 0x5b2a9c47u);
  const bool drawnOffMap = branchRng.Chance(c.branchOffMapChance);
  const float rampFraction = branchRng.Float(0.30f, 0.75f);
  const float rampMax = branchRng.Float(30.0f, 70.0f);
  const auto mergeAt = [&](float t) {
    if (!plan.elevated) return artery.sMin + (0.30f + 0.14f * t) * length;
    if (k == 0) {
      switch (plan.layout) {
        case OverpassLayout::EnterRampUp: return artery.sMin + (0.62f + 0.12f * t) * length;
        case OverpassLayout::EnterRampDown: return artery.sMin + (0.26f + 0.12f * t) * length;
        default: return artery.sMin + (0.46f + 0.08f * t) * length;
      }
    }
    const float lo = std::max(artery.rise1, artery.sMin + 0.15f * length) + 10.0f;
    const float hi = std::min(artery.fall0, artery.sMax - 0.15f * length) - 10.0f;
    return glm::mix(lo, std::max(lo, hi), t);
  };
  // Lays the branch for one merge point and turn; false when a wall crossing
  // cannot be made clean. Fills the walls' branch ramps on success.
  const auto tryBranch = [&](float mergeS, int turnSign, bool offMap, bool strict) {
    BranchSpec b;
    b.width = branch.width;
    b.mergeS = mergeS;
    const glm::vec2 merge = artery.origin + artery.dir * mergeS;
    b.dir = Rotate(artery.dir, static_cast<float>(turnSign) * plan.turn);
    float tMin = 0.0f, tMax = 0.0f;
    ClipLineToSquare(merge, -b.dir, half, &tMin, &tMax);
    b.length = tMax;  // Distance back from the merge to the boundary.
    if (strict && b.length <= half * 0.75f) return false;
    b.start = merge - b.dir * b.length;
    b.side = glm::dot(artery.normal, b.start - artery.origin) > 0.0f ? 1 : -1;
    // The ramp tops out before the branch slab crosses the deck's edge, so
    // the whole junction sits at deck height and the branch only starts its
    // descent once clear of the artery (no wedge cutting under the deck).
    const float sinTurn =
        std::max(0.2f, std::fabs(artery.dir.x * b.dir.y - artery.dir.y * b.dir.x));
    const float junctionClear = (c.arteryWidth * 0.5f + b.width * 0.5f + 2.0f) / sinTurn;
    b.riseEnd = std::max(b.length * 0.5f, b.length - junctionClear);
    b.riseStart = b.riseEnd - std::min(b.riseEnd * rampFraction, rampMax);
    // Over terrain the deck can cruise well above the branch's own level:
    // the on-ramp is stretched to keep the deck grade, and a branch too
    // short for that climb is no on-ramp at all.
    if (plan.elevated && k > 0 && !offMap) {
      const glm::vec2 foot = b.start + b.dir * b.riseStart;
      const float needed = (terrain.deckCruise - TierHeight(terrain, foot)) / kDeckGrade;
      if (b.riseEnd - b.riseStart < needed) b.riseStart = b.riseEnd - needed;
      if (b.riseStart < 2.0f) return false;
    }
    b.offMap = offMap;
    if (offMap) {
      b.riseStart = 0.0f;
      b.riseEnd = 0.0f;
      b.u0 = -kDeckOverhang;
    } else {
      b.u0 = 0.0f;
    }
    // The branch deck (all of it off-map, else past the on-ramp) must clear
    // every tier it runs over: a branch starting on a level higher than
    // the deck would be buried in it.
    if (plan.elevated && k > 0) {
      for (float u = offMap ? 0.0f : b.riseEnd; u <= b.length + 1e-3f; u += 2.0f) {
        const glm::vec2 p = b.start + b.dir * std::min(u, b.length);
        if (TierHeight(terrain, p) + kDeckUnderClearance > terrain.deckCruise) return false;
      }
    }
    // At-grade zone of the branch: everything up to its on-ramp (the whole
    // branch when the artery stays at grade); nothing when it runs off-map.
    const float gradeEnd = !plan.elevated ? b.length : (offMap ? 0.0f : b.riseStart);
    std::vector<std::pair<bool, float>> ramps(terrain.walls.size(), {false, -1000.0f});
    std::vector<std::pair<float, float>> zones;
    for (size_t i = 0; i < terrain.walls.size(); ++i) {
      const TerrainWallSpec& w = terrain.walls[i];
      const float denom = glm::dot(w.raisedN, b.dir);
      const float u = w.CrossingParam(b.start, b.dir);
      if (u <= -8.0f || u >= b.length + 8.0f) continue;
      const float run = w.RampRun(b.dir);
      const float uA = std::min(u, u + run * (denom > 0.0f ? 1.0f : -1.0f));
      const float uB = std::max(u, u + run * (denom > 0.0f ? 1.0f : -1.0f));
      if (uA - 6.0f >= 2.0f && uB + 6.0f <= gradeEnd - 2.0f) {
        for (const auto& z : zones) {
          if (std::min(uB, z.second) + 14.0f > std::max(uA, z.first)) return false;  // Two climbs need a flat window between them.
        }
        zones.emplace_back(uA, uB);
        ramps[i] = {true, u};
      } else if (plan.elevated && (offMap || u - 8.0f > b.riseEnd + 2.0f)) {
        ramps[i] = {false, u};  // The branch deck flies over this wall.
      } else {
        return false;  // Crossing lands on the on-ramp or too near an end.
      }
    }
    branch = b;
    for (size_t i = 0; i < terrain.walls.size(); ++i) {
      terrain.walls[i].branchRamp = ramps[i].first;
      terrain.walls[i].branchU = ramps[i].second;
    }
    return true;
  };
  // Searches merge fractions and turns around the draws for a clean branch.
  const auto planBranch = [&]() {
    const bool canOffMap = plan.elevated && plan.layout != OverpassLayout::Through;
    const bool wantOffMap = canOffMap && (c.branchEnd == BranchEnd::Auto ? drawnOffMap
                                                                         : c.branchEnd == BranchEnd::OffMap);
    std::vector<float> candidates = {plan.mergeUnit};
    for (float step = 0.12f; step <= 1.0f; step += 0.12f) {
      for (const float t : {plan.mergeUnit + step, plan.mergeUnit - step}) {
        if (t >= 0.0f && t <= 1.0f) candidates.push_back(t);
      }
    }
    for (const bool strict : {true, false}) {
      // A turn too short for the map is the last resort, the other turn first.
      for (const int turnSign : {strict ? plan.preferredTurn : -plan.preferredTurn,
                                 strict ? -plan.preferredTurn : plan.preferredTurn}) {
        for (const float t : candidates) {
          for (const bool offMap : {wantOffMap, !wantOffMap}) {
            if (offMap && !canOffMap) continue;
            if (tryBranch(mergeAt(t), turnSign, offMap, strict)) return true;
          }
          if (k == 0) break;  // A flat city keeps its drawn merge exactly.
        }
      }
    }
    return false;
  };

  if (!options.empty()) {
    // The draw picks a layout; one the branch cannot serve (its on-ramp
    // too steep or its wall crossings unclean) yields to the next, and
    // with every layout exhausted the artery stays at grade -- unless the
    // overpass was forced, which keeps the drawn layout regardless.
    const int first = rng.Int(0, static_cast<int>(options.size()) - 1);
    bool settled = false;
    for (size_t i = 0; i < options.size() && !settled; ++i) {
      applyOption(options[(first + i) % options.size()]);
      settled = !plan.hasBranch || planBranch();
    }
    if (settled) return;
    if (c.elevatedHighway != OverpassMode::On) {
      plan.elevated = false;
      plan.layout = OverpassLayout::RampUpRampDown;
      ApplyLegacyLayout(&artery, OverpassLayout::RampUpRampDown);
      for (TerrainWallSpec& w : terrain.walls) w.arteryRamp = true;
      terrain.deckCruise = c.highwayElevation;
      plan.hasBranch = c.arteryCount >= 2;
    } else {
      applyOption(options[static_cast<size_t>(first)]);
    }
  }
  if (!plan.hasBranch || planBranch()) return;
  // No clean crossing anywhere: an on-ramp the overpass itself asked for
  // is dropped; a requested second artery is laid at its draw regardless.
  if (c.arteryCount < 2) {
    plan.hasBranch = false;
    return;
  }
  for (TerrainWallSpec& w : terrain.walls) w.branchRamp = false;
  BranchSpec b;
  b.width = branch.width;
  b.mergeS = mergeAt(plan.mergeUnit);
  const glm::vec2 merge = artery.origin + artery.dir * b.mergeS;
  b.dir = Rotate(artery.dir, static_cast<float>(plan.preferredTurn) * plan.turn);
  float tMin = 0.0f, tMax = 0.0f;
  ClipLineToSquare(merge, -b.dir, half, &tMin, &tMax);
  b.length = tMax;
  b.start = merge - b.dir * b.length;
  b.side = glm::dot(artery.normal, b.start - artery.origin) > 0.0f ? 1 : -1;
  b.riseEnd = b.length * 0.5f;
  b.riseStart = b.riseEnd - std::min(b.riseEnd * rampFraction, rampMax);
  if (plan.elevated) {
    const float needed = (terrain.deckCruise - TierHeight(terrain, b.start)) / kDeckGrade;
    b.riseStart = std::max(0.0f, std::min(b.riseStart, b.riseEnd - needed));
  }
  branch = b;
  // Every wall it crosses short of the junction is ramped through, even
  // under the on-ramp: the deck's lift rides the ramped grade, so the slab
  // stays smooth instead of stepping with the wall beneath.
  for (TerrainWallSpec& w : terrain.walls) {
    const float u = w.CrossingParam(b.start, b.dir);
    if (u > -8.0f && u < b.length + 8.0f && (!plan.elevated || u - 8.0f <= b.riseEnd + 2.0f)) {
      w.branchRamp = true;
      w.branchU = u;
    }
  }
}

// The whole highway plan plus the terrain levels it was planned against.
HighwayPlan BuildHighwayPlan(uint32_t seed, const MapGeneratorConfig& c, TerrainPlan* terrainOut) {
  HighwayPlan plan = BeginHighwayPlan(seed, c);
  *terrainOut = BuildTerrainPlan(seed, c, plan.artery);
  FinishHighwayPlan(&plan, seed, c, terrainOut);
  return plan;
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
  TerrainPlan terrain;
  const HighwayPlan plan = BuildHighwayPlan(seed, c, &terrain);
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

void SimplifyPolygon(std::vector<glm::vec2>* polygon) {
  std::vector<glm::vec2> clean;
  for (glm::vec2 p : *polygon) {
    if (clean.empty() || glm::distance(clean.back(), p) > 1e-4f) clean.push_back(p);
  }
  if (clean.size() > 1 && glm::distance(clean.front(), clean.back()) <= 1e-4f) clean.pop_back();
  bool changed = true;
  while (changed && clean.size() >= 3) {
    changed = false;
    for (size_t i = 0; i < clean.size(); ++i) {
      const glm::vec2 a = clean[(i + clean.size() - 1) % clean.size()];
      const glm::vec2 b = clean[i];
      const glm::vec2 c = clean[(i + 1) % clean.size()];
      const glm::vec2 ab = b - a, bc = c - b;
      if (std::fabs(ab.x * bc.y - ab.y * bc.x) <= 1e-5f) {
        clean.erase(clean.begin() + static_cast<std::ptrdiff_t>(i));
        changed = true;
        break;
      }
    }
  }
  *polygon = std::move(clean);
}

// Cuts an oriented convex model footprint cleanly at the map boundary.
// Keeping this independent of the model kind lets columns, footings and
// their connector beams share exactly the same edge treatment.
std::vector<glm::vec2> ClipModelToMap(std::vector<glm::vec2> polygon, float half) {
  polygon = KeepSide(polygon, {1.0f, 0.0f}, -half);
  polygon = KeepSide(polygon, {-1.0f, 0.0f}, -half);
  polygon = KeepSide(polygon, {0.0f, 1.0f}, -half);
  polygon = KeepSide(polygon, {0.0f, -1.0f}, -half);
  SimplifyPolygon(&polygon);
  if (polygon.size() < 3 || PolygonArea(polygon) < 1e-4f) return {};
  if (PolygonSignedArea(polygon) < 0.0f) std::reverse(polygon.begin(), polygon.end());
  return polygon;
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
// exactly the cuts the street pass chose. A terrain wall strip is carved
// the same way but never paved (it holds the retaining wall, its stairs
// and a walkway); both artery sides get the same line (one entry per side).
struct StreetCut {
  glm::vec2 origin, dir;
  float width;
  int side;
  UrbanStreetClass cls;
  float pave = 0.0f;  // Pavement width; 0 means the full carve width.
  bool terrainBoundary = false;  // A wall strip: unpaved, not a street.
  // Param intervals of the line where the street does not exist: a level
  // cut it off, so it stops at the intersection before the wall (a T
  // there) and resumes at the first one beyond it. Sorted, disjoint.
  std::vector<std::pair<float, float>> gaps;
  // Param intervals its pavement may never cover: the level bands and ramp
  // corridors it would run into (what the gaps grew from). Sorted, disjoint.
  std::vector<std::pair<float, float>> blocked;
  // An avenue widened into an additional artery (MapGeneratorConfig::
  // extraArteries): ramps through every level it meets.
  bool secondaryArtery = false;
  // A short segment dividing a long block (its gaps are everything outside
  // the segment): junction tests use only that segment.
  bool blockSplit = false;

  bool InGap(float t, float tol = 0.0f) const {
    for (const auto& g : gaps) {
      if (t > g.first + tol && t < g.second - tol) return true;
    }
    return false;
  }
  // The street's stretches within [lo, hi] (gaps removed).
  std::vector<std::pair<float, float>> Kept(float lo, float hi) const {
    std::vector<std::pair<float, float>> out;
    float cursor = lo;
    for (const auto& g : gaps) {
      if (g.second <= cursor) continue;
      if (g.first > cursor) out.emplace_back(cursor, std::min(g.first, hi));
      cursor = std::max(cursor, g.second);
      if (cursor >= hi) break;
    }
    if (cursor < hi) out.emplace_back(cursor, hi);
    return out;
  }
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

// Exact urban ground model. The sampled HeightField is for movement and
// queries; emitters use this analytic form so a vertex next to a sheer edge
// never picks up the grid's half-cell interpolation.
float UrbanGroundAt(const HighwayPlan& plan, const TerrainPlan& terrain,
                    const MapGeneratorConfig& c, glm::vec2 p) {
  const ArterySpec& artery = plan.artery;
  // The corridor the road grade owns is exactly the pavement: beside it the
  // tier stands at full height, so a ramp through a level is a clean cut.
  const float bandA = c.arteryWidth * 0.5f;
  const float sA = glm::dot(p - artery.origin, artery.dir);
  if (std::fabs(glm::dot(p - artery.origin, artery.normal)) < bandA &&
      sA > artery.sMin - 2.0f && sA < artery.sMax + 2.0f) {
    return HighwayGradeAt(terrain, p, artery.dir, false);
  }
  if (plan.hasBranch) {
    const glm::vec2 branchN(-plan.branch.dir.y, plan.branch.dir.x);
    const float bandB = plan.branch.width * 0.5f;
    const float u = glm::dot(p - plan.branch.start, plan.branch.dir);
    if (u > -1.0f && u < plan.branch.length + 1.0f &&
        std::fabs(glm::dot(p - plan.branch.start, branchN)) < bandB) {
      return HighwayGradeAt(terrain, p, plan.branch.dir, true);
    }
  }
  float y = 0.0f;
  for (const TerrainWallSpec& wall : terrain.walls) y += wall.rise * wall.Factor(p);
  return y;
}

float UrbanGroundAt(const CityPlan& city, const MapGeneratorConfig& c, glm::vec2 p) {
  return UrbanGroundAt(city.highway, city.terrain, c, p);
}

float RimWidth(const MapGeneratorConfig& c) { return std::max(c.streetWidth, c.localStreetWidth); }
float ArteryBand(const MapGeneratorConfig& c) {
  return c.arteryWidth * 0.5f + c.localStreetWidth * 0.45f;
}

// Param range of a cut's line inside its side's rim rectangle: where the
// street can exist at all (the artery corridor and the rim streets bound it).
bool CutChord(const StreetCut& cut, const HighwayPlan& plan, const MapGeneratorConfig& c,
              float* t0, float* t1) {
  const float hi = UrbanMapHalfExtent(c) - RimWidth(c);
  ClipLineToSquare(cut.origin, cut.dir, hi, t0, t1);
  const glm::vec2 ns = plan.artery.normal * static_cast<float>(cut.side);
  const float f0 = glm::dot(ns, cut.origin) -
                   (static_cast<float>(cut.side) * glm::dot(plan.artery.normal, plan.artery.origin) +
                    ArteryBand(c));
  const float slope = glm::dot(ns, cut.dir);
  if (std::fabs(slope) < 1e-6f) return f0 >= 0.0f && *t1 > *t0;
  const float t = -f0 / slope;
  if (slope > 0.0f) *t0 = std::max(*t0, t);
  else *t1 = std::min(*t1, t);
  return *t1 > *t0;
}

// Param range of a cut's line where its pavement exists: the map square,
// on the cut's own side of the artery corridor (the same clip the pavement
// band gets). Wider than CutChord by the rim streets.
bool CutPavedExtent(const StreetCut& cut, const HighwayPlan& plan, const MapGeneratorConfig& c,
                    float* t0, float* t1) {
  ClipLineToSquare(cut.origin, cut.dir, UrbanMapHalfExtent(c), t0, t1);
  const glm::vec2 ns = plan.artery.normal * static_cast<float>(cut.side);
  const float f0 = glm::dot(ns, cut.origin) -
                   (static_cast<float>(cut.side) * glm::dot(plan.artery.normal, plan.artery.origin) +
                    c.arteryWidth * 0.5f - 0.2f);
  const float slope = glm::dot(ns, cut.dir);
  if (std::fabs(slope) < 1e-6f) return f0 >= 0.0f && *t1 > *t0;
  const float t = -f0 / slope;
  if (slope > 0.0f) *t0 = std::max(*t0, t);
  else *t1 = std::min(*t1, t);
  return *t1 > *t0;
}

// Picks, per wall and artery side, the avenue allowed to ramp through that
// level at road grade (see BuildUrbanTerrain, which carves the opening).
void ChooseRampChannels(uint32_t seed, const MapGeneratorConfig& c, const HighwayPlan& plan,
                        TerrainPlan* terrain, const std::vector<StreetCut>& cuts) {
  const ArterySpec& artery = plan.artery;
  const float half = UrbanMapHalfExtent(c);
  Rng rng(seed ^ kRampChannelStream);
  const glm::vec2 branchN(-plan.branch.dir.y, plan.branch.dir.x);
  for (TerrainWallSpec& w : terrain->walls) {
    for (const int side : {1, -1}) {
      const bool drawn = rng.Chance(0.6f);
      bool hasSecondary = false;
      for (const StreetCut& cut : cuts) hasSecondary |= cut.side == side && cut.secondaryArtery;
      if (!drawn && !hasSecondary) continue;
      // Additional arteries always claim the climb when one fits (they are
      // main roads: they should not dead-end at a level); other avenues only
      // on the usual draw.
      for (int pass = 0; pass < 2; ++pass) {
      if (pass == 1 && !drawn) break;
      bool placed = false;
      for (const StreetCut& cut : cuts) {
        if (cut.side != side || cut.cls == UrbanStreetClass::Highway || cut.terrainBoundary) {
          continue;
        }
        if (pass == 0 && !cut.secondaryArtery) continue;
        const float denom = glm::dot(w.raisedN, cut.dir);
        if (std::fabs(denom) < 0.55f) continue;  // Too oblique for a straight climb.
        const glm::vec2 q = cut.origin + cut.dir * w.CrossingParam(cut.origin, cut.dir);
        const float run = w.RampRun(cut.dir);
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
          for (const TerrainWallSpec& other : terrain->walls) {
            if (&other == &w) continue;
            if (std::fabs(glm::dot(other.raisedN, e) - other.stepD) < 6.0f) fits = false;
          }
        }
        if (!fits) continue;
        w.channels.push_back(TerrainChannel{q, along, cut.width, run});
        placed = true;
        break;
      }
      if (placed) break;
      }
    }
  }
}

// A local street never runs into a level, and never crosses a road that is
// ramping through one. Where a street's pavement would reach a level band
// (the avenue ramped through that level excepted) or a ramp corridor, the
// stretch is cut off: the street ends at the last junction before it (a T
// instead of an X) and resumes at the first one beyond it, so the blocks
// either side of the missing stretch stay one block. With no junction to
// stop at, the street gives way back to the rim.
void TrimStreetsAtWalls(const MapGeneratorConfig& c, const HighwayPlan& plan,
                        const TerrainPlan& terrain, std::vector<StreetCut>* cuts) {
  if (terrain.walls.empty()) return;
  constexpr float kInf = 1e9f;
  const ArterySpec& artery = plan.artery;
  struct Crossing {
    size_t cut;
    float lo, hi;  // Param range on the cut that the obstacle spans.
  };
  std::vector<Crossing> crossings;
  std::vector<std::pair<float, float>> chords(cuts->size());
  std::vector<bool> local(cuts->size(), false);
  // Param range of a cut's line through a convex polygon grown by the
  // street's half width: where its pavement, not just its centerline,
  // would enter the polygon.
  const auto through = [](const StreetCut& cut, const std::vector<glm::vec2>& polygon, float* lo,
                          float* hi) {
    const float orient = PolygonSignedArea(polygon) >= 0.0f ? 1.0f : -1.0f;
    *lo = -kInf;
    *hi = kInf;
    for (size_t i = 0; i < polygon.size(); ++i) {
      const glm::vec2 a = polygon[i], e = polygon[(i + 1) % polygon.size()] - a;
      const float length = glm::length(e);
      if (length < 1e-5f) continue;
      const glm::vec2 inward = glm::vec2(-e.y, e.x) / length * orient;
      const float f0 = glm::dot(inward, cut.origin - a) + cut.width * 0.5f;
      const float slope = glm::dot(inward, cut.dir);
      if (std::fabs(slope) < 1e-6f) {
        if (f0 < 0.0f) return false;
        continue;
      }
      const float t = -f0 / slope;
      if (slope > 0.0f) *lo = std::max(*lo, t);
      else *hi = std::min(*hi, t);
    }
    return *hi > *lo;
  };
  for (size_t i = 0; i < cuts->size(); ++i) {
    const StreetCut& cut = (*cuts)[i];
    if (cut.cls == UrbanStreetClass::Highway || cut.terrainBoundary) continue;
    // Obstacles are met anywhere the pavement runs, the rim stubs included;
    // junctions to stop at only lie within the chord. A street with no
    // chord at all (a stub in a rim corner) has no junction to stop at, so
    // an obstacle takes all of it.
    float paved0 = 0.0f, paved1 = 0.0f;
    if (!CutPavedExtent(cut, plan, c, &paved0, &paved1)) continue;
    if (!CutChord(cut, plan, c, &chords[i].first, &chords[i].second)) {
      chords[i] = {paved1, paved1};
    }
    local[i] = true;
    const auto add = [&](float lo, float hi) {
      if (hi <= paved0 || lo >= paved1) return;
      crossings.push_back({i, lo, hi});
    };
    for (const TerrainWallSpec& w : terrain.walls) {
      const TerrainChannel* own = nullptr;  // This street's own ramp through the wall.
      for (const TerrainChannel& ch : w.channels) {
        const glm::vec2 rel = ch.foot - cut.origin;
        if (std::fabs(rel.x * cut.dir.y - rel.y * cut.dir.x) < 1e-2f &&
            std::fabs(glm::dot(ch.along, cut.dir)) > 0.999f) {
          own = &ch;
        }
      }
      // The level band, grown by the street's own width: a street brushing
      // the band at a shallow angle is cut off as well.
      const float denom = glm::dot(w.raisedN, cut.dir);
      if (!own && std::fabs(denom) > 1e-3f) {
        const float t = (w.stepD - glm::dot(w.raisedN, cut.origin)) / denom;
        const float halfStrip = (w.cutWidth + cut.width) * 0.5f / std::fabs(denom);
        add(t - halfStrip, t + halfStrip);
      }
      // Ramp corridors of the other roads through this wall.
      float lo = 0.0f, hi = 0.0f;
      for (const TerrainChannel& ch : w.channels) {
        if (&ch == own) continue;
        if (through(cut, w.RampZone(ch.foot, ch.along, ch.width * 0.5f), &lo, &hi)) add(lo, hi);
      }
      if (w.arteryRamp) {
        const glm::vec2 foot = artery.origin + artery.dir * w.CrossingParam(artery.origin, artery.dir);
        const glm::vec2 along = artery.dir * (glm::dot(w.raisedN, artery.dir) > 0.0f ? 1.0f : -1.0f);
        // Grown to the artery band, where cross streets end: one ending
        // above the ramp cut would face a cliff instead of the artery.
        if (through(cut, w.RampZone(foot, along, ArteryBand(c) + 0.5f), &lo, &hi)) add(lo, hi);
      }
      if (plan.hasBranch && w.branchRamp) {
        const BranchSpec& branch = plan.branch;
        const glm::vec2 foot = branch.start + branch.dir * w.CrossingParam(branch.start, branch.dir);
        const glm::vec2 along = branch.dir * (glm::dot(w.raisedN, branch.dir) > 0.0f ? 1.0f : -1.0f);
        if (through(cut, w.RampZone(foot, along, branch.width * 0.5f + c.localStreetWidth * 0.4f + 0.5f),
                    &lo, &hi)) {
          add(lo, hi);
        }
      }
    }
  }
  for (const Crossing& x : crossings) (*cuts)[x.cut].blocked.emplace_back(x.lo, x.hi);
  for (StreetCut& cut : *cuts) {
    std::sort(cut.blocked.begin(), cut.blocked.end());
    std::vector<std::pair<float, float>> merged;
    for (const auto& b : cut.blocked) {
      if (!merged.empty() && b.first <= merged.back().second) {
        merged.back().second = std::max(merged.back().second, b.second);
      } else {
        merged.push_back(b);
      }
    }
    cut.blocked = std::move(merged);
  }
  if (crossings.empty()) return;
  // A street stops only at a junction that still exists, so a street cut
  // short itself can lose the junctions others had picked: iterate until
  // the gaps settle (they only ever grow).
  for (int pass = 0; pass < 8; ++pass) {
    bool changed = false;
    for (size_t i = 0; i < cuts->size(); ++i) {
      if (!local[i]) continue;
      StreetCut& cut = (*cuts)[i];
      // Junctions with the other streets on this side: (param on cut).
      std::vector<float> stops;
      for (size_t j = 0; j < cuts->size(); ++j) {
        if (j == i || !local[j] || (*cuts)[j].side != cut.side) continue;
        const StreetCut& other = (*cuts)[j];
        const float cross = cut.dir.x * other.dir.y - cut.dir.y * other.dir.x;
        if (std::fabs(cross) < 0.2f) continue;
        const glm::vec2 rel = other.origin - cut.origin;
        const float t = (rel.x * other.dir.y - rel.y * other.dir.x) / cross;
        const float u = (rel.x * cut.dir.y - rel.y * cut.dir.x) / cross;  // On `other`.
        if (t <= chords[i].first || t >= chords[i].second) continue;
        if (u <= chords[j].first || u >= chords[j].second || other.InGap(u, 0.01f)) continue;
        stops.push_back(t);
      }
      std::vector<std::pair<float, float>> gaps;
      for (const Crossing& x : crossings) {
        if (x.cut != i) continue;
        float lo = chords[i].first, hi = chords[i].second;
        for (const float t : stops) {
          if (t <= x.lo) lo = std::max(lo, t);
          if (t >= x.hi) hi = std::min(hi, t);
        }
        // A gap reaching the street's own end takes the stub beyond it too.
        gaps.emplace_back(lo <= chords[i].first + 1e-3f ? -kInf : lo,
                          hi >= chords[i].second - 1e-3f ? kInf : hi);
      }
      std::sort(gaps.begin(), gaps.end());
      std::vector<std::pair<float, float>> merged;
      for (const auto& g : gaps) {
        if (!merged.empty() && g.first <= merged.back().second) {
          merged.back().second = std::max(merged.back().second, g.second);
        } else {
          merged.push_back(g);
        }
      }
      if (merged != cut.gaps) {
        cut.gaps = std::move(merged);
        changed = true;
      }
    }
    if (!changed) break;
  }
}

// Drops every channel whose avenue no longer runs over the ramp itself
// (cut short by a level or corridor further on and rolled back past it):
// a ramp with no road would be an empty cut in the level. True when any
// was dropped.
bool DropOrphanChannels(TerrainPlan* terrain, const std::vector<StreetCut>& cuts) {
  bool dropped = false;
  for (TerrainWallSpec& w : terrain->walls) {
    const auto orphan = [&](const TerrainChannel& channel) {
      for (const StreetCut& cut : cuts) {
        if (cut.cls == UrbanStreetClass::Highway || cut.terrainBoundary) continue;
        const glm::vec2 rel = channel.foot - cut.origin;
        if (std::fabs(rel.x * cut.dir.y - rel.y * cut.dir.x) >= 1e-2f ||
            std::fabs(glm::dot(channel.along, cut.dir)) <= 0.999f) {
          continue;
        }
        // Its avenue: the climb and a margin either side must be one stretch.
        const float t = glm::dot(rel, cut.dir);
        const float top = t + channel.length * (glm::dot(channel.along, cut.dir) > 0.0f ? 1.0f : -1.0f);
        const float lo = std::min(t, top) - 1.0f, hi = std::max(t, top) + 1.0f;
        const auto kept = cut.Kept(lo, hi);
        return kept.size() != 1 || kept[0].first > lo + 1e-3f || kept[0].second < hi - 1e-3f;
      }
      return true;  // No such street at all.
    };
    auto& channels = w.channels;
    const size_t before = channels.size();
    channels.erase(std::remove_if(channels.begin(), channels.end(), orphan), channels.end());
    dropped = dropped || channels.size() != before;
  }
  return dropped;
}

// How many avenues become additional arteries: pinned by
// config.extraArteries, otherwise drawn per seed (half the seeds keep the
// single artery, 30% add one, 20% add two).
int ExtraArteryCount(uint32_t seed, const MapGeneratorConfig& c) {
  if (c.arteryCount <= 0) return 0;
  Rng rng(seed ^ kExtraArteryStream);
  const float roll = rng.Float(0.0f, 1.0f);
  if (c.extraArteries >= 0) return std::min(c.extraArteries, 2);
  return roll < 0.5f ? 0 : (roll < 0.8f ? 1 : 2);
}

// Street pass: the branch corridor, avenues roughly parallel to the artery
// and oblique cross streets for each side. Each side draws its own, so
// junctions don't line up across the artery and block shapes differ.
StreetGraph BuildStreetGraph(uint32_t seed, const MapGeneratorConfig& c, const HighwayPlan& plan,
                             TerrainPlan* terrainPlan) {
  const TerrainPlan& terrain = *terrainPlan;
  const ArterySpec& artery = plan.artery;
  Rng rng(seed ^ kStreetStream);
  const float half = UrbanMapHalfExtent(c);
  const float band = ArteryBand(c);
  StreetGraph graph;
  graph.roads = BuildUrbanRoads(seed, c);
  // Terrain wall strips first: one full map-spanning line per level step,
  // carved on both artery sides so no block ever straddles a wall. They
  // are not streets: nothing is paved there.
  for (const TerrainWallSpec& w : terrain.walls) {
    for (const int side : {1, -1}) {
      graph.cuts.push_back({w.origin, w.dir, w.cutWidth, side, UrbanStreetClass::Cross, 0.0f,
                            /*terrainBoundary=*/true});
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
  // Additional arteries: some seeds widen one or two avenues into wide
  // through roads beside the main one (two or three arteries in all). They
  // parallel the artery, so they never cross its deck; levels are handled by
  // ChooseRampChannels, which ramps them through every wall.
  if (const int extra = ExtraArteryCount(seed, c); extra > 0) {
    Rng extraRng(seed ^ kExtraArteryStream);
    extraRng.Float(0.0f, 1.0f);  // The count draw (ExtraArteryCount).
    std::vector<size_t> avenues;
    for (size_t i = 0; i < graph.cuts.size(); ++i) {
      const StreetCut& cut = graph.cuts[i];
      if (cut.cls != UrbanStreetClass::Avenue || cut.terrainBoundary) continue;
      const float offset = std::fabs(glm::dot(cut.origin - artery.origin, artery.normal));
      if (offset < half - RimWidth(c) - c.arteryWidth) avenues.push_back(i);
    }
    for (int k = 0; k < extra && !avenues.empty(); ++k) {
      const size_t pick = static_cast<size_t>(extraRng.Int(0, static_cast<int>(avenues.size()) - 1));
      StreetCut& cut = graph.cuts[avenues[pick]];
      cut.width = std::max(cut.width, c.arteryWidth * 0.75f);
      cut.secondaryArtery = true;
      avenues.erase(avenues.begin() + pick);
    }
  }
  ChooseRampChannels(seed, c, plan, terrainPlan, graph.cuts);
  // Trimmed against the channels; a channel left without its avenue over
  // the ramp is dropped and the streets trimmed again without it, so the
  // avenue then stops short of the band like any other street.
  for (;;) {
    for (StreetCut& cut : graph.cuts) {
      cut.gaps.clear();
      cut.blocked.clear();
    }
    TrimStreetsAtWalls(c, plan, terrain, &graph.cuts);
    if (!DropOrphanChannels(terrainPlan, graph.cuts)) break;
  }
  return graph;
}

// Block pass. Convex polygon blocks: the rim rectangle is split by the
// artery corridor and every street cut. Each surviving region is a block
// whose shape follows whatever angles the streets around it happen to make.
std::vector<UrbanBlock> BuildBlocks(uint32_t seed, const MapGeneratorConfig& c,
                                    const HighwayPlan& plan, const TerrainPlan& terrain,
                                    StreetGraph* graph) {
  const ArterySpec& artery = plan.artery;
  const float half = UrbanMapHalfExtent(c);
  const float rim = RimWidth(c);
  const float lo = -half + rim, hi = half - rim;
  const std::vector<glm::vec2> rimRect = {{lo, lo}, {hi, lo}, {hi, hi}, {lo, hi}};
  const float arteryD = glm::dot(artery.normal, artery.origin);

  std::vector<UrbanBlock> blocks;
  // Chords a trimmed street was carved along beyond its existing stretch
  // (see the last pass below), paved back afterwards up to its obstacles.
  struct Ghost {
    size_t cut;
    float r0, r1;
  };
  std::vector<Ghost> ghosts;
  for (const int side : {1, -1}) {
    const glm::vec2 n = artery.normal * static_cast<float>(side);
    std::vector<std::vector<glm::vec2>> regions;
    std::vector<glm::vec2> base =
        KeepSide(rimRect, n, static_cast<float>(side) * arteryD + ArteryBand(c));
    if (base.size() >= 3) regions.push_back(std::move(base));
    // Through streets first, then the ones a level cut short (they stop at
    // junctions the first batch already made).
    std::vector<const StreetCut*> trimmed;
    for (const StreetCut& cut : graph->cuts) {
      if (cut.side != side) continue;
      if (cut.gaps.empty()) {
        CutStreet(&regions, cut.origin, cut.dir, cut.width);
      } else {
        trimmed.push_back(&cut);
      }
    }
    // A trimmed street splits only the regions its existing stretch spans;
    // a region over its gap stays one block. A region the stretch crosses
    // only partly waits for another street (itself trimmed) to split it
    // first. The last pass settles a stretch still dead-ending inside a
    // region: it stopped at a junction with a street that itself ends
    // there (an L, not a T), so nothing else will free it. The region is
    // split along the whole line and the street paved back over that chord
    // up to its obstacle -- a dead end just short of the level or ramp,
    // never a street poking into a block. Re-applying a street is
    // harmless: a split region lies off the band.
    for (int pass = 0; pass < 4; ++pass) {
      for (const StreetCut* cut : trimmed) {
        std::vector<std::vector<glm::vec2>> next;
        for (auto& region : regions) {
          const glm::vec2 n(-cut->dir.y, cut->dir.x);
          const float d = glm::dot(n, cut->origin);
          std::vector<glm::vec2> chord = KeepSide(region, n, d - cut->width * 0.5f);
          chord = KeepSide(chord, -n, -(d + cut->width * 0.5f));
          if (chord.size() < 3 || PolygonArea(chord) <= 1.0f) {
            next.push_back(std::move(region));
            continue;
          }
          float r0 = std::numeric_limits<float>::infinity(), r1 = -r0;
          for (const glm::vec2& p : chord) {
            const float t = glm::dot(p - cut->origin, cut->dir);
            r0 = std::min(r0, t);
            r1 = std::max(r1, t);
          }
          float keptLength = 0.0f;
          for (const auto& k : cut->Kept(r0, r1)) keptLength += k.second - k.first;
          const float tol = 0.05f;
          const bool whole = keptLength >= (r1 - r0) - tol;
          const bool settle = pass == 3 && keptLength >= 1.0f;
          if (whole || settle) {
            if (!whole) ghosts.push_back({static_cast<size_t>(cut - graph->cuts.data()), r0, r1});
            std::vector<std::vector<glm::vec2>> one;
            one.push_back(std::move(region));
            CutStreet(&one, cut->origin, cut->dir, cut->width);
            for (auto& piece : one) next.push_back(std::move(piece));
          } else {
            next.push_back(std::move(region));  // Over the gap, or waiting.
          }
        }
        regions = std::move(next);
      }
    }
    for (auto& region : regions) {
      if (PolygonArea(region) < c.minPolygonArea) continue;
      blocks.push_back(MakeUrbanBlock(std::move(region)));
    }
  }
  // The ghost chords, paved again outside the street's obstacles.
  for (const Ghost& g : ghosts) {
    StreetCut& cut = graph->cuts[g.cut];
    std::vector<std::pair<float, float>> paved;
    float from = g.r0;
    for (const auto& b : cut.blocked) {
      if (b.second <= from) continue;
      if (b.first >= g.r1) break;
      if (b.first > from) paved.emplace_back(from, b.first);
      from = std::max(from, b.second);
    }
    if (from < g.r1) paved.emplace_back(from, g.r1);
    for (const auto& p : paved) {
      std::vector<std::pair<float, float>> gaps;
      for (const auto& gap : cut.gaps) {
        if (gap.second <= p.first || gap.first >= p.second) {
          gaps.push_back(gap);
          continue;
        }
        if (gap.first < p.first) gaps.emplace_back(gap.first, p.first);
        if (gap.second > p.second) gaps.emplace_back(p.second, gap.second);
      }
      cut.gaps = std::move(gaps);
    }
  }

  // A local street trimmed by terrain can leave what would have been two or
  // three ordinary blocks joined into one long polygon. Half become rows of
  // apartment slabs; the rest are split back into 2-3 blocks along a short
  // local street segment (one StreetCut limited to the old block's extent).
  Rng longBlockRng(seed ^ kLongBlockStream);
  std::vector<UrbanBlock> refined;
  for (UrbanBlock& block : blocks) {
    BlockAxis shape;
    if (!IsLongBlock(block, c, &shape)) {
      refined.push_back(std::move(block));
      continue;
    }
    // A row needs room for at least three decent slabs; a block that can't
    // hold them is split or left ordinary.
    std::vector<glm::vec2> rowOuter = InsetConvexPolygon(block.vertices, c.sidewalkWidth);
    const bool rowViable =
        rowOuter.size() >= 3 && !ApartmentRowSlabs(rowOuter, c, 0).empty() &&
        !ApartmentRowSlabs(rowOuter, c, 1).empty();  // Either lean direction.
    const bool wantsRow = longBlockRng.Chance(0.5f);
    const bool apartmentRow = wantsRow && rowViable;
    // A row that was wanted but is too tight becomes two blocks: one left
    // empty, the other built up as an ordinary block.
    const bool tight = wantsRow && !rowViable;
    if (apartmentRow) {
      block.apartmentRow = rowViable;
      refined.push_back(std::move(block));
      continue;
    }

    const float length = shape.hi - shape.lo;
    const int count =
        tight ? 2 : std::clamp(static_cast<int>(std::lround(length / c.blockSize)), 2, 3);
    const size_t emptyPiece = tight ? (longBlockRng.Chance(0.5f) ? 0 : 1) : count;
    std::vector<std::vector<glm::vec2>> pieces = {block.vertices};
    const glm::vec2 cutDirection(-shape.direction.y, shape.direction.x);
    const float splitWidth = c.localStreetWidth;
    std::vector<glm::vec2> splitOrigins;
    // A street never runs into a level: a split whose ends would reach a
    // wall strip (a block lying along a level) keeps the apartment row.
    bool clear = true;
    for (int i = 1; i < count; ++i) {
      const float at = shape.lo + length * static_cast<float>(i) / static_cast<float>(count);
      const glm::vec2 origin = shape.direction * at;
      float t0 = std::numeric_limits<float>::infinity(), t1 = -t0;
      for (const glm::vec2& v : block.vertices) {
        const float t = glm::dot(v - origin, cutDirection);
        t0 = std::min(t0, t);
        t1 = std::max(t1, t);
      }
      for (const StreetCut& wall : graph->cuts) {
        if (!wall.terrainBoundary) continue;
        const glm::vec2 n(-wall.dir.y, wall.dir.x);
        for (const float t : {t0, t1}) {
          const float d = std::fabs(glm::dot(origin + cutDirection * t - wall.origin, n));
          if (d < wall.width * 0.5f + splitWidth * 0.5f + 0.5f) clear = false;
        }
      }
      // ...and the road is level: both ends stand on one tier.
      // (sampled along it and a little past the block, where the street
      // meets its neighbours' corridors).
      const float groundMid = UrbanGroundAt(plan, terrain, c, origin + cutDirection * (0.5f * (t0 + t1)));
      for (const float t : {t0 - 4.0f, t0 - 1.0f, t0, t0 + 1.0f, t1 - 1.0f, t1, t1 + 1.0f, t1 + 4.0f}) {
        if (std::fabs(UrbanGroundAt(plan, terrain, c, origin + cutDirection * t) - groundMid) > 0.05f) {
          clear = false;
        }
      }
      splitOrigins.push_back(origin);
    }
    if (!clear) {
      block.apartmentRow = rowViable;
      refined.push_back(std::move(block));
      continue;
    }
    for (const glm::vec2& origin : splitOrigins) CutStreet(&pieces, origin, cutDirection, splitWidth);
    bool usable = static_cast<int>(pieces.size()) == count;
    for (const auto& piece : pieces) usable &= PolygonArea(piece) >= c.minPolygonArea;
    if (!usable) {
      block.apartmentRow = rowViable;
      refined.push_back(std::move(block));
      continue;
    }
    // Each split becomes one real street segment, spanning the old block
    // from one bounding street to the other (nothing paved beyond it).
    glm::vec2 centroid(0.0f);
    for (const glm::vec2& v : block.vertices) centroid += v;
    centroid /= static_cast<float>(block.vertices.size());
    const int side = glm::dot(centroid - artery.origin, artery.normal) >= 0.0f ? 1 : -1;
    for (const glm::vec2& origin : splitOrigins) {
      float t0 = std::numeric_limits<float>::infinity(), t1 = -t0;
      for (const glm::vec2& v : block.vertices) {
        const float t = glm::dot(v - origin, cutDirection);
        t0 = std::min(t0, t);
        t1 = std::max(t1, t);
      }
      StreetCut split{origin, cutDirection, splitWidth, side, UrbanStreetClass::Cross};
      split.gaps = {{-1.0e6f, t0}, {t1, 1.0e6f}};
      split.blockSplit = true;
      graph->cuts.push_back(std::move(split));
    }
    for (size_t i = 0; i < pieces.size(); ++i) {
      refined.push_back(MakeUrbanBlock(std::move(pieces[i])));
      refined.back().empty = i == emptyPiece;
    }
  }
  return refined;
}

CityPlan BuildCityPlan(uint32_t seed, const MapGeneratorConfig& c) {
  CityPlan plan;
  plan.highway = BuildHighwayPlan(seed, c, &plan.terrain);
  plan.streets = BuildStreetGraph(seed, c, plan.highway, &plan.terrain);
  plan.blocks = BuildBlocks(seed, c, plan.highway, plan.terrain, &plan.streets);
  return plan;
}

std::vector<UrbanBlock> BuildCityBlocks(uint32_t seed, const MapGeneratorConfig& c) {
  TerrainPlan terrain;
  const HighwayPlan highway = BuildHighwayPlan(seed, c, &terrain);
  StreetGraph graph = BuildStreetGraph(seed, c, highway, &terrain);
  return BuildBlocks(seed, c, highway, terrain, &graph);
}

// Ribbon edge; an end vertex sitting exactly on the map boundary slides
// along the road direction onto it so the oblique road is cut off flush
// with the edge, elevated off-map deck ends included. A deck (lift above
// grade) rides the grade under its centerline rather than under each edge:
// where it flies over a wall the edges would otherwise straddle the sharp
// step and the slab would hump or dip across it. `deckGrade` is that grade
// (null: edges follow `groundY`).
std::vector<glm::vec3> RibbonSide(const UrbanRoad& road, bool left, float half,
                                  const GroundY& groundY, const GroundY* deckGrade = nullptr,
                                  const std::vector<float>* extraHalf = nullptr) {
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
    // `extraHalf` widens the ribbon per sample (the twin-column overpass
    // deck, issue #223): zero at grade, so at-grade pavement is untouched.
    const float halfWidth = road.width * 0.5f + (extraHalf ? (*extraHalf)[i] : 0.0f);
    glm::vec2 q(p.x + sign * normal.x * halfWidth, p.z + sign * normal.y * halfWidth);
    const bool first = i == 0, last = i + 1 == road.centerline.size();
    const bool onEdge = std::fabs(std::max(std::fabs(p.x), std::fabs(p.z)) - half) < 1e-2f;
    if ((first || last) && onEdge) {
      float tMin = 0.0f, tMax = 0.0f;
      ClipLineToSquare(q, tangent, half, &tMin, &tMax);
      q += tangent * (first ? tMin : tMax);
    }
    float grade = groundY(q.x, q.y);
    if (deckGrade && p.y > kGradeY + 0.01f) {
      // A raised slab has one cross-road grade. Blending edge samples into
      // it near the foot twists the first ramp quads into visible dips.
      grade = (*deckGrade)(p.x, p.z);
    }
    side.emplace_back(q.x, grade + p.y, q.y);
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
  bool forceResidential = false;  // Detached slabs in a long apartment-row block.
  // Per-parcel draw from the layout rng (the same stream position the old
  // per-building height jitter used, so layouts never shift): picks the
  // BuildingKind from the weighted table.
  float kindRoll = -1.0f;  // Negative until drawn.
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
    if (std::fabs(probe.x) > half - rim) return UrbanStreetClass::Cross;
    best = std::fabs(std::fabs(probe.x) - (half - rim * 0.5f)) - rim * 0.5f;
  } else if (std::fabs(edgeDir.x) > kParallel) {
    if (std::fabs(probe.y) > half - rim) return UrbanStreetClass::Cross;
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
                       float height, int building, const GroundY& groundY,
                       float minArea = kMinBuildingArea) {
  if (footprint.size() < 3 || PolygonArea(footprint) < minArea) return -1;
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
  if (block.empty || (park && !block.apartmentRow)) return;
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
    if (parcel.kindRoll < 0.0f) parcel.kindRoll = rng->Unit01();
    FinishParcel(&parcel);
    out->push_back(std::move(parcel));
  };
  // Sets the parcel's frontage edge and street: the longest edge lying on the
  // building line (edges on it beat any chord), fronting the street beyond it.
  auto placeFrontage = [&](Parcel* parcel) {
    const std::vector<glm::vec2>& fp = parcel->footprint;
    float best = -1.0f;
    bool bestOnLine = false;
    for (size_t i = 0; i < fp.size(); ++i) {
      const glm::vec2 a = fp[i], b = fp[(i + 1) % fp.size()];
      const float length = glm::length(b - a);
      const bool onLine = DistanceToPolygonEdges(a, outer) < 1e-3f &&
                          DistanceToPolygonEdges(b, outer) < 1e-3f &&
                          DistanceToPolygonEdges((a + b) * 0.5f, outer) < 1e-3f;
      if (onLine && !bestOnLine) best = -1.0f;
      if ((onLine || !bestOnLine) && length > best) {
        best = length;
        bestOnLine = onLine;
        parcel->frontageEdge = static_cast<int>(i);
      }
    }
    const glm::vec2 fa = fp[parcel->frontageEdge];
    const glm::vec2 fb = fp[(parcel->frontageEdge + 1) % fp.size()];
    parcel->street = FrontageClass(city, fa, fb, inwardOf(fa, fb), c);
  };

  if (block.apartmentRow) {
    // Freestanding apartment slabs lean across the short dimension, matching
    // the long-block schematic. They sit well inside the block instead of
    // consuming its full depth, and their wide lawns/courts keep the row from
    // reading as one partitioned warehouse.
    bool any = false;
    for (std::vector<glm::vec2>& slab : ApartmentRowSlabs(outer, c, blockIndex)) {
      Parcel parcel;
      parcel.footprint = std::move(slab);
      parcel.forceResidential = true;
      placeFrontage(&parcel);
      emit(std::move(parcel));
      any = true;
    }
    if (any) return;
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
      parcel.footprint = std::move(slab);
      for (const glm::vec2& v : parcel.footprint) {
        for (const glm::vec2& o : outer) parcel.corner |= glm::distance(v, o) < 1e-3f;
      }
      placeFrontage(&parcel);
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
  // A lone piece between two merged corners would chain them into one fat
  // building; split it along the street instead so each corner gets its own.
  for (size_t e = 0; e < edges; ++e) {
    const float length = glm::distance(outer[e], outer[(e + 1) % edges]);
    if (pieces[e] == 1 && !cornerGap[e] && !cornerGap[(e + 1) % edges] && length >= 8.0f) {
      pieces[e] = 2;
      innerGap[e].assign(1, false);
    }
  }

  struct Piece {
    std::vector<glm::vec2> footprint;
    bool atStart, atEnd;  // Touches the corner at the edge's start/end.
  };
  std::vector<std::vector<Piece>> rows(edges);
  std::vector<bool> lateSplit(edges, false);
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
    // Walls beside a merged corner stay square so the corner building's hull
    // never reaches into its neighbor.
    if (!cornerGap[e]) wallSkew[1] = 0.0f;
    if (!cornerGap[next]) wallSkew[count - 1] = 0.0f;
    auto innerU = [&](float t, float skew) {
      const glm::vec2 foot = glm::mix(outer[e], outer[next], t);
      return std::clamp(glm::dot(foot - inner[e], innerDir) + skew, 0.0f, innerLen);
    };
    for (int i = 0; i < count; ++i) {
      float ta = bounds[i], tb = bounds[i + 1];
      // Corner alleys are cut square to a street below, not by trimming ends.
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
      rows[e].push_back({std::move(footprint), i == 0, i == count - 1});
    }
  }

  // Keep the part of `poly` on the side of the line (point, dir) that `keep` is on.
  auto clipToSide = [](const std::vector<glm::vec2>& poly, glm::vec2 point, glm::vec2 dir,
                       glm::vec2 keep) {
    glm::vec2 n(-dir.y, dir.x);
    if (glm::dot(keep - point, n) < 0.0f) n = -n;
    // A hair's width of separation keeps the seam from reading as overlap to
    // exact-collinearity tests.
    point += n * 0.002f;
    std::vector<glm::vec2> result;
    for (size_t k = 0; k < poly.size(); ++k) {
      const glm::vec2 a = poly[k], b = poly[(k + 1) % poly.size()];
      const float da = glm::dot(a - point, n), db = glm::dot(b - point, n);
      if (da >= 0.0f) result.push_back(a);
      if ((da >= 0.0f) != (db >= 0.0f)) result.push_back(glm::mix(a, b, da / (da - db)));
    }
    return result;
  };
  const float minArea = c.minPolygonArea * 0.18f;
  // Corner alleys: both walls are cut square to one of the two streets, so
  // they run parallel and the passage keeps its full width. (Cutting each
  // piece square to its own street leaves a wedge at non-right corners.) If
  // neither orientation leaves usable pieces, the corner merges instead.
  auto cutAlley = [&](size_t e, size_t next, bool randomFirst) {
    if (next == e || rows[e].empty() || rows[next].empty()) return false;
    if (!rows[e].back().atEnd || !rows[next].front().atStart) return false;
    std::vector<glm::vec2>& pf = rows[e].back().footprint;
    std::vector<glm::vec2>& qf = rows[next].front().footprint;
    const glm::vec2 dirs[2] = {glm::normalize(outer[next] - outer[e]),
                               glm::normalize(outer[(next + 1) % edges] - outer[next])};
    const int first = randomFirst && rng->Chance(0.5f) ? 1 : 0;
    for (int attempt = 0; attempt < 2; ++attempt) {
      const int axis = (first + attempt) % 2;
      const glm::vec2 n = dirs[axis];
      const glm::vec2 along(-n.y, n.x);
      // The piece running along `n` backs off by the alley width; the other
      // piece is only trimmed where it would reach back past the corner (an
      // acute one), so at a right angle its wall stays flush with the street.
      const float pOff = axis == 0 ? -(c.gapWidth + 0.01f) : 0.01f;
      const float qOff = axis == 0 ? -0.01f : c.gapWidth + 0.01f;
      const glm::vec2 a = outer[next] + n * pOff;
      const glm::vec2 b = outer[next] + n * qOff;
      std::vector<glm::vec2> pt = clipToSide(pf, a, along, a - n * 1000.0f);
      std::vector<glm::vec2> qt = clipToSide(qf, b, along, b + n * 1000.0f);
      if (pt.size() < 3 || qt.size() < 3 || PolygonArea(pt) < minArea ||
          PolygonArea(qt) < minArea) {
        continue;
      }
      pf = std::move(pt);
      qf = std::move(qt);
      return true;
    }
    return false;
  };
  for (size_t e = 0; e < edges; ++e) {
    const size_t next = (e + 1) % edges;
    if (cornerGap[next] && !cutAlley(e, next, true)) cornerGap[next] = false;
  }

  // A corner alley can be planned but prove too short to cut, turning into a
  // merge above. If that leaves one piece spanning two merged corners, split
  // it now so two corner buildings cannot chain into a three-sided block.
  for (size_t e = 0; e < edges; ++e) {
    const size_t next = (e + 1) % edges;
    if (cornerGap[e] || cornerGap[next] || rows[e].size() != 1 ||
        !rows[e][0].atStart || !rows[e][0].atEnd) {
      continue;
    }
    const std::vector<glm::vec2>& footprint = rows[e][0].footprint;
    const glm::vec2 frontDir = glm::normalize(outer[next] - outer[e]);
    const glm::vec2 cutDir(-frontDir.y, frontDir.x);
    const glm::vec2 cutPoint = (footprint[0] + footprint[1]) * 0.5f;
    std::vector<glm::vec2> first = clipToSide(footprint, cutPoint, cutDir, footprint[0]);
    std::vector<glm::vec2> second = clipToSide(footprint, cutPoint, cutDir, footprint[1]);
    if (first.size() < 3 || second.size() < 3 || PolygonArea(first) < minArea ||
        PolygonArea(second) < minArea) {
      continue;
    }
    rows[e].clear();
    rows[e].push_back({std::move(first), true, false});
    rows[e].push_back({std::move(second), false, true});
    lateSplit[e] = true;
  }

  // Corners without an alley are one L-shaped building wrapping the corner,
  // not two buildings meeting on the miter line. Footprints must stay convex,
  // so the L is two pieces whose seam sits on the inner wall line of the
  // corner-owning piece, away from the frontage corner: the owner grows to
  // cover the whole corner and its neighbor is trimmed back to meet it.
  std::vector<std::pair<size_t, size_t>> order;  // (edge, index) per piece.
  std::vector<std::vector<size_t>> pieceId(edges);
  for (size_t e = 0; e < edges; ++e) {
    for (size_t i = 0; i < rows[e].size(); ++i) {
      pieceId[e].push_back(order.size());
      order.push_back({e, i});
    }
  }
  std::vector<bool> trimmedStart(order.size(), false);
  std::vector<float> rolls(order.size(), -1.0f);
  // A late split replaces one RNG-planned piece, so its halves share one
  // kind draw to keep every later block on the same RNG stream. Offset the
  // second half deterministically so the neighboring Ls do not read as one.
  std::vector<int> splitGroup(order.size(), -1);
  std::vector<float> splitBaseRoll;
  std::vector<size_t> splitBasePiece;
  for (size_t e = 0; e < edges; ++e) {
    if (!lateSplit[e]) continue;
    const int group = static_cast<int>(splitBaseRoll.size());
    splitGroup[pieceId[e][0]] = group;
    splitGroup[pieceId[e][1]] = group;
    splitBaseRoll.push_back(-1.0f);
    splitBasePiece.push_back(order.size());
  }
  auto variedRoll = [](float roll) { return std::fmod(roll + 0.5f, 1.0f); };
  auto rememberSplitRoll = [&](size_t p, float h) {
    const int group = splitGroup[p];
    if (group >= 0 && splitBaseRoll[static_cast<size_t>(group)] < 0.0f) {
      splitBaseRoll[static_cast<size_t>(group)] = h;
      splitBasePiece[static_cast<size_t>(group)] = p;
    }
  };
  auto ensureRoll = [&](size_t p) {
    if (rolls[p] >= 0.0f) return;
    const int group = splitGroup[p];
    if (group < 0) {
      rolls[p] = rng->Unit01();
      return;
    }
    const size_t g = static_cast<size_t>(group);
    if (splitBaseRoll[g] < 0.0f) {
      splitBaseRoll[g] = rng->Unit01();
      splitBasePiece[g] = p;
      rolls[p] = splitBaseRoll[g];
    } else {
      rolls[p] = p == splitBasePiece[g] ? splitBaseRoll[g] : variedRoll(splitBaseRoll[g]);
    }
  };
  // Intersection parameter along `d1` from `p1` with the line (p2, d2).
  auto lineHit = [](glm::vec2 p1, glm::vec2 d1, glm::vec2 p2, glm::vec2 d2, float* t) {
    const float den = d1.x * d2.y - d1.y * d2.x;
    if (std::abs(den) < 1.0e-4f) return false;
    const glm::vec2 w = p2 - p1;
    *t = (w.x * d2.y - w.y * d2.x) / den;
    return true;
  };
  auto minimumWidth = [](const std::vector<glm::vec2>& poly) {
    float result = std::numeric_limits<float>::infinity();
    for (size_t i = 0; i < poly.size(); ++i) {
      const glm::vec2 edge = poly[(i + 1) % poly.size()] - poly[i];
      const float length = glm::length(edge);
      if (length < 1.0e-4f) continue;
      const glm::vec2 normal(-edge.y / length, edge.x / length);
      float lo = std::numeric_limits<float>::infinity();
      float hi = -lo;
      for (const glm::vec2& point : poly) {
        const float projection = glm::dot(point, normal);
        lo = std::min(lo, projection);
        hi = std::max(hi, projection);
      }
      result = std::min(result, hi - lo);
    }
    return result;
  };
  for (size_t e = 0; e < edges; ++e) {
    const size_t next = (e + 1) % edges;
    if (cornerGap[next] || next == e || rows[e].empty() || rows[next].empty()) continue;
    if (!rows[e].back().atEnd || !rows[next].front().atStart) continue;
    const size_t pid = pieceId[e].back(), qid = pieceId[next].front();
    if (pid == qid) continue;
    std::vector<glm::vec2>& pf = rows[e].back().footprint;
    std::vector<glm::vec2>& qf = rows[next].front().footprint;
    const size_t next2 = (next + 1) % edges;
    const glm::vec2 dirE = glm::normalize(inner[next] - inner[e]);
    const glm::vec2 dirN = glm::normalize(inner[next2] - inner[next]);
    const glm::vec2 frontE = glm::normalize(outer[next] - outer[e]);
    const glm::vec2 frontN = glm::normalize(outer[next2] - outer[next]);
    // The first piece owns the corner unless its far end is already trimmed
    // or the second piece is too short to give up the corner's depth.
    bool joined = false;
    for (int attempt = 0; attempt < 2 && !joined; ++attempt) {
      const bool pOwns = (attempt == 0) == !trimmedStart[pid];
      float t = 0.0f;
      if (pOwns) {
        // P's wall line runs on into the next street's frontage at G.
        if (!lineHit(outer[next], frontN, inner[e], dirE, &t) || t <= 0.0f) continue;
        if (glm::dot(qf[1] - outer[next], frontN) <= t + 0.5f) continue;
        const glm::vec2 g = outer[next] + frontN * t;
        std::vector<glm::vec2> trimmed = clipToSide(qf, inner[next], dirE, qf[1]);
        if (trimmed.size() < 3 || PolygonArea(trimmed) < minArea) continue;
        pf = {pf[0], pf[1], g, pf.back()};
        qf = std::move(trimmed);
        trimmedStart[qid] = true;
      } else {
        // Q's wall line runs back into the previous street's frontage at G.
        if (!lineHit(outer[next], -frontE, inner[next], dirN, &t) || t <= 0.0f) continue;
        if (glm::dot(outer[next] - pf[0], frontE) <= t + 0.5f) continue;
        const glm::vec2 g = outer[next] - frontE * t;
        std::vector<glm::vec2> trimmed = clipToSide(pf, inner[next], dirN, pf[0]);
        if (trimmed.size() < 3 || PolygonArea(trimmed) < minArea) continue;
        qf = {g, qf[0], qf[1], qf[2]};
        pf = std::move(trimmed);
      }
      joined = true;
    }
    if (!joined) {
      // Neither piece can give up the corner; open an alley instead of
      // leaving the two meeting at a point.
      cutAlley(e, next, false);
      continue;
    }
    // One building: the L's two halves share a kind draw.
    ensureRoll(pid);
    rolls[qid] = rolls[pid];
    rememberSplitRoll(qid, rolls[qid]);
  }
  for (size_t p = 0; p < order.size(); ++p) {
    ensureRoll(p);
    const size_t e = order[p].first;
    std::vector<glm::vec2>& footprint = rows[e][order[p].second].footprint;
    // Keep narrow temporary pieces available while resolving alleys and
    // corner merges, but do not emit the tapered slivers that can remain at
    // acute corners. Drawing their kind rolls above preserves the seeded RNG
    // stream for every later block.
    if (minimumWidth(footprint) < c.buildingDepth * 0.25f) continue;
    Parcel parcel;
    parcel.footprint = std::move(footprint);
    parcel.kindRoll = rolls[p];
    parcel.corner = (rows[e][order[p].second].atStart && !cornerGap[e]) ||
                    (rows[e][order[p].second].atEnd && !cornerGap[(e + 1) % edges]);
    placeFrontage(&parcel);
    emit(std::move(parcel));
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

// A smooth city-wide height field in [0, 1]: two crossing low-frequency waves
// with seeded direction, wavelength and phase, so neighbouring parcels land
// at similar heights and the skyline rises and falls in broad clusters.
struct SkylineField {
  glm::vec2 dir[2];
  float freq[2], phase[2];
  explicit SkylineField(Rng* rng) {
    for (int i = 0; i < 2; ++i) {
      const float angle = rng->Float(0.0f, kPi);
      dir[i] = glm::vec2(std::cos(angle), std::sin(angle));
      freq[i] = 2.0f * kPi / rng->Float(70.0f, 130.0f);
      phase[i] = rng->Float(0.0f, 2.0f * kPi);
    }
  }
  float At(glm::vec2 p) const {
    float v = 0.5f;
    for (int i = 0; i < 2; ++i) v += 0.25f * std::sin(glm::dot(dir[i], p) * freq[i] + phase[i]);
    return v;
  }
};

// Kind, floors, facade variant and setback per parcel, then the landmark
// towers (Civic, from the layout rng exactly where the old tower pick drew).
std::vector<Building> PlanBuildings(const std::vector<Parcel>& parcels,
                                    const std::vector<float>& characterRolls, uint32_t seed,
                                    const MapGeneratorConfig& c, Rng* layoutRng) {
  Rng rng(seed ^ kArchetypeStream);
  Rng skylineRng(seed ^ kSkylineStream);
  const SkylineField skyline(&skylineRng);
  std::vector<bool> highwayFrontage(characterRolls.size(), false);
  for (const Parcel& parcel : parcels) {
    if (parcel.street == UrbanStreetClass::Highway) highwayFrontage[parcel.block] = true;
  }
  std::vector<glm::vec2> blockCenter(characterRolls.size(), glm::vec2(0.0f));
  std::vector<int> blockParcels(characterRolls.size(), 0);
  for (const Parcel& parcel : parcels) {
    glm::vec2 center(0.0f);
    for (const glm::vec2& v : parcel.footprint) center += v;
    blockCenter[parcel.block] += center / static_cast<float>(parcel.footprint.size());
    ++blockParcels[parcel.block];
  }
  for (size_t i = 0; i < blockCenter.size(); ++i) {
    if (blockParcels[i] > 0) blockCenter[i] /= static_cast<float>(blockParcels[i]);
  }
  std::vector<Building> buildings;
  buildings.reserve(parcels.size());
  for (size_t i = 0; i < parcels.size(); ++i) {
    const Parcel& parcel = parcels[i];
    const BlockCharacter character =
        DrawCharacter(characterRolls[parcel.block], highwayFrontage[parcel.block], c);
    Building b;
    b.kind = parcel.forceResidential
                 ? BuildingKind::Residential
                 : DrawKind(parcel.kindRoll, parcel.street, character);
    b.parcel = static_cast<int>(i);
    b.block = parcel.block;
    DrawFloors(&b, c, &rng);
    // Residential/mixed/office parcels follow the skyline field up to the
    // config's skyline cap: low districts keep the old 2-3 floors, tall ones
    // reach 6-8, with at most a floor of jitter inside a cluster.
    if (b.kind == BuildingKind::Residential || b.kind == BuildingKind::MixedUse ||
        b.kind == BuildingKind::Office) {
      // One field sample per block (the mean of its parcels' centres), so a
      // block's buildings share a height band and neighbours stay close.
      const glm::vec2 center = blockCenter[parcel.block];
      float v = skyline.At(center) + (characterRolls[parcel.block] - 0.5f) * 0.16f;
      const BlockCharacter character =
          DrawCharacter(characterRolls[parcel.block], highwayFrontage[parcel.block], c);
      if (character == BlockCharacter::Downtown) v += 0.2f;
      if (character == BlockCharacter::Industrial) v -= 0.2f;
      v = std::clamp(v, 0.0f, 1.0f);
      const float target = c.minBuildingHeight + (c.skylineMaxHeight - c.minBuildingHeight) * v * v;
      int floors = static_cast<int>(
                       std::lround((target - b.groundFloorHeight) / b.floorHeight)) + 1;
      const float jitter = skylineRng.Float(0.0f, 1.0f);
      if (jitter < 0.2f) --floors;
      else if (jitter > 0.8f) ++floors;
      b.floorCount = std::max(2, floors);
      while (b.floorCount > 2 && b.Height() > c.skylineMaxHeight + 1e-3f) --b.floorCount;
    }
    b.style = static_cast<int>(b.kind) * 2 + (rng.Chance(0.5f) ? 1 : 0);
    // Offices step their top floor back when there is room for it.
    if (b.kind == BuildingKind::Office && b.floorCount >= 3 && rng.Chance(0.6f)) {
      b.upperObstacle = -2;  // Request; EmitBuildings resolves it (or drops it).
    }
    buildings.push_back(b);
  }

  // Landmark towers: the same two picks the old generator made.
  std::vector<size_t> candidates;
  for (size_t i = 0; i < buildings.size(); ++i) {
    if (!parcels[i].forceResidential) candidates.push_back(i);
  }
  const int towers = std::min<int>(c.towerCount, static_cast<int>(candidates.size()));
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
    // Rounding the floor count up can leave a sliver of a ground floor.
    if (b.groundFloorHeight <= 2.0f && b.floorCount > 2) {
      --b.floorCount;
      b.groundFloorHeight = height - static_cast<float>(b.floorCount - 1) * b.floorHeight;
    }
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
// terrain plan in place: avenue ramp-throughs, the along-wall stair flights
// and the wall gaps they need. Levels come from the plan's map-spanning
// steps (sharp tiers, no noise); the ground under the highway corridors
// keeps the road's own grade (ramping through a wall met at grade, stepping
// sharply under a deck that flies over).
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
  const glm::vec2 branchN(-plan.branch.dir.y, plan.branch.dir.x);

  for (TerrainWallSpec& w : terrain.walls) {
    TerrainWall wall;
    wall.origin = w.origin;
    wall.dir = w.dir;
    wall.raisedNormal = w.raisedN;
    wall.lineOrigin = w.lineOrigin;
    wall.rise = w.rise;
    const auto lineAt = [&](float t) { return w.lineOrigin + w.dir * t; };
    float tMin = 0.0f, tMax = 0.0f;
    ClipLineToSquare(w.lineOrigin, w.dir, half, &tMin, &tMax);

    // Param interval of the step line inside a band |dot(n, p) - d0| < hw;
    // an inverted pair means the line never crosses it.
    const auto bandInterval = [&](glm::vec2 n, float d0, float hw) {
      const float slope = glm::dot(n, w.dir);
      if (std::fabs(slope) < 0.15f) return std::make_pair(1.0f, -1.0f);
      const float tc = (d0 - glm::dot(n, w.lineOrigin)) / slope;
      const float ht = hw / std::fabs(slope);
      return std::make_pair(tc - ht, tc + ht);
    };
    const auto branchBand = [&](float extra) {
      auto band = bandInterval(branchN, glm::dot(branchN, plan.branch.start),
                               plan.branch.width * 0.5f + c.localStreetWidth * 0.4f + extra);
      if (plan.hasBranch && band.first < band.second) {
        const glm::vec2 q = lineAt(0.5f * (band.first + band.second));
        const float u = glm::dot(q - plan.branch.start, plan.branch.dir);
        if (u > -8.0f && u < plan.branch.length + 8.0f) return band;
      }
      return std::make_pair(1.0f, -1.0f);
    };

    // Stretches of the wall line where nothing may be placed: the rim
    // streets, the highway corridors and every street crossing. `hard`
    // holds just the rim and corridors, for a last-resort flight across a
    // dead-ending street's mouth.
    std::vector<std::pair<float, float>> blocked, hard;
    const float rim = RimWidth(c) + 2.0f;
    hard.emplace_back(tMin - 1.0f, tMin + rim);
    hard.emplace_back(tMax - rim, tMax + 1.0f);
    const auto arteryBand = bandInterval(artery.normal, glm::dot(artery.normal, artery.origin),
                                         c.arteryWidth * 0.5f + 6.0f);
    hard.push_back(arteryBand);
    {
      const auto band = branchBand(5.0f);
      if (band.first < band.second) hard.push_back(band);
    }
    blocked = hard;
    for (const StreetCut& cut : city.streets.cuts) {
      // A street cut short before this wall leaves its stretch free.
      if (!cut.gaps.empty() && std::fabs(glm::dot(w.raisedN, cut.dir)) > 1e-3f &&
          cut.InGap(w.CrossingParam(cut.origin, cut.dir))) {
        continue;
      }
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

    // Avenue ramp-throughs were chosen with the street graph: their gaps.
    for (const TerrainChannel& channel : w.channels) {
      const glm::vec2 n(-channel.along.y, channel.along.x);
      const auto gap =
          bandInterval(n, glm::dot(n, channel.foot), channel.width * 0.5f + 0.6f);
      w.gaps.push_back(gap);
      blocked.emplace_back(gap.first - 4.0f, gap.second + 4.0f);
      hard.emplace_back(gap.first - 4.0f, gap.second + 4.0f);
    }

    // Sections of the wall line between the splitters that fragment the
    // raised level: the highway corridors, ramped avenues (their flank
    // walls seal the climb's sides) and the other walls' boundary streets.
    // Each gets its own flight, so no fragment strands.
    std::vector<std::pair<float, float>> splitters;
    splitters.emplace_back(tMin - 1.0f, tMin + rim);
    splitters.emplace_back(tMax - rim, tMax + 1.0f);
    splitters.emplace_back(arteryBand.first - 2.0f, arteryBand.second + 2.0f);
    {
      const auto band = branchBand(6.0f);
      if (band.first < band.second) splitters.push_back(band);
    }
    for (const auto& gap : w.gaps) splitters.emplace_back(gap.first - 2.0f, gap.second + 2.0f);
    for (const TerrainWallSpec& other : terrain.walls) {
      if (&other == &w) continue;
      const glm::vec2 n(-other.dir.y, other.dir.x);
      const auto band = bandInterval(n, glm::dot(n, other.origin), other.cutWidth * 0.5f + 3.0f);
      if (band.first < band.second) splitters.push_back(band);
    }
    const auto sections = FreeIntervals(tMin, tMax, splitters);

    // Stairs along the cliff face: a straight flight on the low side climbs
    // parallel to the cliff, then a flat landing at the level's height. A
    // stair needs its whole stretch of the wall line free, with its foot and
    // head on usable ground: on the map, outside the highway corridors and
    // clear of every other wall's step.
    const float run = w.rise / kStairGrade;
    const float need = run + kLanding + 2.0f * kFlightMargin;
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
        if (&other != &w && std::fabs(glm::dot(other.raisedN, e) - other.stepD) < 3.5f) {
          return false;
        }
      }
      return true;
    };
    // Stretches of the line already taken by a flight, so two never collide.
    std::vector<std::pair<float, float>> used;
    const auto clear = [&](float lo, float hi, const std::vector<std::pair<float, float>>& taken) {
      for (const auto& u : taken) {
        if (lo < u.second && hi > u.first) return false;
      }
      return true;
    };
    // A flight occupying [t0, t0 + need] of the line, climbing in `dir`.
    const auto placeFlight = [&](float t0, float dir, bool lenient) {
      if (!clear(t0, t0 + need, used) || !clear(t0, t0 + need, lenient ? hard : blocked)) {
        return false;
      }
      const float tFoot = dir > 0.0f ? t0 + kFlightMargin : t0 + need - kFlightMargin;
      const float tHead = tFoot + dir * (run + kLanding * 0.5f);  // Landing center.
      const glm::vec2 foot = lineAt(tFoot) - w.raisedN * (kFlightWidth * 0.5f);
      const glm::vec2 top = lineAt(tHead) + w.raisedN * 1.2f;
      if (!usableSpot(foot) || !usableSpot(top)) return false;
      used.emplace_back(t0, t0 + need);
      w.flights.push_back(TerrainFlight{tFoot, dir, run});
      // The cliff opens over the landing so the stair lets onto the level.
      const float tLand0 = tFoot + dir * run, tLand1 = tFoot + dir * (run + kLanding);
      w.gaps.emplace_back(std::min(tLand0, tLand1) - 0.3f, std::max(tLand0, tLand1) + 0.3f);
      TerrainAccess access;
      access.width = kFlightWidth;
      access.bottom = glm::vec3(foot.x, 0.0f, foot.y);
      access.top = glm::vec3(top.x, 0.0f, top.y);
      wall.accesses.push_back(access);
      return true;
    };
    // Fragment guarantee first: every section long enough gets a flight,
    // climbing either way; a section with no clear stretch may still take
    // one across a dead-ending street's mouth (never a corridor).
    for (const auto& section : sections) {
      if (section.second - section.first < need + 1.0f) continue;
      const float firstDir = rng.Chance(0.5f) ? 1.0f : -1.0f;
      bool has = false;
      for (const bool lenient : {false, true}) {
        for (const float at : {0.5f, 0.3f, 0.7f, 0.15f, 0.85f, 0.0f, 1.0f}) {
          for (const float dir : {firstDir, -firstDir}) {
            if (has) break;
            has = placeFlight(glm::mix(section.first + 0.5f, section.second - need - 0.5f, at), dir,
                              lenient);
          }
        }
      }
    }
    // Flavor flights on the free stretches between streets, up to two per
    // artery side.
    const float tArtery = 0.5f * (arteryBand.first + arteryBand.second);
    int placedCount[2] = {0, 0};
    for (const auto& freeRun : FreeIntervals(tMin, tMax, blocked)) {
      const float length = freeRun.second - freeRun.first;
      const int sideIndex = 0.5f * (freeRun.first + freeRun.second) < tArtery ? 0 : 1;
      const bool wanted = rng.Chance(placedCount[sideIndex] ? 0.5f : 0.8f);
      const float dir = rng.Chance(0.5f) ? 1.0f : -1.0f;
      if (!wanted || placedCount[sideIndex] >= 2 || length < need + 1.0f) continue;
      const float t0 = freeRun.first + rng.Float(0.5f, length - need - 0.5f);
      if (placeFlight(t0, dir, false)) ++placedCount[sideIndex];
    }
    scene->terrainWalls.push_back(std::move(wall));
  }

  // The assembled ground: the level steps with their channels' and
  // flights' climbs, and the highway corridors' exact road grade.
  HeightField& ground = scene->ground;
  ground.minX = ground.minZ = -half;
  ground.nx = ground.nz = static_cast<int>(std::ceil(2.0f * half / kGroundStep)) + 1;
  ground.step = 2.0f * half / (ground.nx - 1);
  ground.heights.resize(static_cast<size_t>(ground.nx) * ground.nz);
  for (int iz = 0; iz < ground.nz; ++iz) {
    for (int ix = 0; ix < ground.nx; ++ix) {
      ground.heights[static_cast<size_t>(iz) * ground.nx + ix] =
          UrbanGroundAt(city, c,
                        {ground.minX + ix * ground.step, ground.minZ + iz * ground.step});
    }
  }
  for (TerrainWall& wall : scene->terrainWalls) {
    for (TerrainAccess& access : wall.accesses) {
      access.bottom.y = ground.HeightAt(access.bottom.x, access.bottom.z);
      access.top.y = ground.HeightAt(access.top.x, access.top.z);
    }
  }
  // One stair per flight: the sloped flight quad and the flat landing quad
  // at the level's height, hugging the cliff a hair off its face. The
  // renderer extrudes both down to the low ground as one wedge; movement
  // follows the same grades in the heightfield.
  for (const TerrainWallSpec& w : terrain.walls) {
    const auto lineAt = [&](float t) { return w.lineOrigin + w.dir * t; };
    const glm::vec2 inner = -w.raisedN * 0.02f, outer = -w.raisedN * kFlightWidth;
    for (const TerrainFlight& flight : w.flights) {
      const float tHead = flight.tFoot + flight.dir * flight.run;
      const float tEnd = tHead + flight.dir * kLanding;
      const float lowY = UrbanGroundAt(city, c, lineAt(flight.tFoot) + outer * 0.5f);
      const float highY = UrbanGroundAt(city, c, lineAt(0.5f * (tHead + tEnd)) + outer * 0.5f);
      const auto at = [&](float t, glm::vec2 off, float y) {
        const glm::vec2 q = lineAt(t) + off;
        return glm::vec3(q.x, y, q.y);
      };
      TerrainStair stair;
      stair.flight = {at(flight.tFoot, inner, lowY), at(flight.tFoot, outer, lowY),
                      at(tHead, outer, highY), at(tHead, inner, highY)};
      stair.landing = {at(tHead, inner, highY), at(tHead, outer, highY),
                       at(tEnd, outer, highY), at(tEnd, inner, highY)};
      stair.base = lowY;
      scene->terrainStairs.push_back(std::move(stair));
    }
  }
}

// Level edges for movement and LOS, and the openings the roads need in the
// tier tops. Every cliff line (the step lines, gapped where roads ramp
// through and at the stair landings; the sides of the ramp corridors; a
// stair's outer edge and landing end) gets an unseen nav-only wall so
// nothing walks off the edge and the cliff blocks LOS like the terrain
// does. Each ramp corridor is cut out of the tier tops, exactly the width
// of its pavement, so a level never covers a road as it climbs. The visible
// cliff faces are the sides of the tier prisms the renderer extrudes.
void EmitTerrainCliffs(Scene* scene, const CityPlan& city, const MapGeneratorConfig& c) {
  const TerrainPlan& terrain = city.terrain;
  if (!terrain.enabled) return;
  const HighwayPlan& plan = city.highway;
  const ArterySpec& artery = plan.artery;
  const float half = scene->mapHalfExtent;
  const auto groundY = [&](glm::vec2 p) { return UrbanGroundAt(city, c, p); };

  // An unseen wall from a to b: `lowOff` points to the lower side, `highOff`
  // to the upper one (both clear of the heightfield's one-cell smear). Short
  // pieces follow the ground on each side; a stretch with no drop (a ramp or
  // landing meeting the level) gets none.
  const auto navWall = [&](glm::vec2 a, glm::vec2 b, glm::vec2 lowOff, glm::vec2 highOff) {
    const float length = glm::distance(a, b);
    if (length < 0.3f) return;
    const glm::vec2 dir = (b - a) / length;
    const glm::vec2 n(-dir.y, dir.x);
    constexpr float kNavHalfWidth = 0.15f;
    const int pieces = std::max(1, static_cast<int>(std::ceil(length / 4.0f)));
    for (int i = 0; i < pieces; ++i) {
      const glm::vec2 p0 = a + dir * (length * i / pieces);
      const glm::vec2 p1 = a + dir * (length * (i + 1) / pieces);
      const glm::vec2 mid = 0.5f * (p0 + p1);
      const float base =
          std::min({groundY(p0 + lowOff), groundY(mid + lowOff), groundY(p1 + lowOff)}) - 0.15f;
      const float top = std::max({groundY(p0 + highOff), groundY(mid + highOff), groundY(p1 + highOff)});
      if (top - base < 0.45f) continue;
      std::vector<glm::vec2> footprint = {p0 - n * kNavHalfWidth, p1 - n * kNavHalfWidth,
                                          p1 + n * kNavHalfWidth, p0 + n * kNavHalfWidth};
      // Thin by design: no minimum area, or short pieces vanish and leave
      // holes in the cliff line (and the flag lands on the wrong obstacle).
      const int wall = AddPolygonBuilding(scene, footprint, 0.0f, top - base, -1,
                                          [base](float, float) { return base; }, 1e-4f);
      if (wall >= 0) scene->obstacles[wall].navOnly = true;
    }
  };

  for (const TerrainWallSpec& w : terrain.walls) {
    const auto lineAt = [&](float t) { return w.lineOrigin + w.dir * t; };
    float tMin = 0.0f, tMax = 0.0f;
    ClipLineToSquare(w.lineOrigin, w.dir, half - 0.25f, &tMin, &tMax);
    // The step line is open only where a ramp zone or a stair landing lets
    // onto the level. `w.gaps` also holds the margins that keep other things
    // clear of a ramp; those stretches are still cliffs, so they get a wall.
    std::vector<std::pair<float, float>> open;
    const auto emit = [&](float t0, float t1) {
      if (t1 - t0 < 0.1f) return;
      navWall(lineAt(t0), lineAt(t1), -w.raisedN * 0.6f, w.raisedN * 0.6f);
    };

    // A ramp corridor through this wall (an avenue's or a highway's): the
    // zone is cut out of the tier, and its two long sides are cliffs
    // between the ramp and the level beside it.
    const auto rampCut = [&](glm::vec2 onLine, glm::vec2 along, float halfWidth) {
      const std::vector<glm::vec2> zone = w.RampZone(onLine, along, halfWidth);
      scene->terrainCutouts.push_back(zone);
      const float t0 = glm::dot(zone[0] - w.lineOrigin, w.dir);
      const float t1 = glm::dot(zone[1] - w.lineOrigin, w.dir);
      open.emplace_back(std::min(t0, t1), std::max(t0, t1));
      const glm::vec2 across(-along.y, along.x);
      navWall(zone[0], zone[3], across * 0.6f, -across * 0.6f);
      navWall(zone[1], zone[2], -across * 0.6f, across * 0.6f);
    };
    for (const TerrainChannel& channel : w.channels) {
      rampCut(channel.foot, channel.along, channel.width * 0.5f);
    }
    const auto highwayCut = [&](glm::vec2 origin, glm::vec2 roadDir, float corridorHalf) {
      const glm::vec2 foot = origin + roadDir * w.CrossingParam(origin, roadDir);
      const glm::vec2 along = roadDir * (glm::dot(w.raisedN, roadDir) > 0.0f ? 1.0f : -1.0f);
      rampCut(foot, along, corridorHalf);
    };
    if (w.arteryRamp) highwayCut(artery.origin, artery.dir, c.arteryWidth * 0.5f);
    if (plan.hasBranch && w.branchRamp) {
      highwayCut(plan.branch.start, plan.branch.dir, plan.branch.width * 0.5f);
    }
    for (const TerrainFlight& flight : w.flights) {
      const float tLand0 = flight.tFoot + flight.dir * flight.run;
      const float tLand1 = flight.tFoot + flight.dir * (flight.run + kLanding);
      open.emplace_back(std::min(tLand0, tLand1) - 0.3f, std::max(tLand0, tLand1) + 0.3f);
    }
    std::sort(open.begin(), open.end());
    float from = tMin;
    for (const auto& span : open) {
      emit(from, std::min(span.first, tMax));
      from = std::max(from, span.second);
    }
    emit(from, tMax);

    // Stairs: the flight's outer edge over its climb and landing, and the
    // landing's far end, so the stair only lets off onto the level.
    for (const TerrainFlight& flight : w.flights) {
      const float tEnd = flight.tFoot + flight.dir * (flight.run + kLanding);
      navWall(lineAt(flight.tFoot) - w.raisedN * kFlightWidth, lineAt(tEnd) - w.raisedN * kFlightWidth,
              -w.raisedN * 0.6f, w.raisedN * 0.6f);
      navWall(lineAt(tEnd) - w.raisedN * kFlightWidth, lineAt(tEnd), w.dir * (flight.dir * 0.6f),
              -w.dir * (flight.dir * 0.6f));
    }
  }
}

// Pavement for a convex piece lying on one planar stretch of the ground:
// the ground at the piece's centroid and its gradient there, extrapolated
// to each vertex. A vertex on a step or ramp edge then takes this piece's
// plane rather than the neighbour's, and the patch sits exactly on the
// tier or ramp beneath it. Slivers too thin to probe inside are dropped.
template <typename Ground>
bool PlanarPatch(const std::vector<glm::vec2>& piece, const Ground& ground, RoadSurface* out) {
  if (piece.size() < 3 || PolygonArea(piece) < 1e-3f) return false;
  float width = std::numeric_limits<float>::infinity();
  for (size_t i = 0; i < piece.size(); ++i) {
    const glm::vec2 a = piece[i], e = piece[(i + 1) % piece.size()] - a;
    const float length = glm::length(e);
    if (length < 1e-5f) continue;
    float farthest = 0.0f;
    for (const glm::vec2& p : piece) {
      farthest = std::max(farthest, std::fabs(e.x * (p.y - a.y) - e.y * (p.x - a.x)) / length);
    }
    width = std::min(width, farthest);
  }
  if (width < 0.1f) return false;
  glm::vec2 center(0.0f);
  for (const glm::vec2& p : piece) center += p;
  center /= static_cast<float>(piece.size());
  constexpr float h = 0.02f;
  const float y0 = ground(center);
  const float gx = (ground(center + glm::vec2(h, 0.0f)) - ground(center - glm::vec2(h, 0.0f))) / (2.0f * h);
  const float gz = (ground(center + glm::vec2(0.0f, h)) - ground(center - glm::vec2(0.0f, h))) / (2.0f * h);
  out->vertices.clear();
  for (const glm::vec2& p : piece) {
    out->vertices.emplace_back(p.x, y0 + gx * (p.x - center.x) + gz * (p.y - center.y), p.y);
  }
  return true;
}

constexpr float kDeckRailThickness = 0.3f;

struct HighwayRibbon {
  std::vector<glm::vec3> left, right;
};

struct DeckRailJoin {
  glm::vec2 outer{0.0f};
  glm::vec2 inner{0.0f};
};

struct DeckRailJunction {
  bool valid = false;
  int arterySide = 0;
  std::array<DeckRailJoin, 2> branch;  // Left/right branch edges.
};

float Cross2(glm::vec2 a, glm::vec2 b) { return a.x * b.y - a.y * b.x; }

// Intersect the actual ribbon edges, including their height-dependent
// widening. Nominal road widths put the join off the deck over terrain.
// Offsetting both polylines also finds where the inner rail faces meet.
bool IntersectRibbonEdges(const std::vector<glm::vec3>& a, glm::vec2 aOffset,
                          const std::vector<glm::vec3>& b, glm::vec2 bOffset,
                          glm::vec2* intersection) {
  for (size_t i = 0; i + 1 < a.size(); ++i) {
    const glm::vec2 p = glm::vec2(a[i].x, a[i].z) + aOffset;
    const glm::vec2 r(a[i + 1].x - a[i].x, a[i + 1].z - a[i].z);
    for (size_t j = 0; j + 1 < b.size(); ++j) {
      const glm::vec2 q = glm::vec2(b[j].x, b[j].z) + bOffset;
      const glm::vec2 t(b[j + 1].x - b[j].x, b[j + 1].z - b[j].z);
      const float denominator = Cross2(r, t);
      if (std::fabs(denominator) < 1e-5f) continue;
      const float u = Cross2(q - p, t) / denominator;
      const float v = Cross2(q - p, r) / denominator;
      if (u < -1e-5f || u > 1.0f + 1e-5f || v < -1e-5f || v > 1.0f + 1e-5f) continue;
      *intersection = p + r * u;
      return true;
    }
  }
  return false;
}

DeckRailJunction BuildDeckRailJunction(const HighwayPlan& plan,
                                       const std::vector<HighwayRibbon>& ribbons) {
  DeckRailJunction junction;
  if (!plan.hasBranch) return junction;
  junction.arterySide = plan.branch.side;
  const auto& artery = junction.arterySide > 0 ? ribbons[0].left : ribbons[0].right;
  const glm::vec2 arteryInward = -static_cast<float>(junction.arterySide) * plan.artery.normal;
  const glm::vec2 branchNormal(-plan.branch.dir.y, plan.branch.dir.x);
  for (size_t i = 0; i < junction.branch.size(); ++i) {
    const auto& branch = i == 0 ? ribbons[1].left : ribbons[1].right;
    const glm::vec2 branchInward = (i == 0 ? -1.0f : 1.0f) * branchNormal;
    DeckRailJoin& join = junction.branch[i];
    if (!IntersectRibbonEdges(artery, glm::vec2(0.0f), branch, glm::vec2(0.0f), &join.outer) ||
        !IntersectRibbonEdges(artery, arteryInward * kDeckRailThickness,
                              branch, branchInward * kDeckRailThickness, &join.inner)) {
      return DeckRailJunction{};
    }
  }
  junction.valid = true;
  return junction;
}

// A rail stays on the span's edge. At a fork, clip its strip at the shared
// miter instead of moving an endpoint sideways to the join. Clipping also
// handles a miter crossing a sample station without folding a short quad
// back over the preceding rail. Drawing, LOS and walking use this polygon.
void EmitDeckRail(Scene* scene, WalkSurface* span, const HighwayPlan& plan,
                  const MapGeneratorConfig& c, const DeckRailJunction& junction,
                  size_t roadIndex, int edgeSide, glm::vec3 a, glm::vec3 b,
                  glm::vec2 inward) {
  const glm::vec2 outerA(a.x, a.z), outerB(b.x, b.z);
  const glm::vec2 roadDir = roadIndex == 0 ? plan.artery.dir : plan.branch.dir;
  const float length = glm::dot(outerB - outerA, roadDir);
  if (length < 1e-4f) return;
  const std::vector<glm::vec2> strip = {outerA, outerB,
                                      outerB + inward * kDeckRailThickness,
                                      outerA + inward * kDeckRailThickness};
  const auto emit = [&](std::vector<glm::vec2> footprint) {
    SimplifyPolygon(&footprint);
    if (footprint.size() < 3 || PolygonArea(footprint) < 1e-5f) return;
    if (PolygonSignedArea(footprint) < 0.0f) std::reverse(footprint.begin(), footprint.end());
    std::vector<glm::vec3> top;
    float yLo = std::numeric_limits<float>::infinity(), yHi = -yLo;
    for (const glm::vec2 corner : footprint) {
      const float y = glm::mix(a.y, b.y, glm::dot(corner - outerA, roadDir) / length);
      top.emplace_back(corner.x, y + c.deckRailHeight, corner.y);
      yLo = std::min(yLo, y);
      yHi = std::max(yHi, y);
    }
    MassingDetail rail;
    rail.kind = MassingDetail::Kind::DeckRail;
    rail.footprint = footprint;
    rail.depth = c.deckRailHeight + 0.25f;
    rail.y0 = yLo - 0.25f;
    rail.y1 = yHi + c.deckRailHeight;
    rail.top = std::move(top);
    span->blockers.push_back(footprint);
    scene->massing.push_back(std::move(rail));
  };
  const auto trim = [&](const DeckRailJoin& join, float keepDirection) {
    const glm::vec2 cap = join.inner - join.outer;
    glm::vec2 n = glm::normalize(glm::vec2(-cap.y, cap.x));
    if (glm::dot(n, roadDir) * keepDirection < 0.0f) n = -n;
    return KeepSide(strip, n, glm::dot(n, join.outer));
  };
  if (!junction.valid || (roadIndex == 0 && edgeSide != junction.arterySide)) {
    emit(strip);
  } else if (roadIndex == 1) {
    emit(trim(junction.branch[edgeSide > 0 ? 0 : 1], -1.0f));
  } else {
    const DeckRailJoin* lo = &junction.branch[0];
    const DeckRailJoin* hi = &junction.branch[1];
    if (glm::dot(lo->outer - hi->outer, roadDir) > 0.0f) std::swap(lo, hi);
    emit(trim(*lo, -1.0f));
    emit(trim(*hi, 1.0f));
  }
}

// Road pavement, elevated deck chains and their ground/branch links.
void EmitHighwaySurfaces(Scene* scene, const CityPlan& city, const MapGeneratorConfig& c,
                         PierForm pierForm, const GroundY& groundY) {
  const HighwayPlan& plan = city.highway;
  const std::vector<UrbanRoad>& roads = city.streets.roads;
  // Pavement and deck layers come from the same mitered ribbons. At-grade
  // stretches stay ordinary ground roads; everything above grade becomes an
  // explicit WalkSurface chain so the deck never erases the ground beneath.
  struct Chain {
    int first = -1, last = -1;
    std::vector<float> loS, hiS;  // Centerline param range per chain surface.
  };
  const auto splitAt = [](std::vector<std::vector<glm::vec2>>* polygons, glm::vec2 n, float d) {
    std::vector<std::vector<glm::vec2>> split;
    for (const auto& polygon : *polygons) {
      for (float side : {-1.0f, 1.0f}) {
        auto piece = KeepSide(polygon, n * side, d * side);
        if (piece.size() >= 3 && PolygonArea(piece) > 1e-4f) split.push_back(std::move(piece));
      }
    }
    *polygons = std::move(split);
  };
  std::vector<Chain> chains(roads.size());
  std::vector<HighwayRibbon> ribbons;
  for (size_t roadIndex = 0; roadIndex < roads.size(); ++roadIndex) {
    const UrbanRoad& road = roads[roadIndex];
    // Over terrain the ribbon rides the road's exact grade, not the sampled
    // heightfield: a deck flying over a wall would otherwise kink by the
    // step height where the samples smear the sharp step.
    const glm::vec2 roadDir = roadIndex == 0 ? plan.artery.dir : plan.branch.dir;
    const GroundY roadGrade = [&](float x, float z) {
      return HighwayGradeAt(city.terrain, {x, z}, roadDir, roadIndex != 0);
    };
    const GroundY& base = city.terrain.enabled ? roadGrade : groundY;
    const GroundY* deckGrade = city.terrain.enabled ? &roadGrade : nullptr;
    // Twin-column seeds (issue #223) widen the elevated deck so the paired
    // columns sit visibly under its edges. The widening scales with the
    // lift, so at-grade pavement keeps the corridor width exactly.
    std::vector<float> extraHalf;
    if (plan.elevated && pierForm == PierForm::Twin && c.twinDeckExtraWidth > 0.0f) {
      const float cruise =
          (city.terrain.enabled ? city.terrain.deckCruise : c.highwayElevation) - kGradeY;
      extraHalf.resize(road.centerline.size());
      for (size_t i = 0; i < road.centerline.size(); ++i) {
        const float f = std::min(1.0f, std::max(0.0f, (road.centerline[i].y - kGradeY) /
                                                          std::max(1.0f, cruise)));
        extraHalf[i] = 0.5f * c.twinDeckExtraWidth * f;
      }
    }
    const std::vector<float>* widen = extraHalf.empty() ? nullptr : &extraHalf;
    ribbons.push_back({RibbonSide(road, true, scene->mapHalfExtent, base, deckGrade, widen),
                       RibbonSide(road, false, scene->mapHalfExtent, base, deckGrade, widen)});
  }
  const DeckRailJunction railJunction = BuildDeckRailJunction(plan, ribbons);
  for (size_t roadIndex = 0; roadIndex < roads.size(); ++roadIndex) {
    const UrbanRoad& road = roads[roadIndex];
    const auto& left = ribbons[roadIndex].left;
    const auto& right = ribbons[roadIndex].right;
    const glm::vec2 roadDir = roadIndex == 0 ? plan.artery.dir : plan.branch.dir;
    const std::vector<float> params = roadIndex == 0
                                          ? SampleParams(plan.artery.s0, plan.artery.s1)
                                          : SampleParams(plan.branch.u0, plan.branch.length);
    Chain& chain = chains[roadIndex];
    for (size_t i = 0; i + 1 < road.centerline.size(); ++i) {
      const bool deck =
          plan.elevated &&
          std::max(road.centerline[i].y, road.centerline[i + 1].y) > kGradeY + 0.05f;
      if (!deck) {
        std::vector<std::vector<glm::vec2>> pieces = {{{left[i].x, left[i].z},
                                                        {right[i].x, right[i].z},
                                                        {right[i + 1].x, right[i + 1].z},
                                                        {left[i + 1].x, left[i + 1].z}}};
        // Split at every analytic grade boundary. Each resulting pavement
        // patch is one plane, including an oblique crossing of a level.
        for (const TerrainWallSpec& wall : city.terrain.walls) {
          const bool ramp = roadIndex == 0 ? wall.arteryRamp : wall.branchRamp;
          splitAt(&pieces, wall.raisedN, wall.stepD);
          if (ramp) splitAt(&pieces, wall.raisedN, wall.stepD + wall.rise / kRampGrade);
        }
        const glm::vec2 origin = roadIndex == 0 ? plan.artery.origin : plan.branch.start;
        if (plan.elevated) {
          const std::vector<float> profileBreaks =
              roadIndex == 0
                  ? std::vector<float>{plan.artery.rise0, plan.artery.rise1,
                                       plan.artery.fall0, plan.artery.fall1}
                  : std::vector<float>{plan.branch.riseStart, plan.branch.riseEnd};
          for (float at : profileBreaks) {
            splitAt(&pieces, roadDir, glm::dot(roadDir, origin) + at);
          }
        }
        const auto ground = [&](glm::vec2 p) {
          const float param = glm::dot(p - origin, roadDir);
          const float lift = roadIndex == 0 ? ArteryElevationAt(plan, city.terrain, c, param)
                                            : BranchElevationAt(plan, city.terrain, c, param);
          return HighwayGradeAt(city.terrain, p, roadDir, roadIndex != 0) + lift;
        };
        for (auto piece : pieces) {
          SimplifyPolygon(&piece);
          RoadSurface surface;
          if (PlanarPatch(piece, ground, &surface)) scene->roads.push_back(std::move(surface));
        }
        continue;
      }
      WalkSurface surface;
      surface.vertices = {left[i], right[i], right[i + 1], left[i + 1]};
      surface.overpass = true;
      // Raised deck sides (issue #223): guard parapets along both edges of
      // every genuinely lifted span, open across the merge junction. The
      // last half-meter of a ramp stays rail-free at its foot.
      if (c.deckRailHeight > 0.0f &&
          std::max(road.centerline[i].y, road.centerline[i + 1].y) > 0.5f) {
        const glm::vec2 inward(roadDir.y, -roadDir.x);
        EmitDeckRail(scene, &surface, plan, c, railJunction, roadIndex, 1, left[i],
                     left[i + 1], inward);
        EmitDeckRail(scene, &surface, plan, c, railJunction, roadIndex, -1, right[i],
                     right[i + 1], -inward);
      }
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
// band clipped to the map and to its side of the artery corridor, on its
// existing stretches only (a level cut the rest off). Neighbouring bands
// overlap coplanar at junctions, so the crossing is paved by both. Each
// stretch is split along every line where the ground's plane can change
// (the level steps and ramp tops, the ramp corridors' sides, the highway
// corridors' edges) and each piece is one planar patch on the exact
// ground: a ramped avenue is flat, one straight grade, flat again.
void EmitStreetSurfaces(Scene* scene, const CityPlan& city, const MapGeneratorConfig& c) {
  const ArterySpec& artery = city.highway.artery;
  const float arteryD = glm::dot(artery.normal, artery.origin);
  const float half = scene->mapHalfExtent;
  const auto splitAt = [](std::vector<std::vector<glm::vec2>>* polygons, glm::vec2 n, float d) {
    std::vector<std::vector<glm::vec2>> split;
    for (const auto& polygon : *polygons) {
      for (float side : {-1.0f, 1.0f}) {
        auto piece = KeepSide(polygon, n * side, d * side);
        if (piece.size() >= 3 && PolygonArea(piece) > 1e-4f) split.push_back(std::move(piece));
      }
    }
    *polygons = std::move(split);
  };
  struct Line {
    glm::vec2 n;
    float d;
  };
  std::vector<Line> lines;
  for (const TerrainWallSpec& wall : city.terrain.walls) {
    lines.push_back({wall.raisedN, wall.stepD});
    lines.push_back({wall.raisedN, wall.stepD + wall.rise / kRampGrade});
    for (const TerrainChannel& channel : wall.channels) {
      const glm::vec2 across(-channel.along.y, channel.along.x);
      const float sideD = glm::dot(across, channel.foot);
      lines.push_back({across, sideD - channel.width * 0.5f});
      lines.push_back({across, sideD + channel.width * 0.5f});
    }
  }
  if (city.terrain.enabled) {
    lines.push_back({artery.normal, arteryD - c.arteryWidth * 0.5f});
    lines.push_back({artery.normal, arteryD + c.arteryWidth * 0.5f});
    const float s0 = glm::dot(artery.dir, artery.origin);
    lines.push_back({artery.dir, s0 + artery.sMin - 2.0f});
    lines.push_back({artery.dir, s0 + artery.sMax + 2.0f});
    if (city.highway.hasBranch) {
      const BranchSpec& branch = city.highway.branch;
      const glm::vec2 branchN(-branch.dir.y, branch.dir.x);
      const float branchD = glm::dot(branchN, branch.start);
      lines.push_back({branchN, branchD - branch.width * 0.5f});
      lines.push_back({branchN, branchD + branch.width * 0.5f});
      const float u0 = glm::dot(branch.dir, branch.start);
      lines.push_back({branch.dir, u0 - 1.0f});
      lines.push_back({branch.dir, u0 + branch.length + 1.0f});
    }
  }
  const auto ground = [&](glm::vec2 p) { return UrbanGroundAt(city, c, p) + kGradeY; };
  for (const StreetCut& cut : city.streets.cuts) {
    if (cut.cls == UrbanStreetClass::Highway) continue;  // A highway road already.
    if (cut.terrainBoundary) continue;                   // A level band: unpaved.
    float t0, t1;
    ClipLineToSquare(cut.origin, cut.dir, half, &t0, &t1);
    if (!(t1 > t0)) continue;
    const glm::vec2 n(-cut.dir.y, cut.dir.x);
    const float hw = (cut.pave > 0.0f ? cut.pave : cut.width) * 0.5f;
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
    const float base = glm::dot(cut.dir, cut.origin);
    for (const auto& kept : cut.Kept(-1e9f, 1e9f)) {
      std::vector<glm::vec2> piece = band;
      if (kept.first > -1e8f) piece = KeepSide(piece, cut.dir, base + kept.first);
      if (kept.second < 1e8f) piece = KeepSide(piece, -cut.dir, -(base + kept.second));
      if (piece.size() < 3 || PolygonArea(piece) < 1.0f) continue;
      std::vector<std::vector<glm::vec2>> surfaces = {std::move(piece)};
      for (const Line& line : lines) splitAt(&surfaces, line.n, line.d);
      for (auto surface : surfaces) {
        SimplifyPolygon(&surface);
        RoadSurface road;
        if (PlanarPatch(surface, ground, &road)) scene->roads.push_back(std::move(road));
      }
    }
  }
}

// Bridge bents under the elevated spans (issue #223). Per the seed's drawn
// form, a bent is either one thick central column flaring into a hammerhead
// cap (TBent), or twin columns under the deck edges joined by a crossbeam
// cap (Twin). Columns and their stepped footings are real obstacles
// (navmesh, collision, LOS, cover); the cap wrapping the underside of the
// deck is overhead massing, so the ground between bents -- and between twin
// columns -- stays navigable.
void EmitPiers(Scene* scene, const CityPlan& city, const MapGeneratorConfig& c, PierForm form,
               const GroundY& groundY) {
  const HighwayPlan& plan = city.highway;
  // Bridge stands march under the high spans, leaving broad navigable
  // ground between bents. None near the on-ramp's merge, where the two
  // slabs overlap.
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
    // Oriented convex rectangle, counter-clockwise, aligned to the road.
    const auto rect = [](glm::vec2 center, glm::vec2 along, float halfAlong, float halfAcross) {
      const glm::vec2 across(-along.y, along.x);
      std::vector<glm::vec2> r = {center - along * halfAlong - across * halfAcross,
                                  center + along * halfAlong - across * halfAcross,
                                  center + along * halfAlong + across * halfAcross,
                                  center - along * halfAlong + across * halfAcross};
      if (PolygonSignedArea(r) < 0.0f) std::reverse(r.begin(), r.end());
      return r;
    };
    auto pushPier = [&](std::vector<glm::vec2> footprint, float y0, float y1) {
      if (footprint.size() < 3 || PolygonArea(footprint) < 1e-4f) return;
      AABB bounds{glm::vec3(1e9f, y0, 1e9f), glm::vec3(-1e9f, y1, -1e9f)};
      for (const glm::vec2& p : footprint) {
        bounds.min.x = std::min(bounds.min.x, p.x);
        bounds.max.x = std::max(bounds.max.x, p.x);
        bounds.min.z = std::min(bounds.min.z, p.y);
        bounds.max.z = std::max(bounds.max.z, p.y);
      }
      scene->obstacles.push_back(
          Obstacle{bounds, false, std::move(footprint), ObstacleKind::Pier});
    };
    // One bent at `center`: `deckHalf` is the road's half width, `along` its
    // direction, `deckY` the relative deck lift there.
    auto addBent = [&](glm::vec2 center, glm::vec2 along, float deckHalf, float deckY) {
      const float height = deckY - c.highwayThickness - 0.05f;
      const float shaftHalf = 0.5f * c.pierColumnWidth;
      // Footing + shaft; false where the column can't stand.
      auto addColumn = [&](glm::vec2 foot, float halfAlong, float halfAcross) {
        // Stepped footing: a plinth wider than the shaft wrapping its base.
        // Never inside a retaining wall where the deck crosses a level step.
        for (const TerrainWallSpec& w : city.terrain.walls) {
          if (std::fabs(glm::dot(w.raisedN, foot) - w.stepD) < 3.4f) return false;
        }
        if (!columnFits(foot, height)) return false;
        std::vector<glm::vec2> footing = ClipModelToMap(
            rect(foot, along, halfAlong + c.pierBaseExtra, halfAcross + c.pierBaseExtra),
            scene->mapHalfExtent);
        std::vector<glm::vec2> shaft = ClipModelToMap(
            rect(foot, along, halfAlong, halfAcross), scene->mapHalfExtent);
        if (shaft.empty()) return false;
        const float baseY = groundY(foot.x, foot.y);
        pushPier(std::move(footing), baseY,
                 baseY + std::min(c.pierBaseHeight, 0.4f * height));
        pushPier(std::move(shaft), baseY, baseY + height);
        return true;
      };
      const glm::vec2 across(-along.y, along.x);
      bool stood = false;
      float capHalfSpan = deckHalf - 0.8f;  // The T-bent hammerhead wraps most of the deck.
      if (form == PierForm::Twin) {
        const float columnSpan = deckHalf - 0.7f;
        for (const float lat : {1.0f, -1.0f}) {
          stood |= addColumn(center + across * (lat * columnSpan), shaftHalf, shaftHalf);
        }
        capHalfSpan = columnSpan + 2.0f * shaftHalf;
      } else {
        stood = addColumn(center, shaftHalf, 2.0f * shaftHalf);
      }
      if (!stood) return;
      // The cap joining the column(s) to the deck's underside: overhead
      // massing (drawn, blocks LOS), never a ground obstacle.
      const float underside = groundY(center.x, center.y) + deckY - c.highwayThickness;
      MassingDetail cap;
      cap.kind = MassingDetail::Kind::PierCap;
      cap.footprint = ClipModelToMap(
          rect(center, along, form == PierForm::Twin ? 0.7f : 1.0f, capHalfSpan),
          scene->mapHalfExtent);
      if (cap.footprint.empty()) return;
      cap.y0 = underside - c.pierCapDepth;
      cap.y1 = underside - 0.02f;
      scene->massing.push_back(std::move(cap));
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
      addBent(plan.artery.origin + plan.artery.dir * s, plan.artery.dir,
              c.arteryWidth * 0.5f, deckY);
    }
    if (plan.hasBranch) {
      for (float u = std::max(0.0f, plan.branch.riseStart); u < plan.branch.length - 8.0f;
           u += c.supportSpacing * 0.75f) {
        const float deckY = BranchElevationAt(plan, city.terrain, c, u);
        if (deckY < kMinClearance) continue;
        addBent(plan.branch.start + plan.branch.dir * u, plan.branch.dir,
                plan.branch.width * 0.5f, deckY);
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
      if (city.terrain.enabled) {
        // One plane per strip, probed inside it: a block never straddles a
        // level or a ramp, so the strip lies on one tier top. A ring vertex
        // exactly on a ramp corridor's edge then takes the tier's height,
        // not the ramp's below, and the strip stays flat beside the
        // corridor's cliff instead of twisting down into it.
        const auto ground = [&](glm::vec2 q) { return UrbanGroundAt(city, c, q) + kSidewalkHeight; };
        if (!PlanarPatch(strip, ground, &sidewalk)) continue;
      } else {
        for (const glm::vec2& p : strip) {
          sidewalk.vertices.emplace_back(p.x, groundY(p.x, p.y) + kSidewalkHeight, p.y);
        }
      }
      scene->sidewalkSurfaces.push_back(std::move(sidewalk));
    }
  }
  constexpr float kSetbackInset = 1.2f;
  // Parcels make way for the stairs climbing a terrain level: any parcel
  // overlapping an access (plus clearance) is dropped.
  std::vector<std::vector<glm::vec2>> accessClearances;
  for (const TerrainWall& wall : scene->terrainWalls) {
    for (const TerrainAccess& access : wall.accesses) {
      accessClearances.push_back(AccessFootprint(access, 1.8f));
    }
  }
  // A building needs a level foundation. Parcels keep the sidewalk inset
  // from their frontage, except that a sliver block's mitered inset can
  // leave a corner nearly on the block edge, where terrain ground already
  // steps toward a level wall or ramp channel. Such a parcel is dropped
  // rather than built on a slope. Flat cities are untouched.
  const auto dropped = [&](const Parcel& parcel) {
    for (const auto& clearance : accessClearances) {
      if (PolygonsOverlap(parcel.footprint, clearance)) return true;
    }
    if (scene->ground.Empty()) return false;
    const glm::vec2 center = PolygonCenter(parcel.footprint);
    const float base = groundY(center.x, center.y);
    for (const glm::vec2& p : parcel.footprint) {
      if (std::fabs(groundY(p.x, p.y) - base) > 0.02f) return true;
    }
    return false;
  };
  for (size_t i = 0; i < parcels.size(); ++i) {
    if (dropped(parcels[i])) continue;
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

    // Awning/canopy over mixed-use shopfronts. Commercial halls are warehouses
    // with a large loading door, which an awning would cut across.
    if (b.kind == BuildingKind::MixedUse) {
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

// --- Facade openings (issue #152, roadmap 3.4) -------------------------------

constexpr uint32_t kFacadeStream = 0x51f0a3c9u;  // Doors, stoops, platforms, window grids.

// Per-kind window pane proportions: sill above the floor, pane height, pane
// width, and the gap between panes along the facade.
struct WindowSpec {
  float sill, height, width, gap;
};
WindowSpec WindowSpecFor(BuildingKind kind) {
  switch (kind) {
    case BuildingKind::Residential: return {0.9f, 1.2f, 1.1f, 1.0f};
    case BuildingKind::MixedUse: return {0.9f, 1.2f, 1.1f, 1.0f};  // Upper floors.
    case BuildingKind::Office: return {0.5f, 2.4f, 2.6f, 0.45f};   // Full-floor ribbon glazing.
    case BuildingKind::Commercial: return {1.2f, 1.9f, 2.6f, 0.9f};  // Large factory windows.
    case BuildingKind::Civic: return {0.9f, 1.6f, 1.4f, 0.9f};
    case BuildingKind::Garage: return {1.1f, 0.8f, 2.2f, 1.6f};  // Open deck slots.
  }
  return {0.9f, 1.2f, 1.1f, 1.0f};
}

// Street entrance per kind: door size, the chance the entrance gets a raised
// stoop walk-up (instead of opening straight off the ground), and the chance
// of a rear door onto the courtyard.
struct DoorSpec {
  float width, height, walkUpChance, rearChance;
};
DoorSpec DoorSpecFor(BuildingKind kind) {
  switch (kind) {
    case BuildingKind::Residential: return {1.2f, 2.2f, 0.55f, 0.60f};
    case BuildingKind::MixedUse: return {1.6f, 2.6f, 0.10f, 0.70f};
    case BuildingKind::Office: return {1.8f, 2.6f, 0.35f, 0.40f};
    case BuildingKind::Commercial: return {5.0f, 4.2f, 0.00f, 0.55f};  // Warehouse loading bay.
    case BuildingKind::Civic: return {1.8f, 2.8f, 0.50f, 0.30f};
    case BuildingKind::Garage: return {2.6f, 2.4f, 0.00f, 0.35f};
  }
  return {1.2f, 2.2f, 0.3f, 0.5f};
}

// Evenly spaced pane u-ranges along an edge of length `len`, centered within
// the end margins. Empty when even a 0.8-width pane does not fit.
std::vector<std::pair<float, float>> FacadeColumns(float len, float w, float gap, float margin) {
  std::vector<std::pair<float, float>> columns;
  const float usable = len - 2.0f * margin;
  if (usable < 0.8f * w) return columns;
  const int count = std::max(1, static_cast<int>(std::floor((usable + gap) / (w + gap))));
  const float width = std::min(w, usable);
  const float total = count * width + (count - 1) * gap;
  float u = 0.5f * (len - total);
  for (int i = 0; i < count; ++i) {
    columns.emplace_back(u, u + width);
    u += width + gap;
  }
  return columns;
}

// Windows and doors per building, recorded as FacadeOpening data and drawn
// as pane extrusions (roadmap 3.4): a window grid per floor on every street
// facade, one entrance door on the frontage, and a rear/courtyard door for
// some (warehouses get a wide loading bay and no windows). Entrances draw a short stoop ramp walk-up for some kinds; rear doors
// are usually raised on a little platform whose access ramp sits on any of
// its three free sides (both as WalkSurfaces, so figures use them). Own rng
// stream: openings never move a wall or a zipline draw.
void EmitFacades(Scene* scene, const CityPlan& city, const std::vector<Parcel>& parcels,
                 uint32_t seed, const MapGeneratorConfig& c, const GroundY& groundY) {
  if (scene->buildings.empty()) return;
  if (c.features.facades.mode == FeatureMode::Off) return;
  Rng rng(seed ^ kFacadeStream);
  if (c.features.facades.mode == FeatureMode::Auto && !rng.Chance(c.features.facades.chance)) {
    return;
  }
  constexpr float kEndMargin = 0.7f;      // Openings keep clear of footprint corners.
  constexpr float kDoorClearance = 0.35f; // Gap between a door and its flanking windows.
  constexpr float kStoopGrade = 0.4f;     // Rise/run of stoop and platform ramps.
  constexpr float kPlatformDepth = 1.4f;
  constexpr float kPlatformChance = 0.8f; // Rear doors are usually raised.
  constexpr float kRearDoorWidth = 1.2f, kRearDoorHeight = 2.2f;
  constexpr float kWallGap = 0.03f;       // Walk surfaces stand just off the wall face.
  constexpr float kSpawnClearance = 1.5f;

  std::vector<std::vector<glm::vec2>> footprints;
  footprints.reserve(scene->obstacles.size());
  for (const Obstacle& o : scene->obstacles) footprints.push_back(ObstacleFootprint(o));
  // The building line (block outline inset by the sidewalk) per block: a
  // footprint edge lying on it faces a street and gets the window grid.
  std::vector<std::vector<glm::vec2>> buildingLines(city.blocks.size());
  for (size_t i = 0; i < city.blocks.size(); ++i) {
    buildingLines[i] = InsetConvexPolygon(city.blocks[i].vertices, c.sidewalkWidth);
  }
  const auto onLine = [](const std::vector<glm::vec2>& line, glm::vec2 p) {
    float best = std::numeric_limits<float>::infinity();
    for (size_t e = 0; e < line.size(); ++e) {
      best = std::min(best, PointSegmentDistance(p, line[e], line[(e + 1) % line.size()]));
    }
    return best < 0.03f;
  };

  // A stoop/platform/ramp footprint must sit on flat ground at the wall's
  // base, inside the map, clear of every obstacle, every walk surface that
  // is not far overhead (other stoops and platforms included), and the
  // spawn points.
  const auto clearFor = [&](const std::vector<glm::vec2>& poly, float baseY) {
    for (const glm::vec2& p : poly) {
      if (std::max(std::fabs(p.x), std::fabs(p.y)) > scene->mapHalfExtent - 0.2f) return false;
      if (std::fabs(groundY(p.x, p.y) - baseY) > 0.05f) return false;
    }
    const glm::vec2 center = PolygonCenter(poly);
    if (std::fabs(groundY(center.x, center.y) - baseY) > 0.05f) return false;
    for (const auto& footprint : footprints) {
      if (PolygonsOverlap(poly, footprint)) return false;
    }
    for (const WalkSurface& surface : scene->walkSurfaces) {
      if (surface.vertices.size() < 3) continue;
      float lowest = surface.vertices.front().y;
      for (const glm::vec3& v : surface.vertices) lowest = std::min(lowest, v.y);
      if (lowest > baseY + constants::kMinDeckHeadroom) continue;
      std::vector<glm::vec2> xz;
      for (const glm::vec3& v : surface.vertices) xz.emplace_back(v.x, v.z);
      if (PolygonsOverlap(poly, xz)) return false;
    }
    for (const Unit& unit : scene->units) {
      if (DistanceToPolygon(glm::vec2(unit.position.x, unit.position.z), poly) <
          kSpawnClearance) {
        return false;
      }
    }
    return true;
  };
  // Counter-clockwise from above, as every WalkSurface consumer expects.
  const auto pushSurface = [&](std::vector<glm::vec3> vertices, bool connectsToGround) {
    float area = 0.0f;
    for (size_t i = 0; i < vertices.size(); ++i) {
      const glm::vec3& p = vertices[i];
      const glm::vec3& q = vertices[(i + 1) % vertices.size()];
      area += p.x * q.z - q.x * p.z;
    }
    if (area < 0.0f) std::reverse(vertices.begin(), vertices.end());
    WalkSurface surface;
    surface.vertices = std::move(vertices);
    surface.connectsToGround = connectsToGround;
    scene->walkSurfaces.push_back(std::move(surface));
    return static_cast<int>(scene->walkSurfaces.size()) - 1;
  };

  for (size_t bi = 0; bi < scene->buildings.size(); ++bi) {
    const Building& b = scene->buildings[bi];
    const Parcel& parcel = parcels[b.parcel];
    const Obstacle& base = scene->obstacles[b.obstacle];
    const std::vector<glm::vec2>& fp = base.footprint;
    const size_t n = fp.size();
    const float baseY = base.bounds.min.y;
    const float baseTop = base.bounds.max.y;
    const std::vector<glm::vec2>& line = buildingLines[b.block];

    // Street facades: edges on the block's building line. The frontage is
    // one of them; corner parcels have two or more.
    std::vector<char> street(n, 0);
    for (size_t e = 0; e < n; ++e) {
      const glm::vec2 a = fp[e], q = fp[(e + 1) % n];
      street[e] = onLine(line, a) && onLine(line, q) && onLine(line, (a + q) * 0.5f);
    }
    const int frontage = parcel.frontageEdge;
    // Detached apartment rows have no wall on the street building line;
    // their selected long edge is still the principal, windowed facade.
    if (parcel.forceResidential) street[frontage] = true;
    // ...and so is the opposite long face: slabs are glazed on both sides.
    const int backEdge = parcel.forceResidential ? static_cast<int>((frontage + 2) % n) : -1;
    // Rear edge: the longest non-street edge facing away from the frontage.
    const glm::vec2 fa = fp[frontage], fb = fp[(frontage + 1) % n];
    const glm::vec2 frontDir = glm::normalize(fb - fa);
    const glm::vec2 frontOut(frontDir.y, -frontDir.x);
    int rearEdge = -1;
    float rearScore = 0.0f;
    for (size_t e = 0; e < n; ++e) {
      if (street[e] || static_cast<int>(e) == frontage) continue;
      const glm::vec2 a = fp[e], q = fp[(e + 1) % n];
      const float len = glm::distance(a, q);
      if (len < kRearDoorWidth + 2.0f * kEndMargin) continue;
      const glm::vec2 d = (q - a) / len;
      const glm::vec2 out(d.y, -d.x);
      const float facing = -glm::dot(out, frontOut);
      if (facing < 0.3f) continue;
      // The wall must actually face open ground: back-to-back slab parcels
      // touch rear walls, where a door would open straight into the
      // neighbour. Probe a thin strip just outside the edge.
      const glm::vec2 s0 = a + d * kEndMargin + out * 0.05f;
      const glm::vec2 s1 = q - d * kEndMargin + out * 0.05f;
      const std::vector<glm::vec2> strip = {s0, s1, s1 + out * 0.8f, s0 + out * 0.8f};
      bool open = true;
      for (const auto& footprint : footprints) {
        if (PolygonsOverlap(strip, footprint)) {
          open = false;
          break;
        }
      }
      if (!open) continue;
      const float score = facing * len;
      if (score > rearScore) {
        rearScore = score;
        rearEdge = static_cast<int>(e);
      }
    }

    const DoorSpec door = DoorSpecFor(b.kind);
    // Door u-ranges per edge, so the ground-floor window grid skips them.
    std::vector<glm::vec2> doorSpans(n, glm::vec2(0.0f, -1.0f));

    // Street entrance on the frontage, with a stoop walk-up for some
    // buildings and straight off the ground for the rest.
    {
      const glm::vec2 a = fp[frontage], q = fp[(frontage + 1) % n];
      const float len = glm::distance(a, q);
      const glm::vec2 dir = (q - a) / len;
      const glm::vec2 out(dir.y, -dir.x);
      const float width = std::min(door.width, len - 2.0f * kEndMargin);
      if (width >= 0.9f) {
        const float center = rng.Float(kEndMargin + width * 0.5f, len - kEndMargin - width * 0.5f);
        const bool wantStoop = rng.Chance(door.walkUpChance);
        const float stoopHeight = rng.Float(0.3f, 0.45f);
        float doorBase = baseY;
        if (wantStoop) {
          const float run =
              std::clamp(stoopHeight / kStoopGrade, 0.9f, std::max(0.9f, c.sidewalkWidth - 0.2f));
          const float half = std::min(width * 0.5f + 0.5f,
                                      std::min(center, len - center) - 0.15f);
          const glm::vec2 s0 = a + dir * (center - half) + out * kWallGap;
          const glm::vec2 s1 = a + dir * (center + half) + out * kWallGap;
          const std::vector<glm::vec2> poly = {s0, s1, s1 + out * run, s0 + out * run};
          if (clearFor(poly, baseY) &&
              baseY + stoopHeight + 1.4f <= baseY + b.groundFloorHeight - 0.15f) {
            pushSurface({{s0.x, baseY + stoopHeight, s0.y},
                         {s1.x, baseY + stoopHeight, s1.y},
                         {poly[2].x, baseY, poly[2].y},
                         {poly[3].x, baseY, poly[3].y}},
                        /*connectsToGround=*/true);
            doorBase = baseY + stoopHeight;
          }
        }
        const float head = std::min(doorBase + door.height, baseY + b.groundFloorHeight - 0.15f);
        if (head - doorBase >= 1.4f) {
          scene->facadeOpenings.push_back(FacadeOpening{
              FacadeOpening::Kind::Door, static_cast<int>(bi), b.obstacle, frontage,
              center - width * 0.5f, center + width * 0.5f, doorBase, head, 0});
          doorSpans[frontage] = glm::vec2(center - width * 0.5f, center + width * 0.5f);
        }
      }
    }

    // Rear/courtyard door for some buildings: usually raised on a little
    // platform with a ramp on one of its three free sides, otherwise (or
    // when nothing fits) straight off the ground.
    if (rearEdge >= 0 && rng.Chance(door.rearChance)) {
      const glm::vec2 a = fp[rearEdge], q = fp[(rearEdge + 1) % n];
      const float len = glm::distance(a, q);
      const glm::vec2 dir = (q - a) / len;
      const glm::vec2 out(dir.y, -dir.x);
      const float center =
          rng.Float(kEndMargin + kRearDoorWidth * 0.5f, len - kEndMargin - kRearDoorWidth * 0.5f);
      const bool wantPlatform = rng.Chance(kPlatformChance);
      const float platformHeight = rng.Float(0.4f, 0.5f);
      const int sidePick = rng.Int(0, 2);  // 0: toward fp[e], 1: away, 2: front.
      float doorBase = baseY;
      if (wantPlatform) {
        // Full-size first; a cramped courtyard (slab-row blocks, deep
        // neighbours) gets a second, compact attempt before giving up.
        const float fullHalf = std::min(kRearDoorWidth * 0.5f + 0.6f,
                                        std::min(center, len - center) - 0.15f);
        const float fullRun = std::clamp(platformHeight / kStoopGrade, 1.0f, 1.6f);
        const struct { float half, depth, run; } sizes[] = {
            {fullHalf, kPlatformDepth, fullRun},
            {std::min(fullHalf, kRearDoorWidth * 0.5f + 0.3f), 1.0f, 0.9f},
        };
        for (const auto& size : sizes) {
          const glm::vec2 p0 = a + dir * (center - size.half) + out * kWallGap;
          const glm::vec2 p1 = a + dir * (center + size.half) + out * kWallGap;
          const glm::vec2 p2 = p1 + out * size.depth;
          const glm::vec2 p3 = p0 + out * size.depth;
          const std::vector<glm::vec2> platform = {p0, p1, p2, p3};
          for (int attempt = 0; attempt < 3 && doorBase == baseY; ++attempt) {
            const int side = (sidePick + attempt) % 3;
            std::vector<glm::vec2> ramp;
            glm::vec2 top0, top1;  // The platform edge the ramp hangs off.
            if (side == 0) {
              top0 = p0, top1 = p3;
              ramp = {p0, p3, p3 - dir * size.run, p0 - dir * size.run};
            } else if (side == 1) {
              top0 = p1, top1 = p2;
              ramp = {p1, p2, p2 + dir * size.run, p1 + dir * size.run};
            } else {
              top0 = p2, top1 = p3;
              ramp = {p2, p3, p3 + out * size.run, p2 + out * size.run};
            }
            if (!clearFor(platform, baseY) || !clearFor(ramp, baseY)) continue;
            const float top = baseY + platformHeight;
            const int platformIndex = pushSurface({{p0.x, top, p0.y},
                                                   {p1.x, top, p1.y},
                                                   {p2.x, top, p2.y},
                                                   {p3.x, top, p3.y}},
                                                  /*connectsToGround=*/false);
            const int rampIndex = pushSurface({{top0.x, top, top0.y},
                                               {top1.x, top, top1.y},
                                               {ramp[2].x, baseY, ramp[2].y},
                                               {ramp[3].x, baseY, ramp[3].y}},
                                              /*connectsToGround=*/true);
            scene->walkSurfaces[platformIndex].neighbors.push_back(rampIndex);
            scene->walkSurfaces[rampIndex].neighbors.push_back(platformIndex);
            doorBase = top;
          }
          if (doorBase > baseY) break;
        }
      }
      const float head = std::min(doorBase + kRearDoorHeight, baseY + b.groundFloorHeight - 0.15f);
      if (head - doorBase >= 1.4f) {
        scene->facadeOpenings.push_back(FacadeOpening{
            FacadeOpening::Kind::Door, static_cast<int>(bi), b.obstacle, rearEdge,
            center - kRearDoorWidth * 0.5f, center + kRearDoorWidth * 0.5f, doorBase, head, 0});
        doorSpans[rearEdge] = glm::vec2(center - kRearDoorWidth * 0.5f,
                                        center + kRearDoorWidth * 0.5f);
      }
    }

    // Window grids: floorCount x facade length on every street facade, plus
    // a sparser grid on the rear wall's upper floors. Floors above the base
    // prism (a set-back office top) keep their walls blank.
    const WindowSpec spec = WindowSpecFor(b.kind);
    // Commercial halls are warehouses: no windows, just the wide loading
    // entrance on the frontage. Mixed-use shops stay glazed.
    if (b.kind == BuildingKind::Commercial) continue;
    const bool shopfront = b.kind == BuildingKind::MixedUse;
    const bool rearWindows =
        b.kind == BuildingKind::Residential || b.kind == BuildingKind::MixedUse ||
        b.kind == BuildingKind::Office || b.kind == BuildingKind::Civic;
    for (size_t e = 0; e < n; ++e) {
      const bool rear = static_cast<int>(e) == rearEdge && rearWindows;
      const bool back = static_cast<int>(e) == backEdge && rearWindows;
      if (!street[e] && !rear && !back) continue;
      const glm::vec2 a = fp[e], q = fp[(e + 1) % n];
      const float len = glm::distance(a, q);
      const glm::vec2 doorSpan = doorSpans[e];
      for (int floor = 0; floor < b.floorCount; ++floor) {
        const float floorBase =
            floor == 0 ? baseY : baseY + b.groundFloorHeight + (floor - 1) * b.floorHeight;
        const float floorTop = floorBase + (floor == 0 ? b.groundFloorHeight : b.floorHeight);
        if (floorTop > baseTop + 0.01f) break;
        if (rear && floor == 0) continue;  // Courtyard ground floors stay blank.
        float sill = spec.sill, head = spec.sill + spec.height, w = spec.width, gap = spec.gap;
        if (floor == 0 && shopfront && street[e]) {
          // Shopfront glazing: sill near the floor, panes nearly full height.
          sill = 0.4f;
          head = std::min(b.groundFloorHeight - 0.7f, 3.2f);
          w = 2.0f;
          gap = 0.35f;
        }
        if (rear && !back) gap *= 2.0f;  // Sparser on the courtyard side.
        const float y0 = floorBase + sill;
        const float y1 = std::min(floorBase + head, floorTop - 0.2f);
        if (y1 - y0 < 0.4f) continue;
        for (const auto& [u0, u1] : FacadeColumns(len, w, gap, kEndMargin)) {
          if (floor == 0 && doorSpan.y > doorSpan.x && u1 > doorSpan.x - kDoorClearance &&
              u0 < doorSpan.y + kDoorClearance) {
            continue;  // The entrance takes this bay.
          }
          scene->facadeOpenings.push_back(FacadeOpening{
              FacadeOpening::Kind::Window, static_cast<int>(bi), b.obstacle,
              static_cast<int>(e), u0, u1, y0, y1, floor});
        }
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
  if (cut.blockSplit && cut.gaps.size() == 2) {
    *t0 = std::max(*t0, cut.gaps[0].second);
    *t1 = std::min(*t1, cut.gaps[1].first);
  }
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
    if (ArteryElevationAt(city.highway, city.terrain, c, arteryS) > kGradeY + 0.05f) continue;
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
      const bool place = rng.Chance(0.4f);
      if (!place) continue;
      const glm::vec2 inward = glm::normalize(centroid - poly[v]);
      const glm::vec2 nextEdge = glm::normalize(poly[(v + 1) % poly.size()] - poly[v]);
      const glm::vec2 previousEdge =
          glm::normalize(poly[v] - poly[(v + poly.size() - 1) % poly.size()]);
      add(StreetPropKind::StreetNameSign, poly[v] + inward * 0.5f,
          std::atan2(nextEdge.y, nextEdge.x), std::atan2(previousEdge.y, previousEdge.x));
    }
  }
}

void EmitTrees(Scene* scene, const std::vector<UrbanBlock>& blocks,
               uint32_t seed, const MapGeneratorConfig& c) {
  if (c.features.trees.mode == FeatureMode::Off) return;
  Rng rng(seed ^ 0x74ee270bu);
  for (const UrbanBlock& block : blocks) {
    const auto& polygon = block.vertices;
    if (polygon.size() < 3) continue;
    const auto blockInterior = InsetConvexPolygon(polygon, c.sidewalkWidth);
    if (blockInterior.size() < 3) continue;
    // Test the entire block, not just a candidate point: courtyards and gaps
    // in an otherwise built-up block must stay bare. Include piers, furniture
    // obstacles and walk surfaces. Corner traffic signs belong to the wrapping
    // sidewalk; they don't occupy the otherwise empty block interior.
    bool occupied = false;
    for (const Obstacle& o : scene->obstacles) {
      // EmitTrafficControl gives every corner sign a tiny collision pole,
      // including on park blocks. Keep that existing curb detail intact.
      const bool cornerSign = o.kind == ObstacleKind::StreetFurniture &&
          std::any_of(scene->props.begin(), scene->props.end(), [&](const StreetProp& p) {
            return p.kind != StreetPropKind::LampPost &&
                   p.position.x == o.bounds.Center().x && p.position.z == o.bounds.Center().z &&
                   !PointInConvexPolygon({p.position.x, p.position.z}, blockInterior);
          });
      if (cornerSign) continue;
      // Footprints clipped to the map can be slightly non-convex, which makes
      // the half-plane overlap test miss; also treat any vertex inside the
      // block as occupying it.
      const std::vector<glm::vec2> footprint = ObstacleFootprint(o);
      if (PolygonsOverlap(polygon, footprint) ||
          std::any_of(footprint.begin(), footprint.end(), [&](const glm::vec2& v) {
            return PointInConvexPolygon(v, polygon);
          })) {
        occupied = true;
        break;
      }
    }
    for (const StreetProp& p : scene->props)
      if (PointInConvexPolygon({p.position.x, p.position.z}, blockInterior)) occupied = true;
    for (const WalkSurface& s : scene->walkSurfaces) {
      std::vector<glm::vec2> footprint;
      for (const glm::vec3& v : s.vertices) footprint.emplace_back(v.x, v.z);
      if (PolygonSignedArea(footprint) < 0.0f) std::reverse(footprint.begin(), footprint.end());
      if (footprint.size() >= 3 && PolygonsOverlap(polygon, footprint)) occupied = true;
    }
    if (occupied) continue;

    const size_t first = scene->trees.size();
    const int count = rng.Int(10, 16);
    for (int attempt = 0; attempt < 600 && scene->trees.size() < first + count; ++attempt) {
      TreeDecoration tree;
      tree.scale = rng.Float(0.75f, 1.1f);
      const float radius = kTreeCanopyRadius * tree.scale;
      // Sequence RNG draws explicitly: argument evaluation order differs
      // between native compilers and Emscripten.
      const float x = rng.Float(block.x0, block.x1);
      const float z = rng.Float(block.z0, block.z1);
      const glm::vec2 p(x, z);
      if (!PointInConvexPolygon(p, polygon)) continue;
      bool clear = true;
      // Measure the original edges directly: offset-line intersections can
      // fold back inside a very narrow/sliver block when the inset is too large.
      for (size_t e = 0; e < polygon.size(); ++e)
        clear &= PointSegmentDistance(p, polygon[e], polygon[(e + 1) % polygon.size()]) >=
                 c.sidewalkWidth + radius + 0.25f;
      for (size_t i = first; i < scene->trees.size(); ++i) {
        const TreeDecoration& other = scene->trees[i];
        clear &= glm::distance(p, glm::vec2(other.position.x, other.position.z)) >
                 0.45f * (radius + kTreeCanopyRadius * other.scale);  // Crowns overlap.
      }
      // A cable may cross an empty block even if both anchors are outside it.
      for (const Zipline& line : scene->ziplines) {
        const glm::vec2 a(line.a.x, line.a.z), b(line.b.x, line.b.z);
        const glm::vec2 d = b - a;
        const float t = glm::clamp(glm::dot(p - a, d) / std::max(glm::dot(d, d), 1e-6f),
                                    0.0f, 1.0f);
        clear &= glm::distance(p, a + d * t) > radius + 0.4f;
      }
      if (!clear) continue;
      float y = scene->ground.HeightAt(p.x, p.y);
      // Legacy empty lots are paved across their whole interior.
      for (const AABB& slab : scene->sidewalks) {
        if (p.x >= slab.min.x && p.x <= slab.max.x && p.y >= slab.min.z && p.y <= slab.max.z)
          y = std::max(y, slab.max.y);
      }
      tree.position = {p.x, y, p.y};
      tree.variant = rng.Int(0, 2);
      tree.yaw = rng.Float(-kPi, kPi);
      tree.tint = rng.Float(0.88f, 1.12f);
      scene->trees.push_back(tree);
      // Solid and opaque from the ground to the crown top: blocks movement,
      // LOS and FOV, never drawn (the instanced tree is the visual).
      Obstacle solid;
      const float r = 1.4f * tree.scale;
      for (int i = 0; i < 8; ++i) {
        const float a = 2.0f * kPi * static_cast<float>(i) / 8.0f;
        solid.footprint.emplace_back(p.x + r * std::cos(a), p.y + r * std::sin(a));
      }
      solid.bounds.min = {p.x - r, y, p.y - r};
      solid.bounds.max = {p.x + r, y + 5.0f * tree.scale, p.y + r};
      solid.navOnly = true;
      scene->obstacles.push_back(std::move(solid));
    }
  }
}

Scene GenerateArterialUrbanMap(uint32_t seed, const MapGeneratorConfig& c) {
  Rng rng(seed);
  Scene scene;
  scene.mapHalfExtent = UrbanMapHalfExtent(c);
  CityPlan city = BuildCityPlan(seed, c);
  BuildUrbanTerrain(&scene, &city, seed, c);
  const GroundY groundY = [&](float x, float z) { return scene.ground.HeightAt(x, z); };
  const PierForm pierForm = ResolvePierForm(seed, c);
  EmitHighwaySurfaces(&scene, city, c, pierForm, groundY);
  EmitStreetSurfaces(&scene, city, c);
  EmitPiers(&scene, city, c, pierForm, groundY);
  std::vector<float> characterRolls;
  const std::vector<Parcel> parcels = PlanParcels(city, c, &rng, &characterRolls);
  EmitBuildings(&scene, city, parcels, PlanBuildings(parcels, characterRolls, seed, c, &rng), c,
                groundY);
  EmitSpawns(&scene, city, c, groundY);
  // Facades go before the ziplines so an anchor or cable never lands on a
  // stoop or rear platform (their clearance pass checks the walk surfaces).
  EmitFacades(&scene, city, parcels, seed, c, groundY);
  const size_t roofCount = scene.obstacles.size();
  EmitTerrainCliffs(&scene, city, c);
  // Furniture before ziplines: anchors and cables then keep clear of it.
  EmitStreetFurniture(&scene, city, c, seed, groundY);
  EmitTrafficControl(&scene, city, c, seed, groundY);
  PlaceZiplines(&scene, city.blocks, seed, c, roofCount);
  EmitMassing(&scene, parcels, seed);
  EmitTrees(&scene, city.blocks, seed, c);
  return scene;
}

}  // namespace

FeatureToggle* UrbanFeatureByName(UrbanFeatures* f, const std::string& name) {
  if (name == "trees") return &f->trees;
  if (name == "lanes") return &f->lanes;
  if (name == "curbs") return &f->curbs;
  if (name == "furniture") return &f->furniture;
  if (name == "terrain") return &f->terrain;
  if (name == "interiors") return &f->interiors;
  if (name == "rail") return &f->rail;
  if (name == "facades") return &f->facades;
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
  auto roads = BuildUrbanRoads(seed, c);
  TerrainPlan terrain;
  const HighwayPlan plan = BuildHighwayPlan(seed, c, &terrain);
  if (terrain.enabled) {
    // The road's exact grade (what the ground under its corridor follows),
    // so a deck stays smooth where it flies over a sharp step.
    for (size_t i = 0; i < roads.size(); ++i) {
      for (auto& p : roads[i].centerline) {
        p.y += HighwayGradeAt(terrain, {p.x, p.z}, i == 0 ? plan.artery.dir : plan.branch.dir,
                              i != 0);
      }
    }
  }
  return roads;
}

std::vector<UrbanStreet> UrbanStreets(uint32_t seed, const MapGeneratorConfig& c) {
  std::vector<UrbanStreet> out;
  if (c.arteryCount <= 0) return out;
  // The plan the scene is built from, trimmed and settled against its
  // blocks, so the streets reported are exactly the ones paved.
  CityPlan city = BuildCityPlan(seed, c);
  const HighwayPlan& plan = city.highway;
  const StreetGraph& graph = city.streets;
  for (const UrbanRoad& road : UrbanRoads(seed, c)) {
    for (size_t i = 0; i + 1 < road.centerline.size(); ++i) {
      out.push_back({road.centerline[i], road.centerline[i + 1], road.width,
                     UrbanStreetClass::Highway, road.elevated});
    }
  }
  const float half = UrbanMapHalfExtent(c);
  Scene terrain;
  terrain.mapHalfExtent = half;
  if (c.features.terrain.mode != FeatureMode::Off) BuildUrbanTerrain(&terrain, &city, seed, c);
  for (const StreetCut& cut : graph.cuts) {
    if (cut.cls == UrbanStreetClass::Highway) continue;  // Already a road above.
    if (cut.terrainBoundary) continue;                   // A level band, not a street.
    float t0, t1;
    if (!CutPavedExtent(cut, plan, c, &t0, &t1)) continue;  // Its own side only.
    for (const auto& kept : cut.Kept(t0, t1)) {
      if (!(kept.second > kept.first)) continue;
      const glm::vec2 a = cut.origin + cut.dir * kept.first, b = cut.origin + cut.dir * kept.second;
      out.push_back({glm::vec3(a.x, terrain.ground.HeightAt(a.x, a.y), a.y),
                     glm::vec3(b.x, terrain.ground.HeightAt(b.x, b.y), b.y),
                     cut.pave > 0.0f ? cut.pave : cut.width, cut.cls, false});
    }
  }
  return out;
}

OverpassChoice UrbanOverpass(uint32_t seed, const MapGeneratorConfig& c) {
  if (c.arteryCount <= 0) return OverpassChoice{};
  return ResolveOverpass(seed, c);
}

PierForm UrbanPierForm(uint32_t seed, const MapGeneratorConfig& c) {
  return ResolvePierForm(seed, c);
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

namespace {
Scene GenerateUrbanMapImpl(uint32_t seed, const MapGeneratorConfig& c) {
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
}  // namespace

Scene GenerateUrbanMap(uint32_t seed, const MapGeneratorConfig& c) {
  Scene scene = GenerateUrbanMapImpl(seed, c);
  if (c.arteryCount == 0 && c.features.trees.mode != FeatureMode::Off) {
    std::vector<UrbanBlock> emptyBlocks;
    for (const UrbanLot& lot : UrbanLots(seed, c)) {
      if (!lot.empty) continue;
      emptyBlocks.push_back({lot.x0, lot.x1, lot.z0, lot.z1,
                             {{lot.x0, lot.z0}, {lot.x1, lot.z0},
                              {lot.x1, lot.z1}, {lot.x0, lot.z1}}});
    }
    EmitTrees(&scene, emptyBlocks, seed, c);
  }
  SuggestSpawnHints(&scene);
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

namespace {

Scene GenerateHillyMapImpl(uint32_t seed, const HillyMapConfig& c) {
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

}  // namespace

Scene GenerateHillyMap(uint32_t seed, const HillyMapConfig& c) {
  Scene scene = GenerateHillyMapImpl(seed, c);
  SuggestSpawnHints(&scene);
  return scene;
}

}  // namespace tactics
