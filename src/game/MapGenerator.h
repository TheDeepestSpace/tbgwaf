#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "game/Scene.h"

namespace tactics {

// Overpass presence (issue #130). Auto lets the seed decide (overpassChance
// of an overpass per map); Off/On force it, as the `urban`/`urban-elevated`
// map types and the scenario `elevated:` key do.
enum class OverpassMode { Auto, Off, On };

// Deck profile of the overpass, when present. Auto draws one per seed;
// forcing a concrete layout pins scenarios/goldens to one shape. Off-map
// ends extend past the map bounds so the deck reads as continuing rather
// than stopping in a stub at the edge.
enum class OverpassLayout {
  Auto,
  RampUpRampDown,  // Both ramps on-map: up from grade, deck, back down.
  Through,         // Enters and exits off-map at deck height; no artery ramps.
  EnterRampUp,     // At grade on-map, ramps up, exits off-map elevated.
  EnterRampDown,   // Enters off-map elevated, ramps down to grade on-map.
};

// Column form of the bents carrying the overpass (issue #223). Auto draws
// one per seed (own sub-stream, so forcing it never reshuffles the rest of
// a seed); TBent is one thick central column flaring into a hammerhead cap
// under the deck, Twin is a pair of columns under the deck edges joined by
// a crossbeam cap, with the deck widened so the columns visibly carry it.
enum class PierForm { Auto, TBent, Twin };

// How the overpass's branching avenue ends at its map-edge side. Auto draws
// per seed; Ramp climbs from grade at the boundary up to the deck, OffMap
// stays at deck height and runs off the map with no ramp (never drawn for
// the Through layout, where the branch is the only way up).
enum class BranchEnd { Auto, Ramp, OffMap };

// Per-feature switch for the optional urban detail layers (issue #146).
// Auto lets the seed decide, with `chance` as the per-seed probability; Off/On
// force it. Terrain defaults to On (levels are part of every arterial city;
// issue #155). `furniture` is live (issue #150) but unconditional: Auto behaves
// like On, only Off disables it. It gates the street furniture obstacles along
// the sidewalk curbs and the visual-only traffic lights and signage
// (Scene::props). The remaining detail flags are reserved for their emitters.
enum class FeatureMode { Auto, Off, On };
struct FeatureToggle {
  FeatureMode mode = FeatureMode::Auto;
  float chance = 0.5f;
};
struct UrbanFeatures {
  FeatureToggle lanes;
  FeatureToggle curbs;
  FeatureToggle furniture;
  FeatureToggle terrain{FeatureMode::On, 0.5f};
  FeatureToggle interiors;
  FeatureToggle rail;
  // Facade openings (issue #152): windows/doors as decals, entrance stoops
  // and raised rear-door platforms. On by default; Off keeps the walls bare
  // (and adds no stoop/platform walk surfaces).
  FeatureToggle facades{FeatureMode::On, 0.5f};
  // Visual-only trees on sidewalk-only blocks. Auto behaves like On.
  FeatureToggle trees{FeatureMode::On, 0.5f};
};

// Name lookups for scenario YAML / env plumbing. UrbanFeatureByName takes
// lanes/curbs/furniture/terrain/interiors/rail/facades/trees; both return
// null/false for an unknown name. ParseFeatureMode takes auto/off/on.
FeatureToggle* UrbanFeatureByName(UrbanFeatures* features, const std::string& name);
bool ParseFeatureMode(const std::string& text, FeatureMode* mode);

// Neighbourhood character of a block (issue #151, roadmap 3.2): biases the
// street-class-weighted BuildingKind table so a block reads coherently
// (offices cluster, apartments cluster) instead of a per-parcel lottery.
// Auto draws one per block (its own rng stream, so forcing a character never
// moves a wall); the rest force every block to that character.
enum class BlockCharacter { Auto, Mixed, Downtown, Residential, Industrial };
bool ParseBlockCharacter(const std::string& text, BlockCharacter* character);

// Tuning for GenerateUrbanMap. Defaults give an 8x8 city of ~26-unit average
// blocks (~288 units across, ~9.6x the default scene's 30). The overall
// extent is fixed by (blocks, blockSize, streetWidth); per-seed variation
// redistributes it between blocks and streets.
struct MapGeneratorConfig {
  int blocksX = 8;
  int blocksZ = 8;
  float blockSize = 26.0f;      // Average block footprint; real sizes vary per column/row.
  float streetWidth = 8.0f;     // Average corridor between blocks; rim streets use exactly this.
  float streetWidthMin = 0.75f; // Interior street width range, as a multiple of streetWidth.
  float streetWidthMax = 1.4f;
  float minBlockSize = 22.0f;   // Floor so every block fits a sidewalk, a building ring and gaps.
  float sidewalkWidth = 1.5f;   // Obstacle-free strip inside each block edge, along the street.
  float buildingDepth = 5.0f;   // Thickness of the building ring inside the sidewalk setback.
  float minBuildingHeight = 3.5f;  // Taller than a figure can climb: walls block movement and LOS.
  float maxBuildingHeight = 11.0f;  // Cap for ordinary (non-tower) buildings: 3 office floors.
  // Building archetypes (issue #151): each parcel's BuildingKind is drawn
  // from a table weighted by the street class it fronts, biased by its
  // block's character; floor count/height per kind set the building height.
  BlockCharacter character = BlockCharacter::Auto;
  int towerCount = 2;           // Tall landmark buildings scattered across the city.
  // Arterial city: residential/mixed/office buildings follow a smooth height
  // field, so similar heights cluster, up to this cap (maxBuildingHeight
  // still bounds the other kinds).
  float skylineMaxHeight = 24.0f;
  float towerMinHeight = 28.0f;
  float towerMaxHeight = 40.0f;
  float gapWidth = 3.0f;        // Passable alley between two buildings (> 2 * kAgentRadius).
  float gapProbability = 0.45f; // Chance a junction between adjacent buildings is a gap.
  // Hierarchical street layout. Set arteryCount=0 for the original orthogonal
  // grid generator; the default lays one oblique boundary-to-boundary artery.
  int arteryCount = 1;           // 1 oblique through-road; >=2 adds a wide branching avenue.
  // Additional arteries (0-2) widened out of the avenues parallel to the main
  // one, ramping through levels like it; -1 draws 0-2 per seed (half 0).
  int extraArteries = -1;
  float arteryWidth = 14.0f;
  float localStreetWidth = 6.0f;
  float localStreetSkew = 0.45f; // Max radians a cross street deviates from square to the artery.
  // Artery becomes an overpass crossing on piers. Auto draws presence per
  // seed (overpassChance) and one of the four layouts; see the enums above.
  OverpassMode elevatedHighway = OverpassMode::Auto;
  // P(overpass) for an Auto seed: roughly 50/50. A terrain-enabled seed
  // halves the complement (0.5 -> 0.75): flyovers pair well with levels.
  float overpassChance = 0.5f;
  OverpassLayout overpassLayout = OverpassLayout::Auto;
  BranchEnd branchEnd = BranchEnd::Auto;
  float branchOffMapChance = 0.35f;  // P(off-map branch) for an Auto seed that allows one.
  float highwayElevation = 5.0f;
  float highwayThickness = 0.45f;
  float supportSpacing = 18.0f;  // Arc-length between pier bents under the deck.
  // Overpass architecture (issue #223): the bents under the elevated spans
  // and the deck's raised sides. Each bent is a footing (wider than its
  // shaft), the shaft(s), and a cap wrapping the underside of the deck.
  PierForm pierForm = PierForm::Auto;
  float pierColumnWidth = 1.6f;     // Twin shaft width; the T-bent shaft is twice this across the road.
  float pierBaseExtra = 0.45f;      // Footing (plinth) margin outside the shaft, each side.
  float pierBaseHeight = 0.8f;      // Footing height above its ground.
  float pierCapDepth = 0.9f;        // Hammerhead/crossbeam cap depth under the deck slab.
  float twinDeckExtraWidth = 2.4f;  // Total extra deck width at full elevation on twin seeds.
  float deckRailHeight = 0.9f;      // Guard parapet above the deck top along its sides; 0 disables.
  float minPolygonArea = 22.0f;  // Smaller inset/sliver regions remain open.
  // Ziplines (issue #166): each block draws 0..maxZiplinesPerBlock lines
  // uniformly, each from the street up to the roof of one of the block's
  // buildings (which gets a walkable roof slab), from a dedicated RNG
  // stream, so they never shift the layout. Lengths are cable lengths.
  // Set maxZiplinesPerBlock = 0 to disable.
  int maxZiplinesPerBlock = 2;
  float ziplineMinLength = 4.0f;
  float ziplineMaxLength = 22.0f;
  // Urban ground (issue #155): map-spanning raised levels. Each level is a
  // straight step, roughly across the artery, laid along its own boundary
  // street: everything on one side rises 1-5 figure heights behind a
  // retaining wall parallel to that street. Steps overlap into nested
  // tiers (sharp, no rolling noise). The highway profile is planned
  // against the levels: the artery/branch ramp through a wall at grade (a
  // main road cutting through the level) or fly over a run of walls on a
  // deck; an avenue may ramp through; stair flights attached along each
  // wall face climb it elsewhere. terrainLevelCount: -1 draws 1-3 per
  // seed; 0 keeps the city flat. Legacy arteryCount=0 stays flat.
  int terrainLevelCount = -1;
  UrbanFeatures features;
};

// A block's footprint, street edge to street edge. The bounds are retained
// for compatibility; `vertices` is authoritative for angled layouts.
struct UrbanBlock {
  float x0, x1, z0, z1;
  std::vector<glm::vec2> vertices;
  // Long blocks left behind by a trimmed local street either split into
  // smaller blocks (divided by a short local street segment) or use this
  // angled, detached apartment-row layout.
  bool apartmentRow = false;
  bool empty = false;  // Left unbuilt (the vacant half of a tight long block).
};

struct UrbanRoad {
  std::vector<glm::vec3> centerline;
  float width = 0.0f;
  bool artery = false;
  bool elevated = false;
};

// Deterministic hierarchical city. The default cuts the map with one wide
// oblique boundary-to-boundary artery, then angled avenues and oblique cross
// streets on each side; the resulting convex polygon blocks all differ in
// shape and their frontage-following building rows adjust to the angles
// (acute corner wedges become real angled buildings; collapsed slivers stay
// open). arteryCount>=2 adds a wide branching avenue that merges into the
// artery. An elevated seed (or elevatedHighway=On) turns the artery into a
// true overpass: a bridge deck on paired pier columns with usable ground
// beneath, shaped by one of the OverpassLayout profiles (both ramps on-map,
// off-map through, or a single on-map ramp at either end), while the branch
// becomes a connecting ramp that climbs from grade and merges onto the
// elevated deck mid-span. arteryCount=0 retains the original orthogonal
// generator for compatibility. The same (seed, config) always yields an
// identical Scene. Scene::ziplines holds the block ziplines (see
// MapGeneratorConfig::maxZiplinesPerBlock): each runs from obstacle-free
// street level (`a`) to the roof of a building in the block (`b`, with a
// flat walkable roof slab in Scene::walkSurfaces), clear of obstacles, and
// lines are non-overlapping. Scene::buildings carries each building's
// archetype (see BuildingKind; the two landmark towers are Civic) and
// Scene::massing its visual-only parapets, roof furniture and awnings.
Scene GenerateUrbanMap(uint32_t seed, const MapGeneratorConfig& config = {});

// Block footprints GenerateUrbanMap(seed, config) uses. The hierarchical
// layout returns one convex polygon per block (artery-left side first);
// arteryCount=0 returns the legacy grid rectangles in x-major order.
std::vector<UrbanBlock> UrbanBlocks(uint32_t seed, const MapGeneratorConfig& config = {});

// Centerlines used by the generated scene. The oblique artery is returned
// first (sampled end-to-end: boundary-to-boundary at grade, extended past
// the bounds where an elevated layout runs off-map); the branching avenue
// follows, sampled from the map boundary to its merge point on the artery.
std::vector<UrbanRoad> UrbanRoads(uint32_t seed, const MapGeneratorConfig& config = {});

// Street classes of the street graph. Highway: the artery and its branch.
// Avenue: roughly parallel to the artery. Cross: oblique cross street.
enum class UrbanStreetClass { Highway, Avenue, Cross };

// One straight piece of street. Highway pieces follow the sampled
// centerline (so carry elevation in `a.y`/`b.y`); local streets are one
// segment clipped to the map square. Heights include the terrain field when enabled.
struct UrbanStreet {
  glm::vec3 a, b;
  float width = 0.0f;
  UrbanStreetClass cls = UrbanStreetClass::Cross;
  bool elevated = false;
};

// Every street of GenerateUrbanMap(seed, config) as segments: highway
// pieces first, then avenues and cross streets. Empty for arteryCount=0.
std::vector<UrbanStreet> UrbanStreets(uint32_t seed, const MapGeneratorConfig& config = {});

// The overpass decision GenerateUrbanMap(seed, config) makes: whether the
// artery is elevated and, if so, the deck layout used. Seed-driven under
// the Auto config defaults, forced otherwise; arteryCount=0 (the legacy
// grid) never has an overpass.
struct OverpassChoice {
  bool elevated = false;
  OverpassLayout layout = OverpassLayout::RampUpRampDown;
};
OverpassChoice UrbanOverpass(uint32_t seed, const MapGeneratorConfig& config = {});

// The bent column form GenerateUrbanMap(seed, config) uses for the overpass
// piers (issue #223): seed-driven under the Auto default, forced otherwise
// (never returns Auto). Meaningful only when the seed's artery is elevated.
PierForm UrbanPierForm(uint32_t seed, const MapGeneratorConfig& config = {});

// A city block as built: one or more grid cells joined with the streets
// between them (`x0..x1` / `z0..z1` is the bounding box). `notch` marks an
// L of three cells (the box minus one corner cell); `empty` an open area
// with a sidewalk ring but no buildings.
struct UrbanLot { float x0, x1, z0, z1; bool empty, notch; };

// Lot bounds for compatibility/inspection. Hierarchical layouts return one
// bound per polygon block; arteryCount=0 returns the original merged/L lots.
std::vector<UrbanLot> UrbanLots(uint32_t seed, const MapGeneratorConfig& config = {});

// A parcel (issue #151, roadmap 3.1): one buildable lot of a block's frontage
// ring (or detached apartment row), the unit the plan sizes and counts
// buildings by.
// `footprint` is the building's convex outline, counter-clockwise; edge
// `frontageEdge` (footprint[i] -> footprint[i+1]) normally lies on the block's
// building line along the street whose class is `street`; a detached apartment
// uses its principal long facade instead. `depth` is the footprint's extent
// behind the frontage; `corner` marks a piece at a block corner (two street
// frontages). Buildings and parcels correspond 1:1:
// Scene::buildings[i].parcel == i and GenerateUrbanMap's obstacle for
// parcel i has that footprint.
struct UrbanParcel {
  std::vector<glm::vec2> footprint;
  int frontageEdge = 0;
  UrbanStreetClass street = UrbanStreetClass::Cross;
  float depth = 0.0f;
  bool corner = false;
  int block = -1;
};

// Every parcel of GenerateUrbanMap(seed, config), in Scene::buildings order.
// Empty for arteryCount=0 (the legacy grid has no parcel pass).
std::vector<UrbanParcel> UrbanParcels(uint32_t seed, const MapGeneratorConfig& config = {});

// Half-extent of the generated map for `config` (independent of seed).
float UrbanMapHalfExtent(const MapGeneratorConfig& config);

// Tuning for GenerateHillyMap. Defaults give an 80x80 field of rolling
// hills (~10 units of relief over ~16-unit wavelengths -- steep, but
// every slope is still walkable) with a scattering of impassable rocks.
struct HillyMapConfig {
  float halfExtent = 40.0f;
  float cellSize = 1.0f;         // Heightfield sample spacing.
  float hillAmplitude = 12.0f;   // Height scale of the base noise octave.
  float hillWavelength = 16.0f;  // Size of the dominant hills.
  int rockCount = 12;            // Impassable boulders scattered mid-field.
  float rockMinExtent = 1.2f;    // Half-extent range of a rock's footprint.
  float rockMaxExtent = 3.0f;
  float rockHeight = 2.8f;       // Rock top above the local terrain: blocks LOS.
  float spawnMargin = 8.0f;      // Rock-free strip at the west/east spawn edges.
};

// Deterministic rolling-hills map: continuous ground-height variation
// (Scene::ground is filled in) from seeded value noise, a handful of
// impassable rocks embedded in the slopes, and the usual 3v3 spawn rows on
// the west (Blue) and east (Red) edges, each figure standing on the terrain.
// The same (seed, config) always yields an identical Scene.
Scene GenerateHillyMap(uint32_t seed, const HillyMapConfig& config = {});

}  // namespace tactics
