# Urban map roadmap: from blocks-and-streets to an enterable city

This is the staged plan for growing the procedural urban map (`src/game/MapGenerator.cpp`,
`GenerateArterialUrbanMap`) into a full city: lane-marked roads with sidewalks and street
furniture, uneven ground with ramps and retaining walls, railways, and buildings a figure can
actually enter (floors, doorways, windows, stairs, rooftops, bridges between buildings), in
several building types. Everything stays seed-deterministic and fully random per seed.

The plan is ordered by **data-model dependencies**, not by how visible each feature is.
Each stage lands as one or a few PRs, each of which is independently shippable, keeps every
existing seed/golden either byte-identical or deliberately regenerated, and adds its own
tests. Nothing in a later stage should require re-doing an earlier one.

---

## 0. Where the generator is today

What already exists and what each later stage leans on:

| Piece | Today | Relevant limitation |
|---|---|---|
| Street layout | One oblique artery (+ optional branch), avenues and skewed cross streets carve **convex polygon blocks** (`BuildCityBlocks`, `CutStreet`). | Streets are only *negative space* between blocks: there is no street object, so nothing can be placed "in the street" (lanes, signs, stops) by construction. |
| Roads | `UrbanRoad` centerlines only for the artery + branch; `Scene::roads` are flat asphalt quads at `kGradeY`. Local streets have no road surface at all. | Local streets are bare ground. |
| Sidewalks | One `sidewalkSurfaces` polygon per block (whole block at curb height). | No curb geometry, no separation between sidewalk and the buildable interior. |
| Buildings | `Obstacle` = convex XZ footprint prism, `bounds.min.y = 0`, solid, non-climbable (`EmitBlockBuildings`). | A building is a single solid block: no interior, roof is unreachable, LOS is fully blocked. |
| Elevation | `WalkSurface` chains for deck/ramp with explicit `neighbors` and `connectsToGround`; `NavMesh` routes ground ↔ surfaces; `ReachField` carries `surfaceY`. | Only one vertical layer type is modelled (decks). Climbing onto crates is still the legacy `Obstacle::climbable` flag ([#40](https://github.com/TheDeepestSpace/tbgwaf/issues/40)). |
| Terrain | `HeightField` ground, used by the hilly map only; navmesh/LOS/render all sample it. | The urban generator never fills `Scene::ground`; every urban element assumes `y = 0`. |
| LOS / FOV | Ray vs. obstacle prism, deck slab and terrain mesh (`Raycast.cpp`); shadow-map FOV mask renders the same static mesh. | No notion of "see-through but not walk-through" (windows, railings). |
| Determinism | Sub-streams per concern (`seed ^ constant`) so a forced choice never reshuffles the rest. Pinned per-seed fingerprints + map goldens + scenario goldens. | Any new draw must go in its own sub-stream or the fingerprints/goldens of every seed churn. |
| Config | `MapGeneratorConfig` (flat struct), `OverpassMode`/`OverpassLayout`/`BranchEnd` enums, scenario `map.generate` keys, `TBGWAF_MAP` env. | Fine for now; stage 1 formalizes how new knobs are added. |

Two engine facts shape the order below:

1. **Scene is the contract.** Generator → `Scene` (`obstacles`, `roads`, `sidewalkSurfaces`,
   `walkSurfaces`, `ground`, `units`) → navmesh, LOS, renderer, shadow-map FOV, scenario tests.
   Any feature that needs a new *kind* of thing in the world (a doorway, a window, a stair, a
   curb that is also a step) is a `Scene` data-model change first and a generator change second.
   Those are the expensive, cross-cutting steps; the plan groups them so each one lands once.
2. **`ReachField` cost scales with the number of walk surfaces** (`ComputeReachField` runs
   `FindSurfacePath` per grid node per surface). Enterable buildings multiply surfaces by an
   order of magnitude, so a navmesh performance pass is a prerequisite of interiors, not a
   follow-up.

---

## 1. Groundwork (no visible change, unlocks everything)

### 1.1 Generator pipeline refactor
Split `GenerateArterialUrbanMap` into explicit, individually testable passes that share one
intermediate `CityPlan` struct instead of recomputing `BuildHighwayPlan`/`BuildCityBlocks`
from the seed in several places:

```
seed + config
  → HighwayPlan        (exists)
  → StreetGraph        (new: every street as a polyline segment with width, class, and
                        which block edges it fronts; the artery/branch are just class=artery)
  → Blocks             (exists; each block edge now references its StreetGraph segment)
  → Parcels            (new: per-block subdivision; today implicit in EmitBlockBuildings)
  → Scene emission     (one emitter per feature family: roads, sidewalks, buildings, furniture…)
```

The `StreetGraph` is the single most important new intermediate: every later street feature
(lanes, curbs, signs, stops, crossings, rails) is "walk each segment and place things along
it". The avenues and cross streets that `CutStreet` carves today must be recorded as segments
rather than thrown away.

- Determinism: every new pass gets its own `Rng(seed ^ kConstant)` sub-stream.
- Expose read-only inspection APIs like the existing `UrbanBlocks/UrbanRoads` (e.g.
  `UrbanStreets(seed, config)`) so tests can assert on the plan, not only the scene.
- Exit criterion: all existing seeds produce byte-identical `Scene`s (the `SameScene` test
  and all map goldens unchanged).

### 1.2 Feature flags in `MapGeneratorConfig`
Add a nested `UrbanFeatures` block of per-feature `Auto/Off/On` tri-states (same pattern as
`OverpassMode`) plus per-feature chance, e.g. `lanes`, `curbs`, `furniture`, `terrain`,
`interiors`, `rail`. Default `Off` while a feature is in development, then `Auto` once it is
golden-tested. Plumb to scenario YAML (`map.generate.features: {...}`) and to `TBGWAF_MAP`
(e.g. `urban-full`). This is what lets each stage ship behind a flag without disturbing
existing scenarios.

### 1.3 Navmesh performance pass (prerequisite for interiors)
- Precompute surface-to-surface shortest distances once per `Build` (graph is small) instead of
  running `FindSurfacePath` per reach-field node.
- Spatial index for `SurfaceContainsXZ` lookups.
- Add a benchmark test with a synthetic scene of ~300 small surfaces and a budget, asserting
  `ComputeReachField` stays under a time ceiling, so later stages cannot silently regress it.

### 1.4 Terrain-aware urban scene plumbing
Make every urban emitter take a `GroundY(x, z)` sampler and emit geometry relative to it, even
while the sampler still returns `0`: road quads, sidewalk polygons, building `bounds.min.y`,
pier feet, spawn positions. This is a pure refactor on flat ground (goldens unchanged) that
makes stage 4 a generator change instead of an everything change.

---

## 2. Street level: lanes, curbs, crossings, furniture

Pure generator + renderer work on the existing data model (`RoadSurface`, `Obstacle`,
`sidewalkSurfaces`). No navmesh or LOS semantics change, so each item is low risk and can be
goldened immediately.

### 2.1 Road surfaces for every street
From the `StreetGraph`, emit asphalt `RoadSurface` ribbons for avenues and cross streets
(today only the artery/branch have pavement), mitered at junctions. Junction polygons become
explicit (needed for crossings and traffic lights).

### 2.2 Lane markings
Add a `RoadMarking` list to `Scene` (thin polygons: center dashes, lane dividers, stop lines,
zebra crossings, arrows). Rendered as flat decals above the asphalt; ignored by navmesh/LOS.
Lane count derived from street class and width (artery 2+2, avenue 1+1 or 2+2, local 1+1).
Seeded variation: one-way streets, turn arrows, faded/absent markings on back streets.

### 2.3 Curbs and sidewalk strips
Replace "whole block at curb height" with a proper sidewalk ring (`sidewalkWidth` already
exists) and a courtyard/parcel interior at grade. Curb height stays visual (`kSidewalkHeight`,
well under a step). Add dropped curbs at crossings. Sidewalk polygons also exist along
the artery/branch at grade where blocks front them.

### 2.4 Street furniture (first batch: non-blocking)
Small `Obstacle`s placed along sidewalk edges from the `StreetGraph`: trash cans, bollards,
fire hydrants, street signs (post + plate), lamp posts, mailboxes. Rules: never inside the
`gapWidth` alleys or block openings, never within the agent-padded spawn lanes, keep
`2 * kAgentRadius` of clear sidewalk. Posts thinner than the agent radius should be
`Obstacle`s that block LOS only trivially. Map-generator test: sidewalks remain fully
connected (reuse `TestNavMeshFullyReachableFromSpawns` logic per block ring).

### 2.5 Street furniture (second batch: cover-sized)
Bus stops (shelter: roof slab on posts, bench), newsstands, parked cars (low climbable boxes
until stage 5 replaces the flag), planters, dumpsters in alleys. These are gameplay cover,
so each gets a logic test for LOS blocking at the eye probe vs. the foot probe.

### 2.6 Traffic lights and signage
Traffic lights at junctions with ≥ 3 arms (mast arm over the road, or post per corner);
stop/yield signs at smaller junctions; street-name signs at corners. Visual only.

Seeded randomization across 2.x: per-street class draws (one-way, parking lane, planted
median on wide avenues), per-corner draws (which furniture, light vs. sign), a density knob.

Exit criteria for stage 2: new map goldens `city_streets_seed_*`; fingerprint test extended
with furniture counts; the "Map seeds" gallery shows the new tiles.

---

## 3. Building typology and massing (still solid buildings)

Decide *what* each building is before making it enterable; typology drives floors, windows,
doors and roofs later. Still the current solid-prism data model, so no engine change.

### 3.1 Parcel subdivision
Formalize `EmitBlockBuildings`' frontage pieces as `Parcel`s (footprint polygon, frontage
edge → street segment, depth, corner flag). This is where the plan decides building count and
sizes; nothing changes visually yet.

### 3.2 Building archetypes
Add `BuildingKind` to a new `Building` record kept beside the `Obstacle` (or as an index map):
`Residential` (apartments), `MixedUse` (commercial ground floor + apartments), `Office`,
`Commercial` (single-storey shop/warehouse), `Civic`/landmark (the existing towers), `Garage`.
Kind is drawn per parcel from a street-class-weighted table (artery frontage → office/mixed;
back street → residential/warehouse) with a per-block "character" bias so neighbourhoods read
coherently. Determine `floorCount` and `floorHeight` per kind (e.g. commercial ground floor
4.0, residential 3.0, office 3.5) and derive building height from them, replacing the
`cluster ± 1.2` height draw. Keep the two landmark towers.

### 3.3 Massing details (visual)
Setbacks on upper floors for offices, roof parapets, roof furniture (water tanks, AC units,
stair bulkheads: these become real rooftop cover later), awnings/canopies over commercial
ground floors, basement light wells avoided. Facade material/colour per kind via a per-obstacle
style index the renderer reads.

### 3.4 Windows and doors as facade decals (visual only, no semantics)
Window grids derived from `floorCount` × facade length; a street-facing entrance door on each
building's frontage edge (plus a rear/alley door for some). Record each opening as data
(`FacadeOpening{wall edge, u-range, floor, kind}`) now, render as decals, so stage 5 can turn
the same records into real holes without re-rolling where they are.

Exit criteria: goldens per archetype mix; fingerprint test tracks kind histogram per seed.

---

## 4. Uneven ground: slopes, terraces, retaining walls

Introduce `Scene::ground` into the urban map (enabled by 1.4), with the two regimes asked for:
gentle ramps and abrupt level changes that act as natural walls.

### 4.1 Terrain field for the city
Low-frequency value noise (reuse `ValueNoise`) plus a seeded set of **terrace plateaus**: pick
2–4 regions bounded by street segments from the `StreetGraph`, assign each a level offset, and
blend the field with a steep smoothstep across the boundary street (gentle grade) or a near
step (the "disconnection"). Amplitude kept such that street grades stay walkable where the
plan says gentle. Config: `terrainAmplitude`, `terraceCount`, `cliffChance`.

### 4.2 Streets follow the ground, blocks are levelled
Each block's parcel interior is flattened to one pad height (buildings need a flat base);
streets and sidewalks follow the field. Where the pad is higher than the adjacent sidewalk,
emit a **retaining wall** `Obstacle` plus steps (stage 5 gives steps semantics; until then a
`WalkSurface` ramp). Where lower, a short slope. The existing hilly tests
(`TestHillyPathsFollowTerrain`, slope-aware reach costs) are reused on the urban map.

### 4.3 Cliffs and natural walls
Abrupt level changes become `Obstacle` retaining walls with height = Δlevel; LOS over them
depends on the eye probe, so a 1.2 m wall is cover, a 3.5 m wall is a true wall. Guarantee
connectivity: every plateau gets at least one gentle street ramp and one stair, verified by a
reachability test from both spawns to every block ring.

### 4.4 Overpass and terrain
The overpass profile is currently absolute (`highwayElevation` above `kGradeY`); make it
relative to local ground so the deck still clears the streets beneath, and pier heights follow
the field. Branch ramp length adapts to the actual rise.

### 4.5 Underpasses and sunken streets
With terrain available, a street can also run *under* a block pad (cut-and-cover) or be sunk
between retaining walls, giving the "trench" variant for free. Draw per seed.

Exit criteria: `city_terrain_seed_*` goldens; fingerprint test includes terrain stats
(min/max, plateau count); all spawn-reachability tests pass with terrain on; frame-cost
scenario budget (#119) unchanged within tolerance.

---

## 5. Enterable buildings, part 1: data model for openings, floors, stairs

This is the one large engine stage. It is split so each PR changes one semantic concept.

### 5.1 Walls instead of solid prisms
Represent a building as a set of wall `Obstacle` segments (thin convex prisms along each
footprint edge) plus a `floors[]` list, rather than one solid prism. Interiors are empty space.
A flag on the building keeps "render as solid massing" so stage 3 visuals are unchanged for
buildings that are not enterable in a given seed. Mass-to-walls is lossless for navmesh/LOS
when every wall is present, so goldens stay the same up to roof rendering.

### 5.2 Doorways
A wall segment may have a gap `u-range` at ground level: implement as splitting the wall
obstacle into pieces around the opening (exactly how `SplitRun`/gaps already produce alleys),
so navmesh and LOS get doorways with **zero** new engine semantics. Door width ≥ `gapWidth`
rule reused. The entrance records from 3.4 drive where. The lintel over the door is a wall
piece with `bounds.min.y` at head height (obstacles already support a raised base: the hilly
rocks use one), which needs the one navmesh rule below.

**One navmesh rule:** an obstacle whose `bounds.min.y` is at or above `kUnitHeight` does not
block movement (figures walk under it). Today every obstacle footprint is inflated into the
ground mesh regardless of height. This is a one-line filter in `NavMesh::Build` plus a logic
test, and it is what makes lintels, window heads, awnings, skybridge undersides and signs
on posts all work without special cases.

### 5.3 Windows as real cutouts in the walls
Decided: a window is a hole in the wall, not a flag. The wall segment is split into four
plain `Obstacle` pieces: the two flanking full-height jambs, a **sill** piece from the floor
up to sill height, and a **head** piece from head height up to the ceiling. Nothing new is
needed in `Raycast.cpp`, the shadow-map FOV mesh or the renderer: rays pass through the gap
geometrically, the prism renderer already draws from `bounds.min.y` to `max.y` so the hole is
visible, and the sill blocks movement while the head piece is ignored by the navmesh (5.2
rule). Resulting gameplay, all from existing probes:

- A figure inside at the window has an FOV cone out through the opening (a sniper covering an
  intersection from a second-floor window) and is seen from outside at the **eye probe** when
  standing, while the **foot probe** is hidden by the sill. Crouching is not modelled, so a
  figure that steps back from the window is hidden by the jambs.
- A figure outside can see in through the same hole, and later free-aim/ballistic shots
  ([#129](https://github.com/TheDeepestSpace/tbgwaf/issues/129)) and thrown grenades go
  through the opening for free, because every trace is a geometric ray/arc against the same
  pieces. The roadmap does not add grenades; it just guarantees openings will not block them.
- Sill and head heights per archetype (shopfront glazing: sill near the floor, so it hides
  nothing; apartment window: sill ~0.9, head ~2.1; office ribbon windows: sill ~0.8 along the
  whole facade).

Same mechanism gives railings, fences with gaps, parapets and arrow slits. Logic tests: eye
visible/foot hidden through a window; nothing visible through the jamb; a shot through the
window resolves; a figure cannot path through a window.

### 5.4 Floors as `WalkSurface`s with ceilings
Each upper floor is one or more convex `WalkSurface`s per storey at `floorIndex * floorHeight`.
Ceilings/floors need to occlude LOS vertically: a `WalkSurface` already acts as a
`highwayThickness` slab in `RayIntersectsWalkSurface`, which is exactly a floor slab. The
ground-floor interior is just ground inside walls. Floors are only reachable through 5.5.

**Convex vs. non-convex, and why it matters here.** A polygon is *convex* when every corner
bends the same way and any straight line between two points inside it stays inside: rectangles,
triangles, trapezoids, the wedge blocks the street cutter produces. It is *non-convex* when it
has an inward corner or a hole: an L or U shaped floor, a floor with a stairwell cut out, a
corridor with rooms off it. Almost every geometry helper in the engine assumes convex input
(`PointInConvexPolygon`, `InsetConvexPolygon`, `KeepSide`, `SurfaceContainsXZ`,
`Obstacle::footprint`), because for convex shapes those tests are exact and a handful of dot
products. The navmesh also treats each `WalkSurface` as one node a figure can cross in a
straight line, so an L-shaped floor stored as one surface would let figures walk straight
through the missing corner, and an inset of it could fold over itself.

Decided: keep every surface convex and **decompose** non-convex floors into convex pieces
joined by `neighbors` links, exactly as the overpass deck is already a chain of quads. A floor
with a stairwell is four rectangles around the hole; an L is two rectangles; the room-splitting
in 6.1 produces convex rooms by construction. `FindSurfacePath` already walks these links, so
no engine change is needed, only a small `DecomposeToConvex(polygon) -> pieces + links` helper
with its own unit tests.

### 5.5 Stairs
A stair is a short steep `WalkSurface` chain (reuse the ramp machinery with
`connectsToGround` at the bottom and a `neighbors` link at the top), placed inside the
building against a wall, with a stairwell cut-out in each floor above (the floor polygon is
split around it). Rendered as steps (visual) over the ramp slope (gameplay). This also closes
[#40](https://github.com/TheDeepestSpace/tbgwaf/issues/40): crates/parked cars become a tiny
ramp+top surface and `Obstacle::climbable` is removed.

### 5.6 Rooftops
Top floor slab doubles as the roof `WalkSurface` with the stair bulkhead (`3.3`) as a
doorway onto it, and parapets as low walls (cover at the foot probe, see-over at the eye).
Rooftop furniture from 3.3 becomes cover.

### 5.7 Picking, camera and overlays for interiors
Click/hover must resolve to the right floor (`FindWalkSurfaceContaining` with y-hint already
exists); the move frontier draws per floor; the camera needs a "cut-away" or per-floor
opacity when the selected figure is indoors (relates to [#136](https://github.com/TheDeepestSpace/tbgwaf/issues/136)).
Shadow-map FOV must treat ceilings as casters (it already uses the static mesh).

Exit criteria: scenario tests `enter_building_ground_floor`, `climb_stairs_two_floors`,
`shoot_through_window`, `rooftop_sightline`; map test that every floor surface is reachable
from its building's door; reach-field benchmark from 1.3 still green on a full city.

---

## 6. Enterable buildings, part 2: interiors per archetype, random generation

Generator-only work on the model from stage 5.

### 6.1 Floor plans per archetype
- Commercial ground floor: one open hall, wide street door(s), large windows.
- Apartments: a central corridor/stair core, 2–6 units with internal walls and doors; windows
  on the facade; rear door to the courtyard.
- Office: open plan with a core, column grid as cover, glass facade (dense windows).
- Mixed use: commercial plan on floor 0, apartment plan above, separate street entrance to the
  stair core.
- Warehouse/garage: single volume, roll-up door (wide doorway), mezzanine.
Plans are templated room-splitting of the footprint (BSP along the long axis), fully seeded.

### 6.2 Which buildings are enterable (staged rollout, FPS-permitting)
Decided: start with a random subset and grow it as the performance budget allows. Each
building draws `enterable` from a per-kind `interiorChance` (its own seed sub-stream, so
changing the chance later never reshuffles layouts), with a hard rule that every block has
at least one enterable building and every artery frontage a few. Solid buildings keep the
stage 3 massing. The rollout is three config presets, each gated on the reach-field benchmark
(1.3) and the scenario frame-cost tracking
([#119](https://github.com/TheDeepestSpace/tbgwaf/issues/119)) staying within budget on the
web build:

| Preset | Enterable share | Floors opened | When |
|---|---|---|---|
| `sparse` (default after 5.x) | ~1 per block | ground floor + roof | first ship |
| `mixed` | ~40% | all floors, stairs | once 1.3 + per-floor culling in the renderer are in |
| `dense` | ~80%, every mixed-use/office | all floors, bridges, fire escapes | if budget allows |

The preset is a `MapGeneratorConfig` field and a scenario key, so tests can pin `dense` on
one seed regardless of the shipped default.

### 6.3 Vertical variety
Fire escapes (external stair `WalkSurface` chain on an alley wall → rooftop), basements
(stage 4 retaining walls make a half-sunk floor cheap), roof-access-only buildings, blocked
stairs (random stair removed on one floor to force the fire escape route).

### 6.4 Connections between buildings
- Rooftop-to-rooftop bridges/planks where adjacent roofs are within a step height (reuse the
  `SpansTouchAtSameHeight` link logic).
- Skybridges across a street between two office buildings (a `WalkSurface` with railings).
- Shared party-wall doors between adjacent apartment buildings (rare).
- Rooftop entrances from the overpass deck where a deck runs past a roof at matching height.

Exit criteria: per-archetype goldens on fixed seeds; scenario tests for a fire-escape route and
a skybridge crossing; interior reachability test over many seeds.

---

## 7. Rail

Rail reuses the artery/overpass machinery almost entirely.

### 7.1 At-grade railway corridor
A second linear corridor type in the `HighwayPlan` → `StreetGraph`: ballast `RoadSurface`,
two rail `RoadMarking`s, sleepers as decals, level crossings where it meets streets (barriers
as thin obstacles, crossing signs). Blocks are cut against it like against the artery. Fenced
on both sides except at crossings (fence = 5.3 see-through obstacle).

### 7.2 Elevated railway / viaduct
Same `WalkSurface` chain + pier bents as the overpass with a different profile (arches or
closer bents), no ramps (trains don't need them, so it is `Through`-only), stations as
platforms (`WalkSurface` at deck height reached by stairs from 5.5) with a roof. A railway
overpassing the road overpass is the intersection of two deck chains at different heights,
which the layer model already supports.

### 7.3 Rolling stock as cover
Static train cars on a seeded subset of track (long low climbable surfaces via the 5.5 stair
model, or solid obstacles).

---

## 8. Cross-cutting: randomization, testing and tooling (runs alongside every stage)

- **Seed streams:** one `Rng(seed ^ kStageConstant)` per emitter; document the constants in a
  table in `MapGenerator.cpp` so no two features share a stream.
- **Fingerprints:** extend `TestPinnedSeedFingerprints` into a per-seed struct (buildings by
  kind, surfaces, openings, furniture count, terrain stats) with a single place to update.
- **Invariants over many seeds** (not just the three pinned ones): a `for seed in 1..200`
  test for spawn reachability, no overlapping obstacles, every door reachable, every floor
  reachable from its door, every plateau reachable. This is the main safety net for random
  generation.
- **Map goldens** per stage and the gallery's "Map seeds" section pick them up automatically.
- **Scenario tests**: one or two per gameplay-affecting feature, as listed in each stage.
- **Performance budget**: the reach-field benchmark (1.3) and frame-cost tracking (#119) run on
  a "full feature" seed.
- **Debug overlay**: extend the `N` navmesh overlay to show walk-surface ids/floors and street
  graph segments, which makes each stage reviewable in the PR preview build.

---

## Suggested delivery order and sizing

| Order | Stage | Size | Depends on | Visible to players |
|---|---|---|---|---|
| 1 | 1.1 pipeline refactor + 1.2 feature flags | M | – | no |
| 2 | 1.4 terrain plumbing | S | 1.1 | no |
| 3 | 2.1 road surfaces + 2.3 curbs | S | 1.1 | yes |
| 4 | 2.2 lane markings | S | 2.1 | yes |
| 5 | 2.4 + 2.6 furniture (non-blocking) + lights/signs | M | 2.3 | yes |
| 6 | 3.1 parcels + 3.2 archetypes + 3.3 massing | M | 1.1 | yes |
| 7 | 3.4 facade openings as data + decals | S | 3.2 | yes |
| 8 | 2.5 cover-sized furniture (bus stops etc.) | S | 2.4 | yes |
| 9 | 1.3 navmesh performance pass | M | – | no |
| 10 | 4.1–4.3 terrain, terraces, retaining walls | L | 1.4, 1.3 | yes |
| 11 | 4.4–4.5 overpass over terrain, sunken streets | M | 4.1 | yes |
| 12 | 5.1–5.2 walls + doorways | M | 3.4 | yes (ground floors) |
| 13 | 5.3 windows (see-through obstacles) | M | 5.1 | yes |
| 14 | 5.4–5.5 floors + stairs (closes #40) | L | 1.3, 5.1 | yes |
| 15 | 5.6–5.7 rooftops + interior picking/camera | M | 5.5 | yes |
| 16 | 6.1–6.2 archetype floor plans, enterable subset | L | 5.x | yes |
| 17 | 6.3–6.4 fire escapes, bridges, rooftop links | M | 6.1 | yes |
| 18 | 7.1 at-grade rail | M | 1.1, 2.x | yes |
| 19 | 7.2–7.3 viaduct, stations, rolling stock | M | 7.1, 5.5 | yes |

S ≈ one PR of a day or two, M ≈ a few PRs over a week, L ≈ a multi-PR stage with its own
feature flag kept `Off` until the last PR.

Items 3–8 (street level and massing) are independent of items 9–15 (terrain and interiors),
so the two tracks can proceed in parallel once the stage 1 refactor is in.

---

## Decisions taken

1. **Windows are real cutouts** (5.3): a wall is split into jambs, sill and head pieces, so
   figures peep in or out, snipers cover a street from an upper window, and future free-aim
   shots or grenades pass through the opening geometrically. Cost: one navmesh rule that
   obstacles starting above head height do not block movement (5.2).
2. **Floors stay convex and are decomposed** (5.4): non-convex plans become linked convex
   pieces, the same pattern the overpass deck already uses, so no navmesh change is needed.
3. **Interiors roll out by preset** (6.2): `sparse` first (about one enterable building per
   block, ground floor and roof), then `mixed` and `dense` as the reach-field benchmark and
   frame-cost tracking allow.
