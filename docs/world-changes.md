# World-change log

Map edits after generation (destructible world) are an ordered, timestamped
log of `WorldChange`s (`src/game/WorldChange.h`). Server and clients apply the
same log to the same seeded map, so drawn == simulated geometry.

- **Kinds:** obstacle removed / replaced by pieces, walk surface removed / cut,
  heightfield patch, wall hole (`FacadeOpening::Kind::Breach`), floor hole,
  massing cut, debris, opening removed.
- **Fracture (issue #350):** a blast pre-cracks every concrete element it
  reaches into seeded Voronoi cells in the element's own plane, the usual
  pre-fractured-destruction scheme (cells + damage radius): walls, the jamb /
  sill / head strips around windows and corner posts in the wall plane;
  ceiling / roof slabs, floor surfaces and roof massing in XZ. The crack
  pattern is world-anchored (sites are offsets from the bomb, projected into
  each plane), so coplanar elements crack along the same lines. Cells near
  the blast are knocked out; the rest stay standing as convex pieces, so
  every torn edge sits at the fracture's own angle -- no rect cuts anywhere.
- **Walls / posts** are an `ObstacleReplaced`: the prism dies and the standing
  cells are appended, each carrying its cell as a wall-plane `profile`
  (`Obstacle::profile`, a convex (u, y) polygon extruded through the
  thickness). LOS, shots and grenade flight clip against the profile, so they
  pass through exactly the drawn gap. The ground navmesh narrows a profiled
  piece to where its silhouette crosses a figure's body band (0.35 m step-over
  to head height), so a breach is a doorway wherever the torn edges leave
  room. Pieces re-fracture in their own plane on later blasts.
- **Thick solids** (thicker than a wall, 0.7 m: overpass piers and footings,
  solid non-enterable building blocks, crates) fracture in 3D, also as an
  `ObstacleReplaced`: the part within the blast's height is split into layers
  (0.5-1.2 m, anchored to the bomb's height), each layer pre-cracked in XZ
  with its own turn of the crack pattern, and the cells within the blast come
  out; the standing cells are plain prisms (cell footprint x layer), what is
  above or below the blast's reach stays in one piece. Nothing collapses for
  lack of support yet (upper parts may levitate). Terrain is not cut.
- **Slabs** (ceilings, roofs) are an `ObstacleReplaced` with the standing
  cells as plain prisms. **Floor holes** (`FloorHole`) carry the standing
  cells as `pieces` footprints -- the same cells as the slab under the floor
  -- clipped to the patch; the patch dies, the pieces are appended and linked
  to each other and to the old neighbours still touching them; a figure left
  without footing falls (ragdoll, with the drop to whatever support remains
  below). **Massing cuts** (`MassingCut`) break the visual-only roof
  furniture the same way: `Scene::massing[target]` (the parapet band wrapping
  the roof edge, roof fixtures, awnings) dies and its standing cells are
  appended (sight prisms included -- `SightObstacles` rebuilds from the live
  massing); a sloped-top detail (an overpass deck rail) keeps its `top`
  on every piece. Overpass decks, ramps and other walk surfaces with nothing
  cut under them are cut by the same `FloorHole` (cell pieces may cut a
  sloped planar patch) and shed their own chunks; a **stair flight** the
  blast knocks a cell out of comes down whole (`SurfaceRemoved`, every cell
  a chunk). Both
  kinds keep their rect form (no `pieces`: cut around `rect`), and `WallHole`
  (rect or u-monotone outline cut, Breach `FacadeOpening`) still applies, so
  older logs replay unchanged; the planner no longer emits them.
- **Window frames / doors:** a `FacadeOpening` within the blast is blown off
  (`OpeningRemoved`; its frame bars or door leaf leave as chunks). The others
  stay: when the prism an opening was laid out on is fractured
  (`ObstacleReplaced` -- e.g. the roof slab every window of an enterable
  building is indexed against), the opening is *detached* onto its own
  world-space edge (`FacadeOpening::detached`, `FacadeOpeningEdge`) instead
  of dying with the prism's index. Only `ObstacleRemoved` still kills them.
- **Debris:** everything knocked out is accounted for -- every cell (and
  every standing sliver too small to keep) detaches as a chunk of exactly its
  own shape; only outlines under 0.004 m^2 are dust. No cap on chunks, pieces
  or changes drops anything: past the cut budget (600 elements per blast)
  the remaining elements are simply left intact. The chunks are a rigid-body
  simulation (`src/game/DebrisPhysics.cpp`) run inside `PlanExplosion` -- on
  the authority planning the blast, i.e. the server for networked matches --
  against the post-cut scene: each chunk is a convex prism thrown by the
  blast with a launch velocity and spin, under gravity, colliding (SAT
  contacts with face clipping, sequential impulses, friction, a little
  restitution) with the other chunks, standing geometry, the ground, floors
  and roofs, the map edge, and figures (immovable posts: a chunk is deflected
  off a figure, never lies on one). It comes to rest in whatever pose the
  contacts leave it in -- flat, tilted, leaning on a neighbour, heaped in a
  doorway. Nothing steers chunks clear of the breach: if the physics seals it,
  it is sealed. Fixed 120 Hz step, fixed iteration order, no clock; contacts
  are cold-started and penetration is removed by moving bodies apart, not by
  adding velocity, so piles do not creep; a body that has gone still (by
  velocity, or by pose) is put to sleep -- frozen exactly -- and wakes only on
  a hard hit, and a body that keeps shuffling on the spot (under 0.15 m of
  net travel for over a second) is damped ever harder until it stops, so
  nothing "seizes"; a body still going somewhere (falling, sliding, rolling
  off another) is never slowed, however late. A blast is cut off at 12 s.
  Litter-sized chunks
  collide with everything but never push a heavier chunk. `Debris` changes
  (batches of 60) carry the chunks as they left the wall / slab (`pieces`:
  the prism it was cut from) plus one `DebrisFlight` per piece: its pose
  (centre of mass + quaternion) sampled every 1/20 s until it stopped. The
  clients integrate nothing -- they interpolate the track (`DebrisPose`;
  identity at launch, the rest pose from the end on), so a chunk that hits a
  figure or a wall does so identically everywhere. Chunks of 0.1 m^2 and up
  that settle are `solid`: appended as `ObstacleKind::Rubble` obstacles whose
  footprint / bounds are the vertical envelope of the tilted body at rest
  (`DebrisRestObstacle`; `Obstacle::rubble` keeps the true body), blocking
  sight and movement from the change's `at` (rubble lying no higher than a
  0.35 m step is walked over, like a stub at the foot of a breach; heaps
  block); a later blast throws them again as the very bodies they are. A
  chunk that would lie over a figure, and every chunk under 0.1 m^2, is
  litter: drawn where it came to rest, never an obstacle. **Timing:** a blast
  inside a round (`at` > 0) is drawn against the round clock alone
  (`DebrisRoundAge`: launch pose on the frame it goes off; a replay paused on
  any frame holds the chunks still; a flight that outlasts its turn freezes
  at the turn's end and carries on in the next). A networked mirror keeps
  that clock during round playback. Only a detonation outside a round
  (`at` = 0) falls on a local wall clock, and the app does not count the
  time spent planning a blast as game time. In the figures'
  sight (FoV) maps the renderer draws solid chunks at their pose *right now*,
  not at their landing spot. Street fixtures (cars, benches) in a blast are
  still removed outright.
- **Stable ids:** nothing is erased. Removed obstacles / surfaces / buildings /
  openings / massing get `dead = true` and stay in their vectors, so every
  index holder (`Obstacle::building`, `Building::obstacle|upperObstacle|
  floorSurfaces|stairSurfaces`, `FacadeOpening::obstacle`,
  `MassingDetail::building`, `WalkSurface::neighbors`,
  `TerrainTunnel::roofSurface`, `TeamVisibility::visibleObstacle`) stays valid.
  A dead obstacle is parked below the world with no footprint (inert even for
  code that ignores `dead`); a dead surface has no vertices/neighbours.
  Replacement pieces are appended, so their indices are deterministic.
- **State:** `GameLogic::WorldLog()`; part of `GameSnapshot` (`world`), the
  snapshot text (only when non-empty, so old hashes are unchanged) and hence
  `StateHash()`. `ImportState` syncs geometry to the snapshot's log (rewinding
  from `BaseScene()` when it is shorter/different). `Reset` clears it.
- **Wire (protocol 7):** `round.world` = changes not yet sent (each has `at`,
  seconds into the round; the client applies them as playback passes it);
  `state.world` = full log (join/reconnect/spectator); `{"t":"world"}` for
  changes made outside a round. `RemoteClient` ignores ids it already has.
- **Debug:** control tap `{"t":"remove_obstacle","room":..,"obstacle":N}`
  removes an obstacle mid-match; `dump_state` reports `world_changes`,
  `checksum` and the full `world` log.
- **Debug bomb (issue #350):** a movable debug explosive that emits the same
  explosion record a grenade will later (position, radius, seed =
  hash(match seed, round, bomb id)). `PlanExplosion`
  (`src/game/Destruction.h`) turns it into a deterministic ordered change
  list: street fixtures blown away, then the fracture described above (~30-40
  wall sites and ~12-16 floor sites, dense near the bomb, plus a ring of
  always-standing anchor sites at the blast's in-plane radius that bounds
  the hole), and the Debris changes with the chunks' simulated pose tracks.
  App: the "Debug bomb" section of the debug panel places it at the world
  coordinates typed / dragged there (X / Y / Z; "Put on ground" sets Y to
  1 m above the ground; a marker in the 3D view shows the placed bomb, and
  editing the coordinates or radius moves it live), detonates it now, or
  arms it for a time into the next round (the changes then carry that `at`,
  so playback and replays blow it at the same moment). In a server-backed
  match the panel sends `place_bomb` / `detonate_bomb` to the server instead
  (seated players only); the marker there is this client's own request.
  Server: control taps
  `{"t":"place_bomb","room":..,"pos":[x,y,z],"radius":r}` and
  `{"t":"detonate_bomb","room":..}` (optional `"at":s` arms instead; an
  immediate detonation broadcasts the delta and re-sends each seat's
  fog-filtered state, since a fresh hole can reveal figures). Scenarios:
  `{action: bomb, position: [x,y,z], radius: r, at: s?}`.
