# World-change log

Map edits after generation (destructible world) are an ordered, timestamped
log of `WorldChange`s (`src/game/WorldChange.h`). Server and clients apply the
same log to the same seeded map, so drawn == simulated geometry.

- **Kinds:** obstacle removed / replaced by pieces, walk surface removed / cut,
  heightfield patch, wall hole (`FacadeOpening::Kind::Breach`), floor hole,
  massing cut, debris.
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
  room. Pieces re-fracture in their own plane on later blasts. Solids thicker
  than a wall (0.7 m: solid non-enterable building blocks, overpass piers)
  are not fractured yet.
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
  massing). Sloped-top details (deck rails) are only removed whole. Both
  kinds keep their rect form (no `pieces`: cut around `rect`), and `WallHole`
  (rect or u-monotone outline cut, Breach `FacadeOpening`) still applies, so
  older logs replay unchanged; the planner no longer emits them.
- **Debris:** every knocked-out cell detaches as a chunk of exactly its own
  shape, is thrown away from the blast and lands lying flat. The ballistics
  run inside `PlanExplosion` -- on the authority planning the blast, i.e. the
  server for networked matches -- against the post-cut scene, seeded and
  deterministic; the `Debris` change carries the rest-pose prisms (the cell
  outline as footprint) plus one `DebrisFlight` per piece (launch point,
  launch/rest yaw, flight time, tumble). Clients replay the same closed-form
  motion (`DebrisPose`): at launch the chunk sits exactly in the cell it was
  knocked out of, at the end exactly on its rest pose. The largest chunks
  (up to 8 per blast, >= 0.25 m^2) are `solid`: appended as
  `ObstacleKind::Rubble` obstacles, blocking movement and sight from the
  change's `at` (only the drawn fall spans the flight time), never landing on
  a figure or back in the breach; a later blast blows them away like a street
  fixture. Smaller chunks are litter: drawn where they came to rest, never
  simulated. Cells that find no free spot to rest are dust.
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
- **Wire (protocol 5):** `round.world` = changes not yet sent (each has `at`,
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
  the hole), and one Debris change with the chunks' simulated falls.
  Native app: the "Debug bomb" section of the debug
  panel places it at the camera focus, detonates it now, or arms it for a
  time into the next round (the changes then carry that `at`, so playback
  and replays blow it at the same moment). Server: control taps
  `{"t":"place_bomb","room":..,"pos":[x,y,z],"radius":r}` and
  `{"t":"detonate_bomb","room":..}` (optional `"at":s` arms instead; an
  immediate detonation broadcasts the delta and re-sends each seat's
  fog-filtered state, since a fresh hole can reveal figures). Scenarios:
  `{action: bomb, position: [x,y,z], radius: r, at: s?}`.
