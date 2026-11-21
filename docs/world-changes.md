# World-change log

Map edits after generation (destructible world) are an ordered, timestamped
log of `WorldChange`s (`src/game/WorldChange.h`). Server and clients apply the
same log to the same seeded map, so drawn == simulated geometry.

- **Kinds:** obstacle removed / replaced by pieces, walk surface removed / cut,
  heightfield patch, wall hole (`FacadeOpening::Kind::Breach`), floor hole,
  massing cut, debris.
- **Wall holes** really cut the wall (issue #350): the target prism dies and
  the convex leftovers of its silhouette are appended around the opening --
  plain jamb prisms beside it plus pieces carrying a wall-plane `profile`
  (`Obstacle::profile`, a convex (u, y) polygon extruded through the
  thickness) for the slanted torn edges, so the fracture sits at random
  angles rather than stair-steps while LOS, shots and grenade flight still
  pass through exactly the drawn gap (raycast clips against the profile; XZ
  movement collision stays footprint-based, conservatively). The change's
  `polygon` holds the u-monotone fracture outline; an empty outline keeps the
  plain u0/u1/y0/y1 rect, so pre-fracture logs replay bit-identically. A
  hole reaching the floor with head-height clearance becomes a doorway for
  the navmesh automatically; the Breach `FacadeOpening` records the
  outline's bounding box. **Floor holes** cut a rect out
  of a building floor surface: the patch dies, up to four convex pieces are
  appended and linked to each other and to the old neighbours still touching
  them; a figure standing over the hole falls (ragdoll, with the drop to
  whatever support remains below). The matching ceiling-slab cut is a
  separate `ObstacleReplaced` change.
- **Massing cuts** break the visual-only roof furniture with the building:
  `Scene::massing[target]` (the parapet band wrapping the roof edge, roof
  fixtures, awnings) dies and the convex parts of its footprint outside the
  change's rect are appended, so the wrap-around above a breached wall opens
  with it (sight prisms included -- `SightObstacles` rebuilds from the live
  massing). Sloped-top details (deck rails) are only removed whole.
- **Debris:** chunks knocked out of the openings detach, fly away from the
  blast and land as `ObstacleKind::Rubble` obstacles that block movement and
  sight like anything else. The ballistics run inside `PlanExplosion` -- on
  the authority planning the blast, i.e. the server for networked matches --
  against the post-cut scene, seeded and deterministic; the change carries
  the rest-pose pieces plus one `DebrisFlight` (launch point/yaw, flight
  time, tumble) per piece, and clients replay the same closed-form arc
  cosmetically (the renderer keeps rubble out of the static mesh and
  animates it), landing exactly on the solid rest pose. Simulation-side the
  rubble is solid from the change's `at`; only the drawn fall spans the
  flight time. A later blast near rubble blows it away like a street
  fixture.
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
- **Wire (protocol 4):** `round.world` = changes not yet sent (each has `at`,
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
  list: seeded graded Voronoi fracture of the wall prisms in the blast
  (~30-40 sites at distance R x random^2, core/shard cells removed, outer
  chunks left standing; each connected opening becomes one jagged outline
  whose simplified torn edges follow the cells' own angles, piece-budgeted),
  ceiling/roof slab cuts, massing cuts (parapets break with the wall), floor
  holes, street fixtures blown away, and one Debris change with the rubble
  chunks' simulated falls. Native app: the "Debug bomb" section of the debug
  panel places it at the camera focus, detonates it now, or arms it for a
  time into the next round (the changes then carry that `at`, so playback
  and replays blow it at the same moment). Server: control taps
  `{"t":"place_bomb","room":..,"pos":[x,y,z],"radius":r}` and
  `{"t":"detonate_bomb","room":..}` (optional `"at":s` arms instead; an
  immediate detonation broadcasts the delta and re-sends each seat's
  fog-filtered state, since a fresh hole can reveal figures). Scenarios:
  `{action: bomb, position: [x,y,z], radius: r, at: s?}`.
