# World-change log

Map edits after generation (destructible world) are an ordered, timestamped
log of `WorldChange`s (`src/game/WorldChange.h`). Server and clients apply the
same log to the same seeded map, so drawn == simulated geometry.

- **Kinds:** obstacle removed / replaced by pieces, walk surface removed / cut,
  heightfield patch, wall hole (`FacadeOpening::Kind::Breach`), floor hole.
- **Wall holes** really cut the wall (issue #350): the target prism dies and
  up to four plain prisms (jambs, sill, head) are appended around the
  opening, CutWindows-style, so LOS, shots and grenade flight pass through
  the gap with no downstream special cases; a hole reaching the floor with
  head-height clearance becomes a doorway for the navmesh automatically. The
  Breach `FacadeOpening` records the outline. **Floor holes** cut a rect out
  of a building floor surface: the patch dies, up to four convex pieces are
  appended and linked to each other and to the old neighbours still touching
  them; a figure standing over the hole falls (ragdoll, with the drop to
  whatever support remains below). The matching ceiling-slab cut is a
  separate `ObstacleReplaced` change.
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
- **Wire (protocol 3):** `round.world` = changes not yet sent (each has `at`,
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
  chunks left standing, rect-merged per wall so every piece stays an upright
  prism, piece-budgeted), ceiling/roof slab cuts, floor holes and street
  fixtures blown away. Native app: the "Debug bomb" section of the debug
  panel places it at the camera focus, detonates it now, or arms it for a
  time into the next round (the changes then carry that `at`, so playback
  and replays blow it at the same moment). Server: control taps
  `{"t":"place_bomb","room":..,"pos":[x,y,z],"radius":r}` and
  `{"t":"detonate_bomb","room":..}` (optional `"at":s` arms instead; an
  immediate detonation broadcasts the delta and re-sends each seat's
  fog-filtered state, since a fresh hole can reveal figures). Scenarios:
  `{action: bomb, position: [x,y,z], radius: r, at: s?}`.
