# World-change log

Map edits after generation (destructible world) are an ordered, timestamped
log of `WorldChange`s (`src/game/WorldChange.h`). Server and clients apply the
same log to the same seeded map, so drawn == simulated geometry.

- **Kinds:** obstacle removed / replaced by pieces, walk surface removed / cut,
  heightfield patch, wall hole (`FacadeOpening::Kind::Breach`).
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
- **Wire (protocol 2):** `round.world` = changes not yet sent (each has `at`,
  seconds into the round; the client applies them as playback passes it);
  `state.world` = full log (join/reconnect/spectator); `{"t":"world"}` for
  changes made outside a round. `RemoteClient` ignores ids it already has.
- **Debug:** control tap `{"t":"remove_obstacle","room":..,"obstacle":N}`
  removes an obstacle mid-match; `dump_state` reports `world_changes` and
  `checksum`.
