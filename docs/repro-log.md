# Demo-scene repro logs

The **Demo Scenes** game mode (issue #240) exists to reproduce odd behaviour
and hand the evidence to a developer or an agent. Game Mode → Demo Scenes
shows one card per registered scene (thumbnail, name, what to do); picking
one starts a normal match on that small fixed setup with the usual controls,
and everything needed to reproduce the session exactly is recorded. **Download
log** in the top-center panel exports it: a single self-describing JSON Lines
file. It is the VCD-file equivalent of a hardware simulation: a complete
input trace with checkpoints, replayable headlessly.

## Replaying a log

```sh
build/tactics_repro_replay repro-odd-ragdoll-repro-20261027-101500.jsonl
```

rebuilds the scene, feeds every recorded input back through the same
`GameLogic` calls, prints the simulation's own log lines and checks the state
hash after every entry against the recorded one (exit 0 = exact
reproduction). To watch it instead:

```sh
TBGWAF_REPRO_LOG=repro-....jsonl build/tactics_app
```

opens the app straight on the replayed final state with the whole session on
the turn timeline, so it can be scrubbed and viewed through either team's
fog. (Native only; the loaded session is not re-recorded.)

`tests/repro_log_tests.cpp` proves record → replay equality on the first
scene (and that a tampered log is caught); ctest's `demo_smoke_test` runs the
real app headlessly through the Demo Scenes flow, writes a log and replays
it.

### Determinism

A replay is bit-exact on the same build and platform: the shot RNG is seeded
from the log, every float is written with enough digits to round-trip, and
each frame's `dt` is recorded (the game's idle/animation clocks live in the
hashed state, so even planning-phase frames count). Across platforms (a log
recorded in the browser replayed natively) libm differences can shift a
float in the last place; the replay then reports the first diverging entry
rather than failing outright, and the sim log lines still say what happened.

## Format (version 1)

One JSON object per line. The first line is the header; every later line has
`type`, `seq` (1-based, increasing), `round` (the round number after the
entry) and `t` (wall-clock milliseconds since the recording started).

```jsonl
{"type":"header","format":"tbgwaf-repro-log","version":1,"scene":"odd-ragdoll-repro","seed":239,"build":"1938d0b","created":"2026-10-27T10:15:00Z","initial_hash":"5f1c…"}
{"type":"updates","seq":1,"round":1,"t":0,"dts":[0.0166999996,0.0170000009],"hash":"…"}
{"type":"input","seq":2,"round":1,"t":540,"op":"click_unit","unit":2,"team":"red","hash":"…"}
{"type":"input","seq":3,"round":1,"t":540,"op":"choose_move","hash":"…"}
{"type":"input","seq":4,"round":1,"t":2210,"op":"click_ground","point":[5,0,6],"team":"red","hash":"…"}
{"type":"input","seq":9,"round":1,"t":6100,"op":"commit","hash":"…"}
{"type":"log","seq":11,"round":1,"t":8120,"msg":"bullet shooter=1 index=0 at=0.0000 from=(…) to=(…)"}
{"type":"log","seq":15,"round":1,"t":10160,"msg":"knockdown unit=2 pos=(5.0000,0.0000,0.3200) dir=(0.9980,0.0000,0.0620) exec=2.0010 round=1"}
```

### Header

| field | meaning |
| --- | --- |
| `version` | Format version; a reader rejects versions it does not know. |
| `scene` | Demo scene id (`DemoScene::id`); the replayer rebuilds the scene from the registry, so the map and figures are not in the log. |
| `seed` | Shot-RNG seed applied before the first input. |
| `build` | Git short hash of the recording build (`unknown` outside a checkout). |
| `created` | UTC timestamp, informational. |
| `initial_hash` | State hash before the first input. |

### `updates`

Consecutive frames: `dts` is the list of `Update(dt)` seconds, in order, and
`hash` the state after the last of them. A batch closes at the next other
input, at a sim log line, when the frame changed the game's mode (a round
ended, the game ended) or after 600 frames.

### `input`

`op` names the `GameLogic` call; the other fields are its arguments:

| `op` | fields | call |
| --- | --- | --- |
| `click_unit` | `unit`, `team` | `ClickUnit` |
| `click_ground` | `point`, `team` | `ClickGround` |
| `finish_move` | | `FinishMovePlan` |
| `choose_move` / `choose_shoot` / `choose_pass` / `cancel` | | `ChooseMove` / `ChooseShoot` / `ChoosePass` / `CancelAction` |
| `set_move_facing` | `unit`, `yaw`, `team` | `SetPlannedMoveFacing` |
| `click_aim_ray` | `origin`, `dir`, `team` | `ClickAimRay` |
| `place_aim` | `point`, `team` | `PlaceAimPoint` |
| `confirm_aim` | `team` | `ConfirmAim` |
| `set_shots` | `count`, `team` | `SetPlannedShotCount` |
| `set_playbook` | `team`, `table` (4 reaction ids, `[stationary-unseen, stationary-seen, moving-unseen, moving-seen]`), `ignore_idle` | `SetPlaybook` |
| `set_friendly_fire` | `enabled` | `SetFriendlyFireEnabled` |
| `commit` | | `CommitRound` |
| `clear_plans` | | `ClearAllPlans` |
| `reset` | | `Reset` with the demo scene (New Match) |
| `import_plans` | `team`, `snapshot` | `ImportTeamPlans` (web two-canvas play: the other side's plans arrive as a snapshot) |
| `import_state` | `snapshot` | `ImportState` |

`hash` is the state after the call. Teams are `"blue"` / `"red"`; points are
`[x, y, z]`.

### `log`

`msg` is a free-form simulation line, emitted from inside the frame it
happened in: `bullet …` for every bullet fired, `shot …` for a reaction
shot, `knockdown unit=… dir=…` when a figure goes down (with the direction it
is knocked in and the execution-clock time), `round N finished`. Replay
ignores them; they are there for the reader.

### State hash

`GameLogic::StateHash()`: FNV-1a (64-bit, hex) of `SerializeSnapshot` of
`ExportState()`, i.e. the same text the two-canvas web build already sends
between its instances: every figure's position, facing, alive flag, plan,
knockdown and animation clocks, the bullet lines, playbooks, mode, round,
execution clock, winner and flag.

## Adding a scene

Append a `DemoScene` to `kDemoScenes` in `src/game/DemoScenes.cpp`: id, name,
description (the instructions on the card), RNG seed, a function building the
`Scene`, and the thumbnail camera (look-at point, zoom delta). Add a
`tests/scenarios/demo_<id>.yaml` with `map: {demo_scene: <id>}` that scripts
the behaviour the scene is for, and regenerate its goldens with
`scripts/update_golden_baselines.sh`. Thumbnails are rendered by the app the
first time the Demo Scenes screen opens, so no asset is needed.
