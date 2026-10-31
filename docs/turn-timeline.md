# Turn timeline (issue #143)

An on-screen strip at the bottom-middle of each pane for replaying the match
from the start or from any completed round ("turn" = one WEGO round).

## UI

- **Timeline line** with a circle at each turn boundary: `Start` (game
  start) plus `T1, T2, ...` (each round's end), spaced evenly (each
  turn is the same width; frames sit proportionally within their turn). A draggable handle moves freely along it (any recorded frame, not
  just boundaries) but **magnetizes** to a circle within ~14 px.
- **Play / Pause** replays forward from wherever the handle is
  (`TimelinePlayback::kReplaySpeed`, currently 1x) and pauses at the live
  end. Disabled while live (nothing ahead to play).
- **Live** is a lock-style toggle, shown pushed in while the live game is
  showing (handle parked at the right end). Dragging the handle to the right
  end locks Live; pressing Live while replaying returns to it.
- Widgets use tall frame padding and a wide slider grab so the strip is
  touch-friendly; it is anchored to the pane's bottom edge, clear of the
  Round panel (top-left), the team label (bottom-left corner) and the
  floating action menu (anchored above the selected figure).

## How it works

`src/game/TurnTimeline.h` holds both halves:

- **`TurnTimeline`** records `GameSnapshot` frames from the *local*
  `GameLogic`: one at game start, a ~30 Hz stream while a round executes (or
  a knockdown is still falling), and one at each round end / game over.
  Round boundaries become the labelled ticks. Replay therefore re-shows
  exactly what happened, frame by frame -- nothing is re-simulated, so shot
  outcomes can't diverge from the recorded match.
- **`TimelinePlayback`** imports those frames into its own private
  `GameLogic` (built from a copy of the live scene). While it is active the
  pane renders that instance instead of the live game, through the same
  `ComputeVisibility`/`RenderPane` path -- so each viewer sees the replayed
  moment with **their own team's fog of war** recomputed from the replayed
  positions. Ghosts and shot lines reappear in sequence as the replay plays
  forward (rebuilt, not recorded). Unlike live play they don't wait for the
  turn to end: on the replay instance a ghost holds `kReplayGhostHold` (1 s)
  after appearing and then fades, and a bullet line fades out within
  `kReplayTracerLife` (1 s) of the shot, carrying over into the next turn.

Replayed (non-current) turns show no plan details -- planned/armed next
moves, shot lines and plan rings are stripped from the replay instance.

## Decisions (as asked in the issue)

- **View-only replay, no branching.** Scrubbing back and pressing Play never
  truncates history or resumes live play from the past; the live match keeps
  running (and syncing) untouched underneath. While a replay is showing, all
  planning input and HUD game actions are inert -- return to Live first. A
  separate "Resume from here" can follow later.
- **Bus safety.** The replay `GameLogic` is a separate instance and nothing
  about it is ever serialized onto the page-level message bus, so the
  Blue-authority / Red-mirror protocol is untouched. Each web client records
  its own history from its own (imported) state and replays locally --
  **Red has its own timeline**, which is the simpler option because it needs
  no new bus messages; a follower that joins mid-match simply starts its
  history at the state it joined on. The native split-screen window keeps
  one shared timeline/replay for its one shared `GameLogic`: scrubbing in
  either pane shows the replay in both, each through its own fog.
- **New game clears history** (cf. #124). Every `Reset` path restarts the
  timeline at a fresh `Start` tick; a follower detects the restart from the
  round counter going backwards (same heuristic as `ImportState`).

## Tests

- `tests/logic_tests.cpp`: `TestTurnTimelineRecordsTicksAndReplays`,
  `TestTurnTimelineResetsOnNewMatch`.
- Scenario steps `timeline_seek` / `timeline_play` / `timeline_pause` and
  the `timeline_ticks` assertion drive the same controller from YAML; while
  a replay is active, assertions check the replayed state.
  `tests/scenarios/timeline_scrub_replay.yaml` covers replay from start /
  from a mid-game turn / scrub while playing;
  `tests/scenarios/timeline_new_game.yaml` covers the reset.
  The visual runner draws the timeline strip (and captures goldens of the
  replayed states) for scenarios that script timeline steps; other
  scenarios render unchanged, keeping their goldens stable.

Shot lines (live and replay) share `GameLogic::TracerAlphaForAge`: each fades
linearly to nothing within `kReplayTracerLife` (1 s) of its own shot. Ghosts
keep their per-turn behaviour.

## Turn animation (issue #237)

Visual-only: the sim facing still changes at once, but `Unit::turnLag` holds
the remaining angle (shortest arc, any heading change up to 180°) between it
and the drawn heading. `GameLogic::Update` eases it to 0 (~0.15 s for
45–90°, ~0.2 s for an about-face) while the figure is already walking, so a
turn adds no dead time. `Unit::VisualYaw()` drives the body, FoV cone and
shadow mask. The rig leads with torso/head, trails the legs, steps the foot
on the turning side when standing (pivot) and banks into the turn when
moving (path corners). `turnLag` is part of the snapshot, so followers and
replays draw the same pose. Scenario: `turn_animation_angles`.

## Figures may not overlap while walking (issue #267)

`GameLogic::PathOverlapsFigure` sweeps the round timeline in fixed 1/40 s
steps: the mover follows its candidate path from t = 0 (zipline rides priced
like the move budget), every other living figure follows its own planned move
or stands still, and any time two figures come within 2 × `kAgentRadius`
(and overlap vertically) the path is rejected. Pairs already overlapping at
t = 0 are ignored.

- **Crossing paths are rejected, not time-offset** -- walks all start at t = 0,
  so the player must route around the other figure's path. Only the first leg
  of a chain is checked (later legs run in later rounds).
- Unplanned *teammates* are ignored when planning (they will move); unplanned
  enemies count as standing. `CanCommitRound` is also false while any planned
  move overlaps (`HasPlanOverlap`), and the HUD says why; hover previews
  turn invalid (`MovePreviewBlockedByFigure`).
- **Jev**: `ApplyJevChoice` plans non-movers first, then each mover; a
  rejected move falls back to stopping 1-4 units short, the actor's other
  offered moves, then waiting. Routed legs never end on another figure.
- Tests/scenarios that deliberately collide use
  `SetFigureOverlapCheckEnabled(false)` / `figure_overlap: false`.
