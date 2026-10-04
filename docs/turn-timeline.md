# Turn timeline (issue #143)

An on-screen strip at the bottom-middle of each pane for replaying the match
from the start or from any completed round ("turn" = one WEGO round).

## UI

- **Slider** with one stop per timeline tick: `Start` (game start) plus
  `T1, T2, ...` (each round's end). It snaps to ticks and shows the current
  tick's label; selecting a tick restores the state at that point.
- **Play / Pause** replays forward from the selected tick at replay speed
  (`TimelinePlayback::kReplaySpeed`, currently 1x) and pauses when it
  reaches the live end.
- **Live** (shown only while a replay is being viewed) returns to the live
  game. Scrubbing the slider to the newest tick while paused does the same.
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
  positions. Sighting-memory ghost trails are rebuilt as the replay plays
  forward (approximate, not recorded).

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
