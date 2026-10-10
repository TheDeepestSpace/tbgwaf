#pragma once

// YAML gameplay scenarios: a map + starting units + a scripted sequence of
// player-equivalent actions and state assertions, all driven through the
// same GameLogic click/choose API a real player uses (ClickUnit/ChooseMove/
// ChooseShoot/ChoosePass/ClickGround), so a scenario exercises exactly the
// same code path as the interactive game. No rendering: assertions only see
// game *state* (positions, alive/dead, FOV visibility, round number, winner).
//
// Actions only ever plan a figure's move/shoot/pass. Both teams plan the
// same WEGO round concurrently, so a script interleaves actors from either
// side freely; nothing executes until the script's own explicit
// `action: commit` step, which mirrors clicking "Commit Round" and plays
// out every figure's plan on both teams simultaneously.
//
// Optional top-level `flag: {position: [x, z] | [x, y, z], win_on_grab: bool}`
// enables the neutral flag objective (position defaults to the map center or
// the nearest free spot; win_on_grab defaults to true, false just carries/
// drops it and play goes on). `round_limit: N` ends the match as a draw after
// N rounds with no winner.
//
// `map.half_extent` sizes the ground square; units must start inside it.
//
// See tests/scenarios/*.yaml for the file format by example, and
// tests/scenario_tests.cpp for how these are run in CI.

#include <functional>
#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "game/Scene.h"
#include "game/Types.h"

namespace tactics {
class GameLogic;
class TurnTimeline;
class TimelinePlayback;
}

namespace tactics::scenario {

struct ScenarioAction {
  // Timeline* kinds (issue #143) drive the match's TurnTimeline replay the
  // way the HUD strip does: `timeline_seek` (field `tick`) snaps the replay
  // to a tick (0 = Start, 1 = T1, ...; the newest tick returns to the live
  // view), `timeline_play` starts/resumes playback and advances it by
  // `seconds` of replay time, `timeline_pause` pauses it. While the replay
  // is active, subsequent `assert` steps check the *replayed* state.
  // Aim (issue #129) is the tap-to-place half of a free-aim shot: it enters
  // shoot-target mode and places the "+" aim marker without confirming, so
  // the step ends mid-aim (for visual captures of the aiming UI). A
  // following `shoot` step by the same actor confirms it (the Fire button).
  // BeginMove opens move planning (select + Move) and deliberately stops
  // there, leaving the movement frontier up for the visual runner's
  // post-action capture; no destination is clicked and no plan is recorded.
  enum class Kind {
    Move, BeginMove, Shoot, Aim, Pass, Cancel, Commit, Focus, NewGame,
    TimelineSeek, TimelinePlay, TimelinePause,
  };

  int actor = -1;  // Unused (and not required in YAML) for Commit/NewGame/Timeline*.
  Kind kind = Kind::Pass;
  glm::vec3 destination{0.0f};  // Move only: the last leg's end.
  std::vector<glm::vec3> waypoints;  // Move only: earlier leg ends, clicked in
                                      // order before `destination` (multi-leg
                                      // plan, one leg executes per round).
  std::optional<float> finalFacingDegrees;  // Move only: re-aims the planned
                                             // wireframe before commit.
  int target = -1;              // Shoot only: locked-on figure target.
  bool expectNoop = false;      // Shoot only: the click is expected not to
                                 // resolve (e.g. target outside the
                                 // shooter's team FOV) and not record a plan.
  int timelineTick = 0;         // TimelineSeek only.
  float playSeconds = 0.0f;     // TimelinePlay only: replay time to advance.
  // Shoot/Aim free-aim forms (issue #129), mutually exclusive with `target`:
  // either the already-resolved world aim point (`at`, the protocol form --
  // deliberate blind fire at any point), or a camera-style ray
  // (`aim_from`/`aim_dir`) run through GameLogic::ResolveAimRay exactly like
  // a real click, so unit-under-cursor/surface precedence is what the
  // player would get; a ray with no aimable surface places nothing.
  std::optional<glm::vec3> shootAt;
  std::optional<glm::vec3> aimRayFrom;
  std::optional<glm::vec3> aimRayDir;
  // Shoot/Aim only (issue #138): burst size dialed in on the shot-level bar
  // before the target click / Fire press. Routed through
  // GameLogic::SetPlannedShotCount, so it clamps exactly like the UI
  // (never below 1, never past the weapon's magazine/round-window cap).
  // Absent means "bar untouched": the plan keeps whatever level is dialed in
  // (1 unless an earlier step of the same aim set it).
  std::optional<int> shots;
};

// Every field is optional; only the ones present in the YAML step are
// checked. `unit`-scoped fields (alive/position/visibleTo) require `unit` to
// also be set.
struct ScenarioAssertion {
  std::optional<int> unit;
  std::optional<bool> alive;
  std::optional<glm::vec3> position;
  std::optional<float> facingDegrees;  // Paired with `unit`; degrees, atan2(dz, dx).
  float tolerance = 0.05f;

  std::optional<Team> visibleToTeam;  // Paired with `visible`.
  std::optional<bool> visible;

  bool checkWinner = false;
  std::optional<Team> expectedWinner;  // nullopt means "no winner yet".

  // Sighting memory: `remembered_by` team has (remembered=true) or has no
  // (false) remembered trail of `unit`; `memory_age` is the oldest sample's
  // ageRounds (rounds completed since it was recorded).
  std::optional<Team> rememberedByTeam;
  std::optional<bool> remembered;
  std::optional<int> memoryAge;

  std::optional<int> round;

  // Number of ticks on the match's turn timeline (game start + one per
  // completed round; a new game resets it to 1). Issue #143.
  std::optional<int> timelineTicks;
  // Neutral flag (CTF part 1). `flag_carrier`: id of the carrying figure, or
  // -1 when nobody carries it. `flag_state`: "rest" (never picked up),
  // "carried" or "dropped". `flag_position`: where it is now (the carrier's
  // feet while carried), within `tolerance`. `flag_visible_to`: the team's
  // fog-of-war view includes the flag.
  std::optional<int> flagCarrier;
  std::optional<std::string> flagState;
  std::optional<glm::vec3> flagPosition;
  std::optional<Team> flagVisibleTo;
};

struct ScenarioStep {
  std::optional<ScenarioAction> action;
  std::optional<ScenarioAssertion> assertion;
};

struct Scenario {
  std::string name;
  std::string sourcePath;
  Scene scene;
  SquadPlaybook playbooks[2] = {SquadPlaybook::Passive(), SquadPlaybook::Passive()};  // Indexed by Team; passive unless the YAML sets `playbook`.
  // `friendly_fire: false` flips the single config flag that makes
  // same-team figures transparent to free-aim ballistic traces.
  bool friendlyFire = true;
  // `shot_rolls: [0.1, 0.9, ...]`: pins the shot RNG for deterministic
  // outcomes -- rolls are consumed in resolution order and the list repeats
  // when exhausted. Empty keeps the default seeded RNG.
  std::vector<float> shotRolls;
  std::vector<ScenarioStep> steps;
  // Optional visual-runner camera adjustments (both panes), applied after
  // the initial view is fitted to the map: orbit target on the ground plane,
  // and zoom delta (positive = further out).
  std::optional<glm::vec2> cameraTarget;
  float cameraZoom = 0.0f;
  // Optional visual-runner render settings (`render:` section). Ignored by
  // the logic-only runner.
  //   fov_overlay: "cpu" (default) | "shadow_map" -- issue #110 prototype.
  //   fov_probe_height: shadow_map only; see SceneRenderer::SetFovProbeHeight.
  enum class FovOverlay { Cpu, ShadowMap };
  FovOverlay fovOverlay = FovOverlay::Cpu;
  float fovProbeHeight = 0.0f;
};

// Throws std::runtime_error with a descriptive message on malformed YAML.
Scenario LoadScenarioFromFile(const std::string& path);

struct ScenarioResult {
  bool Passed() const { return failures.empty(); }
  std::vector<std::string> failures;
};

// Optional observation points for the expensive/visual run mode (issue #14
// stage 2/3). The default-constructed value reproduces the cheap/logic-only
// mode exactly: move animations are fast-forwarded in a single Update()
// call and no callbacks fire.
struct PlaybackHooks {
  // When > 0, move animations are advanced in fixed ticks of this many
  // seconds (with onFrame fired after each tick) instead of being
  // fast-forwarded, so a frame-by-frame capture sees the actual motion.
  float tickSeconds = 0.0f;

  // Extra onFrame calls emitted while the state is at rest: once at the
  // initial state and once after each completed action, so captured video
  // holds on each turn's outcome instead of cutting instantly.
  int holdFramesAfterAction = 0;

  // Fired for every playback frame (initial holds, each move tick, and
  // post-action holds). The game state is mid-scenario; do not mutate it.
  std::function<void(const GameLogic&)> onFrame;

  // Fired just before each scripted click is applied (unit selection, move
  // destination, shoot target) by `team`'s player, with the world-space point the equivalent
  // real mouse click would land on (unit head for figures, the ground point
  // for destinations). Lets a renderer show the cursor landing before the
  // state changes. The game state is the pre-click state; do not mutate it.
  std::function<void(const GameLogic&, Team team, const glm::vec3& worldPoint)> onClick;

  // Fired just before a scripted press of a HUD action-menu button ("Move",
  // "Shoot", "Pass", "Cancel") by `team`'s player, after the actor has been
  // selected. The game state is the pre-press state (so the menu the player
  // would be clicking is actually visible); do not mutate it. `button` is
  // the button's on-screen label.
  std::function<void(const GameLogic&, Team team, const char* button)> onMenuClick;

  // Fired holdFramesAfterAction times (min 1) for each Move step, right after
  // ChooseMove() while the game is in AwaitingMoveDestination, so a capture
  // can show the mover's movement frontier. Team is the mover's. Only
  // consumers that record video need set it; screenshots are unaffected.
  std::function<void(const GameLogic&, Team)> onMoveFrontier;

  // Fired once at the initial state (completedActions == 0) and once after
  // each action step resolves (completedActions == 1, 2, ...). Assert-only
  // steps do not fire it: a "turn" for capture purposes is one executed
  // action.
  std::function<void(const GameLogic&, int completedActions)> onActionComplete;

  // Fired for a `focus` step (a double-click on a figure) right after the
  // figure is selected. The visual runner eases the owning pane's camera
  // over the figure's movement frontier here, emitting its own frames.
  std::function<void(const GameLogic&, int unitId, Team team)> onFocus;

  // Fired once before the first step with the turn-timeline recorder and
  // replay controller that drive the scenario's `timeline_*` steps (both
  // outlive the RunScenario call). Lets the visual runner draw the HUD
  // timeline strip for scenarios that script it. When a replay is active,
  // the GameLogic passed to onFrame/onActionComplete is the *replayed*
  // state, exactly what the app would render.
  std::function<void(const TurnTimeline&, const TimelinePlayback&)> onTimeline;
};

// Runs `scenario` against a fresh GameLogic instance built from its scene,
// executing each step's action or checking its assertion in order.
ScenarioResult RunScenario(const Scenario& scenario, const PlaybackHooks& hooks);
ScenarioResult RunScenario(const Scenario& scenario);

}  // namespace tactics::scenario
