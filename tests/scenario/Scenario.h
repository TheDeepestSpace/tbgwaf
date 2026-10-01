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
}

namespace tactics::scenario {

struct ScenarioAction {
  enum class Kind { Move, Shoot, Pass, Cancel, Commit };

  int actor = -1;  // Unused (and not required in YAML) for Commit.
  Kind kind = Kind::Pass;
  glm::vec3 destination{0.0f};  // Move only.
  std::optional<float> finalFacingDegrees;  // Move only: re-aims the planned
                                             // wireframe before commit.
  int target = -1;              // Shoot only.
  bool expectNoop = false;      // Shoot only: the click is expected not to
                                 // resolve (e.g. target outside the
                                 // shooter's team FOV) and not record a plan.
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

  std::optional<int> round;
};

struct ScenarioStep {
  std::optional<ScenarioAction> action;
  std::optional<ScenarioAssertion> assertion;
};

struct Scenario {
  std::string name;
  std::string sourcePath;
  Scene scene;
  std::vector<ScenarioStep> steps;
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

  // Fired once at the initial state (completedActions == 0) and once after
  // each action step resolves (completedActions == 1, 2, ...). Assert-only
  // steps do not fire it: a "turn" for capture purposes is one executed
  // action.
  std::function<void(const GameLogic&, int completedActions)> onActionComplete;
};

// Runs `scenario` against a fresh GameLogic instance built from its scene,
// executing each step's action or checking its assertion in order.
ScenarioResult RunScenario(const Scenario& scenario, const PlaybackHooks& hooks);
ScenarioResult RunScenario(const Scenario& scenario);

}  // namespace tactics::scenario
