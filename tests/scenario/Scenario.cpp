#include "scenario/Scenario.h"

#include <algorithm>
#include <cstdio>
#include <stdexcept>
#include <utility>

#include <yaml-cpp/yaml.h>

#include "game/GameLogic.h"

namespace tactics::scenario {
namespace {

constexpr float kPi = 3.14159265358979323846f;

std::string ToString(const glm::vec3& v) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), "(%.3f, %.3f, %.3f)", v.x, v.y, v.z);
  return buf;
}

std::string ToString(Team team) { return team == Team::Blue ? "blue" : "red"; }

std::string ToString(const std::optional<Team>& team) {
  if (!team) return "none";
  return ToString(*team);
}

Team ParseTeam(const std::string& raw, const std::string& context) {
  if (raw == "blue") return Team::Blue;
  if (raw == "red") return Team::Red;
  throw std::runtime_error("unknown team '" + raw + "' in " + context);
}

glm::vec3 ParseVec3(const YAML::Node& node, const std::string& context) {
  if (!node || !node.IsSequence() || node.size() != 3) {
    throw std::runtime_error(context + " must be a 3-element [x, y, z] list");
  }
  return glm::vec3(node[0].as<float>(), node[1].as<float>(), node[2].as<float>());
}

glm::vec2 ParseVec2(const YAML::Node& node, const std::string& context) {
  if (!node || !node.IsSequence() || node.size() != 2) {
    throw std::runtime_error(context + " must be a 2-element [x, z] list");
  }
  return glm::vec2(node[0].as<float>(), node[1].as<float>());
}

Scene ParseScene(const YAML::Node& root) {
  Scene scene;

  if (const YAML::Node mapNode = root["map"]) {
    if (const YAML::Node obstaclesNode = mapNode["obstacles"]) {
      for (const auto& obsNode : obstaclesNode) {
        const glm::vec2 center = ParseVec2(obsNode["center"], "map.obstacles[].center");
        const glm::vec2 halfExtent =
            ParseVec2(obsNode["half_extent"], "map.obstacles[].half_extent");
        if (!obsNode["height"]) {
          throw std::runtime_error("map.obstacles[] requires 'height'");
        }
        const float height = obsNode["height"].as<float>();

        Obstacle obstacle;
        obstacle.bounds =
            AABB{glm::vec3(center.x - halfExtent.x, 0.0f, center.y - halfExtent.y),
                 glm::vec3(center.x + halfExtent.x, height, center.y + halfExtent.y)};
        obstacle.climbable = obsNode["climbable"] ? obsNode["climbable"].as<bool>() : false;
        scene.obstacles.push_back(obstacle);
      }
    }
  }

  const YAML::Node unitsNode = root["units"];
  if (!unitsNode || !unitsNode.IsSequence() || unitsNode.size() == 0) {
    throw std::runtime_error("scenario must declare at least one unit under 'units'");
  }
  for (const auto& unitNode : unitsNode) {
    if (!unitNode["id"] || !unitNode["team"] || !unitNode["position"]) {
      throw std::runtime_error("units[] requires 'id', 'team', and 'position'");
    }
    Unit unit;
    unit.id = unitNode["id"].as<int>();
    unit.team = ParseTeam(unitNode["team"].as<std::string>(), "units[].team");
    unit.position = ParseVec3(unitNode["position"], "units[].position");
    unit.facingYaw =
        unitNode["facing_degrees"] ? unitNode["facing_degrees"].as<float>() * kPi / 180.0f : 0.0f;
    unit.alive = true;
    scene.units.push_back(unit);
  }
  return scene;
}

ScenarioAction ParseAction(const YAML::Node& node) {
  ScenarioAction action;
  const std::string kind = node["action"].as<std::string>();
  if (kind == "commit") {
    action.kind = ScenarioAction::Kind::Commit;
    return action;
  }

  if (!node["actor"]) throw std::runtime_error("script action step requires 'actor'");
  action.actor = node["actor"].as<int>();
  if (kind == "move") {
    action.kind = ScenarioAction::Kind::Move;
    action.destination = ParseVec3(node["destination"], "script[].destination");
  } else if (kind == "shoot") {
    action.kind = ScenarioAction::Kind::Shoot;
    if (!node["target"]) throw std::runtime_error("script 'shoot' action requires 'target'");
    action.target = node["target"].as<int>();
    action.expectNoop = node["expect_noop"] && node["expect_noop"].as<bool>();
  } else if (kind == "pass") {
    action.kind = ScenarioAction::Kind::Pass;
  } else if (kind == "cancel") {
    action.kind = ScenarioAction::Kind::Cancel;
  } else {
    throw std::runtime_error("unknown script action '" + kind +
                              "' (expected move/shoot/pass/cancel/commit)");
  }
  return action;
}

ScenarioAssertion ParseAssertion(const YAML::Node& node) {
  ScenarioAssertion assertion;
  if (node["unit"]) assertion.unit = node["unit"].as<int>();
  if (node["alive"]) assertion.alive = node["alive"].as<bool>();
  if (node["position"]) assertion.position = ParseVec3(node["position"], "assert.position");
  if (node["tolerance"]) assertion.tolerance = node["tolerance"].as<float>();
  if (node["visible_to"]) {
    assertion.visibleToTeam = ParseTeam(node["visible_to"].as<std::string>(), "assert.visible_to");
  }
  if (node["visible"]) assertion.visible = node["visible"].as<bool>();
  if (node["current_team"]) {
    assertion.currentTeam = ParseTeam(node["current_team"].as<std::string>(), "assert.current_team");
  }
  if (node["round"]) assertion.round = node["round"].as<int>();
  if (node["winner"]) {
    assertion.checkWinner = true;
    const std::string raw = node["winner"].as<std::string>();
    assertion.expectedWinner = raw == "none" ? std::optional<Team>() : ParseTeam(raw, "assert.winner");
  }

  if ((assertion.alive || assertion.position || assertion.visible) && !assertion.unit) {
    throw std::runtime_error("assert checking alive/position/visible requires 'unit'");
  }
  if (assertion.visible.has_value() != assertion.visibleToTeam.has_value()) {
    throw std::runtime_error("assert 'visible' and 'visible_to' must be set together");
  }
  return assertion;
}

bool ExecuteAction(GameLogic& game, const ScenarioAction& action, int stepIndex,
                    const PlaybackHooks& hooks, ScenarioResult* result) {
  auto Fail = [&](const std::string& msg) {
    result->failures.push_back("step " + std::to_string(stepIndex) + " (actor " +
                                std::to_string(action.actor) + "): " + msg);
    return false;
  };

  if (action.kind == ScenarioAction::Kind::Commit) {
    if (!game.CanCommitTurn()) {
      return Fail("cannot commit: not every living figure on " + ToString(game.CurrentTeam()) +
                  "'s team has a plan yet");
    }
    game.CommitTurn();
    if (hooks.tickSeconds > 0.0f) {
      // Visual mode: advance in fixed ticks and let the observer capture
      // each in-between frame of the commit's animation. Bounded so a stuck
      // animation fails the scenario instead of hanging the runner.
      constexpr int kMaxMoveTicks = 20000;
      int ticks = 0;
      while (game.Mode() == InputMode::Moving && ++ticks <= kMaxMoveTicks) {
        game.Update(hooks.tickSeconds);
        if (hooks.onFrame) hooks.onFrame(game);
      }
      if (game.Mode() == InputMode::Moving) {
        return Fail("commit animation did not complete within " +
                    std::to_string(kMaxMoveTicks) + " ticks");
      }
    } else {
      game.Update(1.0e6f);  // Fast-forward past any planned moves' animation.
    }
    return true;
  }

  if (action.kind == ScenarioAction::Kind::Cancel) {
    game.CancelAction();
    return true;
  }

  // Move/Shoot/Pass only ever record a plan on the acting figure, so it must
  // belong to the team currently planning its turn.
  const Unit* actorUnit = game.FindUnit(action.actor);
  if (!actorUnit || !actorUnit->alive) {
    return Fail("is dead or does not exist");
  }
  if (actorUnit->team != game.CurrentTeam()) {
    return Fail("belongs to a team that isn't planning right now (it's " +
                ToString(game.CurrentTeam()) + "'s turn)");
  }

  game.ClickUnit(action.actor);
  if (game.SelectedUnitId() != action.actor || game.Mode() != InputMode::ActionMenu) {
    return Fail("could not be selected (already game over?)");
  }

  switch (action.kind) {
    case ScenarioAction::Kind::Move: {
      game.ChooseMove();
      game.ClickGround(action.destination);
      game.ConfirmMove();  // Untouched ghost: natural travel-direction facing.
      if (game.Mode() != InputMode::AwaitingSelection) {
        return Fail("has no path to destination " + ToString(action.destination));
      }
      return true;
    }
    case ScenarioAction::Kind::Shoot: {
      game.ChooseShoot();
      game.ClickUnit(action.target);
      const bool planned = game.Mode() != InputMode::AwaitingShootTarget;
      if (action.expectNoop && planned) {
        return Fail("shot at " + std::to_string(action.target) +
                    " was expected to be a gated no-op, but it was planned");
      }
      if (!action.expectNoop && !planned) {
        return Fail("shot at " + std::to_string(action.target) +
                    " could not be planned (invalid target, or outside the shooter's team FOV?)");
      }
      if (action.expectNoop) game.CancelAction();  // Return to ActionMenu, mirroring a real player.
      return true;
    }
    case ScenarioAction::Kind::Pass:
      game.ChoosePass();
      return true;
    case ScenarioAction::Kind::Cancel:
    case ScenarioAction::Kind::Commit:
      break;  // Handled above.
  }
  return true;
}

void CheckAssertion(const GameLogic& game, const ScenarioAssertion& a, int stepIndex,
                     ScenarioResult* result) {
  auto Fail = [&](const std::string& msg) {
    result->failures.push_back("step " + std::to_string(stepIndex) + " (assert): " + msg);
  };

  if (a.unit) {
    const Unit* unit = game.FindUnit(*a.unit);
    if (!unit) {
      Fail("references unknown unit " + std::to_string(*a.unit));
    } else {
      if (a.alive && unit->alive != *a.alive) {
        Fail("unit " + std::to_string(*a.unit) + " expected alive=" +
             (*a.alive ? "true" : "false") + " but was " + (unit->alive ? "true" : "false"));
      }
      if (a.position) {
        const float dist = glm::distance(unit->position, *a.position);
        if (dist > a.tolerance) {
          Fail("unit " + std::to_string(*a.unit) + " expected position near " +
               ToString(*a.position) + " but is at " + ToString(unit->position) + " (off by " +
               std::to_string(dist) + ")");
        }
      }
      if (a.visible) {
        const bool actual = game.ComputeVisibility(*a.visibleToTeam).UnitVisible(*a.unit);
        if (actual != *a.visible) {
          Fail("unit " + std::to_string(*a.unit) + " expected visible_to " +
               ToString(a.visibleToTeam) + "=" + (*a.visible ? "true" : "false") + " but was " +
               (actual ? "true" : "false"));
        }
      }
    }
  }

  if (a.checkWinner && game.Winner() != a.expectedWinner) {
    Fail("expected winner=" + ToString(a.expectedWinner) + " but was " + ToString(game.Winner()));
  }

  if (a.currentTeam && game.CurrentTeam() != *a.currentTeam) {
    Fail("expected current team " + ToString(*a.currentTeam) + " but it's " +
         ToString(game.CurrentTeam()) + "'s turn");
  }

  if (a.round && game.RoundNumber() != *a.round) {
    Fail("expected round " + std::to_string(*a.round) + " but was " +
         std::to_string(game.RoundNumber()));
  }
}

}  // namespace

Scenario LoadScenarioFromFile(const std::string& path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("failed to parse " + path + ": " + e.what());
  }

  Scenario scenario;
  scenario.sourcePath = path;
  scenario.name = root["name"] ? root["name"].as<std::string>() : path;
  scenario.scene = ParseScene(root);

  if (const YAML::Node scriptNode = root["script"]) {
    for (const auto& stepNode : scriptNode) {
      ScenarioStep step;
      if (stepNode["action"]) {
        step.action = ParseAction(stepNode);
      } else if (stepNode["assert"]) {
        step.assertion = ParseAssertion(stepNode["assert"]);
      } else {
        throw std::runtime_error("script step must have either 'action' or 'assert'");
      }
      scenario.steps.push_back(std::move(step));
    }
  }
  return scenario;
}

ScenarioResult RunScenario(const Scenario& scenario, const PlaybackHooks& hooks) {
  ScenarioResult result;
  GameLogic game(scenario.scene);

  auto EmitHoldFrames = [&] {
    if (!hooks.onFrame) return;
    for (int i = 0; i < std::max(1, hooks.holdFramesAfterAction); ++i) hooks.onFrame(game);
  };

  EmitHoldFrames();
  if (hooks.onActionComplete) hooks.onActionComplete(game, 0);

  int completedActions = 0;
  for (int i = 0; i < static_cast<int>(scenario.steps.size()); ++i) {
    const ScenarioStep& step = scenario.steps[i];
    if (step.action) {
      if (!ExecuteAction(game, *step.action, i, hooks, &result)) {
        break;  // The script's own preconditions were violated; state past this point is unreliable.
      }
      ++completedActions;
      EmitHoldFrames();
      if (hooks.onActionComplete) hooks.onActionComplete(game, completedActions);
    } else {
      CheckAssertion(game, *step.assertion, i, &result);
    }
  }
  return result;
}

ScenarioResult RunScenario(const Scenario& scenario) { return RunScenario(scenario, {}); }

}  // namespace tactics::scenario
