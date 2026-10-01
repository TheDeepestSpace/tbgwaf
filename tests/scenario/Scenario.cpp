#include <cmath>
#include "scenario/Scenario.h"

#include <algorithm>
#include <cstdio>
#include <stdexcept>
#include <utility>

#include <yaml-cpp/yaml.h>

#include "game/MapGenerator.h"
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

  bool generated = false;
  if (const YAML::Node mapNode = root["map"]) {
    // `generate: {seed: N}` builds a procedural city (units come from the
    // generator unless the scenario lists its own).
    if (const YAML::Node genNode = mapNode["generate"]) {
      if (!genNode["seed"]) throw std::runtime_error("map.generate requires 'seed'");
      scene = GenerateUrbanMap(genNode["seed"].as<uint32_t>());
      generated = true;
    }
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
  if (generated && !unitsNode) return scene;
  if (generated) scene.units.clear();
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
    if (const YAML::Node rule = unitNode["reaction_on_stationary"]) {
      const std::string name = rule.as<std::string>();
      if (name == "shoot") {
        unit.reactionOnStationary = ReactionRule::Shoot;
      } else if (name != "do_nothing") {
        throw std::runtime_error("units[].reaction_on_stationary must be 'shoot' or 'do_nothing', got '" +
                                 name + "'");
      }
    }
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
    if (const YAML::Node waypoints = node["waypoints"]) {
      if (!waypoints.IsSequence()) {
        throw std::runtime_error("script[].waypoints must be a list of [x, y, z]");
      }
      for (const auto& wp : waypoints) {
        action.waypoints.push_back(ParseVec3(wp, "script[].waypoints[]"));
      }
    }
    if (node["final_facing_degrees"]) {
      action.finalFacingDegrees = node["final_facing_degrees"].as<float>();
    }
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
  if (node["facing_degrees"]) assertion.facingDegrees = node["facing_degrees"].as<float>();
  if (node["tolerance"]) assertion.tolerance = node["tolerance"].as<float>();
  if (node["visible_to"]) {
    assertion.visibleToTeam = ParseTeam(node["visible_to"].as<std::string>(), "assert.visible_to");
  }
  if (node["visible"]) assertion.visible = node["visible"].as<bool>();
  if (node["round"]) assertion.round = node["round"].as<int>();
  if (node["winner"]) {
    assertion.checkWinner = true;
    const std::string raw = node["winner"].as<std::string>();
    assertion.expectedWinner = raw == "none" ? std::optional<Team>() : ParseTeam(raw, "assert.winner");
  }

  if ((assertion.alive || assertion.position || assertion.facingDegrees ||
       assertion.visible) && !assertion.unit) {
    throw std::runtime_error("assert checking alive/position/facing_degrees/visible requires 'unit'");
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

  // Reports where the equivalent mouse click would land (a figure's head, to
  // match the HUD's anchor point) before driving the state.
  auto NotifyClick = [&](Team team, const glm::vec3& worldPoint) {
    if (hooks.onClick) hooks.onClick(game, team, worldPoint);
  };
  auto ClickUnitAt = [&](int id, Team team) {
    if (const Unit* unit = game.FindUnit(id)) {
      NotifyClick(team, unit->position + glm::vec3(0.0f, 1.9f, 0.0f));
    }
    game.ClickUnit(id, team);
  };
  // Reports the HUD action-menu button the equivalent real player would
  // press, while the pre-press state (menu included) is still showing.
  auto NotifyMenuClick = [&](Team team, const char* button) {
    if (hooks.onMenuClick) hooks.onMenuClick(game, team, button);
  };

  if (action.kind == ScenarioAction::Kind::Commit) {
    if (!game.CanCommitRound()) {
      return Fail("cannot commit: not every living figure (on both teams) has a plan yet");
    }
    // Either pane has the button; show the press in Blue's.
    NotifyMenuClick(Team::Blue, "Commit Round");
    game.CommitRound();
    if (hooks.tickSeconds > 0.0f) {
      // Visual mode: advance in fixed ticks and let the observer capture
      // each in-between frame of the round's execution. Bounded so a stuck
      // animation fails the scenario instead of hanging the runner.
      constexpr int kMaxMoveTicks = 20000;
      int ticks = 0;
      while (game.Mode() == InputMode::Executing && ++ticks <= kMaxMoveTicks) {
        game.Update(hooks.tickSeconds);
        game.UpdateSightingMemory(hooks.tickSeconds);
        if (hooks.onFrame) hooks.onFrame(game);
      }
      // Shots resolve at commit, so the fall may outlive (or entirely
      // precede) the round's execution; keep ticking until it lands.
      while (game.HasActiveKnockdown() && ++ticks <= kMaxMoveTicks) {
        game.Update(hooks.tickSeconds);
        game.UpdateSightingMemory(hooks.tickSeconds);
        if (hooks.onFrame) hooks.onFrame(game);
      }
      if (game.Mode() == InputMode::Executing) {
        return Fail("round execution did not complete within " +
                    std::to_string(kMaxMoveTicks) + " ticks");
      }
    } else {
      // Fast-forward the executing round. Step coarsely (not one giant tick)
      // so sighting memory still samples the figures along the way.
      constexpr float kFastStepSeconds = 0.1f;
      constexpr int kMaxFastSteps = 20000;
      for (int i = 0; game.Mode() == InputMode::Executing && i < kMaxFastSteps; ++i) {
        game.Update(kFastStepSeconds);
        game.UpdateSightingMemory(kFastStepSeconds);
      }
      game.Update(1.0e6f);  // Settle anything still pending (e.g. knockdowns).
      game.UpdateSightingMemory(0.0f);
    }
    return true;
  }

  if (action.kind == ScenarioAction::Kind::Cancel) {
    game.CancelAction();
    return true;
  }

  // Move/Shoot/Pass only ever record a plan on the acting figure. Both
  // teams plan concurrently, so scripts may freely interleave actors from
  // either side before a single round commit; each click is tagged with the
  // actor's own team, the same as a click landing in that player's pane.
  const Unit* actorUnit = game.FindUnit(action.actor);
  if (!actorUnit || !actorUnit->alive) {
    return Fail("is dead or does not exist");
  }
  const Team actorTeam = actorUnit->team;

  ClickUnitAt(action.actor, actorTeam);
  if (game.SelectedUnitId() != action.actor || game.Mode() != InputMode::ActionMenu) {
    return Fail("could not be selected (already game over?)");
  }

  switch (action.kind) {
    case ScenarioAction::Kind::Move: {
      NotifyMenuClick(actorTeam, "Move");
      game.ChooseMove();
      if (hooks.onMoveFrontier) {
        for (int i = 0; i < std::max(1, hooks.holdFramesAfterAction); ++i) {
          hooks.onMoveFrontier(game, actorTeam);
        }
      }
      for (const glm::vec3& waypoint : action.waypoints) {
        NotifyClick(actorTeam, waypoint);
        game.ClickGround(waypoint, actorTeam);
        if (glm::distance(game.MoveChainEnd(), waypoint) > 0.01f) {
          return Fail("waypoint " + ToString(waypoint) + " was rejected (no path, or beyond one round's reach of the previous leg)");
        }
      }
      NotifyClick(actorTeam, action.destination);
      game.ClickGround(action.destination, actorTeam);
      game.FinishMovePlan();
      if (game.Mode() != InputMode::AwaitingSelection) {
        return Fail("has no path to destination " + ToString(action.destination) +
                    " (unreachable, or beyond the mover's round move budget)");
      }
      if (action.finalFacingDegrees) {
        game.SetPlannedMoveFacing(action.actor, *action.finalFacingDegrees * 3.14159265f / 180.0f,
                                  actorTeam);
      }
      return true;
    }
    case ScenarioAction::Kind::Shoot: {
      NotifyMenuClick(actorTeam, "Shoot");
      game.ChooseShoot();
      ClickUnitAt(action.target, actorTeam);
      const bool planned = game.Mode() != InputMode::AwaitingShootTarget;
      if (action.expectNoop && planned) {
        return Fail("shot at " + std::to_string(action.target) +
                    " was expected to be a gated no-op, but it was planned");
      }
      if (!action.expectNoop && !planned) {
        return Fail("shot at " + std::to_string(action.target) +
                    " could not be planned (invalid target, or outside the shooter's team FOV?)");
      }
      if (action.expectNoop) {
        NotifyMenuClick(actorTeam, "Cancel");
        game.CancelAction();  // Return to ActionMenu, mirroring a real player.
      }
      return true;
    }
    case ScenarioAction::Kind::Pass:
      NotifyMenuClick(actorTeam, "Pass");
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
      if (a.facingDegrees) {
        // Compare on the circle so -180 and 180 are the same heading.
        float diff = std::fmod(unit->facingYaw * 180.0f / 3.14159265f - *a.facingDegrees, 360.0f);
        if (diff > 180.0f) diff -= 360.0f;
        if (diff < -180.0f) diff += 360.0f;
        if (std::fabs(diff) > a.tolerance) {
          Fail("unit " + std::to_string(*a.unit) + " expected facing near " +
               std::to_string(*a.facingDegrees) + " deg but is at " +
               std::to_string(unit->facingYaw * 180.0f / 3.14159265f));
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
  if (const YAML::Node cam = root["camera"]) {
    if (cam["target"]) scenario.cameraTarget = ParseVec2(cam["target"], "camera.target");
    if (cam["zoom"]) scenario.cameraZoom = cam["zoom"].as<float>();
  }

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

  // Mirrors main.cpp's frame loop so captured frames include sighting memory.
  game.UpdateSightingMemory(0.0f);
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
