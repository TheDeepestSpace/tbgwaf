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

ReactionAction ParseReaction(const std::string& name) {
  if (name == "do_nothing") return ReactionAction::DoNothing;
  if (name == "shoot") return ReactionAction::Shoot;
  if (name == "stop") return ReactionAction::Stop;
  if (name == "continue") return ReactionAction::Continue;
  if (name == "shoot_stop") return ReactionAction::ShootStop;
  if (name == "shoot_continue") return ReactionAction::ShootContinue;
  throw std::runtime_error("unknown playbook action '" + name + "'");
}

// playbook: {blue: {moving_seen: stop, stationary_unseen: shoot, ...}}
void ParsePlaybooks(const YAML::Node& root, SquadPlaybook (&out)[2]) {
  const YAML::Node node = root["playbook"];
  if (!node) return;
  for (const auto& teamEntry : node) {
    const Team team = ParseTeam(teamEntry.first.as<std::string>(), "playbook");
    SquadPlaybook& pb = out[static_cast<int>(team)];
    for (const auto& slot : teamEntry.second) {
      const std::string key = slot.first.as<std::string>();
      const ReactionAction action = ParseReaction(slot.second.as<std::string>());
      if (key == "moving_seen") pb.At(true, true) = action;
      else if (key == "moving_unseen") pb.At(true, false) = action;
      else if (key == "stationary_seen") pb.At(false, true) = action;
      else if (key == "stationary_unseen") pb.At(false, false) = action;
      else throw std::runtime_error("unknown playbook slot '" + key + "'");
    }
  }
}

Scene ParseScene(const YAML::Node& root) {
  Scene scene;

  bool generated = false;
  if (const YAML::Node mapNode = root["map"]) {
    // `generate: {seed: N, type: urban|hilly}` builds a procedural map
    // (units come from the generator unless the scenario lists its own).
    // `type` defaults to the urban city generator.
    if (const YAML::Node genNode = mapNode["generate"]) {
      if (!genNode["seed"]) throw std::runtime_error("map.generate requires 'seed'");
      const uint32_t seed = genNode["seed"].as<uint32_t>();
      const std::string type = genNode["type"] ? genNode["type"].as<std::string>() : "urban";
      if (type == "urban") {
        MapGeneratorConfig config;
        if (genNode["arteries"]) config.arteryCount = genNode["arteries"].as<int>();
        if (genNode["artery_width"]) config.arteryWidth = genNode["artery_width"].as<float>();
        if (genNode["local_street_width"]) {
          config.localStreetWidth = genNode["local_street_width"].as<float>();
        }
        if (genNode["elevated"]) config.elevatedHighway = genNode["elevated"].as<bool>();
        scene = GenerateUrbanMap(seed, config);
      } else if (type == "hilly") {
        scene = GenerateHillyMap(seed);
      } else {
        throw std::runtime_error("map.generate.type must be 'urban' or 'hilly', got '" + type +
                                 "'");
      }
      generated = true;
    }
    // `half_extent: N` sizes the ground square (default 15); scenarios whose
    // units sit far apart must grow the map so every figure stands on it.
    if (const YAML::Node halfNode = mapNode["half_extent"]) {
      scene.mapHalfExtent = halfNode.as<float>();
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
    unit.alive = true;
    if (std::abs(unit.position.x) > scene.mapHalfExtent ||
        std::abs(unit.position.z) > scene.mapHalfExtent) {
      throw std::runtime_error("units[] id " + std::to_string(unit.id) +
                               " is outside the map; raise map.half_extent (currently " +
                               std::to_string(scene.mapHalfExtent) + ")");
    }
    unit.weapon = DefaultWeaponForUnit(unit.id);
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

  if (kind == "new_game") {
    action.kind = ScenarioAction::Kind::NewGame;
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
  } else if (kind == "shoot" || kind == "aim") {
    action.kind = kind == "aim" ? ScenarioAction::Kind::Aim : ScenarioAction::Kind::Shoot;
    if (node["at"]) action.shootAt = ParseVec3(node["at"], "script[].at");
    if (node["aim_from"] || node["aim_dir"]) {
      action.aimRayFrom = ParseVec3(node["aim_from"], "script[].aim_from");
      action.aimRayDir = ParseVec3(node["aim_dir"], "script[].aim_dir");
    }
    if (node["target"]) action.target = node["target"].as<int>();
    const bool freeAim = action.shootAt || action.aimRayFrom;
    if (kind == "aim" && (action.target >= 0 || !freeAim)) {
      throw std::runtime_error("script 'aim' action requires 'at' or 'aim_from'/'aim_dir'");
    }
    if (kind == "shoot" && action.target < 0 && !freeAim &&
        !(node["confirm"] && node["confirm"].as<bool>())) {
      throw std::runtime_error(
          "script 'shoot' action requires 'target', 'at', 'aim_from'/'aim_dir', or 'confirm'");
    }
    if (action.target >= 0 && freeAim) {
      throw std::runtime_error("script 'shoot' cannot mix 'target' with a free-aim point/ray");
    }
    if (node["shots"]) action.shots = node["shots"].as<int>();
    action.expectNoop = node["expect_noop"] && node["expect_noop"].as<bool>();
  } else if (kind == "pass") {
    action.kind = ScenarioAction::Kind::Pass;
  } else if (kind == "cancel") {
    action.kind = ScenarioAction::Kind::Cancel;
  } else if (kind == "focus") {
    action.kind = ScenarioAction::Kind::Focus;
  } else {
    throw std::runtime_error("unknown script action '" + kind +
                              "' (expected move/shoot/aim/pass/cancel/focus/commit/new_game)");
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
  if (node["remembered_by"]) {
    assertion.rememberedByTeam = ParseTeam(node["remembered_by"].as<std::string>(), "assert.remembered_by");
  }
  if (node["remembered"]) assertion.remembered = node["remembered"].as<bool>();
  if (node["memory_age"]) assertion.memoryAge = node["memory_age"].as<int>();
  if (node["round"]) assertion.round = node["round"].as<int>();
  if (node["winner"]) {
    assertion.checkWinner = true;
    const std::string raw = node["winner"].as<std::string>();
    assertion.expectedWinner = raw == "none" ? std::optional<Team>() : ParseTeam(raw, "assert.winner");
  }

  if ((assertion.alive || assertion.position || assertion.facingDegrees ||
       assertion.visible || assertion.remembered || assertion.memoryAge) && !assertion.unit) {
    throw std::runtime_error(
        "assert checking alive/position/facing_degrees/visible/remembered/memory_age requires 'unit'");
  }
  if ((assertion.remembered || assertion.memoryAge) && !assertion.rememberedByTeam) {
    throw std::runtime_error("assert 'remembered'/'memory_age' require 'remembered_by'");
  }
  if (assertion.visible.has_value() != assertion.visibleToTeam.has_value()) {
    throw std::runtime_error("assert 'visible' and 'visible_to' must be set together");
  }
  return assertion;
}

bool ExecuteAction(GameLogic& game, const Scene& scene, const ScenarioAction& action,
                    int stepIndex, const PlaybackHooks& hooks, ScenarioResult* result) {
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

  if (action.kind == ScenarioAction::Kind::NewGame) {
    // Same call the UI's new-game paths make (playbooks survive it).
    NotifyMenuClick(Team::Blue, "New Match");
    game.Reset(scene);
    game.UpdateSightingMemory(0.0f);
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

  // A shoot/aim step may continue an earlier `aim` step's unfinished
  // placement: the actor is then already selected and mid-aim, so no fresh
  // selection click happens (the real player is still in the same flow).
  const bool continuingAim =
      (action.kind == ScenarioAction::Kind::Shoot || action.kind == ScenarioAction::Kind::Aim) &&
      game.Mode() == InputMode::AwaitingShootTarget && game.SelectedUnitId() == action.actor;
  if (!continuingAim) {
    ClickUnitAt(action.actor, actorTeam);
    if (game.SelectedUnitId() != action.actor || game.Mode() != InputMode::ActionMenu) {
      return Fail("could not be selected (already game over?)");
    }
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
    case ScenarioAction::Kind::Shoot:
    case ScenarioAction::Kind::Aim: {
      if (!continuingAim) {
        NotifyMenuClick(actorTeam, "Shoot");
        game.ChooseShoot();
      }
      // Burst size (issue #138): the step's `shots` is the level the player
      // dragged the shot bar to before picking the target; the same clamped
      // setter the bar uses.
      if (action.shots) game.SetPlannedShotCount(*action.shots, actorTeam);

      // Locked-on figure target: the pre-#129 flow, unchanged.
      if (action.target >= 0) {
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

      // Free-aim: place the "+" marker, either from the already-resolved
      // world point (`at`) or by running a camera-style ray through the same
      // ResolveAimRay the interactive click uses.
      if (action.aimRayFrom) {
        const tactics::GameLogic::AimRayResult aim =
            game.ResolveAimRay(*action.aimRayFrom, *action.aimRayDir);
        if (aim.kind == tactics::GameLogic::AimRayResult::Kind::Unit) {
          // Unit under the cursor beats the surface behind it: the ray click
          // becomes the existing lock-on plan.
          ClickUnitAt(aim.unitId, actorTeam);
          if (game.Mode() == InputMode::AwaitingShootTarget) {
            return Fail("aim ray locked onto unit " + std::to_string(aim.unitId) +
                        " but the shot could not be planned");
          }
          return true;
        }
        if (aim.kind == tactics::GameLogic::AimRayResult::Kind::Surface) {
          NotifyClick(actorTeam, aim.point);
          game.PlaceAimPoint(aim.point, actorTeam);
        }
      } else if (action.shootAt) {
        NotifyClick(actorTeam, *action.shootAt);
        game.PlaceAimPoint(*action.shootAt, actorTeam);
      }
      if (!game.GetAimPreview()) {
        return Fail("free-aim point could not be placed (actor not aiming?)");
      }
      if (action.kind == ScenarioAction::Kind::Aim) {
        return true;  // Tap-to-place only; a later step confirms or cancels.
      }
      NotifyMenuClick(actorTeam, "Fire");
      game.ConfirmAim(actorTeam);
      if (game.Mode() != InputMode::AwaitingSelection) {
        return Fail("free-aim shot could not be confirmed");
      }
      return true;
    }
    case ScenarioAction::Kind::Pass:
      NotifyMenuClick(actorTeam, "Pass");
      game.ChoosePass();
      return true;
    case ScenarioAction::Kind::Focus:
      // Double-click: the first click's selection already happened above;
      // the camera easing is a rendering concern, so only observers act.
      if (hooks.onFocus) hooks.onFocus(game, action.actor, actorTeam);
      return true;
    case ScenarioAction::Kind::Cancel:
    case ScenarioAction::Kind::Commit:
    case ScenarioAction::Kind::NewGame:
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
      if (a.rememberedByTeam) {
        const auto& samples = game.Sightings(*a.rememberedByTeam, *a.unit);
        if (a.remembered && samples.empty() == *a.remembered) {
          Fail("unit " + std::to_string(*a.unit) + " expected remembered_by " +
               ToString(a.rememberedByTeam) + "=" + (*a.remembered ? "true" : "false") + " but had " +
               std::to_string(samples.size()) + " samples");
        }
        if (a.memoryAge) {
          if (samples.empty()) {
            Fail("unit " + std::to_string(*a.unit) + " expected memory_age " +
                 std::to_string(*a.memoryAge) + " but has no remembered samples");
          } else if (samples.front().ageRounds != *a.memoryAge) {
            Fail("unit " + std::to_string(*a.unit) + " expected memory_age " +
                 std::to_string(*a.memoryAge) + " but was " + std::to_string(samples.front().ageRounds));
          }
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
  ParsePlaybooks(root, scenario.playbooks);
  if (root["friendly_fire"]) scenario.friendlyFire = root["friendly_fire"].as<bool>();
  if (const YAML::Node rolls = root["shot_rolls"]) {
    if (!rolls.IsSequence() || rolls.size() == 0) {
      throw std::runtime_error("shot_rolls must be a non-empty list of [0,1) rolls");
    }
    for (const auto& roll : rolls) scenario.shotRolls.push_back(roll.as<float>());
  }
  if (const YAML::Node cam = root["camera"]) {
    if (cam["target"]) scenario.cameraTarget = ParseVec2(cam["target"], "camera.target");
    if (cam["zoom"]) scenario.cameraZoom = cam["zoom"].as<float>();
  }
  if (const YAML::Node render = root["render"]) {
    if (render["fov_overlay"]) {
      const std::string mode = render["fov_overlay"].as<std::string>();
      if (mode == "cpu") {
        scenario.fovOverlay = Scenario::FovOverlay::Cpu;
      } else if (mode == "shadow_map") {
        scenario.fovOverlay = Scenario::FovOverlay::ShadowMap;
      } else {
        throw std::runtime_error("render.fov_overlay must be 'cpu' or 'shadow_map', got '" +
                                 mode + "'");
      }
    }
    if (render["fov_probe_height"]) {
      scenario.fovProbeHeight = render["fov_probe_height"].as<float>();
    }
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
  game.SetPlaybook(Team::Blue, scenario.playbooks[0]);
  game.SetPlaybook(Team::Red, scenario.playbooks[1]);
  game.SetFriendlyFireEnabled(scenario.friendlyFire);
  // Pinned rolls make probabilistic shots deterministic: consumed in
  // resolution order, repeating the list when it runs out.
  size_t nextRoll = 0;
  if (!scenario.shotRolls.empty()) {
    game.SetShotRollSource([&scenario, &nextRoll]() {
      return scenario.shotRolls[nextRoll++ % scenario.shotRolls.size()];
    });
  }
  // A second page that only mirrors the simulator's snapshots, like the
  // networked follower pane. Its sighting memory is checked too.
  GameLogic follower(scenario.scene);
  auto SyncFollower = [&] {
    follower.ImportState(game.ExportState());
    follower.UpdateSightingMemory(0.0f);
  };

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
      if (!ExecuteAction(game, scenario.scene, *step.action, i, hooks, &result)) {
        break;  // The script's own preconditions were violated; state past this point is unreliable.
      }
      SyncFollower();
      ++completedActions;
      EmitHoldFrames();
      if (hooks.onActionComplete) hooks.onActionComplete(game, completedActions);
    } else {
      CheckAssertion(game, *step.assertion, i, &result);
      if (step.assertion->rememberedByTeam &&
          (!step.assertion->memoryAge || *step.assertion->memoryAge == 0)) {
        ScenarioResult followerResult;
        CheckAssertion(follower, *step.assertion, i, &followerResult);
        for (auto& f : followerResult.failures) result.failures.push_back("follower " + f);
      }
    }
  }
  return result;
}

ScenarioResult RunScenario(const Scenario& scenario) { return RunScenario(scenario, {}); }

}  // namespace tactics::scenario
