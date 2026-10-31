#include "game/DemoScenes.h"

#include <cmath>

#include "game/Weapon.h"

namespace tactics {
namespace {

constexpr float kPi = 3.14159265358979323846f;

Obstacle Box(float cx, float cz, float hx, float hz, float height, bool climbable) {
  Obstacle o;
  o.bounds = AABB{glm::vec3(cx - hx, 0.0f, cz - hz), glm::vec3(cx + hx, height, cz + hz)};
  o.climbable = climbable;
  return o;
}

Unit Figure(int id, Team team, float x, float z, float facingDegrees) {
  Unit u;
  u.id = id;
  u.team = team;
  u.position = glm::vec3(x, 0.0f, z);
  u.facingYaw = facingDegrees * kPi / 180.0f;
  u.alive = true;
  u.weapon = DefaultWeaponForUnit(id);
  return u;
}

// "Odd ragdoll behavior repro" (issue #239): a sniper on the west side with
// a clear line east across an open field; a red figure starts just south of
// that line and is meant to walk north across it, so it is hit while moving
// and should ragdoll along the shot line (eastward). A low crate sits a few
// units past the crossing point, so the body's collision with geometry is
// in play too. Blue 0 (rifle) and red 3 pad the squads out so the round
// has walkers/passers either side.
Scene BuildOddRagdollRepro() {
  Scene scene;
  scene.mapHalfExtent = 15.0f;
  scene.obstacles.push_back(Box(9.0f, 0.5f, 0.8f, 0.8f, 1.2f, true));   // Crate east of the crossing.
  scene.obstacles.push_back(Box(-3.0f, -9.0f, 3.0f, 0.6f, 2.5f, false));  // Wall, out of the way.
  scene.obstacles.push_back(Box(3.0f, 9.0f, 3.0f, 0.6f, 2.5f, false));
  scene.units.push_back(Figure(0, Team::Blue, -9.0f, -5.0f, 0.0f));    // Assault rifle.
  scene.units.push_back(Figure(1, Team::Blue, -9.0f, 0.0f, 0.0f));     // Sniper: the shooter.
  scene.units.push_back(Figure(2, Team::Red, 5.0f, -6.0f, 90.0f));     // Desert Eagle: walks north across the line.
  scene.units.push_back(Figure(3, Team::Red, 11.0f, 11.0f, 180.0f));   // Out of the way.
  return scene;
}

const std::vector<DemoScene> kDemoScenes = {
    {"odd-ragdoll-repro", "Odd ragdoll behavior repro",
     "Issue #239: a shot figure ragdolls, then snaps sideways. Have red 2 walk north "
     "across blue 1's line of fire (e.g. to (5, 6)), have blue 1 shoot red 2 with a "
     "full burst, pass the others and commit; watch where the body ends up relative "
     "to the shot line. Download the log once it has happened.",
     239u, &BuildOddRagdollRepro, 1.0f, 0.0f, -2.0f},
};

}  // namespace

const std::vector<DemoScene>& DemoScenes() { return kDemoScenes; }

const DemoScene* FindDemoScene(const std::string& id) {
  for (const DemoScene& scene : kDemoScenes) {
    if (id == scene.id) return &scene;
  }
  return nullptr;
}

void InitDemoGame(const DemoScene& scene, GameLogic* game) {
  // Session config that outlives Reset(): back to the defaults the log
  // header implies.
  game->SetInputRecorder(nullptr);
  game->SetShotRollSource({});
  game->SetPlaybook(Team::Blue, SquadPlaybook{});
  game->SetPlaybook(Team::Red, SquadPlaybook{});
  game->SetFriendlyFireEnabled(constants::kFriendlyFireDefault);
  game->SeedShotRng(scene.seed);
  game->Reset(scene.build());
  game->UpdateSightingMemory(0.0f);
}

ReplayReport ReplayDemoLog(const std::string& text, GameLogic* game, ReproLog* parsed,
                           const std::function<void(const ReproEntry&, float dt)>& afterEntry) {
  ReplayReport report;
  ReproLog local;
  ReproLog& log = parsed ? *parsed : local;
  if (!ParseReproLog(text, &log, &report.error)) return report;
  const DemoScene* scene = FindDemoScene(log.header.scene);
  if (!scene) {
    report.error = "unknown demo scene '" + log.header.scene + "'";
    return report;
  }
  InitDemoGame(*scene, game);
  if (log.header.seed != scene->seed) game->SeedShotRng(log.header.seed);
  return ReplayReproLog(log, scene->build(), game, afterEntry);
}

}  // namespace tactics
