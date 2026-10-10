#include <algorithm>
#include <cstdio>
#include <string>

#include "ai/JevPlanner.h"
#include "game/GameLogic.h"

namespace {

int failures = 0;
#define CHECK(condition)                                                                     \
  do {                                                                                       \
    if (!(condition)) {                                                                      \
      std::fprintf(stderr, "%s:%d: check failed: %s\n", __FILE__, __LINE__, #condition); \
      ++failures;                                                                            \
    }                                                                                        \
  } while (false)

using tactics::GameLogic;
using tactics::InputMode;
using tactics::PlannedActionType;
using tactics::Scene;
using tactics::Team;
using tactics::Unit;
using tactics::ai::ApplyJevChoice;
using tactics::ai::BuildJevRequest;
using tactics::ai::DeterministicFallbackChoice;
using tactics::ai::JevActionKind;

Scene TwoUnitScene(float distance, bool faceEachOther) {
  Scene scene;
  scene.mapHalfExtent = 30.0f;
  Unit blue;
  blue.id = 0;
  blue.team = Team::Blue;
  blue.position = {-distance * 0.5f, 0.0f, 0.0f};
  blue.facingYaw = faceEachOther ? 0.0f : 3.14159265f;
  scene.units.push_back(blue);
  Unit red;
  red.id = 1;
  red.team = Team::Red;
  red.position = {distance * 0.5f, 0.0f, 0.0f};
  red.facingYaw = faceEachOther ? 3.14159265f : 0.0f;
  scene.units.push_back(red);
  return scene;
}

void TestIncrementalBuilderMatchesBlockingBuild() {
  GameLogic game(TwoUnitScene(24.0f, false));
  const auto expected = BuildJevRequest(game, Team::Blue, 7);
  tactics::ai::JevRequestBuilder builder(game, Team::Blue, 7);
  int steps = 1;
  while (!builder.Step(0.0)) ++steps;  // zero budget: one candidate per step
  const auto actual = builder.Finish();
  CHECK(expected.has_value() && actual.has_value());
  CHECK(steps > 1);
  CHECK(actual->json == expected->json);
}

void TestBoundedCandidatesUseNormalLegalPath() {
  GameLogic game(TwoUnitScene(24.0f, false));
  const auto request = BuildJevRequest(game, Team::Blue, 1);
  CHECK(request.has_value());
  CHECK(request->candidates.size() <= 26);
  bool foundMove = false;
  bool foundWait = false;
  for (const auto& candidate : request->candidates) {
    foundMove |= candidate.kind == JevActionKind::Move;
    foundWait |= candidate.kind == JevActionKind::Wait;
    GameLogic copy = game;
    CHECK(ApplyJevChoice(&copy, *request, {candidate.id}));
    CHECK(copy.FindUnit(0)->plan.type != PlannedActionType::None);
  }
  CHECK(foundMove);
  CHECK(foundWait);
}

void TestShootCandidateIsFullBurstAndStateDescribesWeapons() {
  GameLogic game(TwoUnitScene(10.0f, true));
  const auto request = BuildJevRequest(game, Team::Blue, 3);
  CHECK(request.has_value());
  const auto shoot = std::find_if(request->candidates.begin(), request->candidates.end(),
                                  [](const auto& c) { return c.kind == JevActionKind::Shoot; });
  CHECK(shoot != request->candidates.end());
  if (shoot != request->candidates.end()) {
    CHECK(shoot->shots == tactics::MaxShotsPerAction(game.FindUnit(0)->weapon,
                                                     tactics::constants::kRoundDuration));
    CHECK(shoot->shots > 1);
    CHECK(shoot->description.find("hit chance") != std::string::npos);
    GameLogic copy = game;
    CHECK(ApplyJevChoice(&copy, *request, {shoot->id}));
    CHECK(copy.FindUnit(0)->plan.shots == shoot->shots);
  }
  CHECK(request->json.find("scatter_half_angle_deg") != std::string::npos);
  CHECK(request->json.find("max_burst") != std::string::npos);
  CHECK(request->json.find("friendly fire") != std::string::npos);
}

void TestMapGhostsAndTacticalMovesAreCommunicated() {
  Scene scene = TwoUnitScene(20.0f, true);
  tactics::Obstacle wall;
  wall.bounds.min = {-2.0f, 0.0f, 2.0f};
  wall.bounds.max = {2.0f, 3.0f, 4.0f};
  scene.obstacles.push_back(wall);
  GameLogic game(scene);
  game.UpdateSightingMemory(1.0f);  // Blue sights red.
  game.FindUnit(0)->facingYaw = 3.14159265f;  // Turn away: red leaves view.
  const auto request = BuildJevRequest(game, Team::Blue, 4);
  CHECK(request.has_value());
  CHECK(request->json.find("\"grid\":[") != std::string::npos);
  CHECK(request->json.find("\"obstacles\":[{\"id\":0") != std::string::npos);
  CHECK(request->json.find("\"ghosts\":[{\"id\":1") != std::string::npos);
  CHECK(request->json.find("\"playbook\"") != std::string::npos);
  CHECK(request->json.find("visible_enemies\":[]") != std::string::npos);
  bool hunt = false;
  for (const auto& c : request->candidates) hunt |= c.id == "f0_hunt_1";
  CHECK(hunt);
  bool cover = false;
  for (const auto& c : request->candidates) cover |= c.id == "f0_cover_0";
  CHECK(cover);
  CHECK(request->json.find("exposure") != std::string::npos ||
        request->json.find("no enemy is currently visible") != std::string::npos);
}

void TestCaptureTheFlagIsExplained() {
  Scene scene = TwoUnitScene(24.0f, false);
  GameLogic plain(scene);
  const auto off = BuildJevRequest(plain, Team::Blue, 1);
  CHECK(off.has_value());
  CHECK(off->json.find("\"flag\":null") != std::string::npos);
  CHECK(off->json.find("Capture the flag") == std::string::npos);

  scene.flag.enabled = true;
  scene.flag.position = glm::vec3(-6.0f, 0.0f, 0.0f);
  GameLogic game(scene);
  const auto request = BuildJevRequest(game, Team::Blue, 1);
  CHECK(request.has_value());
  CHECK(request->json.find("Capture the flag") != std::string::npos);
  CHECK(request->json.find("\"status\":\"at_rest\"") != std::string::npos);
  CHECK(request->json.find("\"win_on_grab\":true") != std::string::npos);
  const auto flag = std::find_if(request->candidates.begin(), request->candidates.end(),
                                 [](const auto& c) { return c.id == "f0_flag"; });
  CHECK(flag != request->candidates.end());
  if (flag != request->candidates.end()) {
    CHECK(flag->description.find("wins the match immediately") != std::string::npos);
    GameLogic copy = game;
    CHECK(ApplyJevChoice(&copy, *request, {flag->id}));
  }
}

void TestWholeSquadIsPlannedAtOnce() {
  Scene scene = TwoUnitScene(24.0f, false);
  Unit second;
  second.id = 2;
  second.team = Team::Blue;
  second.position = {-12.0f, 0.0f, 6.0f};
  scene.units.push_back(second);
  tactics::Obstacle wall;
  wall.bounds.min = {-1.0f, 0.0f, 2.0f};
  wall.bounds.max = {1.0f, 3.0f, 4.0f};
  scene.obstacles.push_back(wall);
  GameLogic game(scene);

  const auto request = BuildJevRequest(game, Team::Blue, 9);
  CHECK(request.has_value());
  CHECK(request->actorIds.size() == 2);
  CHECK(request->json.find("\"acting_figures\":[0,2]") != std::string::npos);
  CHECK(request->json.find("\"figure\":2") != std::string::npos);
  // Higher-res map: 0.75-unit cells (rounded to the map), with cell size given.
  CHECK(request->json.find("\"grid_cells\":80") != std::string::npos);
  CHECK(request->json.find("\"cell_size\":") != std::string::npos);

  const auto fallback = DeterministicFallbackChoice(*request);
  CHECK(fallback.size() == 2);
  GameLogic copy = game;
  CHECK(ApplyJevChoice(&copy, *request, fallback));
  CHECK(copy.FindUnit(0)->plan.type != PlannedActionType::None);
  CHECK(copy.FindUnit(2)->plan.type != PlannedActionType::None);
  CHECK(copy.CanCommitRound() == false);  // Red is not planned yet.

  // Incomplete, duplicate-figure and unknown plans change nothing.
  GameLogic untouched = game;
  CHECK(!ApplyJevChoice(&untouched, *request, {fallback[0]}));
  CHECK(!ApplyJevChoice(&untouched, *request, {fallback[0], fallback[0]}));
  CHECK(!ApplyJevChoice(&untouched, *request, {fallback[0], "f2_teleport"}));
  CHECK(untouched.FindUnit(0)->plan.type == PlannedActionType::None);
  CHECK(untouched.FindUnit(2)->plan.type == PlannedActionType::None);
}

void TestHiddenEnemyNeverSerialized() {
  GameLogic game(TwoUnitScene(17.25f, false));
  const auto request = BuildJevRequest(game, Team::Blue, 2);
  CHECK(request.has_value());
  CHECK(request->json.find("visible_enemies\":[]") != std::string::npos);
  CHECK(request->json.find("\"position\":[8.63") == std::string::npos);
  for (const auto& candidate : request->candidates) {
    CHECK(candidate.kind != JevActionKind::Shoot);
  }
}

void TestInvalidAndStaleResponsesDoNotMutate() {
  GameLogic game(TwoUnitScene(8.0f, true));
  const auto request = BuildJevRequest(game, Team::Blue, 3);
  CHECK(request.has_value());
  CHECK(!ApplyJevChoice(&game, *request, {"invented_action"}));
  CHECK(game.FindUnit(0)->plan.type == PlannedActionType::None);

  GameLogic advanced = game;
  auto snapshot = advanced.ExportState();
  snapshot.roundNumber += 1;
  CHECK(advanced.ImportState(snapshot));
  CHECK(!ApplyJevChoice(&advanced, *request, DeterministicFallbackChoice(*request)));
  CHECK(advanced.FindUnit(0)->plan.type == PlannedActionType::None);
}

void TestPlayerVsAiTurnAutomaticallyBecomesCommittable() {
  GameLogic game(TwoUnitScene(8.0f, true));
  game.ClickUnit(0, Team::Blue);
  game.ChoosePass();
  const auto red = BuildJevRequest(game, Team::Red, 4);
  CHECK(red.has_value());
  CHECK(ApplyJevChoice(&game, *red, DeterministicFallbackChoice(*red)));
  CHECK(game.CanCommitRound());
  game.SetShotRollSource([] { return 0.0f; });
  game.CommitRound();
  game.Update(10.0f);  // Bursts are paced in real time; fast-forward the round.
  CHECK(game.Mode() == InputMode::GameOver);
  // Blue passed, but the default playbook lets an idle figure return fire, so
  // the duel may end in mutual elimination (no winner) -- never a Blue win.
  CHECK(game.Winner() != Team::Blue);
  CHECK(!game.FindUnit(0)->alive);
}

void TestCompleteAiVsAiMatchAndTerminalStop() {
  GameLogic game(TwoUnitScene(8.0f, true));
  game.SetShotRollSource([] { return 0.0f; });

  const auto blue = BuildJevRequest(game, Team::Blue, 5);
  CHECK(blue.has_value());
  bool blueHasShoot = false;
  for (const auto& candidate : blue->candidates) blueHasShoot |= candidate.kind == JevActionKind::Shoot;
  CHECK(blueHasShoot);
  CHECK(ApplyJevChoice(&game, *blue, DeterministicFallbackChoice(*blue)));

  const auto red = BuildJevRequest(game, Team::Red, 6);
  CHECK(red.has_value());
  CHECK(ApplyJevChoice(&game, *red, DeterministicFallbackChoice(*red)));
  CHECK(game.CanCommitRound());
  game.CommitRound();
  game.Update(10.0f);  // Bursts are paced in real time; fast-forward the round.

  CHECK(game.Mode() == InputMode::GameOver);
  CHECK(!game.Winner().has_value());  // Simultaneous mutual elimination is a complete draw.
  CHECK(!BuildJevRequest(game, Team::Blue, 7).has_value());
  CHECK(!BuildJevRequest(game, Team::Red, 8).has_value());
}

}  // namespace

int main() {
  TestIncrementalBuilderMatchesBlockingBuild();
  TestBoundedCandidatesUseNormalLegalPath();
  TestShootCandidateIsFullBurstAndStateDescribesWeapons();
  TestMapGhostsAndTacticalMovesAreCommunicated();
  TestCaptureTheFlagIsExplained();
  TestWholeSquadIsPlannedAtOnce();
  TestHiddenEnemyNeverSerialized();
  TestInvalidAndStaleResponsesDoNotMutate();
  TestPlayerVsAiTurnAutomaticallyBecomesCommittable();
  TestCompleteAiVsAiMatchAndTerminalStop();
  if (failures == 0) {
    std::puts("All Jev planner scenarios passed.");
    return 0;
  }
  std::fprintf(stderr, "%d Jev planner check(s) failed.\n", failures);
  return 1;
}
