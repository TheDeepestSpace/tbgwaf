// Headless logic tests for Stage A: navmesh/pathfinding, raycast LOS/FOV,
// turn ordering, and the click-driven game state machine. No SDL/GL/ImGui
// dependency, so this runs in plain CI without a display.

#include <cmath>
#include <cstdio>
#include <vector>

#include "game/GameLogic.h"
#include "game/NavMesh.h"
#include "game/Raycast.h"
#include "game/Scene.h"
#include "game/TurnManager.h"
#include "game/Types.h"

namespace {

int g_failures = 0;

void ReportFailure(const char* file, int line, const char* expr) {
  std::fprintf(stderr, "CHECK FAILED at %s:%d: %s\n", file, line, expr);
  ++g_failures;
}

#define CHECK(expr) \
  do { \
    if (!(expr)) ReportFailure(__FILE__, __LINE__, #expr); \
  } while (0)

using namespace tactics;

// Coarse sampling check: true if the closed segment [a,b] ever enters the
// strict interior of `box` (XZ only). Used to confirm a path doesn't cut
// through an obstacle footprint.
bool SegmentEntersFootprint(const glm::vec3& a, const glm::vec3& b, const AABB& box) {
  constexpr int kSamples = 400;
  for (int i = 0; i <= kSamples; ++i) {
    const float t = static_cast<float>(i) / kSamples;
    const float x = a.x + (b.x - a.x) * t;
    const float z = a.z + (b.z - a.z) * t;
    constexpr float kInset = 1e-3f;
    if (x > box.min.x + kInset && x < box.max.x - kInset && z > box.min.z + kInset &&
        z < box.max.z - kInset) {
      return true;
    }
  }
  return false;
}

bool PathEntersFootprint(const std::vector<glm::vec3>& path, const AABB& box) {
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    if (SegmentEntersFootprint(path[i], path[i + 1], box)) return true;
  }
  return false;
}

void TestNavMeshRoutesAroundObstacle() {
  std::vector<AABB> obstacles = {
      AABB{glm::vec3(-2.0f, 0.0f, -2.0f), glm::vec3(2.0f, 2.0f, 2.0f)},
  };
  NavMesh nav;
  nav.Build(obstacles, /*mapHalfExtent=*/10.0f, /*agentRadius=*/0.4f);

  const glm::vec3 start(-8.0f, 0.0f, 0.0f);
  const glm::vec3 goal(8.0f, 0.0f, 0.0f);

  std::vector<glm::vec3> path;
  CHECK(nav.FindPath(start, goal, &path));
  CHECK(path.size() >= 2);
  CHECK(glm::distance(path.front(), start) < 1e-3f);
  CHECK(glm::distance(path.back(), goal) < 1e-3f);
  CHECK(!PathEntersFootprint(path, obstacles[0]));

  // A direct straight line WOULD cross the obstacle, proving the detour is
  // actually necessary (i.e. this test would fail to catch a no-op navmesh).
  CHECK(SegmentEntersFootprint(start, goal, obstacles[0]));

  // The path should have picked up at least one bend to go around the box.
  CHECK(path.size() > 2);
}

void TestNavMeshDirectPathWhenUnobstructed() {
  std::vector<AABB> obstacles = {
      AABB{glm::vec3(-2.0f, 0.0f, 5.0f), glm::vec3(2.0f, 2.0f, 9.0f)},
  };
  NavMesh nav;
  nav.Build(obstacles, 10.0f, 0.4f);

  const glm::vec3 start(-8.0f, 0.0f, -8.0f);
  const glm::vec3 goal(8.0f, 0.0f, -8.0f);
  std::vector<glm::vec3> path;
  CHECK(nav.FindPath(start, goal, &path));
  // Far from the obstacle, the funnel should collapse to a straight line.
  CHECK(path.size() == 2);
}

void TestNavMeshRejectsPointsInsideObstacle() {
  std::vector<AABB> obstacles = {
      AABB{glm::vec3(-2.0f, 0.0f, -2.0f), glm::vec3(2.0f, 2.0f, 2.0f)},
  };
  NavMesh nav;
  nav.Build(obstacles, 10.0f, 0.4f);
  CHECK(!nav.IsWalkable(0.0f, 0.0f));
  CHECK(nav.IsWalkable(5.0f, 5.0f));

  std::vector<glm::vec3> path;
  CHECK(!nav.FindPath(glm::vec3(-8, 0, 0), glm::vec3(0, 0, 0), &path));
}

void TestRaycastLineOfSight() {
  std::vector<AABB> obstacles = {
      AABB{glm::vec3(-1.0f, 0.0f, -1.0f), glm::vec3(1.0f, 2.0f, 1.0f)},
  };
  // Blocked: straight through the obstacle.
  CHECK(!LineOfSightClear(glm::vec3(-8, 1.5f, 0), glm::vec3(8, 1.5f, 0), obstacles));
  // Clear: passes well above the obstacle's height.
  CHECK(LineOfSightClear(glm::vec3(-8, 5.0f, 0), glm::vec3(8, 5.0f, 0), obstacles));
  // Clear: offset row that never enters the obstacle's footprint.
  CHECK(LineOfSightClear(glm::vec3(-8, 1.5f, 5), glm::vec3(8, 1.5f, 5), obstacles));
}

void TestFovCone() {
  const glm::vec3 origin(0, 1.5f, 0);
  const glm::vec3 forward(1, 0, 0);  // Facing +X.

  CHECK(InFovCone(origin, forward, glm::vec3(5, 1.5f, 0), 75.0f, 30.0f));   // Dead ahead.
  CHECK(InFovCone(origin, forward, glm::vec3(5, 1.5f, 4), 75.0f, 30.0f));   // Within cone.
  CHECK(!InFovCone(origin, forward, glm::vec3(-5, 1.5f, 0), 75.0f, 30.0f));  // Behind.
  CHECK(!InFovCone(origin, forward, glm::vec3(0.1f, 1.5f, 5), 75.0f, 30.0f));  // ~87 deg, outside 75.
  CHECK(!InFovCone(origin, forward, glm::vec3(100, 1.5f, 0), 75.0f, 30.0f));  // Out of range.
}

void TestTurnManagerAlternatesAndSkipsDead() {
  std::vector<Unit> units(4);
  units[0].id = 0;
  units[0].team = Team::Blue;
  units[1].id = 1;
  units[1].team = Team::Blue;
  units[2].id = 2;
  units[2].team = Team::Red;
  units[3].id = 3;
  units[3].team = Team::Red;

  TurnManager tm;
  tm.StartRound(units);
  CHECK(tm.CurrentActorId(units) == 0);
  tm.AdvanceTurn(units);
  CHECK(tm.CurrentActorId(units) == 2);
  tm.AdvanceTurn(units);
  CHECK(tm.CurrentActorId(units) == 1);

  // Kill unit 3 mid-round; it should be skipped when its turn comes up.
  units[3].alive = false;
  tm.AdvanceTurn(units);
  // unit 3 (dead) is skipped -> round exhausted -> new round starts at 0.
  CHECK(tm.CurrentActorId(units) == 0);
  CHECK(tm.RoundNumber() == 2);
}

void TestCheckWinner() {
  std::vector<Unit> units(2);
  units[0].id = 0;
  units[0].team = Team::Blue;
  units[0].alive = true;
  units[1].id = 1;
  units[1].team = Team::Red;
  units[1].alive = true;
  CHECK(!CheckWinner(units).has_value());

  units[1].alive = false;
  CHECK(CheckWinner(units) == Team::Blue);

  units[0].alive = false;
  CHECK(!CheckWinner(units).has_value());  // Both dead: no winner.
}

void TestGameLogicSelectionGating() {
  GameLogic game;
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  CHECK(game.CurrentActorId() == 0);  // Blue0 acts first (Scene builds Blue ids 0-2, Red 3-5).

  // Clicking a unit that isn't the current actor must be a no-op.
  game.ClickUnit(3);
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  CHECK(!game.SelectedUnitId().has_value());

  game.ClickUnit(0);
  CHECK(game.Mode() == InputMode::ActionMenu);
  CHECK(game.SelectedUnitId() == 0);

  game.CancelAction();
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  CHECK(!game.SelectedUnitId().has_value());
}

void TestGameLogicShootRowsMatchLayout() {
  GameLogic game;
  // Row z=-4 (blue id0 vs red id3) is behind the first wall: must miss.
  CHECK(game.CurrentActorId() == 0);
  game.ClickUnit(0);
  game.ChooseShoot();
  CHECK(game.Mode() == InputMode::AwaitingShootTarget);
  game.ClickUnit(3);
  CHECK(game.FindUnit(3)->alive);         // Blocked shot: miss.
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  CHECK(game.CurrentActorId() == 3);       // Turn advanced to red0.

  // Symmetric: red0 shooting blue0 across the same blocked row also misses.
  game.ClickUnit(3);
  game.ChooseShoot();
  game.ClickUnit(0);
  CHECK(game.FindUnit(0)->alive);
  CHECK(game.CurrentActorId() == 1);  // blue1.

  // Row z=0 (blue id1 vs red id4) is the open lane: must hit.
  game.ClickUnit(1);
  game.ChooseShoot();
  game.ClickUnit(4);
  CHECK(!game.FindUnit(4)->alive);
  CHECK(!game.Winner().has_value());  // Red still has id3, id5 alive.
}

void TestGameLogicMoveUpdatesPositionAndFacing() {
  GameLogic game;
  game.ClickUnit(0);
  game.ChooseMove();
  CHECK(game.Mode() == InputMode::AwaitingMoveDestination);

  // Hovering a point inside an obstacle footprint should not produce a
  // valid preview.
  game.HoverGround(glm::vec3(0.0f, 0.0f, -4.0f));
  CHECK(!game.MovePreviewValid());

  // Hovering the open middle lane should produce a valid preview.
  game.HoverGround(glm::vec3(0.0f, 0.0f, 0.0f));
  CHECK(game.MovePreviewValid());

  const glm::vec3 destination(0.0f, 0.0f, 0.0f);
  game.ClickGround(destination);
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  const Unit* moved = game.FindUnit(0);
  CHECK(std::fabs(moved->position.x - destination.x) < 1e-3f);
  CHECK(std::fabs(moved->position.z - destination.z) < 1e-3f);
  // Started at (-8,0,-4), moved to (0,0,0): facing should point roughly
  // toward +X, +Z (atan2(4, 8)).
  const float expectedYaw = std::atan2(4.0f, 8.0f);
  CHECK(std::fabs(moved->facingYaw - expectedYaw) < 1e-3f);

  CHECK(game.CurrentActorId() == 3);  // Turn advanced.
}

void TestGameLogicWinCondition() {
  GameLogic game;
  // Directly eliminate the Red team to drive the game-over transition
  // without depending on precise shot geometry (already covered above).
  game.FindUnit(3)->alive = false;
  game.FindUnit(4)->alive = false;
  game.FindUnit(5)->alive = false;

  CHECK(game.CurrentActorId() == 0);
  game.ClickUnit(0);
  game.ChoosePass();

  CHECK(game.Mode() == InputMode::GameOver);
  CHECK(game.Winner() == Team::Blue);

  // Further input after game over must be inert.
  game.ClickUnit(1);
  CHECK(game.Mode() == InputMode::GameOver);
}

}  // namespace

int main() {
  TestNavMeshRoutesAroundObstacle();
  TestNavMeshDirectPathWhenUnobstructed();
  TestNavMeshRejectsPointsInsideObstacle();
  TestRaycastLineOfSight();
  TestFovCone();
  TestTurnManagerAlternatesAndSkipsDead();
  TestCheckWinner();
  TestGameLogicSelectionGating();
  TestGameLogicShootRowsMatchLayout();
  TestGameLogicMoveUpdatesPositionAndFacing();
  TestGameLogicWinCondition();

  if (g_failures == 0) {
    std::printf("All logic tests passed.\n");
    return 0;
  }
  std::fprintf(stderr, "%d check(s) failed.\n", g_failures);
  return 1;
}
