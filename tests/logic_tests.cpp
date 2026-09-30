// Headless logic tests for Stage A: navmesh/pathfinding, raycast LOS/FOV,
// WEGO round phases, and the click-driven game state machine. No SDL/GL/ImGui
// dependency, so this runs in plain CI without a display.

#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <vector>

#include "game/GameLogic.h"
#include "game/NavMesh.h"
#include "game/Raycast.h"
#include "game/Scene.h"
#include "game/Types.h"
#include "game/Visibility.h"

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

// The original open-lane 20x20 layout (walls at z=+-4, four crates). Many
// tests rely on its clear middle lane; the richer default scene has no
// initial sightlines between squads (see TestDefaultSceneSquadsStartHidden).
tactics::Scene LegacyScene() {
  auto box = [](float cx, float cz, float hx, float hz, float h, bool climbable) {
    return tactics::Obstacle{tactics::AABB{glm::vec3(cx - hx, 0.0f, cz - hz), glm::vec3(cx + hx, h, cz + hz)},
                    climbable};
  };
  tactics::Scene scene;
  scene.obstacles.push_back(box(0.0f, -4.0f, 1.0f, 2.0f, 2.0f, false));
  scene.obstacles.push_back(box(0.0f, 4.0f, 1.0f, 2.0f, 2.0f, false));
  scene.obstacles.push_back(box(-4.0f, 6.5f, 0.6f, 0.6f, 1.2f, true));
  scene.obstacles.push_back(box(4.0f, -6.5f, 0.6f, 0.6f, 1.2f, true));
  scene.obstacles.push_back(box(-3.5f, -7.5f, 0.6f, 0.6f, 1.2f, true));
  scene.obstacles.push_back(box(3.5f, 7.5f, 0.6f, 0.6f, 1.2f, true));
  const float rows[3] = {-4.0f, 0.0f, 4.0f};
  for (int i = 0; i < 3; ++i) {
    tactics::Unit blue;
    blue.id = i;
    blue.team = tactics::Team::Blue;
    blue.position = glm::vec3(-8.0f, 0.0f, rows[i]);
    blue.facingYaw = 0.0f;
    scene.units.push_back(blue);
  }
  for (int i = 0; i < 3; ++i) {
    tactics::Unit red;
    red.id = 3 + i;
    red.team = tactics::Team::Red;
    red.position = glm::vec3(8.0f, 0.0f, rows[i]);
    red.facingYaw = 3.14159265358979f;
    scene.units.push_back(red);
  }
  return scene;
}

constexpr float kPi = 3.14159265358979323846f;

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

void TestNavMeshPathIsTight() {
  // One box padded to [-2.4, 2.4]^2. The shortest path from start to goal
  // wraps the padded corners, so its length is computable analytically.
  std::vector<AABB> obstacles = {
      AABB{glm::vec3(-2.0f, 0.0f, -2.0f), glm::vec3(2.0f, 2.0f, 2.0f)},
  };
  NavMesh nav;
  nav.Build(obstacles, 10.0f, 0.4f);

  const glm::vec3 start(-8.0f, 0.0f, -1.0f);
  const glm::vec3 goal(8.0f, 0.0f, 1.0f);
  std::vector<glm::vec3> path;
  CHECK(nav.FindPath(start, goal, &path));

  float length = 0.0f;
  for (size_t i = 0; i + 1 < path.size(); ++i) length += glm::distance(path[i], path[i + 1]);

  const float c = 2.4f;
  auto viaCorners = [&](float z) {
    return glm::distance(start, glm::vec3(-c, 0, z)) + 2.0f * c +
           glm::distance(glm::vec3(c, 0, z), goal);
  };
  const float optimal = std::fmin(viaCorners(-c), viaCorners(c));
  CHECK(length <= optimal + 1e-3f);

  // Every interior vertex must be a padded obstacle corner (no mid-cell or
  // portal-midpoint wobble), and the path must stay outside the padded box.
  CHECK(path.size() == 4);
  for (size_t i = 1; i + 1 < path.size(); ++i) {
    CHECK(std::fabs(std::fabs(path[i].x) - c) < 1e-3f);
    CHECK(std::fabs(std::fabs(path[i].z) - c) < 1e-3f);
  }
}

void TestNavMeshClimbsOntoClimbableObstacle() {
  std::vector<Obstacle> obstacles = {
      Obstacle{AABB{glm::vec3(-1.0f, 0.0f, -1.0f), glm::vec3(1.0f, 1.2f, 1.0f)},
               /*climbable=*/true},
  };
  NavMesh nav;
  nav.Build(obstacles, /*mapHalfExtent=*/10.0f, /*agentRadius=*/0.4f);

  const glm::vec3 start(-8.0f, 0.0f, 0.0f);
  const glm::vec3 top(0.0f, 1.2f, 0.0f);  // Center of the crate's top surface.

  std::vector<glm::vec3> path;
  CHECK(nav.FindPath(start, top, &path));
  CHECK(path.size() >= 2);
  CHECK(std::fabs(path.front().x - start.x) < 1e-3f);
  CHECK(std::fabs(path.front().z - start.z) < 1e-3f);
  // The path must actually climb: it should end at the obstacle's top
  // elevation, and pass through ground level along the way (not spawn
  // directly on top or treat the obstacle as flat-ground-passable).
  CHECK(std::fabs(path.back().y - 1.2f) < 1e-3f);
  bool sawGroundLevel = false;
  bool sawTopLevel = false;
  for (const auto& p : path) {
    if (std::fabs(p.y) < 1e-3f) sawGroundLevel = true;
    if (std::fabs(p.y - 1.2f) < 1e-3f) sawTopLevel = true;
  }
  CHECK(sawGroundLevel);
  CHECK(sawTopLevel);
}

void TestNavMeshObstacleOverloadStillRoutesAroundNonClimbable() {
  std::vector<Obstacle> obstacles = {
      Obstacle{AABB{glm::vec3(-2.0f, 0.0f, -2.0f), glm::vec3(2.0f, 2.0f, 2.0f)},
               /*climbable=*/false},
  };
  NavMesh nav;
  nav.Build(obstacles, 10.0f, 0.4f);

  const glm::vec3 start(-8.0f, 0.0f, 0.0f);
  const glm::vec3 goal(8.0f, 0.0f, 0.0f);
  std::vector<glm::vec3> path;
  CHECK(nav.FindPath(start, goal, &path));
  CHECK(path.size() > 2);  // Must detour; a non-climbable obstacle has no top connection.
  for (const auto& p : path) CHECK(std::fabs(p.y) < 1e-3f);  // Never leaves ground level.
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

void TestElevatedEyePositionSeesOverObstacle() {
  // Stage-C sanity check: a figure standing atop a climbed obstacle has a
  // raised eye position, and existing 3D LOS/FOV logic (Stage A/B) should
  // handle that correctly with no special-casing -- a shot blocked at ground
  // level becomes clear once the shooter's eye is above the obstacle.
  std::vector<AABB> obstacles = {
      AABB{glm::vec3(-1.0f, 0.0f, -1.0f), glm::vec3(1.0f, 2.0f, 1.0f)},
  };
  CHECK(!LineOfSightClear(glm::vec3(-5.0f, 1.5f, 0.0f), glm::vec3(5.0f, 1.5f, 0.0f), obstacles));
  CHECK(LineOfSightClear(glm::vec3(-5.0f, 3.5f, 0.0f), glm::vec3(5.0f, 1.5f, 0.0f), obstacles));
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

void TestTeamVisibilityAggregatesAcrossFigures() {
  // A wall blocks the straight z=0 line; blueA (on that line) can't see the
  // Red figure directly, but blueB's diagonal line of sight is clear. Team
  // visibility should be the union across all of a team's living figures,
  // not just any single one of them.
  std::vector<AABB> obstacles = {
      AABB{glm::vec3(-1.0f, 0.0f, -3.0f), glm::vec3(1.0f, 2.0f, 3.0f)},
      // A small crate placed directly behind blueB (opposite its facing),
      // clearly outside every living Blue figure's FOV cone.
      AABB{glm::vec3(-11.18f, 0.0f, -9.84f), glm::vec3(-10.18f, 1.0f, -8.84f)},
  };

  std::vector<Unit> units(3);
  units[0].id = 0;
  units[0].team = Team::Blue;
  units[0].position = glm::vec3(-8.0f, 0.0f, 0.0f);
  units[0].facingYaw = 0.0f;  // Faces +X, straight down the blocked z=0 line.

  units[1].id = 1;
  units[1].team = Team::Blue;
  units[1].position = glm::vec3(-8.0f, 0.0f, -8.0f);
  units[1].facingYaw = std::atan2(8.0f, 16.0f);  // Faces the Red figure diagonally.

  units[2].id = 2;
  units[2].team = Team::Red;
  units[2].position = glm::vec3(8.0f, 0.0f, 0.0f);
  units[2].facingYaw = kPi;

  const TeamVisibility visibility = ComputeTeamVisibility(Team::Blue, units, obstacles);
  CHECK(visibility.UnitVisible(2));       // Visible via blueB even though blueA is blocked.
  CHECK(visibility.ObstacleVisible(0));   // The wall itself is in view.
  CHECK(!visibility.ObstacleVisible(1));  // Crate behind blueB: outside every cone.
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
  CHECK(!CheckWinner(units).has_value());  // Both dead: no winner (a draw).
}

void TestRoundPlanningTeamGating() {
  GameLogic game(LegacyScene());
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  CHECK(game.RoundNumber() == 1);

  // Input is team-tagged (the pane a click lands in): a click from Blue's
  // side must never select a Red figure, even though both teams plan the
  // same round.
  game.ClickUnit(3, Team::Blue);
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  CHECK(!game.SelectedUnitId().has_value());

  // Any living figure can be (re)selected by its own side to set or revise
  // its plan -- there's no single "current actor", the whole squad plans.
  game.ClickUnit(1, Team::Blue);
  CHECK(game.Mode() == InputMode::ActionMenu);
  CHECK(game.SelectedUnitId() == 1);
  game.CancelAction();

  // Both teams plan concurrently: Red can select its own figure without
  // waiting for Blue to finish (or commit) anything.
  game.ClickUnit(3, Team::Red);
  CHECK(game.Mode() == InputMode::ActionMenu);
  CHECK(game.SelectedUnitId() == 3);
  game.CancelAction();

  // While Blue is aiming a shot, a click from Red's side must not pick the
  // target for it (it could otherwise steer Blue's shot at Red's choosing).
  game.ClickUnit(0, Team::Blue);
  game.ChooseShoot();
  CHECK(game.Mode() == InputMode::AwaitingShootTarget);
  game.ClickUnit(4, Team::Red);
  CHECK(game.Mode() == InputMode::AwaitingShootTarget);
  CHECK(game.FindUnit(0)->plan.type == tactics::PlannedActionType::None);
  game.ClickUnit(4, Team::Blue);
  CHECK(game.FindUnit(0)->plan.type == tactics::PlannedActionType::Shoot);
  CHECK(game.FindUnit(0)->plan.shootTargetId == 4);
}

void TestRoundCommitRequiresBothTeamsPlanned() {
  GameLogic game(LegacyScene());

  const glm::vec3 blue0Start = game.FindUnit(0)->position;

  // Blue plans a move, a shot down the open middle lane, and a pass.
  game.ClickUnit(0, Team::Blue);
  game.ChooseMove();
  const glm::vec3 destination(-3.0f, 0.0f, -4.0f);
  game.ClickGround(destination, Team::Blue);
  CHECK(game.FindUnit(0)->plan.type == tactics::PlannedActionType::Move);
  game.ClickUnit(1, Team::Blue);
  game.ChooseShoot();
  game.ClickUnit(4, Team::Blue);
  game.ClickUnit(2, Team::Blue);
  game.ChoosePass();

  // Every living Blue figure has a plan, but the round can't commit until
  // Red's squad is fully planned too -- both teams execute together.
  CHECK(!game.CanCommitRound());

  for (int redId : {3, 4, 5}) {
    game.ClickUnit(redId, Team::Red);
    game.ChoosePass();
  }
  CHECK(game.CanCommitRound());

  // World state is completely unchanged by planning alone.
  CHECK(glm::distance(game.FindUnit(0)->position, blue0Start) < 1e-6f);
  CHECK(game.FindUnit(4)->alive);
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  CHECK(game.RoundNumber() == 1);

  game.CommitRound();

  // blue1's shot already connects at the pre-move positions, so it fires
  // the instant the round starts -- it doesn't wait on blue0's move.
  CHECK(game.Mode() == InputMode::Executing);
  CHECK(!game.FindUnit(4)->alive);
  CHECK(game.FindUnit(0)->plan.type == tactics::PlannedActionType::None);  // Plans cleared.
  CHECK(game.FindUnit(1)->plan.type == tactics::PlannedActionType::None);
  CHECK(game.IsUnitMoving(0));  // blue0's planned move is animating.

  game.Update(100.0f);  // Finish blue0's move (the only thing left in flight).
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  CHECK(glm::distance(game.FindUnit(0)->position, destination) < 1e-3f);
  CHECK(game.RoundNumber() == 2);  // One commit resolved the whole round.
}

void TestRoundExecutesBothTeamsMovesConcurrently() {
  GameLogic game(LegacyScene());
  game.ClickUnit(0, Team::Blue);
  game.ChooseMove();
  const glm::vec3 blueDestination(-3.0f, 0.0f, -4.0f);
  game.ClickGround(blueDestination, Team::Blue);
  game.ClickUnit(1, Team::Blue);
  game.ChoosePass();
  game.ClickUnit(2, Team::Blue);
  game.ChoosePass();

  game.ClickUnit(5, Team::Red);
  game.ChooseMove();
  const glm::vec3 redDestination(3.0f, 0.0f, 4.0f);
  game.ClickGround(redDestination, Team::Red);
  game.ClickUnit(3, Team::Red);
  game.ChoosePass();
  game.ClickUnit(4, Team::Red);
  game.ChoosePass();

  CHECK(game.CanCommitRound());
  game.CommitRound();
  CHECK(game.Mode() == InputMode::Executing);
  CHECK(game.IsUnitMoving(0));
  CHECK(game.IsUnitMoving(5));

  const glm::vec3 blueStart(-8.0f, 0.0f, -4.0f);
  const glm::vec3 redStart(8.0f, 0.0f, 4.0f);
  const float totalDistance = glm::distance(blueStart, blueDestination);
  const float halfwayDt = (totalDistance * 0.5f) / tactics::constants::kMoveSpeed;

  // A single Update() call advances figures of *both* teams at once -- Red's
  // mover doesn't sit frozen waiting for a separate Red turn.
  game.Update(halfwayDt);
  CHECK(game.Mode() == InputMode::Executing);
  CHECK(glm::distance(game.FindUnit(0)->position, blueStart) > totalDistance * 0.25f);
  CHECK(glm::distance(game.FindUnit(5)->position, redStart) > totalDistance * 0.25f);
  CHECK(glm::distance(game.FindUnit(0)->position, blueDestination) > 1e-3f);
  CHECK(glm::distance(game.FindUnit(5)->position, redDestination) > 1e-3f);

  game.Update(100.0f);  // Fast-forward the rest.
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  CHECK(glm::distance(game.FindUnit(0)->position, blueDestination) < 1e-3f);
  CHECK(glm::distance(game.FindUnit(5)->position, redDestination) < 1e-3f);
  CHECK(game.RoundNumber() == 2);
}

void TestShootRowsResolveSimultaneouslyAcrossTeams() {
  GameLogic game(LegacyScene());
  // Every figure shoots its opposite number in the same round. Rows z=-4 and
  // z=4 are behind the walls (all four of those shots must miss); row z=0 is
  // the open lane, so blue1 and red4 fire at each other simultaneously --
  // judged against the same snapshot, both shots land.
  const int pairs[3][2] = {{0, 3}, {1, 4}, {2, 5}};
  for (const auto& pair : pairs) {
    game.ClickUnit(pair[0], Team::Blue);
    game.ChooseShoot();
    game.ClickUnit(pair[1], Team::Blue);
    game.ClickUnit(pair[1], Team::Red);
    game.ChooseShoot();
    game.ClickUnit(pair[0], Team::Red);
  }
  CHECK(game.CanCommitRound());
  game.CommitRound();

  // No moves were planned, so the round resolves and finishes in the commit
  // call itself: still-blocked shots can never connect once nobody moves.
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  CHECK(game.FindUnit(0)->alive);
  CHECK(!game.FindUnit(1)->alive);  // Mutual open-lane exchange downs both...
  CHECK(!game.FindUnit(4)->alive);  // ...shooters at the same instant.
  CHECK(game.FindUnit(2)->alive);
  CHECK(game.FindUnit(3)->alive);
  CHECK(game.FindUnit(5)->alive);
  CHECK(!game.Winner().has_value());
  CHECK(game.RoundNumber() == 2);
}

void TestMutualEliminationIsDraw() {
  GameLogic game(LegacyScene());
  // Leave only the open middle lane's pair alive, shooting each other.
  for (int id : {0, 2, 3, 5}) game.FindUnit(id)->alive = false;

  game.ClickUnit(1, Team::Blue);
  game.ChooseShoot();
  game.ClickUnit(4, Team::Blue);
  game.ClickUnit(4, Team::Red);
  game.ChooseShoot();
  game.ClickUnit(1, Team::Red);
  CHECK(game.CanCommitRound());
  game.CommitRound();

  // Both last figures down each other in the same tick: game over, no winner.
  CHECK(!game.FindUnit(1)->alive);
  CHECK(!game.FindUnit(4)->alive);
  CHECK(game.Mode() == InputMode::GameOver);
  CHECK(!game.Winner().has_value());
}

void TestMoveBudgetCapsPlannedPaths() {
  GameLogic game(LegacyScene());
  game.ClickUnit(0, Team::Blue);
  game.ChooseMove();

  // blue0 can cover runSpeed * kRoundDuration world units per round. A far
  // corner beyond that budget must be rejected at plan time -- the round's
  // execution window is fixed, so the figure could never get there in time.
  const Unit* mover = game.FindUnit(0);
  const glm::vec3 tooFar(11.0f, 0.0f, 8.0f);
  CHECK(glm::distance(mover->position, tooFar) > mover->MoveBudget());
  game.HoverGround(tooFar, Team::Blue);
  CHECK(!game.MovePreviewValid());
  game.ClickGround(tooFar, Team::Blue);
  CHECK(game.Mode() == InputMode::AwaitingMoveDestination);  // Rejected: no plan.
  CHECK(game.FindUnit(0)->plan.type == tactics::PlannedActionType::None);

  // A destination inside the budget plans normally.
  const glm::vec3 nearEnough(-3.0f, 0.0f, -4.0f);
  game.HoverGround(nearEnough, Team::Blue);
  CHECK(game.MovePreviewValid());
  game.ClickGround(nearEnough, Team::Blue);
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  CHECK(game.FindUnit(0)->plan.type == tactics::PlannedActionType::Move);
}

void TestPendingShotFiresWhenTargetWalksIntoView() {
  GameLogic game(LegacyScene());

  // Park red4 behind the z=-4 wall from blue1's perspective: blue1's own
  // line of sight is blocked at plan time, but blue2's diagonal view is
  // clear, so red4 is inside Blue's *team* FOV and a legal shot target.
  Unit* red4 = game.FindUnit(4);
  red4->position = glm::vec3(5.0f, 0.0f, -4.0f);
  CHECK(game.ComputeVisibility(Team::Blue).UnitVisible(4));
  CHECK(!game.ResolveShot(*game.FindUnit(1), *red4));  // Blocked right now: no kill.
  CHECK(red4->alive);

  game.ClickUnit(0, Team::Blue);
  game.ChoosePass();
  game.ClickUnit(1, Team::Blue);
  game.ChooseShoot();
  game.ClickUnit(4, Team::Blue);
  game.ClickUnit(2, Team::Blue);
  game.ChoosePass();

  // Red sends red4 out of cover, north along x=5 into the open lane.
  game.ClickUnit(3, Team::Red);
  game.ChoosePass();
  game.ClickUnit(4, Team::Red);
  game.ChooseMove();
  const glm::vec3 destination(5.0f, 0.0f, 0.0f);
  game.ClickGround(destination, Team::Red);
  game.ClickUnit(5, Team::Red);
  game.ChoosePass();

  CHECK(game.CanCommitRound());
  game.CommitRound();

  // Tick 0: still behind the wall, so the shot holds instead of resolving
  // as a one-time miss -- the continuous re-check is the point of WEGO.
  CHECK(game.Mode() == InputMode::Executing);
  CHECK(game.FindUnit(4)->alive);

  // Step frame-by-frame so the per-tick FOV/LOS re-check samples red4's
  // position incrementally: blue1 must fire the moment red4 clears the
  // wall's cover, dropping it mid-path well short of the destination.
  int steps = 0;
  while (game.Mode() == InputMode::Executing && steps < 10000) {
    game.Update(0.02f);
    ++steps;
  }
  CHECK(steps < 10000);  // Sanity: the loop above actually terminated.

  CHECK(!game.FindUnit(4)->alive);
  const glm::vec3 moverStop = game.FindUnit(4)->position;
  CHECK(moverStop.z > -3.5f);                            // Had actually started moving...
  CHECK(glm::distance(moverStop, destination) > 1.5f);   // ...but died short of the goal.
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  CHECK(game.RoundNumber() == 2);
  CHECK(!game.Winner().has_value());
}

void TestDefaultSceneSquadsStartHidden() {
  GameLogic game;
  for (Team team : {Team::Blue, Team::Red}) {
    const auto visibility = game.ComputeVisibility(team);
    for (const Unit& unit : game.GetScene().units) {
      if (unit.team != team) CHECK(!visibility.UnitVisible(unit.id));
    }
  }
}

void TestGameLogicShootGatingRequiresTeamVisibility() {
  GameLogic game(LegacyScene());

  // Turn every living Blue figure to face away from Red (-X instead of +X):
  // Red is now entirely outside Blue's combined FOV, regardless of LOS.
  for (int id = 0; id <= 2; ++id) {
    game.FindUnit(id)->facingYaw = kPi;
  }
  for (int redId = 3; redId <= 5; ++redId) {
    CHECK(!game.ComputeVisibility(Team::Blue).UnitVisible(redId));
  }

  game.ClickUnit(0, Team::Blue);
  game.ChooseShoot();
  CHECK(game.Mode() == InputMode::AwaitingShootTarget);

  // Red5 is alive and would otherwise be a legal target, but it's outside
  // Blue's team FOV: the click must be a no-op (not a guaranteed miss) --
  // no plan is recorded and blue0 stays selected for targeting.
  game.ClickUnit(5, Team::Blue);
  CHECK(game.Mode() == InputMode::AwaitingShootTarget);
  CHECK(game.FindUnit(0)->plan.type == tactics::PlannedActionType::None);
  CHECK(game.FindUnit(5)->alive);

  // Turn blue0 back to face Red: Red5 re-enters Blue's FOV with clear LOS
  // and becomes a valid planning target again.
  game.FindUnit(0)->facingYaw = 0.0f;
  CHECK(game.ComputeVisibility(Team::Blue).UnitVisible(5));
  game.ClickUnit(5, Team::Blue);
  CHECK(game.FindUnit(0)->plan.type == tactics::PlannedActionType::Shoot);
  CHECK(game.FindUnit(0)->plan.shootTargetId == 5);
  CHECK(game.FindUnit(5)->alive);  // Still just a plan; nothing resolved yet.
  CHECK(game.Mode() == InputMode::AwaitingSelection);
}

void TestGameLogicDownedEnemyStaysVisibleInFov() {
  GameLogic game(LegacyScene());
  CHECK(game.ComputeVisibility(Team::Blue).UnitVisible(5));
  game.FindUnit(5)->alive = false;
  CHECK(game.ComputeVisibility(Team::Blue).UnitVisible(5));
  for (int id = 0; id <= 2; ++id) game.FindUnit(id)->facingYaw = kPi;
  CHECK(!game.ComputeVisibility(Team::Blue).UnitVisible(5));
}

// Plans a pass for every living figure except the ids in `except`, on both
// teams -- the boilerplate for tests that only care about one or two units'
// plans now that a round commit needs everyone planned.
void PassEveryoneElse(GameLogic& game, std::initializer_list<int> except) {
  for (const Unit& unit : game.GetScene().units) {
    if (!unit.alive) continue;
    bool skip = false;
    for (int id : except) skip |= (unit.id == id);
    if (skip || unit.plan.type != tactics::PlannedActionType::None) continue;
    game.ClickUnit(unit.id, unit.team);
    game.ChoosePass();
  }
}

void TestGameLogicMoveUpdatesPositionAndFacing() {
  GameLogic game(LegacyScene());
  game.ClickUnit(0, Team::Blue);
  game.ChooseMove();
  CHECK(game.Mode() == InputMode::AwaitingMoveDestination);

  // Hovering a point inside an obstacle footprint should not produce a
  // valid preview.
  game.HoverGround(glm::vec3(0.0f, 0.0f, -4.0f), Team::Blue);
  CHECK(!game.MovePreviewValid());

  // Hovering the open middle lane should produce a valid preview.
  game.HoverGround(glm::vec3(0.0f, 0.0f, 0.0f), Team::Blue);
  CHECK(game.MovePreviewValid());

  const glm::vec3 destination(0.0f, 0.0f, 0.0f);
  game.ClickGround(destination, Team::Blue);
  // Planning only: the click records blue0's plan and returns to unit
  // selection -- nothing moves and the round does not advance yet.
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  const Unit* planned = game.FindUnit(0);
  CHECK(planned->plan.type == tactics::PlannedActionType::Move);
  CHECK(std::fabs(planned->position.x - (-8.0f)) < 1e-3f);
  CHECK(std::fabs(planned->position.z - (-4.0f)) < 1e-3f);
  CHECK(game.RoundNumber() == 1);

  // Fill out the rest of both squads' plans and commit the round.
  PassEveryoneElse(game, {0});
  CHECK(game.CanCommitRound());
  game.CommitRound();

  // blue0's move animates rather than teleporting.
  CHECK(game.Mode() == InputMode::Executing);
  const Unit* moving = game.FindUnit(0);
  CHECK(std::fabs(moving->position.x - (-8.0f)) < 1e-3f);
  CHECK(std::fabs(moving->position.z - (-4.0f)) < 1e-3f);

  // A large fast-forward dt should consume the whole path and complete the
  // move action (and the rest of the round) in one Update() call.
  game.Update(100.0f);
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  const Unit* moved = game.FindUnit(0);
  CHECK(std::fabs(moved->position.x - destination.x) < 1e-3f);
  CHECK(std::fabs(moved->position.z - destination.z) < 1e-3f);
  // Started at (-8,0,-4), moved to (0,0,0): facing should point roughly
  // toward +X, +Z (atan2(4, 8)).
  const float expectedYaw = std::atan2(4.0f, 8.0f);
  CHECK(std::fabs(moved->facingYaw - expectedYaw) < 1e-3f);

  CHECK(game.RoundNumber() == 2);  // Round advanced.
}

void TestGameLogicMoveAnimatesProgressively() {
  GameLogic game(LegacyScene());
  game.ClickUnit(0, Team::Blue);
  game.ChooseMove();
  // Straight line, same row, short of the wall at x in [-1,1] so the path
  // collapses to a direct two-point segment (no detour to complicate the
  // expected travel distance).
  const glm::vec3 destination(-3.0f, 0.0f, -4.0f);
  game.HoverGround(destination, Team::Blue);
  CHECK(game.MovePreviewValid());
  game.ClickGround(destination, Team::Blue);
  CHECK(game.Mode() == InputMode::AwaitingSelection);  // Planned only.

  PassEveryoneElse(game, {0});
  game.CommitRound();
  CHECK(game.Mode() == InputMode::Executing);

  const glm::vec3 start(-8.0f, 0.0f, -4.0f);
  const float totalDistance = glm::distance(start, destination);
  const float halfwayDt = (totalDistance * 0.5f) / tactics::constants::kMoveSpeed;

  game.Update(halfwayDt);
  CHECK(game.Mode() == InputMode::Executing);  // Not there yet.
  const Unit* midway = game.FindUnit(0);
  // Should have advanced roughly half the distance, but strictly less than
  // the full distance -- proving this is a real interpolation, not a
  // disguised teleport.
  CHECK(glm::distance(midway->position, start) > totalDistance * 0.25f);
  CHECK(glm::distance(midway->position, destination) > 1e-3f);
  // Facing already snapped to the direction of travel (+X), matching the
  // original instant-turn behavior.
  CHECK(std::fabs(midway->facingYaw - 0.0f) < 1e-3f);

  game.Update(100.0f);  // Fast-forward the rest.
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  CHECK(glm::distance(game.FindUnit(0)->position, destination) < 1e-3f);
  CHECK(game.RoundNumber() == 2);
}

void TestGameLogicMoveFacingAdjustableBeforeCommit() {
  GameLogic game(LegacyScene());
  const int id = 0;
  game.ClickUnit(id, Team::Blue);
  game.ChooseMove();
  const glm::vec3 destination = game.FindUnit(id)->position + glm::vec3(3.0f, 0.0f, 0.0f);
  game.ClickGround(destination, Team::Blue);
  // Planned right away, defaulting to the natural direction (+X).
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  CHECK(game.FindUnit(id)->plan.type == PlannedActionType::Move);
  CHECK(std::fabs(game.FindUnit(id)->plan.endFacingYaw) < 1e-3f);

  // Still rotatable later, without reselecting the unit.
  game.SetPlannedMoveFacing(id, 1.5f, Team::Blue);
  CHECK(std::fabs(game.FindUnit(id)->plan.endFacingYaw - 1.5f) < 1e-4f);

  PassEveryoneElse(game, {id});
  game.CommitRound();
  game.SetPlannedMoveFacing(id, -1.0f, Team::Blue);  // Ignored once committed.
  game.Update(100.0f);
  CHECK(std::fabs(game.FindUnit(id)->facingYaw - 1.5f) < 1e-4f);
}

void TestGameLogicIgnoresInputWhileExecuting() {
  GameLogic game(LegacyScene());
  game.ClickUnit(0, Team::Blue);
  game.ChooseMove();
  const glm::vec3 destination(-3.0f, 0.0f, -4.0f);
  game.ClickGround(destination, Team::Blue);
  PassEveryoneElse(game, {0});
  game.CommitRound();
  CHECK(game.Mode() == InputMode::Executing);

  const glm::vec3 midStart = game.FindUnit(0)->position;

  // Input during the round's execution must be a no-op, not a desync:
  // clicking another unit (from either side), re-clicking ground, cancel,
  // pass, or even re-triggering commit should all leave the in-flight move
  // untouched.
  game.ClickUnit(3, Team::Red);
  CHECK(game.Mode() == InputMode::Executing);
  game.ClickGround(glm::vec3(5.0f, 0.0f, 5.0f), Team::Blue);
  CHECK(game.Mode() == InputMode::Executing);
  game.CancelAction();
  CHECK(game.Mode() == InputMode::Executing);
  game.ChoosePass();
  CHECK(game.Mode() == InputMode::Executing);
  game.CommitRound();
  CHECK(game.Mode() == InputMode::Executing);
  CHECK(glm::distance(game.FindUnit(0)->position, midStart) < 1e-6f);

  game.Update(100.0f);
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  CHECK(glm::distance(game.FindUnit(0)->position, destination) < 1e-3f);
  CHECK(game.RoundNumber() == 2);  // Round advanced exactly once.
}

void TestGameLogicMoveCanClimbOntoObstacle() {
  // LegacyScene marks the standalone crates climbable; the one at
  // (-4, 6.5) is a 1.2x1.2 footprint, 1.2 tall.
  GameLogic game(LegacyScene());
  game.ClickUnit(0, Team::Blue);
  game.ChooseMove();

  const glm::vec3 crateTop(-4.0f, 1.2f, 6.5f);
  game.HoverGround(crateTop, Team::Blue);
  CHECK(game.MovePreviewValid());

  game.ClickGround(crateTop, Team::Blue);
  CHECK(game.Mode() == InputMode::AwaitingSelection);  // Planned only.

  PassEveryoneElse(game, {0});
  game.CommitRound();
  CHECK(game.Mode() == InputMode::Executing);

  game.Update(100.0f);  // Fast-forward through the climb animation.
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  const Unit* moved = game.FindUnit(0);
  CHECK(std::fabs(moved->position.x - crateTop.x) < 1e-3f);
  CHECK(std::fabs(moved->position.z - crateTop.z) < 1e-3f);
  CHECK(std::fabs(moved->position.y - crateTop.y) < 1e-3f);
  CHECK(game.RoundNumber() == 2);  // Round advanced.
}

void TestGameLogicOverwatchFiresOnEnemyEnteringFov() {
  GameLogic game(LegacyScene());

  // Reposition red4 due west of blue1 -- squarely behind blue1's fixed +X
  // facing, so it starts outside blue1's FOV cone regardless of LOS -- then
  // send it walking east along the open z=0 lane, straight through blue1's
  // position and into its watched cone. Under WEGO the overwatch arms and
  // the enemy move it interrupts happen in the *same* round's commit.
  game.FindUnit(4)->position = glm::vec3(-9.5f, 0.0f, 0.0f);

  game.ClickUnit(1, Team::Blue);
  game.ChooseOverwatch();
  CHECK(game.FindUnit(1)->plan.type == tactics::PlannedActionType::Overwatch);
  CHECK(game.FindUnit(1)->triggerAction == TriggerAction::None);  // Not armed until commit.

  game.ClickUnit(4, Team::Red);
  game.ChooseMove();
  const glm::vec3 destination(0.0f, 0.0f, 0.0f);
  game.ClickGround(destination, Team::Red);

  PassEveryoneElse(game, {1, 4});
  CHECK(game.CanCommitRound());
  game.CommitRound();

  // Committing arms blue1's trigger and kicks off red4's move concurrently.
  CHECK(game.FindUnit(1)->triggerAction == TriggerAction::Shoot);
  CHECK(game.Mode() == InputMode::Executing);

  // Step frame-by-frame (rather than one huge fast-forward dt) so the
  // overwatch check actually samples red4's position incrementally along
  // the path -- a single giant dt would jump it straight from start to
  // destination in one position update, skipping the mid-path FOV entry
  // this test exists to catch. blue1 should spot red4 and fire the moment
  // it crosses into FOV with clear LOS, interrupting the move well short of
  // the destination.
  int steps = 0;
  while (game.Mode() == InputMode::Executing && steps < 10000) {
    game.Update(0.02f);
    ++steps;
  }
  CHECK(steps < 10000);  // Sanity: the loop above actually terminated.

  CHECK(!game.FindUnit(4)->alive);
  CHECK(game.FindUnit(1)->triggerAction == TriggerAction::None);  // One-shot: trigger consumed.
  const glm::vec3 moverStop = game.FindUnit(4)->position;
  CHECK(glm::distance(moverStop, destination) > 1.0f);  // Died mid-path, short of the destination.
  CHECK(moverStop.x > -9.5f + 1e-3f);                    // But had actually started moving.

  // The interrupted move still finishes the round.
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  CHECK(game.RoundNumber() == 2);
  CHECK(!game.Winner().has_value());  // Red still has id3 and id5 alive.
}

void TestGameLogicWinCondition() {
  GameLogic game(LegacyScene());
  // Directly eliminate the Red team to drive the game-over transition
  // without depending on precise shot geometry (already covered above).
  game.FindUnit(3)->alive = false;
  game.FindUnit(4)->alive = false;
  game.FindUnit(5)->alive = false;

  // Only living figures need plans, so Blue alone can commit the round.
  for (int id : {0, 1, 2}) {
    game.ClickUnit(id, Team::Blue);
    game.ChoosePass();
  }
  CHECK(game.CanCommitRound());
  game.CommitRound();

  CHECK(game.Mode() == InputMode::GameOver);
  CHECK(game.Winner() == Team::Blue);

  // Further input after game over must be inert.
  game.ClickUnit(1, Team::Blue);
  CHECK(game.Mode() == InputMode::GameOver);
}

}  // namespace

void TestSnapshotMirrorsMatchAndTeamPlans() {
  GameLogic blue, red;
  // Each canvas's instance plans only its own team.
  for (int id : {0, 1, 2}) {
    blue.ClickUnit(id, Team::Blue);
    blue.ChoosePass();
  }
  red.ClickUnit(3, Team::Red);
  red.ChooseMove();
  red.ClickGround(red.FindUnit(3)->position + glm::vec3(1.0f, 0.0f, 0.0f), Team::Red);
  CHECK(red.FindUnit(3)->plan.type == PlannedActionType::Move);
  red.SetPlannedMoveFacing(3, 1.25f, Team::Red);
  for (int id : {4, 5}) {
    red.ClickUnit(id, Team::Red);
    red.ChoosePass();
  }
  CHECK(!blue.CanCommitRound());

  GameSnapshot redSnap;
  CHECK(DeserializeSnapshot(SerializeSnapshot(red.ExportState()), &redSnap));
  // Blue learns Red's plans (incl. the move path) without touching its own.
  CHECK(blue.ImportTeamPlans(redSnap, Team::Red));
  CHECK(blue.FindUnit(3)->plan.type == PlannedActionType::Move);
  CHECK(blue.FindUnit(3)->plan.movePath == red.FindUnit(3)->plan.movePath);
  CHECK(std::fabs(blue.FindUnit(3)->plan.endFacingYaw - 1.25f) < 1e-4f);
  CHECK(blue.FindUnit(0)->plan.type == PlannedActionType::Pass);
  CHECK(blue.CanCommitRound());

  blue.CommitRound();
  blue.Update(0.5f);  // Mid-move.
  GameSnapshot decoded;
  CHECK(DeserializeSnapshot(SerializeSnapshot(blue.ExportState()), &decoded));
  CHECK(red.ImportState(decoded));
  CHECK(red.Mode() == blue.Mode());
  CHECK(red.RoundNumber() == blue.RoundNumber());
  CHECK(red.IsUnitMoving(3) == blue.IsUnitMoving(3));
  for (const auto& unit : blue.GetScene().units) {
    const Unit* mirrored = red.FindUnit(unit.id);
    CHECK(mirrored && mirrored->position == unit.position);
    CHECK(mirrored && mirrored->facingYaw == unit.facingYaw);
    CHECK(mirrored && mirrored->plan.type == unit.plan.type);
  }

  GameSnapshot bad;
  CHECK(!DeserializeSnapshot("garbage", &bad));
  CHECK(!DeserializeSnapshot("", &bad));
}

int main() {
  TestNavMeshRoutesAroundObstacle();
  TestNavMeshDirectPathWhenUnobstructed();
  TestNavMeshRejectsPointsInsideObstacle();
  TestNavMeshPathIsTight();
  TestNavMeshClimbsOntoClimbableObstacle();
  TestNavMeshObstacleOverloadStillRoutesAroundNonClimbable();
  TestRaycastLineOfSight();
  TestElevatedEyePositionSeesOverObstacle();
  TestFovCone();
  TestTeamVisibilityAggregatesAcrossFigures();
  TestCheckWinner();
  TestRoundPlanningTeamGating();
  TestRoundCommitRequiresBothTeamsPlanned();
  TestRoundExecutesBothTeamsMovesConcurrently();
  TestShootRowsResolveSimultaneouslyAcrossTeams();
  TestMutualEliminationIsDraw();
  TestMoveBudgetCapsPlannedPaths();
  TestPendingShotFiresWhenTargetWalksIntoView();
  TestDefaultSceneSquadsStartHidden();
  TestGameLogicShootGatingRequiresTeamVisibility();
  TestGameLogicDownedEnemyStaysVisibleInFov();
  TestGameLogicMoveUpdatesPositionAndFacing();
  TestGameLogicMoveAnimatesProgressively();
  TestGameLogicMoveFacingAdjustableBeforeCommit();
  TestGameLogicIgnoresInputWhileExecuting();
  TestGameLogicMoveCanClimbOntoObstacle();
  TestGameLogicOverwatchFiresOnEnemyEnteringFov();
  TestGameLogicWinCondition();
  TestSnapshotMirrorsMatchAndTeamPlans();

  if (g_failures == 0) {
    std::printf("All logic tests passed.\n");
    return 0;
  }
  std::fprintf(stderr, "%d check(s) failed.\n", g_failures);
  return 1;
}
