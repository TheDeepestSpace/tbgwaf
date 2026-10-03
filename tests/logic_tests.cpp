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
  game.FinishMovePlan();
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

// Neutral squad playbooks, so movement/visibility tests aren't affected by
// the default table's shoot reactions.
void MakePassive(GameLogic& game) {
  game.SetPlaybook(Team::Blue, SquadPlaybook::Passive());
  game.SetPlaybook(Team::Red, SquadPlaybook::Passive());
}

void TestRoundExecutesBothTeamsMovesConcurrently() {
  GameLogic game(LegacyScene());
  MakePassive(game);
  game.ClickUnit(0, Team::Blue);
  game.ChooseMove();
  const glm::vec3 blueDestination(-3.0f, 0.0f, -4.0f);
  game.ClickGround(blueDestination, Team::Blue);
  game.FinishMovePlan();
  game.ClickUnit(1, Team::Blue);
  game.ChoosePass();
  game.ClickUnit(2, Team::Blue);
  game.ChoosePass();

  game.ClickUnit(5, Team::Red);
  game.ChooseMove();
  const glm::vec3 redDestination(3.0f, 0.0f, 4.0f);
  game.ClickGround(redDestination, Team::Red);
  game.FinishMovePlan();
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

void TestMoveFrontierRoutesAroundObstacle() {
  std::vector<AABB> obstacles = {
      AABB{glm::vec3(-1.0f, 0.0f, -4.0f), glm::vec3(1.0f, 2.0f, 4.0f)},
  };
  NavMesh nav;
  nav.Build(obstacles, /*mapHalfExtent=*/10.0f, /*agentRadius=*/0.4f);
  const glm::vec3 start(-3.0f, 0.0f, 0.0f);
  const ReachField f = nav.ComputeReachField(start, 8.0f);
  CHECK(f.nx > 0);

  auto nodeAt = [&](float x, float z) {
    return std::make_pair(static_cast<int>(std::lround((x - f.minX) / f.step)),
                          static_cast<int>(std::lround((z - f.minZ) / f.step)));
  };
  const auto s = nodeAt(start.x, start.z);
  CHECK(f.Reached(s.first, s.second));
  CHECK(f.Dist(s.first, s.second) == 0.0f);
  // Straight-line 6 away but behind the wall: routed distance exceeds budget.
  const auto behind = nodeAt(3.0f, 0.0f);
  CHECK(!f.Reached(behind.first, behind.second));
  // Inside the padded obstacle is never reached.
  const auto inside = nodeAt(0.0f, 0.0f);
  CHECK(!f.Reached(inside.first, inside.second));
  // Open ground to the left: distance ~ Euclidean.
  const auto left = nodeAt(-6.0f, 0.0f);
  CHECK(std::fabs(f.Dist(left.first, left.second) - 3.0f) < 0.1f);
  // Frontier is cached when entering move mode and cleared on cancel.
  GameLogic game(LegacyScene());
  CHECK(game.MoveFrontier() == nullptr);
  game.ClickUnit(0, Team::Blue);
  game.ChooseMove();
  CHECK(game.MoveFrontier() != nullptr);
  game.CancelAction();
  CHECK(game.MoveFrontier() == nullptr);
}

void TestClickChainsLegsAcrossRounds() {
  GameLogic game(LegacyScene());
  MakePassive(game);  // Blue0 crosses red lines of sight over several rounds.
  game.ClickUnit(0, Team::Blue);
  game.ChooseMove();

  const Unit* mover = game.FindUnit(0);
  const float budget = mover->MoveBudget();
  const glm::vec3 start = mover->position;

  // A click beyond one round's reach is rejected (no plan, still choosing).
  const glm::vec3 far(11.0f, 0.0f, 8.0f);
  CHECK(glm::distance(start, far) > budget);
  game.HoverGround(far, Team::Blue);
  CHECK(!game.MovePreviewValid());
  game.ClickGround(far, Team::Blue);
  CHECK(mover->plan.type == tactics::PlannedActionType::None);
  CHECK(game.Mode() == InputMode::AwaitingMoveDestination);

  // Each in-reach click adds a leg from the previous leg's end; the figure
  // stays selected until the chain is finished.
  const glm::vec3 first = start + glm::vec3(budget * 0.9f, 0.0f, 0.0f);
  game.ClickGround(first, Team::Blue);
  CHECK(mover->plan.type == tactics::PlannedActionType::Move);
  CHECK(mover->plan.queuedLegs.empty());
  CHECK(game.Mode() == InputMode::AwaitingMoveDestination);
  CHECK(glm::distance(game.MoveChainEnd(), first) < 1e-3f);
  const glm::vec3 second = first + glm::vec3(0.0f, 0.0f, budget * 0.9f);
  game.ClickGround(second, Team::Blue);
  CHECK(mover->plan.queuedLegs.size() == 1);
  CHECK(glm::distance(mover->plan.queuedLegs[0].front(), first) < 1e-3f);
  CHECK(glm::distance(game.MoveChainEnd(), second) < 1e-3f);
  game.FinishMovePlan();
  CHECK(game.Mode() == InputMode::AwaitingSelection);

  // Legs auto-arm across rounds with no re-clicking, ending at the last click.
  for (int round = 0; round < 5 && game.FindUnit(0)->plan.type == tactics::PlannedActionType::Move;
       ++round) {
    for (const Unit& u : game.GetScene().units) {
      if (u.id == 0 || u.plan.type != tactics::PlannedActionType::None) continue;
      game.ClickUnit(u.id, u.team);
      game.ChoosePass();
    }
    game.CommitRound();
    while (game.Mode() == InputMode::Executing) game.Update(0.05f);
  }
  CHECK(glm::distance(game.FindUnit(0)->position, second) < 0.1f);
  CHECK(game.FindUnit(0)->plan.type == tactics::PlannedActionType::None);
  CHECK(game.FindUnit(0)->plan.queuedLegs.empty());
}

void TestManualReplanClearsQueuedLegs() {
  GameLogic game(LegacyScene());
  auto planChain = [&] {
    game.ClickUnit(0, Team::Blue);
    game.ChooseMove();
    const glm::vec3 start = game.FindUnit(0)->position;
    const float budget = game.FindUnit(0)->MoveBudget();
    game.ClickGround(start + glm::vec3(budget * 0.9f, 0.0f, 0.0f), Team::Blue);
    game.ClickGround(game.MoveChainEnd() + glm::vec3(0.0f, 0.0f, budget * 0.9f), Team::Blue);
    CHECK(!game.FindUnit(0)->plan.queuedLegs.empty());
    game.FinishMovePlan();
  };

  planChain();
  game.ClickUnit(0, Team::Blue);
  game.ChoosePass();
  CHECK(game.FindUnit(0)->plan.queuedLegs.empty());

  // Choosing Move again starts a fresh chain; Esc abandons the one in progress.
  planChain();
  game.ClickUnit(0, Team::Blue);
  game.ChooseMove();
  CHECK(game.FindUnit(0)->plan.type == tactics::PlannedActionType::None);
  game.ClickGround(game.FindUnit(0)->position + glm::vec3(1.0f, 0.0f, 0.0f), Team::Blue);
  game.CancelAction();
  CHECK(game.FindUnit(0)->plan.type == tactics::PlannedActionType::None);
  CHECK(game.FindUnit(0)->plan.queuedLegs.empty());
}

void TestQueuedLegsSnapshotRoundTrip() {
  GameLogic game(LegacyScene());
  game.ClickUnit(0, Team::Blue);
  game.ChooseMove();
  const glm::vec3 start = game.FindUnit(0)->position;
  const float budget = game.FindUnit(0)->MoveBudget();
  game.ClickGround(start + glm::vec3(budget * 0.9f, 0.0f, 0.0f), Team::Blue);
  game.ClickGround(game.MoveChainEnd() + glm::vec3(0.0f, 0.0f, budget * 0.9f), Team::Blue);
  game.ClickGround(game.MoveChainEnd() + glm::vec3(-budget * 0.5f, 0.0f, 0.0f), Team::Blue);
  game.FinishMovePlan();
  const auto& queued = game.FindUnit(0)->plan.queuedLegs;
  CHECK(queued.size() == 2);
  tactics::GameSnapshot out;
  CHECK(tactics::DeserializeSnapshot(tactics::SerializeSnapshot(game.ExportState()), &out));
  GameLogic mirror(LegacyScene());
  CHECK(mirror.ImportState(out));
  CHECK(mirror.FindUnit(0)->plan.queuedLegs.size() == queued.size());
  CHECK(mirror.FindUnit(0)->plan.queuedLegs[1].size() == queued[1].size());
}

void TestPendingShotFiresWhenTargetWalksIntoView() {
  GameLogic game(LegacyScene());
  MakePassive(game);

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
  game.FinishMovePlan();
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
  game.FinishMovePlan();
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
  MakePassive(game);
  game.ClickUnit(0, Team::Blue);
  game.ChooseMove();
  // Straight line, same row, short of the wall at x in [-1,1] so the path
  // collapses to a direct two-point segment (no detour to complicate the
  // expected travel distance).
  const glm::vec3 destination(-3.0f, 0.0f, -4.0f);
  game.HoverGround(destination, Team::Blue);
  CHECK(game.MovePreviewValid());
  game.ClickGround(destination, Team::Blue);
  game.FinishMovePlan();
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
  game.FinishMovePlan();
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
  game.FinishMovePlan();
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
  game.FinishMovePlan();
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
  game.FinishMovePlan();
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

// Shared setup: blue passes while red4 -- repositioned due west of blue1,
// behind its fixed +X facing -- plans a walk east through blue1's open
// lane. Commits the round, leaving the move animating.
void StartRed4WalkThroughBlue1Lane(GameLogic& game, const glm::vec3& destination) {
  game.FindUnit(4)->position = glm::vec3(-9.5f, 0.0f, 0.0f);
  for (int id : {0, 1, 2}) {
    game.ClickUnit(id, Team::Blue);
    game.ChoosePass();
  }
  game.ClickUnit(3, Team::Red);
  game.ChoosePass();
  game.ClickUnit(4, Team::Red);
  game.ChooseMove();
  game.ClickGround(destination, Team::Red);
  game.FinishMovePlan();
  game.ClickUnit(5, Team::Red);
  game.ChoosePass();
  CHECK(game.CanCommitRound());
  game.CommitRound();
  CHECK(game.Mode() == InputMode::Executing);
}

void SetStationaryShoot(GameLogic& game, Team team) {
  SquadPlaybook pb = game.Playbook(team);
  pb.At(false, true) = ReactionAction::Shoot;
  pb.At(false, false) = ReactionAction::Shoot;
  game.SetPlaybook(team, pb);
}

// Runs the committed round to completion.
void RunRound(GameLogic& game) {
  int steps = 0;
  while (game.Mode() == InputMode::Executing && steps < 10000) {
    game.Update(0.02f);
    ++steps;
  }
  CHECK(steps < 10000);
}

// Red4 walks east from behind blue1 (which faces +X). Applies `redPb` to red
// and runs the round; returns red4's final distance to the destination.
float RunRed4Walk(const SquadPlaybook& redPb, bool* red4Alive, bool* blue1Alive,
                  bool blueShoots) {
  GameLogic game(LegacyScene());
  game.SetPlaybook(Team::Red, redPb);
  if (blueShoots) SetStationaryShoot(game, Team::Blue);
  const glm::vec3 destination(0.0f, 0.0f, 0.0f);
  game.FindUnit(4)->position = glm::vec3(-9.5f, 0.0f, 0.0f);
  for (int id : {0, 1, 2}) {
    game.ClickUnit(id, Team::Blue);
    game.ChoosePass();
  }
  game.ClickUnit(3, Team::Red);
  game.ChoosePass();
  game.ClickUnit(4, Team::Red);
  game.ChooseMove();
  game.ClickGround(destination, Team::Red);
  game.FinishMovePlan();
  game.ClickUnit(5, Team::Red);
  game.ChoosePass();
  game.CommitRound();
  RunRound(game);
  *red4Alive = game.FindUnit(4)->alive;
  *blue1Alive = game.FindUnit(1)->alive;
  return glm::distance(game.FindUnit(4)->position, destination);
}

void TestGameLogicPlaybookShootsOnFovEntryAndPersistsAcrossRounds() {
  GameLogic game(LegacyScene());
  // Flipping the field directly mirrors what the config popup does; it
  // doesn't touch turn state, so blue1's plan below is still just Pass.
  Unit* blue1 = game.FindUnit(1);
  SetStationaryShoot(game, Team::Blue);

  const glm::vec3 destination(0.0f, 0.0f, 0.0f);
  StartRed4WalkThroughBlue1Lane(game, destination);

  int steps = 0;
  while (game.Mode() == InputMode::Executing && steps < 10000) {
    game.Update(0.02f);
    ++steps;
  }
  CHECK(steps < 10000);
  CHECK(!game.FindUnit(4)->alive);
  CHECK(glm::distance(game.FindUnit(4)->position, destination) > 1.0f);
  CHECK(game.FindUnit(4)->position.x > -9.5f + 1e-3f);
  // Standing rule: stays in force after firing.
  CHECK(game.Playbook(Team::Blue).At(false, false) == ReactionAction::Shoot);
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  CHECK(game.RoundNumber() == 2);
}

void TestGameLogicPlaybookDefaultTable() {
  GameLogic game(LegacyScene());
  for (Team t : {Team::Blue, Team::Red}) {
    CHECK(game.Playbook(t).At(true, true) == ReactionAction::ShootContinue);
    CHECK(game.Playbook(t).At(true, false) == ReactionAction::Continue);
    CHECK(game.Playbook(t).At(false, true) == ReactionAction::Shoot);
    CHECK(game.Playbook(t).At(false, false) == ReactionAction::DoNothing);
  }
  game.SetPlaybook(Team::Blue, SquadPlaybook::Passive());
  CHECK(game.Playbook(Team::Blue).At(false, true) == ReactionAction::DoNothing);

  const glm::vec3 destination(0.0f, 0.0f, 0.0f);
  StartRed4WalkThroughBlue1Lane(game, destination);
  game.Update(100.0f);  // Fast-forward: nothing should interrupt this move.
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  CHECK(game.FindUnit(4)->alive);
  CHECK(glm::distance(game.FindUnit(4)->position, destination) < 1e-3f);
}

// Regression: the Shoot rule only fires on enemies. Arms red3 (Shoot) and
// walks red4 through its FOV; the teammate must be left alone.
void TestGameLogicPlaybookIgnoresSameTeamMover() {
  GameLogic game(LegacyScene());
  SetStationaryShoot(game, Team::Red);

  const glm::vec3 destination(0.0f, 0.0f, 0.0f);
  StartRed4WalkThroughBlue1Lane(game, destination);
  game.Update(100.0f);
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  CHECK(game.FindUnit(4)->alive);
  CHECK(glm::distance(game.FindUnit(4)->position, destination) < 1e-3f);
}

void TestGameLogicPlaybookMovingRows() {
  bool red4Alive = false, blue1Alive = false;
  SquadPlaybook pb;

  // Moving + continue: finishes the walk uninterrupted.
  float dist = RunRed4Walk(pb, &red4Alive, &blue1Alive, false);
  CHECK(red4Alive && dist < 1e-3f);

  // Moving + stop (both visibility columns): halts mid-path, alive.
  pb.At(true, true) = ReactionAction::Stop;
  pb.At(true, false) = ReactionAction::Stop;
  dist = RunRed4Walk(pb, &red4Alive, &blue1Alive, false);
  CHECK(red4Alive && dist > 1.0f);

  // Moving + shoot->continue: blue1 is downed, red4 still completes its move.
  pb.At(true, true) = ReactionAction::ShootContinue;
  pb.At(true, false) = ReactionAction::ShootContinue;
  dist = RunRed4Walk(pb, &red4Alive, &blue1Alive, false);
  CHECK(!blue1Alive && red4Alive && dist < 1e-3f);

  // Moving + shoot->stop: shoots and halts.
  pb.At(true, true) = ReactionAction::ShootStop;
  pb.At(true, false) = ReactionAction::ShootStop;
  dist = RunRed4Walk(pb, &red4Alive, &blue1Alive, false);
  CHECK(!blue1Alive && red4Alive && dist > 1.0f);
}

// The visibility axis selects the column: red4 walks away from blue1's gaze
// and can't be seen by it only if blue1 faces away; here blue1 faces +X and
// red4 starts west of it, so blue1 sees red4 only once red4 is ahead of it,
// while red4 (moving east) sees blue1 first. Setting only one column to Stop
// shows which column applies on each side of that crossing.
void TestGameLogicPlaybookVisibilityColumns() {
  bool red4Alive = false, blue1Alive = false;
  SquadPlaybook seenOnly;
  seenOnly.At(true, true) = ReactionAction::Stop;
  SquadPlaybook unseenOnly;
  unseenOnly.At(true, false) = ReactionAction::Stop;
  const float seenDist = RunRed4Walk(seenOnly, &red4Alive, &blue1Alive, false);
  const float unseenDist = RunRed4Walk(unseenOnly, &red4Alive, &blue1Alive, false);
  // Exactly one of the two columns governs this walk.
  CHECK((seenDist < 1e-3f) != (unseenDist < 1e-3f));
}

// Red4 walks +Z toward two blue figures at z=6 (clear of the scene's obstacles). `blueA` faces red4 (sees it
// back); `blueB` faces away. With "seen" = Stop and "unseen" = Continue, the
// walk halts only if the any-enemy-sees-back tie-break treats red4 as seen.
float RunTieBreakWalk(bool includeSeeingEnemy) {
  GameLogic game(LegacyScene());
  for (Unit& u : const_cast<std::vector<Unit>&>(game.GetScene().units)) {
    // Park everyone else far away; blue faces away so it can't see red4.
    u.position = glm::vec3(u.team == Team::Blue ? -40.0f : 40.0f, 0.0f, 40.0f + u.id);
    u.facingYaw = 3.14159265f;
  }
  const float kHalfPi = 1.57079632679f;
  Unit* red4 = game.FindUnit(4);
  red4->position = glm::vec3(8.0f, 0.0f, 0.0f);
  red4->facingYaw = kHalfPi;
  Unit* blueA = game.FindUnit(0);
  blueA->position = glm::vec3(8.0f, 0.0f, 6.0f);
  blueA->facingYaw = includeSeeingEnemy ? -kHalfPi : kHalfPi;
  Unit* blueB = game.FindUnit(1);
  blueB->position = glm::vec3(9.0f, 0.0f, 6.0f);
  blueB->facingYaw = kHalfPi;

  SquadPlaybook pb;
  pb.At(true, true) = ReactionAction::Stop;
  pb.At(true, false) = ReactionAction::Continue;
  game.SetPlaybook(Team::Red, pb);

  const glm::vec3 destination(8.0f, 0.0f, 3.0f);
  for (int id : {0, 1, 2}) {
    game.ClickUnit(id, Team::Blue);
    game.ChoosePass();
  }
  game.ClickUnit(3, Team::Red);
  game.ChoosePass();
  game.ClickUnit(4, Team::Red);
  game.ChooseMove();
  game.ClickGround(destination, Team::Red);
  game.FinishMovePlan();
  game.ClickUnit(5, Team::Red);
  game.ChoosePass();
  game.CommitRound();
  RunRound(game);
  return glm::distance(game.FindUnit(4)->position, destination);
}

void TestGameLogicPlaybookMultiEnemyTieBreak() {
  // Only an enemy that can't see back: "unseen" column -> continue.
  CHECK(RunTieBreakWalk(false) < 1e-3f);
  // One enemy sees back, the other doesn't: treated as seen -> stop.
  CHECK(RunTieBreakWalk(true) > 0.5f);
}

// Isolated run of one playbook checkbox: red4 walks a straight lane past a
// stationary blue0 while every other cell of both teams' tables stays
// passive. `movingRow` picks whose table holds the cell under test -- the
// mover's (red) for the moving rows, the watcher's (blue) for the stationary
// rows. Geometry pins the visibility column for the whole walk: the actor
// always has the enemy in its own FOV (a reaction needs a sighted enemy),
// and facing/walk direction decides whether that enemy sees it back.
struct PlaybookCellOutcome {
  float moverDistToDest;
  bool moverAlive;
  bool watcherAlive;
};

PlaybookCellOutcome RunPlaybookCell(bool movingRow, bool canSeeMe, ReactionAction action) {
  GameLogic game(LegacyScene());
  for (Unit& u : const_cast<std::vector<Unit>&>(game.GetScene().units)) {
    // Park everyone uninvolved far away, facing nothing.
    u.position = glm::vec3(u.team == Team::Blue ? -40.0f : 40.0f, 0.0f, 40.0f + u.id);
    u.facingYaw = 3.14159265f;
  }
  const float kHalfPi = 1.57079632679f;
  Unit* watcher = game.FindUnit(0);  // Blue; stationary all round.
  watcher->position = glm::vec3(8.0f, 0.0f, 6.0f);
  Unit* mover = game.FindUnit(4);  // Red; walks the x=8 lane.
  glm::vec3 destination;
  SquadPlaybook pb = SquadPlaybook::Passive();
  pb.At(movingRow, canSeeMe) = action;
  if (movingRow) {
    // Mover's row: it walks toward the watcher (so the enemy stays sighted);
    // the watcher faces the lane only in the "seen" column.
    mover->position = glm::vec3(8.0f, 0.0f, 0.0f);
    mover->facingYaw = kHalfPi;
    destination = glm::vec3(8.0f, 0.0f, 3.0f);
    watcher->facingYaw = canSeeMe ? -kHalfPi : kHalfPi;
    game.SetPlaybook(Team::Red, pb);
    game.SetPlaybook(Team::Blue, SquadPlaybook::Passive());
  } else {
    // Watcher's row: it always faces the lane; the mover walks toward it
    // (the sighted enemy sees the watcher back: "seen") or away ("unseen").
    watcher->facingYaw = -kHalfPi;
    mover->position = glm::vec3(8.0f, 0.0f, canSeeMe ? 0.0f : 3.0f);
    mover->facingYaw = canSeeMe ? kHalfPi : -kHalfPi;
    destination = glm::vec3(8.0f, 0.0f, canSeeMe ? 3.0f : 0.0f);
    game.SetPlaybook(Team::Blue, pb);
    game.SetPlaybook(Team::Red, SquadPlaybook::Passive());
  }

  for (int id : {0, 1, 2}) {
    game.ClickUnit(id, Team::Blue);
    game.ChoosePass();
  }
  game.ClickUnit(3, Team::Red);
  game.ChoosePass();
  game.ClickUnit(4, Team::Red);
  game.ChooseMove();
  game.ClickGround(destination, Team::Red);
  game.FinishMovePlan();
  game.ClickUnit(5, Team::Red);
  game.ChoosePass();
  CHECK(game.CanCommitRound());
  game.CommitRound();
  RunRound(game);
  return PlaybookCellOutcome{glm::distance(game.FindUnit(4)->position, destination),
                             game.FindUnit(4)->alive, game.FindUnit(0)->alive};
}

// Regression sweep over the whole playbook grid: every checkbox the HUD
// offers (moving rows: Stop/Continue/ShootStop/ShootContinue; stationary
// rows: DoNothing/Shoot -- mirroring the enabled columns in Hud.cpp's
// DrawPlaybookView), each selected on its own in an otherwise passive
// table, must produce exactly its advertised effect and nothing else.
void TestGameLogicPlaybookEveryCheckbox() {
  for (bool canSeeMe : {false, true}) {
    for (ReactionAction action : {ReactionAction::Stop, ReactionAction::Continue,
                                  ReactionAction::ShootStop, ReactionAction::ShootContinue}) {
      const PlaybookCellOutcome out = RunPlaybookCell(true, canSeeMe, action);
      CHECK(out.moverAlive);  // The watcher's table is passive: nobody shoots back.
      CHECK(out.watcherAlive != ReactionShoots(action));
      if (ReactionStops(action)) CHECK(out.moverDistToDest > 0.5f);
      else CHECK(out.moverDistToDest < 1e-3f);
    }
    for (ReactionAction action : {ReactionAction::DoNothing, ReactionAction::Shoot}) {
      const PlaybookCellOutcome out = RunPlaybookCell(false, canSeeMe, action);
      CHECK(out.watcherAlive);  // The mover's table is passive: it never shoots.
      CHECK(out.moverAlive != ReactionShoots(action));
      // A downed mover drops short of its destination; otherwise it arrives.
      if (ReactionShoots(action)) CHECK(out.moverDistToDest > 0.5f);
      else CHECK(out.moverDistToDest < 1e-3f);
    }
  }
}

void TestSnapshotCarriesSquadPlaybook() {
  GameLogic a(LegacyScene());
  SquadPlaybook pb;
  pb.At(true, true) = ReactionAction::ShootStop;
  pb.At(false, false) = ReactionAction::Shoot;
  a.SetPlaybook(Team::Red, pb);
  GameSnapshot snap;
  CHECK(DeserializeSnapshot(SerializeSnapshot(a.ExportState()), &snap));
  GameLogic b(LegacyScene());
  CHECK(b.ImportState(snap));
  CHECK(b.Playbook(Team::Red) == pb);
  CHECK(b.Playbook(Team::Blue) == SquadPlaybook{});
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

// Walk cycle: phase advances with distance walked (2*pi per stride length)
// and the blend eases in only for figures with an in-flight move, then
// snaps back to rest when the round is fast-forwarded to completion.
void TestWalkCycleTracksInFlightMove() {
  GameLogic game(LegacyScene());
  MakePassive(game);
  Unit* mover = game.FindUnit(0);
  const glm::vec3 start = mover->position;
  game.ClickUnit(0, Team::Blue);
  game.ChooseMove();
  game.ClickGround(start + glm::vec3(4.0f, 0.0f, 0.0f), Team::Blue);
  game.FinishMovePlan();
  CHECK(mover->plan.type == PlannedActionType::Move);
  for (int id : {1, 2, 3, 4, 5}) {
    game.ClickUnit(id, id < 3 ? Team::Blue : Team::Red);
    game.ChoosePass();
  }
  CHECK(mover->walkPhase == 0.0f && mover->walkBlend == 0.0f);

  game.CommitRound();
  CHECK(game.Mode() == InputMode::Executing);
  game.Update(0.1f);
  const float walked = glm::distance(start, mover->position);
  CHECK(walked > 0.0f);
  const float kTwoPi = 6.28318530717958647692f;
  CHECK(std::fabs(mover->walkPhase - walked * kTwoPi / constants::kWalkStrideLength) < 1e-4f);
  CHECK(mover->walkBlend > 0.0f && mover->walkBlend <= 1.0f);
  // Figures standing still never pick up any walk pose.
  for (int id : {1, 2, 3, 4, 5}) {
    CHECK(game.FindUnit(id)->walkPhase == 0.0f);
    CHECK(game.FindUnit(id)->walkBlend == 0.0f);
  }

  game.Update(1.0e6f);  // Fast-forward: the move finishes inside this tick.
  CHECK(game.Mode() != InputMode::Executing);
  CHECK(mover->walkBlend == 0.0f);  // Judged after the move step, so already at rest.
  CHECK(mover->walkPhase >= 0.0f && mover->walkPhase < kTwoPi);
}

// A resolved (hitting) shot starts the shooter's quick-draw beat aimed at
// the target's bearing; it times out back to idle. Hit resolution itself
// is unchanged, and a blocked shot doesn't play anything.
void TestResolvedShotStartsShootAnimation() {
  GameLogic game(LegacyScene());
  Unit* shooter = game.FindUnit(1);  // Open middle lane: blue1 <-> red4.
  Unit* target = game.FindUnit(4);
  // Aim the shooter a bit off the target so the aim yaw is distinguishable
  // from its facing yaw (still well inside the 150 degree cone).
  const glm::vec3 toTarget = target->position - shooter->position;
  const float bearing = std::atan2(toTarget.z, toTarget.x);
  shooter->facingYaw = bearing + 0.4f;
  CHECK(shooter->shootElapsed < 0.0f);

  CHECK(game.ResolveShot(*shooter, *target));
  CHECK(!target->alive);
  CHECK(target->knockdownElapsed == 0.0f);
  CHECK(shooter->shootElapsed == 0.0f);
  CHECK(std::fabs(shooter->shootAimYaw - bearing) < 1e-4f);
  CHECK(target->shootElapsed < 0.0f);  // Only the shooter animates.

  game.Update(0.1f);  // Advances in any mode, like the knockdown timer.
  CHECK(std::fabs(shooter->shootElapsed - 0.1f) < 1e-5f);
  game.Update(constants::kShootAnimDuration);
  CHECK(shooter->shootElapsed < 0.0f);  // Beat over: back to idle.
  CHECK(target->knockdownElapsed == constants::kKnockdownDuration);

  // A shot that can't connect (target behind the shooter) fires nothing.
  Unit* other = game.FindUnit(5);
  shooter->facingYaw = bearing + 3.0f;
  CHECK(!game.ResolveShot(*shooter, *other));
  CHECK(other->alive);
  CHECK(shooter->shootElapsed < 0.0f);
}

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
  red.FinishMovePlan();
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
  // The mover has walked a stride by now, so the animation state being
  // mirrored is non-trivial.
  CHECK(blue.FindUnit(3)->walkPhase > 0.0f);
  for (const auto& unit : blue.GetScene().units) {
    const Unit* mirrored = red.FindUnit(unit.id);
    CHECK(mirrored && mirrored->position == unit.position);
    CHECK(mirrored && mirrored->facingYaw == unit.facingYaw);
    CHECK(mirrored && mirrored->plan.type == unit.plan.type);
    CHECK(mirrored && mirrored->walkPhase == unit.walkPhase);
    CHECK(mirrored && mirrored->walkBlend == unit.walkBlend);
    CHECK(mirrored && mirrored->idleElapsed == unit.idleElapsed);
    CHECK(mirrored && mirrored->shootElapsed == unit.shootElapsed);
    CHECK(mirrored && mirrored->shootAimYaw == unit.shootAimYaw);
  }

  GameSnapshot bad;
  CHECK(!DeserializeSnapshot("garbage", &bad));
  CHECK(!DeserializeSnapshot("", &bad));
}

// --- Enemy sighting memory ---------------------------------------------

void StepSightings(GameLogic& game, float seconds, float dt = 0.05f) {
  const int steps = static_cast<int>(std::lround(seconds / dt));
  for (int i = 0; i < steps; ++i) game.UpdateSightingMemory(dt);
}

// Simulates a round ending: bumps the round number the way a real round
// transition would, then ticks memory once so it ages.
void AdvanceRounds(GameLogic& game, int rounds) {
  GameSnapshot snap = game.ExportState();
  snap.roundNumber += rounds;
  CHECK(game.ImportState(snap));
  game.UpdateSightingMemory(0.0f);
}

void BlueLookAway(GameLogic& game, float yaw) {
  for (int id = 0; id <= 2; ++id) game.FindUnit(id)->facingYaw = yaw;
}

void TestSightingRecordedImmediatelyOnEntry() {
  GameLogic game(LegacyScene());
  BlueLookAway(game, kPi);
  StepSightings(game, 1.0f);
  CHECK(game.Sightings(Team::Blue, 4).empty());
  BlueLookAway(game, 0.0f);
  game.UpdateSightingMemory(0.05f);
  CHECK(game.Sightings(Team::Blue, 4).size() == 1);
  CHECK(game.Sightings(Team::Red, 4).empty());  // Own figures aren't "sighted".
}

void TestSightingSamplesAccumulateWhileInFov() {
  GameLogic game(LegacyScene());
  StepSightings(game, 1.0f);
  // Stationary figure: only the entry sample, no stacked duplicates.
  CHECK(game.Sightings(Team::Blue, 4).size() == 1);
  // Once it moves, a sample is taken per 0.5s interval.
  Unit* enemy = game.FindUnit(4);
  for (int i = 0; i < 60; ++i) {
    enemy->position.x += 0.01f;
    game.UpdateSightingMemory(0.05f);
  }
  const auto& samples = game.Sightings(Team::Blue, 4);
  CHECK(samples.size() >= 6 && samples.size() <= 7);
  for (size_t i = 0; i + 1 < samples.size(); ++i) {
    CHECK(samples[i].ageRounds == 0);  // No fading in real time.
  }
}

void TestSightingMoveDirectionOnlyWhenMoving() {
  GameLogic game(LegacyScene());
  StepSightings(game, 1.0f);
  for (const auto& s : game.Sightings(Team::Blue, 4)) {
    CHECK(glm::length(s.moveDirection) == 0.0f);
  }
  const size_t stationaryCount = game.Sightings(Team::Blue, 4).size();
  Unit* red = game.FindUnit(4);
  for (int i = 0; i < 20; ++i) {
    red->position.x -= 0.1f;
    game.UpdateSightingMemory(0.05f);
  }
  const auto& samples = game.Sightings(Team::Blue, 4);
  CHECK(samples.size() > stationaryCount);
  const auto& last = samples.back();
  CHECK(std::fabs(last.moveDirection.x + 1.0f) < 1e-3f);
  CHECK(std::fabs(last.moveDirection.z) < 1e-3f);
}

void TestSightingCapturesAnimationPose() {
  GameLogic game(LegacyScene());
  Unit* red = game.FindUnit(4);
  red->walkPhase = 1.25f;
  red->walkBlend = 0.75f;
  red->idleElapsed = 2.5f;
  StepSightings(game, 1.0f);
  const auto& samples = game.Sightings(Team::Blue, 4);
  CHECK(!samples.empty());
  const GameLogic::EnemySighting first = samples.front();
  CHECK(first.walkPhase == 1.25f);
  CHECK(first.walkBlend == 0.75f);
  CHECK(first.idleElapsed == 2.5f);
  red->walkPhase = 3.0f;
  red->walkBlend = 0.1f;
  red->idleElapsed = 9.0f;
  CHECK(game.Sightings(Team::Blue, 4).front().walkPhase == 1.25f);
  CHECK(game.Sightings(Team::Blue, 4).front().walkBlend == 0.75f);
  CHECK(game.Sightings(Team::Blue, 4).front().idleElapsed == 2.5f);
}

void TestSightingsPersistAfterLeavingFovThenExpire() {
  GameLogic game(LegacyScene());
  StepSightings(game, 2.0f);
  const size_t count = game.Sightings(Team::Blue, 4).size();
  CHECK(count > 0);
  BlueLookAway(game, kPi);
  StepSightings(game, 2.0f);
  CHECK(game.Sightings(Team::Blue, 4).size() == count);  // Not cleared, none added.
  StepSightings(game, 60.0f);
  CHECK(game.Sightings(Team::Blue, 4).size() == count);  // Real time doesn't expire.
  AdvanceRounds(game, constants::kSightingMemoryRounds - 1);
  CHECK(game.Sightings(Team::Blue, 4).size() == count);
  CHECK(game.Sightings(Team::Blue, 4).front().ageRounds == constants::kSightingMemoryRounds - 1);
  AdvanceRounds(game, 1);
  CHECK(game.Sightings(Team::Blue, 4).empty());
}

void TestSightingReentryAppendsToAgingTrail() {
  GameLogic game(LegacyScene());
  StepSightings(game, 1.0f);
  const size_t before = game.Sightings(Team::Blue, 4).size();
  BlueLookAway(game, kPi);
  AdvanceRounds(game, 2);
  BlueLookAway(game, 0.0f);
  game.UpdateSightingMemory(0.05f);
  const auto& samples = game.Sightings(Team::Blue, 4);
  CHECK(samples.size() == before + 1);
  CHECK(samples.front().ageRounds == 2);  // Kept aging, not reset.
  CHECK(samples.back().ageRounds == 0);
}

void TestResetClearsSightings() {
  GameLogic game(LegacyScene());
  StepSightings(game, 1.0f);
  CHECK(!game.Sightings(Team::Blue, 4).empty());
  game.Reset(LegacyScene());
  CHECK(game.Sightings(Team::Blue, 4).empty());
}

void TestImportStateOfNewGameClearsFollowerSightings() {
  GameLogic sim(LegacyScene()), follower(LegacyScene());
  GameSnapshot lateRound = sim.ExportState();
  lateRound.roundNumber = 3;
  CHECK(follower.ImportState(lateRound));
  StepSightings(follower, 1.0f);
  CHECK(!follower.Sightings(Team::Blue, 4).empty());
  sim.Reset(LegacyScene());  // New game: same unit ids, round back to 1.
  CHECK(follower.ImportState(sim.ExportState()));
  CHECK(follower.Sightings(Team::Blue, 4).empty());
}

void TestFollowerBuildsSightingsWithoutPhysicsUpdate() {
  GameLogic sim(LegacyScene()), follower(LegacyScene());
  MakePassive(sim);
  MakePassive(follower);
  sim.ClickUnit(4, Team::Red);
  sim.ChooseMove();
  sim.ClickGround(glm::vec3(-2.0f, 0.0f, 0.0f), Team::Red);
  sim.FinishMovePlan();
  CHECK(sim.FindUnit(4)->plan.type == PlannedActionType::Move);
  PassEveryoneElse(sim, {4});
  CHECK(sim.CanCommitRound());
  sim.CommitRound();
  CHECK(sim.Mode() == InputMode::Executing);

  // Mirror main.cpp: only the simulator runs Update() while Executing; both
  // pages tick sighting memory every frame.
  int frames = 0;
  while (sim.Mode() == InputMode::Executing && frames < 10000) {
    sim.Update(0.05f);
    CHECK(follower.ImportState(sim.ExportState()));
    if (follower.Mode() != InputMode::Executing) follower.Update(0.05f);
    sim.UpdateSightingMemory(0.05f);
    follower.UpdateSightingMemory(0.05f);
    ++frames;
  }
  CHECK(frames < 10000);
  const auto& simSamples = sim.Sightings(Team::Blue, 4);
  const auto& followerSamples = follower.Sightings(Team::Blue, 4);
  CHECK(simSamples.size() >= 4);
  CHECK(followerSamples.size() == simSamples.size());
  bool anyMoving = false;
  for (const auto& sample : followerSamples) anyMoving |= glm::length(sample.moveDirection) > 0.5f;
  CHECK(anyMoving);
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
  TestMoveFrontierRoutesAroundObstacle();
  TestClickChainsLegsAcrossRounds();
  TestManualReplanClearsQueuedLegs();
  TestQueuedLegsSnapshotRoundTrip();
  TestPendingShotFiresWhenTargetWalksIntoView();
  TestDefaultSceneSquadsStartHidden();
  TestGameLogicShootGatingRequiresTeamVisibility();
  TestGameLogicDownedEnemyStaysVisibleInFov();
  TestGameLogicMoveUpdatesPositionAndFacing();
  TestGameLogicMoveAnimatesProgressively();
  TestGameLogicMoveFacingAdjustableBeforeCommit();
  TestGameLogicIgnoresInputWhileExecuting();
  TestGameLogicMoveCanClimbOntoObstacle();
  TestGameLogicPlaybookShootsOnFovEntryAndPersistsAcrossRounds();
  TestGameLogicPlaybookDefaultTable();
  TestGameLogicPlaybookIgnoresSameTeamMover();
  TestGameLogicPlaybookMovingRows();
  TestGameLogicPlaybookVisibilityColumns();
  TestGameLogicPlaybookMultiEnemyTieBreak();
  TestGameLogicPlaybookEveryCheckbox();
  TestSnapshotCarriesSquadPlaybook();
  TestGameLogicWinCondition();
  TestWalkCycleTracksInFlightMove();
  TestResolvedShotStartsShootAnimation();
  TestSnapshotMirrorsMatchAndTeamPlans();
  TestSightingRecordedImmediatelyOnEntry();
  TestSightingSamplesAccumulateWhileInFov();
  TestSightingMoveDirectionOnlyWhenMoving();
  TestSightingCapturesAnimationPose();
  TestSightingsPersistAfterLeavingFovThenExpire();
  TestSightingReentryAppendsToAgingTrail();
  TestResetClearsSightings();
  TestImportStateOfNewGameClearsFollowerSightings();
  TestFollowerBuildsSightingsWithoutPhysicsUpdate();

  if (g_failures == 0) {
    std::printf("All logic tests passed.\n");
    return 0;
  }
  std::fprintf(stderr, "%d check(s) failed.\n", g_failures);
  return 1;
}
