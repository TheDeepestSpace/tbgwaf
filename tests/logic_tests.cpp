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

void TestTurnManagerAlternatesByTeamAndRounds() {
  // Plan-then-commit turn model: a "turn" is a whole team's block, not a
  // single figure, so the manager just alternates Blue/Red and bumps the
  // round once both have gone.
  TurnManager tm;
  tm.StartRound();
  CHECK(tm.CurrentTeam() == Team::Blue);
  CHECK(tm.RoundNumber() == 1);

  tm.AdvanceTurn();
  CHECK(tm.CurrentTeam() == Team::Red);
  CHECK(tm.RoundNumber() == 1);

  tm.AdvanceTurn();
  CHECK(tm.CurrentTeam() == Team::Blue);
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
  GameLogic game(LegacyScene());
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  CHECK(game.CurrentTeam() == Team::Blue);  // Blue plans first (Scene builds Blue ids 0-2, Red 3-5).

  // Clicking a figure on the non-acting team must be a no-op.
  game.ClickUnit(3);
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  CHECK(!game.SelectedUnitId().has_value());

  // Any living figure on the acting team can be selected to plan its action
  // -- there's no single "current actor" any more, the whole squad plans.
  game.ClickUnit(1);
  CHECK(game.Mode() == InputMode::ActionMenu);
  CHECK(game.SelectedUnitId() == 1);

  game.CancelAction();
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  CHECK(!game.SelectedUnitId().has_value());
}

void TestGameLogicPlanThenCommitDefersExecutionAndAppliesSimultaneously() {
  GameLogic game(LegacyScene());
  CHECK(game.CurrentTeam() == Team::Blue);

  const glm::vec3 blue0Start = game.FindUnit(0)->position;
  const glm::vec3 blue1Start = game.FindUnit(1)->position;

  // Plan a move on blue0, a shoot (open lane, would hit) on blue1, and a
  // pass on blue2 -- one action per figure, nothing executes yet.
  game.ClickUnit(0);
  game.ChooseMove();
  const glm::vec3 destination(-3.0f, 0.0f, -4.0f);
  game.ClickGround(destination);
  CHECK(game.FindUnit(0)->plan.type == tactics::PlannedActionType::Move);

  game.ClickUnit(1);
  game.ChooseShoot();
  game.ClickUnit(4);
  CHECK(game.FindUnit(1)->plan.type == tactics::PlannedActionType::Shoot);
  CHECK(game.FindUnit(1)->plan.shootTargetId == 4);

  CHECK(!game.CanCommitTurn());  // blue2 hasn't planned yet.
  game.ClickUnit(2);
  game.ChoosePass();
  CHECK(game.FindUnit(2)->plan.type == tactics::PlannedActionType::Pass);
  CHECK(game.CanCommitTurn());

  // World state is completely unchanged by planning alone.
  CHECK(glm::distance(game.FindUnit(0)->position, blue0Start) < 1e-6f);
  CHECK(glm::distance(game.FindUnit(1)->position, blue1Start) < 1e-6f);
  CHECK(game.FindUnit(4)->alive);
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  CHECK(game.CurrentTeam() == Team::Blue);

  game.CommitTurn();

  // Shots resolve the instant the turn is committed -- blue1's shot doesn't
  // wait on blue0's move animation to finish.
  CHECK(game.Mode() == InputMode::Moving);
  CHECK(!game.FindUnit(4)->alive);          // blue1's planned shot already applied.
  CHECK(game.FindUnit(0)->plan.type == tactics::PlannedActionType::None);  // Plans cleared.
  CHECK(game.FindUnit(1)->plan.type == tactics::PlannedActionType::None);
  CHECK(game.IsUnitMoving(0));               // blue0's planned move is animating.

  game.Update(100.0f);  // Finish blue0's move (the only thing left in flight).
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  CHECK(glm::distance(game.FindUnit(0)->position, destination) < 1e-3f);
  CHECK(game.CurrentTeam() == Team::Red);  // Turn passed to the other team.
}

void TestGameLogicCommitAnimatesMultipleMovesConcurrently() {
  GameLogic game(LegacyScene());
  game.ClickUnit(0);
  game.ChooseMove();
  const glm::vec3 destination0(-3.0f, 0.0f, -4.0f);
  game.ClickGround(destination0);

  game.ClickUnit(1);
  game.ChooseMove();
  const glm::vec3 destination1(-3.0f, 0.0f, 0.0f);
  game.ClickGround(destination1);

  game.ClickUnit(2);
  game.ChoosePass();
  CHECK(game.CanCommitTurn());
  game.CommitTurn();
  CHECK(game.Mode() == InputMode::Moving);
  CHECK(game.IsUnitMoving(0));
  CHECK(game.IsUnitMoving(1));

  const glm::vec3 start0(-8.0f, 0.0f, -4.0f);
  const glm::vec3 start1(-8.0f, 0.0f, 0.0f);
  const float totalDistance = glm::distance(start0, destination0);
  const float halfwayDt = (totalDistance * 0.5f) / tactics::constants::kMoveSpeed;

  // A single Update() call advances every animating figure at once -- blue1
  // doesn't sit idle waiting for blue0 to finish moving first.
  game.Update(halfwayDt);
  CHECK(game.Mode() == InputMode::Moving);
  CHECK(glm::distance(game.FindUnit(0)->position, start0) > totalDistance * 0.25f);
  CHECK(glm::distance(game.FindUnit(1)->position, start1) > totalDistance * 0.25f);
  CHECK(glm::distance(game.FindUnit(0)->position, destination0) > 1e-3f);
  CHECK(glm::distance(game.FindUnit(1)->position, destination1) > 1e-3f);

  game.Update(100.0f);  // Fast-forward the rest.
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  CHECK(glm::distance(game.FindUnit(0)->position, destination0) < 1e-3f);
  CHECK(glm::distance(game.FindUnit(1)->position, destination1) < 1e-3f);
  CHECK(game.CurrentTeam() == Team::Red);
}

void TestGameLogicShootRowsMatchLayout() {
  GameLogic game(LegacyScene());
  // Row z=-4 (blue id0 vs red id3) is behind the first wall: must miss. Row
  // z=0 (blue id1 vs red id4) is the open lane: must hit. Plan both plus a
  // pass for blue2, then commit the whole squad at once.
  CHECK(game.CurrentTeam() == Team::Blue);
  game.ClickUnit(0);
  game.ChooseShoot();
  CHECK(game.Mode() == InputMode::AwaitingShootTarget);
  game.ClickUnit(3);
  CHECK(game.FindUnit(3)->alive);  // Still just a plan.

  game.ClickUnit(1);
  game.ChooseShoot();
  game.ClickUnit(4);

  game.ClickUnit(2);
  game.ChoosePass();

  CHECK(game.CanCommitTurn());
  game.CommitTurn();

  CHECK(game.FindUnit(3)->alive);      // Blocked shot: miss.
  CHECK(!game.FindUnit(4)->alive);     // Open lane: hit.
  CHECK(!game.Winner().has_value());   // Red still has id3, id5 alive.
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  CHECK(game.CurrentTeam() == Team::Red);  // Turn advanced to Red.

  // Symmetric: red0 shooting blue0 across the same blocked row also misses,
  // and Red's other figures just pass.
  game.ClickUnit(3);
  game.ChooseShoot();
  game.ClickUnit(0);
  game.ClickUnit(4);
  game.ChoosePass();
  game.ClickUnit(5);
  game.ChoosePass();
  game.CommitTurn();
  CHECK(game.FindUnit(0)->alive);
  CHECK(game.CurrentTeam() == Team::Blue);
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

  game.ClickUnit(0);
  game.ChooseShoot();
  CHECK(game.Mode() == InputMode::AwaitingShootTarget);

  // Red5 is alive and would otherwise be a legal target, but it's outside
  // Blue's team FOV: the click must be a no-op (not a guaranteed miss) --
  // no plan is recorded and blue0 stays selected for targeting.
  game.ClickUnit(5);
  CHECK(game.Mode() == InputMode::AwaitingShootTarget);
  CHECK(game.FindUnit(0)->plan.type == tactics::PlannedActionType::None);
  CHECK(game.FindUnit(5)->alive);

  // Turn blue0 back to face Red: Red5 re-enters Blue's FOV with clear LOS
  // and becomes a valid planning target again.
  game.FindUnit(0)->facingYaw = 0.0f;
  CHECK(game.ComputeVisibility(Team::Blue).UnitVisible(5));
  game.ClickUnit(5);
  CHECK(game.FindUnit(0)->plan.type == tactics::PlannedActionType::Shoot);
  CHECK(game.FindUnit(0)->plan.shootTargetId == 5);
  CHECK(game.FindUnit(5)->alive);  // Still just a plan; nothing resolved yet.
  CHECK(game.Mode() == InputMode::AwaitingSelection);
}

void TestGameLogicMoveUpdatesPositionAndFacing() {
  GameLogic game(LegacyScene());
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
  // Planning only: the click records blue0's plan and returns to unit
  // selection -- nothing moves and the turn does not advance yet.
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  const Unit* planned = game.FindUnit(0);
  CHECK(planned->plan.type == tactics::PlannedActionType::Move);
  CHECK(std::fabs(planned->position.x - (-8.0f)) < 1e-3f);
  CHECK(std::fabs(planned->position.z - (-4.0f)) < 1e-3f);
  CHECK(game.CurrentTeam() == Team::Blue);

  // Fill out the rest of Blue's plan and commit the turn.
  game.ClickUnit(1);
  game.ChoosePass();
  game.ClickUnit(2);
  game.ChoosePass();
  CHECK(game.CanCommitTurn());
  game.CommitTurn();

  // blue0's move animates rather than teleporting: it's first in squad
  // order, so the commit starts by animating it.
  CHECK(game.Mode() == InputMode::Moving);
  const Unit* moving = game.FindUnit(0);
  CHECK(std::fabs(moving->position.x - (-8.0f)) < 1e-3f);
  CHECK(std::fabs(moving->position.z - (-4.0f)) < 1e-3f);

  // A large fast-forward dt should consume the whole path and complete the
  // move action (and the rest of the commit) in one Update() call.
  game.Update(100.0f);
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  const Unit* moved = game.FindUnit(0);
  CHECK(std::fabs(moved->position.x - destination.x) < 1e-3f);
  CHECK(std::fabs(moved->position.z - destination.z) < 1e-3f);
  // Started at (-8,0,-4), moved to (0,0,0): facing should point roughly
  // toward +X, +Z (atan2(4, 8)).
  const float expectedYaw = std::atan2(4.0f, 8.0f);
  CHECK(std::fabs(moved->facingYaw - expectedYaw) < 1e-3f);

  CHECK(game.CurrentTeam() == Team::Red);  // Turn advanced.
}

void TestGameLogicMoveAnimatesProgressively() {
  GameLogic game(LegacyScene());
  game.ClickUnit(0);
  game.ChooseMove();
  // Straight line, same row, short of the wall at x in [-1,1] so the path
  // collapses to a direct two-point segment (no detour to complicate the
  // expected travel distance).
  const glm::vec3 destination(-3.0f, 0.0f, -4.0f);
  game.HoverGround(destination);
  CHECK(game.MovePreviewValid());
  game.ClickGround(destination);
  CHECK(game.Mode() == InputMode::AwaitingSelection);  // Planned only.

  game.ClickUnit(1);
  game.ChoosePass();
  game.ClickUnit(2);
  game.ChoosePass();
  game.CommitTurn();
  CHECK(game.Mode() == InputMode::Moving);

  const glm::vec3 start(-8.0f, 0.0f, -4.0f);
  const float totalDistance = glm::distance(start, destination);
  const float halfwayDt = (totalDistance * 0.5f) / tactics::constants::kMoveSpeed;

  game.Update(halfwayDt);
  CHECK(game.Mode() == InputMode::Moving);  // Not there yet.
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
  CHECK(game.CurrentTeam() == Team::Red);
}

void TestGameLogicMoveIgnoresInputWhileAnimating() {
  GameLogic game(LegacyScene());
  game.ClickUnit(0);
  game.ChooseMove();
  const glm::vec3 destination(-3.0f, 0.0f, -4.0f);
  game.ClickGround(destination);
  game.ClickUnit(1);
  game.ChoosePass();
  game.ClickUnit(2);
  game.ChoosePass();
  game.CommitTurn();
  CHECK(game.Mode() == InputMode::Moving);

  const glm::vec3 midStart = game.FindUnit(0)->position;

  // Input during the commit's move animation must be a no-op, not a
  // desync: clicking another unit, re-clicking ground, cancel, pass, or
  // even re-triggering commit should all leave the in-flight move untouched.
  game.ClickUnit(3);
  CHECK(game.Mode() == InputMode::Moving);
  game.ClickGround(glm::vec3(5.0f, 0.0f, 5.0f));
  CHECK(game.Mode() == InputMode::Moving);
  game.CancelAction();
  CHECK(game.Mode() == InputMode::Moving);
  game.ChoosePass();
  CHECK(game.Mode() == InputMode::Moving);
  game.CommitTurn();
  CHECK(game.Mode() == InputMode::Moving);
  CHECK(glm::distance(game.FindUnit(0)->position, midStart) < 1e-6f);

  game.Update(100.0f);
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  CHECK(glm::distance(game.FindUnit(0)->position, destination) < 1e-3f);
  CHECK(game.CurrentTeam() == Team::Red);  // Turn advanced exactly once.
}

void TestGameLogicMoveCanClimbOntoObstacle() {
  // BuildDefaultScene marks the standalone crates climbable; the one at
  // (-4, 6.5) is a 1.2x1.2 footprint, 1.2 tall (see Scene.cpp).
  GameLogic game(LegacyScene());
  CHECK(game.CurrentTeam() == Team::Blue);
  game.ClickUnit(0);
  game.ChooseMove();

  const glm::vec3 crateTop(-4.0f, 1.2f, 6.5f);
  game.HoverGround(crateTop);
  CHECK(game.MovePreviewValid());

  game.ClickGround(crateTop);
  CHECK(game.Mode() == InputMode::AwaitingSelection);  // Planned only.

  game.ClickUnit(1);
  game.ChoosePass();
  game.ClickUnit(2);
  game.ChoosePass();
  game.CommitTurn();
  CHECK(game.Mode() == InputMode::Moving);

  game.Update(100.0f);  // Fast-forward through the climb animation.
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  const Unit* moved = game.FindUnit(0);
  CHECK(std::fabs(moved->position.x - crateTop.x) < 1e-3f);
  CHECK(std::fabs(moved->position.z - crateTop.z) < 1e-3f);
  CHECK(std::fabs(moved->position.y - crateTop.y) < 1e-3f);
  CHECK(game.CurrentTeam() == Team::Red);  // Turn advanced.
}

void TestGameLogicOverwatchFiresOnEnemyEnteringFov() {
  GameLogic game(LegacyScene());

  // Reposition red4 due west of blue1 -- squarely behind blue1's fixed +X
  // facing, so it starts outside blue1's FOV cone regardless of LOS -- then
  // send it walking east along the open z=0 lane, straight through blue1's
  // position and into its watched cone.
  game.FindUnit(4)->position = glm::vec3(-9.5f, 0.0f, 0.0f);

  // Blue plans: blue0 and blue2 pass, blue1 arms overwatch instead of a
  // Move/Shoot/Pass -- nothing fires yet, since a plan is just recorded
  // until the whole team's turn is committed.
  CHECK(game.CurrentTeam() == Team::Blue);
  game.ClickUnit(0);
  game.ChoosePass();
  game.ClickUnit(1);
  game.ChooseOverwatch();
  CHECK(game.FindUnit(1)->plan.type == tactics::PlannedActionType::Overwatch);
  CHECK(game.FindUnit(1)->triggerAction == TriggerAction::None);  // Not armed until commit.
  game.ClickUnit(2);
  game.ChoosePass();
  CHECK(game.CanCommitTurn());
  game.CommitTurn();

  // Committing arms blue1's trigger and hands the turn to Red.
  CHECK(game.FindUnit(1)->triggerAction == TriggerAction::Shoot);
  CHECK(game.CurrentTeam() == Team::Red);

  // Red plans: red3 and red5 pass, red4 moves east through blue1's watched
  // lane toward the far side. Committing kicks off red4's move.
  game.ClickUnit(3);
  game.ChoosePass();
  game.ClickUnit(4);
  game.ChooseMove();
  const glm::vec3 destination(0.0f, 0.0f, 0.0f);
  game.ClickGround(destination);
  game.ClickUnit(5);
  game.ChoosePass();
  CHECK(game.CanCommitTurn());
  game.CommitTurn();
  CHECK(game.Mode() == InputMode::Moving);

  // Step frame-by-frame (rather than one huge fast-forward dt) so the
  // overwatch check actually samples red4's position incrementally along
  // the path -- a single giant dt would jump it straight from start to
  // destination in one position update, skipping the mid-path FOV entry
  // this test exists to catch. blue1 should spot red4 and fire the moment
  // it crosses into FOV with clear LOS, interrupting the move well short of
  // the destination.
  int steps = 0;
  while (game.Mode() == InputMode::Moving && steps < 10000) {
    game.Update(0.02f);
    ++steps;
  }
  CHECK(steps < 10000);  // Sanity: the loop above actually terminated.

  CHECK(!game.FindUnit(4)->alive);
  CHECK(game.FindUnit(1)->triggerAction == TriggerAction::None);  // One-shot: trigger consumed.
  const glm::vec3 moverStop = game.FindUnit(4)->position;
  CHECK(glm::distance(moverStop, destination) > 1.0f);  // Died mid-path, short of the destination.
  CHECK(moverStop.x > -9.5f + 1e-3f);                    // But had actually started moving.

  // The interrupted move still finishes Red's commit and passes the turn.
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  CHECK(game.CurrentTeam() == Team::Blue);
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

  CHECK(game.CurrentTeam() == Team::Blue);
  game.ClickUnit(0);
  game.ChoosePass();
  game.ClickUnit(1);
  game.ChoosePass();
  game.ClickUnit(2);
  game.ChoosePass();
  CHECK(game.CanCommitTurn());
  game.CommitTurn();

  CHECK(game.Mode() == InputMode::GameOver);
  CHECK(game.Winner() == Team::Blue);

  // Further input after game over must be inert.
  game.ClickUnit(1);
  CHECK(game.Mode() == InputMode::GameOver);
}

}  // namespace

void TestSnapshotRoundTripMirrorsMatch() {
  GameLogic a;
  a.ClickUnit(0);
  a.ChooseMove();
  a.ClickGround(glm::vec3(0.0f, 0.0f, 0.0f));
  a.ClickUnit(1);
  a.ChoosePass();
  a.ClickUnit(2);
  a.ChoosePass();
  a.CommitTurn();
  a.Update(0.5f);  // Mid-move: positions/facing have changed.

  GameSnapshot decoded;
  CHECK(DeserializeSnapshot(SerializeSnapshot(a.ExportState()), &decoded));

  GameLogic b;
  CHECK(b.ImportState(decoded));
  CHECK(b.Mode() == a.Mode());
  CHECK(b.CurrentTeam() == a.CurrentTeam());
  CHECK(b.SelectedUnitId() == a.SelectedUnitId());
  CHECK(b.RoundNumber() == a.RoundNumber());
  for (const auto& unit : a.GetScene().units) {
    const Unit* mirrored = b.FindUnit(unit.id);
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
  TestNavMeshClimbsOntoClimbableObstacle();
  TestNavMeshObstacleOverloadStillRoutesAroundNonClimbable();
  TestRaycastLineOfSight();
  TestElevatedEyePositionSeesOverObstacle();
  TestFovCone();
  TestTeamVisibilityAggregatesAcrossFigures();
  TestTurnManagerAlternatesByTeamAndRounds();
  TestCheckWinner();
  TestGameLogicSelectionGating();
  TestGameLogicPlanThenCommitDefersExecutionAndAppliesSimultaneously();
  TestGameLogicCommitAnimatesMultipleMovesConcurrently();
  TestGameLogicShootRowsMatchLayout();
  TestDefaultSceneSquadsStartHidden();
  TestGameLogicShootGatingRequiresTeamVisibility();
  TestGameLogicMoveUpdatesPositionAndFacing();
  TestGameLogicMoveAnimatesProgressively();
  TestGameLogicMoveIgnoresInputWhileAnimating();
  TestGameLogicMoveCanClimbOntoObstacle();
  TestGameLogicOverwatchFiresOnEnemyEnteringFov();
  TestGameLogicWinCondition();
  TestSnapshotRoundTripMirrorsMatch();

  if (g_failures == 0) {
    std::printf("All logic tests passed.\n");
    return 0;
  }
  std::fprintf(stderr, "%d check(s) failed.\n", g_failures);
  return 1;
}
