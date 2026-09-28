#include "game/Scene.h"

#include <cmath>

namespace tactics {
namespace {

constexpr float kPi = 3.14159265358979323846f;

Obstacle MakeObstacle(float centerX, float centerZ, float halfWidthX, float halfWidthZ,
                       float height, bool climbable) {
  return Obstacle{AABB{glm::vec3(centerX - halfWidthX, 0.0f, centerZ - halfWidthZ),
                        glm::vec3(centerX + halfWidthX, height, centerZ + halfWidthZ)},
                   climbable};
}

}  // namespace

Scene BuildDefaultScene() {
  Scene scene;

  // Two wall segments split the field into three lanes: the top and bottom
  // lanes are blocked (forcing pathfinding to route around, and blocking
  // straight-line shots), the middle lane is open (a clean shot is possible).
  // They're taller than a figure can climb, so the navmesh treats them as
  // fully impassable rather than climbable.
  scene.obstacles.push_back(MakeObstacle(0.0f, -4.0f, 1.0f, 2.0f, 2.0f, /*climbable=*/false));
  scene.obstacles.push_back(MakeObstacle(0.0f, 4.0f, 1.0f, 2.0f, 2.0f, /*climbable=*/false));

  // A handful of standalone crates for visual variety and extra path
  // variety. Low enough to be climbable: the navmesh connects ground level
  // to their tops (Stage C).
  scene.obstacles.push_back(MakeObstacle(-4.0f, 6.5f, 0.6f, 0.6f, 1.2f, /*climbable=*/true));
  scene.obstacles.push_back(MakeObstacle(4.0f, -6.5f, 0.6f, 0.6f, 1.2f, /*climbable=*/true));
  scene.obstacles.push_back(MakeObstacle(-3.5f, -7.5f, 0.6f, 0.6f, 1.2f, /*climbable=*/true));
  scene.obstacles.push_back(MakeObstacle(3.5f, 7.5f, 0.6f, 0.6f, 1.2f, /*climbable=*/true));

  const float spawnX = 8.0f;
  const float rows[3] = {-4.0f, 0.0f, 4.0f};

  for (int i = 0; i < 3; ++i) {
    Unit blue;
    blue.id = i;
    blue.team = Team::Blue;
    blue.position = glm::vec3(-spawnX, 0.0f, rows[i]);
    blue.facingYaw = 0.0f;  // Faces +X, toward the Red side.
    scene.units.push_back(blue);
  }
  for (int i = 0; i < 3; ++i) {
    Unit red;
    red.id = 3 + i;
    red.team = Team::Red;
    red.position = glm::vec3(spawnX, 0.0f, rows[i]);
    red.facingYaw = kPi;  // Faces -X, toward the Blue side.
    scene.units.push_back(red);
  }

  return scene;
}

}  // namespace tactics
