#include "game/Scene.h"

#include <cmath>

namespace tactics {
namespace {

constexpr float kPi = 3.14159265358979323846f;

AABB MakeBoxXZ(float centerX, float centerZ, float halfWidthX, float halfWidthZ, float height) {
  return AABB{glm::vec3(centerX - halfWidthX, 0.0f, centerZ - halfWidthZ),
              glm::vec3(centerX + halfWidthX, height, centerZ + halfWidthZ)};
}

}  // namespace

Scene BuildDefaultScene() {
  Scene scene;

  // Two wall segments split the field into three lanes: the top and bottom
  // lanes are blocked (forcing pathfinding to route around, and blocking
  // straight-line shots), the middle lane is open (a clean shot is possible).
  scene.obstacles.push_back(MakeBoxXZ(0.0f, -4.0f, 1.0f, 2.0f, 2.0f));
  scene.obstacles.push_back(MakeBoxXZ(0.0f, 4.0f, 1.0f, 2.0f, 2.0f));

  // A handful of standalone crates for visual variety and extra path variety.
  scene.obstacles.push_back(MakeBoxXZ(-4.0f, 6.5f, 0.6f, 0.6f, 1.2f));
  scene.obstacles.push_back(MakeBoxXZ(4.0f, -6.5f, 0.6f, 0.6f, 1.2f));
  scene.obstacles.push_back(MakeBoxXZ(-3.5f, -7.5f, 0.6f, 0.6f, 1.2f));
  scene.obstacles.push_back(MakeBoxXZ(3.5f, 7.5f, 0.6f, 0.6f, 1.2f));

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
