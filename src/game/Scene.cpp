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

  // 30x30 field. Non-climbable structures (taller than a figure can climb)
  // are fully impassable to the navmesh and block shots; the squads start at
  // opposite ends with no clear sightline between them.
  constexpr float kWallHeight = 2.5f;
  constexpr float kFenceHeight = 1.7f;
  auto wall = [&](float cx, float cz, float hx, float hz) {
    scene.obstacles.push_back(MakeObstacle(cx, cz, hx, hz, kWallHeight, /*climbable=*/false));
  };
  auto fence = [&](float cx, float cz, float hx, float hz) {
    scene.obstacles.push_back(MakeObstacle(cx, cz, hx, hz, kFenceHeight, /*climbable=*/false));
  };

  // Central building: a walled room with a door gap (x in [-1, 1]) in its
  // south wall, blocking the middle lane.
  wall(0.0f, -3.5f, 3.75f, 0.25f);   // North wall.
  wall(-3.5f, 0.0f, 0.25f, 3.25f);   // West wall.
  wall(3.5f, 0.0f, 0.25f, 3.25f);    // East wall.
  wall(-2.25f, 3.5f, 1.5f, 0.25f);   // South wall, west of the door.
  wall(2.25f, 3.5f, 1.5f, 0.25f);    // South wall, east of the door.

  // Two small solid buildings in the outer corners.
  wall(-8.0f, -11.5f, 2.0f, 1.5f);
  wall(8.0f, 11.5f, 2.0f, 1.5f);

  // Fences screening the outer lanes near each squad, plus a pair guarding
  // the approach to the central building.
  fence(-7.0f, -6.0f, 0.15f, 2.5f);
  fence(7.0f, 6.0f, 0.15f, 2.5f);
  fence(-7.0f, 6.0f, 2.4f, 0.15f);
  fence(7.0f, -6.0f, 2.4f, 0.15f);

  // A handful of standalone crates for extra path variety. Low enough to be
  // climbable: the navmesh connects ground level to their tops (Stage C).
  scene.obstacles.push_back(MakeObstacle(-4.0f, 6.5f, 0.6f, 0.6f, 1.2f, /*climbable=*/true));
  scene.obstacles.push_back(MakeObstacle(4.0f, -6.5f, 0.6f, 0.6f, 1.2f, /*climbable=*/true));
  scene.obstacles.push_back(MakeObstacle(-3.5f, -7.5f, 0.6f, 0.6f, 1.2f, /*climbable=*/true));
  scene.obstacles.push_back(MakeObstacle(3.5f, 7.5f, 0.6f, 0.6f, 1.2f, /*climbable=*/true));
  scene.obstacles.push_back(MakeObstacle(0.0f, 0.0f, 0.6f, 0.6f, 1.2f, /*climbable=*/true));

  const float spawnX = 12.0f;
  const float rows[3] = {-6.0f, 0.0f, 6.0f};

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
