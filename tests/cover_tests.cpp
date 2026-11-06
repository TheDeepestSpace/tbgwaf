#include <cstdio>
#include <vector>

#include "game/Cover.h"

using namespace tactics;

static int gFailures = 0;
#define CHECK(expr) \
  do { \
    if (!(expr)) { \
      std::fprintf(stderr, "CHECK FAILED at %s:%d: %s\n", __FILE__, __LINE__, #expr); \
      ++gFailures; \
    } \
  } while (0)

static Obstacle Box(float x0, float z0, float x1, float z1, float h = 3.0f) {
  Obstacle o;
  o.bounds.min = glm::vec3(x0, 0.0f, z0);
  o.bounds.max = glm::vec3(x1, h, z1);
  return o;
}

int main() {
  // A long wall along z in [0,1], x in [-20,20].
  const std::vector<Obstacle> wall = {Box(-20, 0, 20, 1)};
  CHECK(ClassifyCover(wall, {0, 0, -3}).kind == CoverKind::None);
  const CoverTile flat = ClassifyCover(wall, {0, 0, -0.4f});
  CHECK(flat.kind == CoverKind::Flat);
  CHECK(flat.dirMask & (1u << 2));  // +Z side.
  float yaw = 0;
  ClassifyCover(wall, {0, 0, -0.4f}, &yaw);
  CHECK(yaw > 1.5f && yaw < 1.65f);

  // Near the wall's end: tactical; deep in the wall: flat.
  const std::vector<Obstacle> block = {Box(0, 0, 1.2f, 1)};
  CHECK(ClassifyCover(block, {0.6f, 0, -0.4f}).kind == CoverKind::Tactical);  // Short car-sized block.
  const std::vector<Obstacle> longer = {Box(0, 0, 12, 1)};
  CHECK(ClassifyCover(longer, {6, 0, -0.4f}).kind == CoverKind::Flat);
  CHECK(ClassifyCover(longer, {0.3f, 0, -0.4f}).kind == CoverKind::Tactical);

  // Low blockers (below cover height) and overhead ones don't count.
  CHECK(ClassifyCover({Box(-20, 0, 20, 1, 0.5f)}, {0, 0, -0.4f}).kind == CoverKind::None);
  Obstacle crown = Box(-20, 0, 20, 1);
  crown.bounds.min.y = 2.0f;
  CHECK(ClassifyCover({crown}, {0, 0, -0.4f}).kind == CoverKind::None);

  // Height decides the pose: tall = stand with back to it, low = crouch,
  // and below the 70%-of-crouched-height line it is not cover at all.
  CHECK(!ClassifyCover({Box(-20, 0, 20, 1, 3.0f)}, {0, 0, -0.4f}).crouch);
  CHECK(!ClassifyCover({Box(-20, 0, 20, 1, kCoverStandHeight)}, {0, 0, -0.4f}).crouch);
  CHECK(ClassifyCover({Box(-20, 0, 20, 1, 1.0f)}, {0, 0, -0.4f}).crouch);
  CHECK(ClassifyCover({Box(-20, 0, 20, 1, kCoverMinHeight + 0.01f)}, {0, 0, -0.4f}).kind ==
        CoverKind::Flat);
  CHECK(ClassifyCover({Box(-20, 0, 20, 1, kCoverMinHeight - 0.01f)}, {0, 0, -0.4f}).kind ==
        CoverKind::None);
  // Crouched, no more than 30% of the figure shows above the cover.
  CHECK((kCrouchedHeight - kCoverMinHeight) / kCrouchedHeight <= 0.3f + 1e-4f);

  // Deterministic and map consistent with point classification.
  ReachField f;
  f.minX = -1; f.minZ = -2; f.step = 0.25f; f.nx = 9; f.nz = 9; f.budget = 5;
  f.dist.assign(81, 1.0f);
  f.surfaceY.assign(81, 0.0f);
  const CoverMap a = ComputeCoverMap(longer, f), b = ComputeCoverMap(longer, f);
  bool same = true, any = false;
  for (int i = 0; i < 81; ++i) {
    same &= a.tiles[i].kind == b.tiles[i].kind && a.tiles[i].dirMask == b.tiles[i].dirMask;
    any |= a.tiles[i].kind != CoverKind::None;
  }
  CHECK(same);
  CHECK(any);
  CHECK(a.At(0, 8).kind == ClassifyCover(longer, f.Node(0, 8)).kind);

  // --- Scenarios showcasing various covers ---
  const float kPi = 3.14159265f;

  // Open ground: no cover anywhere near.
  CHECK(ClassifyCover({}, {0, 0, 0}).kind == CoverKind::None);
  CHECK(ClassifyCover(wall, {0, 0, 5}).kind == CoverKind::None);

  // Too far from the wall face (beyond probe distance) is not cover.
  CHECK(ClassifyCover(wall, {0, 0, -1.0f}).kind == CoverKind::None);

  // Wall on the other side faces the opposite way (-Z blocker => yaw ~ -pi/2).
  {
    float y = 0;
    CHECK(ClassifyCover(wall, {0, 0, 1.4f}, &y).kind == CoverKind::Flat);
    CHECK(y < -1.5f && y > -1.65f);
  }

  // Wall facing +X / -X use the matching compass bits and yaw.
  {
    const std::vector<Obstacle> tall = {Box(0, -20, 1, 20)};
    float y = 9;
    const CoverTile t = ClassifyCover(tall, {-0.4f, 0, 0}, &y);
    CHECK(t.kind == CoverKind::Flat);
    CHECK(t.dirMask & (1u << 0));
    CHECK(y > -0.1f && y < 0.1f);
    CHECK(ClassifyCover(tall, {1.4f, 0, 0}, &y).kind == CoverKind::Flat);
    CHECK(y > kPi - 0.1f || y < -kPi + 0.1f);
  }

  // Building corner: standing at the outside corner is tactical.
  {
    const std::vector<Obstacle> building = {Box(0, 0, 10, 10)};
    CHECK(ClassifyCover(building, {-0.4f, 0, 5}).kind == CoverKind::Flat);   // Mid-face.
    CHECK(ClassifyCover(building, {-0.4f, 0, 0.3f}).kind == CoverKind::Tactical);  // Near corner.
    CHECK(ClassifyCover(building, {-0.4f, 0, -0.4f}).kind == CoverKind::Tactical);  // Diagonal touch.
    CHECK(ClassifyCover(building, {-3, 0, -3}).kind == CoverKind::None);
  }

  // Alley between two parallel walls: cover on both sides at once.
  {
    const std::vector<Obstacle> alley = {Box(-20, 0, 20, 1), Box(-20, 2.0f, 20, 3.0f)};
    const CoverTile t = ClassifyCover(alley, {0, 0, 1.5f});
    CHECK(t.kind == CoverKind::Flat);
    CHECK((t.dirMask & (1u << 2)) && (t.dirMask & (1u << 6)));
  }

  // Row of cars with a gap.
  {
    const std::vector<Obstacle> cars = {Box(0, 0, 4, 1.5f, 1.4f), Box(6, 0, 10, 1.5f, 1.4f)};
    CHECK(ClassifyCover(cars, {4.3f, 0, -0.4f}).kind == CoverKind::Tactical);  // Beside the gap.
    CHECK(ClassifyCover(cars, {4.4f, 0, 0.7f}).kind != CoverKind::None);      // Inside the gap, beside a car end.
    CHECK(ClassifyCover(cars, {2.0f, 0, -0.4f}).kind != CoverKind::None);
  }

  // Waist-high barrier is not cover; a tall one next to it is.
  {
    const std::vector<Obstacle> mixed = {Box(-5, 0, 5, 1, 0.6f), Box(-5, 4, 5, 5, 3.0f)};
    CHECK(ClassifyCover(mixed, {0, 0, -0.4f}).kind == CoverKind::None);
    CHECK(ClassifyCover(mixed, {0, 0, 3.6f}).kind == CoverKind::Flat);
  }

  // Unreached nodes in a ReachField stay None even beside a wall.
  {
    ReachField g;
    g.minX = -1; g.minZ = -2; g.step = 0.25f; g.nx = 9; g.nz = 9; g.budget = 5;
    g.dist.assign(81, 1e9f);
    g.surfaceY.assign(81, 0.0f);
    const CoverMap m = ComputeCoverMap(longer, g);
    bool allNone = true;
    for (const CoverTile& t : m.tiles) allNone &= t.kind == CoverKind::None;
    CHECK(allNone);
  }

  if (gFailures) return 1;
  std::puts("cover_tests passed");
  return 0;
}
