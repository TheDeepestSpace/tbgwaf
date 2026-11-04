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

  if (gFailures) return 1;
  std::puts("cover_tests passed");
  return 0;
}
