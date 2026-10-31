// Headless tests for the render-side ragdoll (issue #217). glm + game Unit
// only; no GL.

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "game/Types.h"
#include "gfx/FigureRig.h"
#include "game/Geometry.h"
#include "gfx/Ragdoll.h"

namespace {

int g_failures = 0;

#define CHECK(expr) \
  do { \
    if (!(expr)) { \
      std::fprintf(stderr, "CHECK FAILED at %s:%d: %s\n", __FILE__, __LINE__, #expr); \
      ++g_failures; \
    } \
  } while (0)

// A unit just killed by a shot travelling +X (same encoding as KnockDown()).
tactics::Unit Downed(int id, float elapsed) {
  tactics::Unit u;
  u.id = id;
  u.position = glm::vec3(3.0f, 0.5f, -2.0f);
  u.alive = false;
  u.knockdownAxis = glm::vec3(0.0f, 0.0f, -1.0f);  // dir = +X
  u.knockdownElapsed = elapsed;
  return u;
}

glm::vec3 PoseCentre(const gfx::RagdollPose& pose) {
  glm::vec3 centre(0.0f);
  for (const glm::vec3& p : pose) centre += p;
  return centre / static_cast<float>(gfx::kRagJointCount);
}

void TestOnlyDownedUnitsRagdoll() {
  tactics::Unit u;
  CHECK(!gfx::RagdollActive(u));
  CHECK(gfx::RagdollActive(Downed(1, 0.0f)));
  u = Downed(1, -1.0f);
  CHECK(!gfx::RagdollActive(u));
}

void TestFallsAlongShotAndRests() {
  const tactics::Unit u = Downed(1, tactics::constants::kKnockdownMaxSeconds);
  const gfx::RagdollPose start = gfx::SampleRagdollStep(u, 0);
  const gfx::RagdollPose end = gfx::SampleRagdoll(u);
  CHECK(start[gfx::kRagHead].y > u.position.y + 1.2f);
  CHECK(end[gfx::kRagHead].y < u.position.y + 0.6f);  // Head came down.
  CHECK(end[gfx::kRagHead].x > start[gfx::kRagHead].x + 0.3f);  // Toward the shot.
  for (const glm::vec3& p : end) CHECK(p.y >= u.position.y - 1e-4f);  // Never below ground.
  // At rest: the last frames are identical (asleep or settled).
  const gfx::RagdollPose prev = gfx::SampleRagdollStep(u, gfx::kRagdollSteps - 1);
  for (int i = 0; i < gfx::kRagJointCount; ++i) CHECK(glm::length(end[i] - prev[i]) < 0.02f);
}

void TestLimbLengthsPreserved() {
  const tactics::Unit u = Downed(2, tactics::constants::kKnockdownMaxSeconds);
  const tactics::Unit stand = [] {
    tactics::Unit s = Downed(2, 0.0f);
    return s;
  }();
  const gfx::RagdollPose a = gfx::SampleRagdollStep(stand, 0);
  const gfx::RagdollPose b = gfx::SampleRagdoll(u);
  auto len = [](const gfx::RagdollPose& p, int i, int j) { return glm::length(p[i] - p[j]); };
  CHECK(std::fabs(len(a, gfx::kRagHipR, gfx::kRagKneeR) - len(b, gfx::kRagHipR, gfx::kRagKneeR)) <
        0.05f);
  CHECK(std::fabs(len(a, gfx::kRagShoulderL, gfx::kRagElbowL) -
                  len(b, gfx::kRagShoulderL, gfx::kRagElbowL)) < 0.05f);
}

void TestDeterministicAndCacheBounded() {
  const tactics::Unit u = Downed(7, 0.2f);
  const gfx::RagdollPose a = gfx::SampleRagdoll(u);
  // Thrash the cache well past its cap; evicted ragdolls must resimulate identically.
  for (int id = 100; id < 100 + 3 * gfx::kRagdollMaxCached; ++id) {
    gfx::SampleRagdoll(Downed(id, 0.4f));
  }
  const gfx::RagdollPose b = gfx::SampleRagdoll(u);
  for (int i = 0; i < gfx::kRagJointCount; ++i) CHECK(a[i] == b[i]);
  // Different units fall differently.
  const gfx::RagdollPose c = gfx::SampleRagdoll(Downed(8, 0.2f));
  CHECK(c[gfx::kRagHead] != a[gfx::kRagHead]);
}

void TestDisplacementStaysOnShotAxisAndReplaysExactly() {
  gfx::SetRagdollObstacles({});
  const glm::vec3 shotDir = glm::normalize(glm::vec3(0.6f, 0.0f, -0.8f));
  const glm::vec3 side(-shotDir.z, 0.0f, shotDir.x);

  // Exercise every normal figure id. The varied launch may change how far a
  // body travels, but without a replicated lateral force its centre must stay
  // on the shot axis. Varying the hit position represents moving figures
  // being shot at different points along their paths.
  for (int id = 0; id < 40; ++id) {
    tactics::Unit u = Downed(id, tactics::constants::kKnockdownMaxSeconds);
    u.position += glm::vec3(0.17f * id, 0.0f, -0.11f * id);
    u.facingYaw = 0.37f;
    u.knockdownAxis = glm::vec3(shotDir.z, 0.0f, -shotDir.x);
    const glm::vec3 start = PoseCentre(gfx::SampleRagdollStep(u, 0));
    float maxSideways = 0.0f;
    for (int step = 1; step <= gfx::kRagdollSteps; ++step) {
      const glm::vec3 displacement = PoseCentre(gfx::SampleRagdollStep(u, step)) - start;
      maxSideways = std::max(maxSideways, std::fabs(glm::dot(displacement, side)));
    }
    CHECK(maxSideways < 0.15f);
    const glm::vec3 displacement = PoseCentre(gfx::SampleRagdoll(u)) - start;
    CHECK(glm::dot(displacement, shotDir) > 0.2f);
  }

  // Replay may sample the fall out of order after the live trajectory has
  // been evicted. It must reconstruct the identical physical frames.
  tactics::Unit u = Downed(7, 0.0f);
  u.position = glm::vec3(-1.3f, 0.0f, 4.7f);
  u.knockdownAxis = glm::vec3(shotDir.z, 0.0f, -shotDir.x);
  const int sampleSteps[] = {0, 7, 31, 79, 143, gfx::kRagdollSteps};
  gfx::RagdollPose live[6];
  for (int i = 0; i < 6; ++i) live[i] = gfx::SampleRagdollStep(u, sampleSteps[i]);
  for (int id = 100; id < 100 + 2 * gfx::kRagdollMaxCached; ++id) {
    gfx::SampleRagdoll(Downed(id, 0.3f));
  }
  for (int i = 5; i >= 0; --i) {
    const gfx::RagdollPose replay = gfx::SampleRagdollStep(u, sampleSteps[i]);
    for (int joint = 0; joint < gfx::kRagJointCount; ++joint) {
      CHECK(replay[joint] == live[i][joint]);
    }
  }
}

void TestFigurePartsFollowRagdoll() {
  const gfx::FigureParts standing = gfx::BuildFigure(tactics::Unit{});
  const tactics::Unit down = Downed(1, tactics::constants::kKnockdownMaxSeconds);
  const gfx::FigureParts parts = gfx::BuildFigure(down);
  CHECK(!parts.empty());
  CHECK(parts.size() >= 12);
  // Every part's centre sits near the ground once at rest.
  for (const gfx::FigurePart& p : parts) CHECK(p.model[3].y < down.position.y + 0.9f);
  const gfx::FigureParts mid = gfx::BuildFigure(Downed(1, 0.1f));
  CHECK(mid.size() == parts.size());
}

void TestGunReleaseAndDrop() {
  int dropped = 0;
  for (int id = 1; id <= 40; ++id) {
    const tactics::Unit u = Downed(id, tactics::constants::kKnockdownMaxSeconds);
    const gfx::RagdollGun start = gfx::SampleRagdollGunStep(u, 0);
    const gfx::RagdollGun end = gfx::SampleRagdollGun(u);
    CHECK(start.dropped == end.dropped);
    if (!end.dropped) continue;
    ++dropped;
    // Falls under gravity: lands on (not through) the ground, away from the hand.
    CHECK(end.pos.y >= u.position.y - 0.25f);
    CHECK(end.pos.y < u.position.y + 0.4f);
    CHECK(glm::length(end.pos - start.pos) > 0.2f);
    // Deterministic and settled.
    const gfx::RagdollGun again = gfx::SampleRagdollGun(u);
    CHECK(again.pos == end.pos);
    const gfx::RagdollGun prev = gfx::SampleRagdollGunStep(u, gfx::kRagdollSteps - 1);
    CHECK(glm::length(prev.pos - end.pos) < 0.05f);
  }
  CHECK(dropped > 5 && dropped < 35);  // Random mix of droppers and holders.
}

void TestFallsOffWallAndStaysOutside() {
  tactics::Obstacle wall;
  wall.bounds.min = glm::vec3(-0.6f, 0.0f, -3.0f);
  wall.bounds.max = glm::vec3(0.6f, 3.0f, 3.0f);
  gfx::SetRagdollObstacles({wall});
  tactics::Unit u = Downed(3, tactics::constants::kKnockdownMaxSeconds);
  u.position = glm::vec3(0.3f, 3.0f, 0.0f);
  u.knockdownAxis = glm::vec3(0.0f, 0.0f, -1.0f);  // Shot travels +X, off the wall.
  const gfx::RagdollPose end = gfx::SampleRagdoll(u);
  for (int s = 0; s <= gfx::kRagdollSteps; ++s) {
    for (const glm::vec3& p : gfx::SampleRagdollStep(u, s)) {
      // Never inside the wall's body.
      CHECK(!(p.x > -0.55f && p.x < 0.55f && p.y < 2.9f));
    }
  }
  CHECK(end[gfx::kRagHead].y < 1.0f);  // Landed on the ground, not the wall top.
  CHECK(end[gfx::kRagHead].x > 0.6f);
  // The whole body (legs included) ends lying on the ground, not frozen
  // mid-fall with the feet in the air.
  for (const glm::vec3& p : end) CHECK(p.y < 0.5f);
  // And it is actually at rest there.
  const gfx::RagdollPose prev = gfx::SampleRagdollStep(u, gfx::kRagdollSteps - 1);
  for (int i = 0; i < gfx::kRagJointCount; ++i) CHECK(glm::length(end[i] - prev[i]) < 0.05f);
  gfx::SetRagdollObstacles({});
}

// Dying flush against a wall must not turn the wall push-out into velocity
// (the body used to be flung away and through the map).
void TestWallPushOutDoesNotLaunchBody() {
  tactics::Obstacle wall;
  wall.bounds.min = glm::vec3(0.4f, 0.0f, -3.0f);
  wall.bounds.max = glm::vec3(3.0f, 3.0f, 3.0f);
  gfx::SetRagdollObstacles({wall});
  for (int id = 1; id <= 20; ++id) {
    tactics::Unit u = Downed(id, tactics::constants::kKnockdownMaxSeconds);
    u.position = glm::vec3(0.1f, 0.0f, 0.0f);  // Body overlaps the wall face.
    gfx::RagdollPose last = gfx::SampleRagdollStep(u, 0);
    for (int s = 1; s <= gfx::kRagdollSteps; ++s) {
      const gfx::RagdollPose cur = gfx::SampleRagdollStep(u, s);
      for (int i = 0; i < gfx::kRagJointCount; ++i) {
        CHECK(glm::length(cur[i] - last[i]) < 0.3f);  // No per-step teleport/launch.
        CHECK(glm::length(cur[i] - u.position) < 6.0f);
      }
      last = cur;
    }
  }
  gfx::SetRagdollObstacles({});
}

// Overlapping low slabs (sidewalk + curb) around a standing figure, with a
// neighbour: neither body may be launched (issue #262).
void TestOverlappingSidewalkDoesNotLaunch() {
  tactics::Obstacle a, b, c;
  a.bounds.min = glm::vec3(2.0f, 0.0f, -4.0f);
  a.bounds.max = glm::vec3(3.6f, 0.15f, 4.0f);
  b.bounds.min = glm::vec3(2.9f, 0.0f, -4.0f);  // Overlaps a.
  b.bounds.max = glm::vec3(3.2f, 0.4f, 4.0f);
  c.bounds.min = glm::vec3(2.5f, 0.0f, -2.5f);
  c.bounds.max = glm::vec3(3.3f, 0.3f, -1.5f);
  gfx::SetRagdollObstacles({a, b, c});
  for (int id = 1; id <= 20; ++id) {
    for (float x : {2.2f, 3.0f, 3.5f}) {
      tactics::Unit u = Downed(id, tactics::constants::kKnockdownMaxSeconds);
      u.position = glm::vec3(x, 0.0f, -2.0f);
      u.knockdownImpacts.push_back({0.4f, glm::vec3(0.0f, 0.0f, 1.0f)});
      gfx::RagdollPose last = gfx::SampleRagdollStep(u, 0);
      for (int s = 1; s <= gfx::kRagdollSteps; ++s) {
        const gfx::RagdollPose cur = gfx::SampleRagdollStep(u, s);
        for (int i = 0; i < gfx::kRagJointCount; ++i) {
          CHECK(glm::length(cur[i] - last[i]) < 0.3f);
          CHECK(glm::length(cur[i] - u.position) < 6.0f);
        }
        last = cur;
      }
    }
  }
  gfx::SetRagdollObstacles({});
}

// A second hit on the falling body shoves it: same fall until the hit, then
// a different one (here a hit from the opposite side).
void TestSecondHitShovesFallingBody() {
  const float kHitAt = 0.3f;
  const tactics::Unit plain = Downed(7, tactics::constants::kKnockdownMaxSeconds);
  tactics::Unit hit = plain;
  hit.knockdownImpacts.push_back({kHitAt, glm::vec3(0.0f, 0.0f, 1.0f)});  // dir = -X
  const int hitStep = static_cast<int>(std::lround(kHitAt / gfx::kRagdollStep));
  const gfx::RagdollPose a = gfx::SampleRagdollStep(plain, hitStep);
  const gfx::RagdollPose b = gfx::SampleRagdollStep(hit, hitStep);
  for (int i = 0; i < gfx::kRagJointCount; ++i) CHECK(glm::length(a[i] - b[i]) < 1e-5f);
  const gfx::RagdollPose endPlain = gfx::SampleRagdoll(plain);
  const gfx::RagdollPose endHit = gfx::SampleRagdoll(hit);
  CHECK(glm::length(endPlain[gfx::kRagHead] - endHit[gfx::kRagHead]) > 0.1f);
  CHECK(endHit[gfx::kRagHead].x < endPlain[gfx::kRagHead].x);  // Pushed back.
  // A hit after the body came to rest wakes it up again.
  tactics::Unit late = plain;
  late.knockdownImpacts.push_back({2.0f, glm::vec3(0.0f, 0.0f, -1.0f)});
  const gfx::RagdollPose before = gfx::SampleRagdollStep(late, 119);
  const gfx::RagdollPose after = gfx::SampleRagdollStep(late, 140);
  CHECK(glm::length(after[gfx::kRagHead] - before[gfx::kRagHead]) > 0.05f);
}

}  // namespace

// The fall begins from the pose held at the hit: a unit caught mid-stride
// starts from different joint positions than one standing still.
void TestStartsFromAnimatedPose() {
  tactics::Unit still = Downed(1, 0.0f);
  tactics::Unit running = still;
  running.walkBlend = 1.0f;
  running.walkPhase = 1.2f;
  const gfx::RagdollPose a = gfx::SampleRagdollStep(still, 0);
  const gfx::RagdollPose b = gfx::SampleRagdollStep(running, 0);
  CHECK(glm::length(a[gfx::kRagFootL] - b[gfx::kRagFootL]) > 0.02f ||
        glm::length(a[gfx::kRagHandL] - b[gfx::kRagHandL]) > 0.02f);
  // Matches the live animated pose exactly (no snap to a neutral stand).
  const gfx::RagdollPose live = gfx::AnimatedPose(running);
  for (int i = 0; i < gfx::kRagJointCount; ++i) CHECK(glm::length(b[i] - live[i]) < 1e-4f);
}

// Shot on a zipline cable: the body drops to the ground below, not rest in mid-air.
void TestRiderFallsToGroundBelowCable() {
  gfx::SetRagdollObstacles({});
  tactics::Unit u = Downed(3, tactics::constants::kKnockdownMaxSeconds);
  u.position.y = 3.0f;
  u.knockdownDrop = 3.0f;
  const gfx::RagdollPose end = gfx::SampleRagdoll(u);
  float lowest = 1e9f;
  for (const glm::vec3& p : end) lowest = std::min(lowest, p.y);
  CHECK(lowest < 0.3f);
}

// A figure killed on a ramp comes to rest lying on it, not hovering at its
// starting height above the lower ground.
void TestRestsOnRamp() {
  tactics::WalkSurface ramp;  // Falls from y=1.6 at x=-2 to y=0 at x=14.
  ramp.vertices = {{-2.0f, 1.6f, 4.0f}, {14.0f, 0.0f, 4.0f}, {14.0f, 0.0f, -4.0f},
                   {-2.0f, 1.6f, -4.0f}};
  gfx::SetRagdollObstacles({});
  gfx::SetRagdollGround({ramp}, tactics::HeightField{});
  for (int id = 1; id <= 10; ++id) {
    tactics::Unit u = Downed(id, tactics::constants::kKnockdownMaxSeconds);
    u.position = glm::vec3(0.0f, 1.6f * 14.0f / 16.0f, 0.0f);
    const gfx::RagdollPose end = gfx::SampleRagdoll(u);
    for (int s = 0; s <= gfx::kRagdollSteps; ++s) {
      for (const glm::vec3& p : gfx::SampleRagdollStep(u, s)) {
        if (p.x > -2.0f && p.x < 14.0f) {
          CHECK(p.y > tactics::SurfaceHeightAt(ramp, p.x, p.z) - 0.35f);  // Never under it.
        }
      }
    }
    for (const glm::vec3& p : end) {
      const float floorY = p.x < 14.0f ? tactics::SurfaceHeightAt(ramp, p.x, p.z) : 0.0f;
      CHECK(p.y - floorY < 0.5f);  // Lying on the ramp, not levitating.
    }
  }
  gfx::SetRagdollGround({}, tactics::HeightField{});
}

int main() {
  TestRiderFallsToGroundBelowCable();
  TestStartsFromAnimatedPose();
  TestWallPushOutDoesNotLaunchBody();
  TestOverlappingSidewalkDoesNotLaunch();
  TestSecondHitShovesFallingBody();
  TestFallsOffWallAndStaysOutside();
  TestRestsOnRamp();
  TestGunReleaseAndDrop();
  TestOnlyDownedUnitsRagdoll();
  TestFallsAlongShotAndRests();
  TestLimbLengthsPreserved();
  TestDeterministicAndCacheBounded();
  TestDisplacementStaysOnShotAxisAndReplaysExactly();
  TestFigurePartsFollowRagdoll();
  if (g_failures == 0) std::puts("ragdoll_tests: OK");
  return g_failures == 0 ? 0 : 1;
}
