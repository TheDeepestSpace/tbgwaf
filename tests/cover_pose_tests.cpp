// Cover poses are real joint poses (issue #290): the crouch must lower the
// head with the feet planted, and both poses must lean the head toward the wall.
#include <cmath>
#include <cstdio>

#include "game/Unit.h"
#include "gfx/FigureRig.h"

using namespace tactics;

static int gFailures = 0;
#define CHECK(expr) \
  do { \
    if (!(expr)) { \
      std::fprintf(stderr, "CHECK FAILED at %s:%d: %s\n", __FILE__, __LINE__, #expr); \
      ++gFailures; \
    } \
  } while (0)

namespace {
// Highest part center = the head.
glm::vec3 Head(const Unit& u) {
  glm::vec3 best(0.0f, -1e9f, 0.0f);
  for (const gfx::FigurePart& p : gfx::BuildFigure(u)) {
    const glm::vec3 c(p.model[3]);
    if (p.primitive == gfx::FigurePrimitive::Rounded && c.y > best.y) best = c;
  }
  return best;
}
}  // namespace

int main() {
  Unit u;
  u.alive = true;
  u.position = glm::vec3(0.0f);
  u.facingYaw = 0.0f;  // Facing +X; the wall is toward +Z (the figure's right).
  u.coverYaw = glm::half_pi<float>();
  const glm::vec3 stand = Head(u);

  u.coverKind = 2;  // Tactical: crouch.
  u.coverBlend = 1.0f;
  const glm::vec3 crouch = Head(u);
  CHECK(crouch.y < stand.y - 0.15f);
  CHECK(crouch.z > stand.z + 0.05f);

  u.coverKind = 1;  // Flat: lean against the wall.
  const glm::vec3 lean = Head(u);
  CHECK(lean.z > stand.z + 0.1f);
  CHECK(lean.y > stand.y - 0.1f);  // Still upright-ish, not crouched.

  // The cover poses are looping retargeted clips, not static poses: the
  // figure keeps breathing/shifting while holding cover.
  u.coverKind = 2;
  const glm::vec3 crouchA = Head(u);
  u.idleElapsed = 0.4f * gfx::kCoverCrouchLoopSeconds;
  const glm::vec3 crouchB = Head(u);
  CHECK(glm::length(crouchB - crouchA) > 1e-3f);
  u.idleElapsed = 0.0f;

  // Attack position (coverKind 3, issue #369): leans forward over the edge
  // (toward the obstacle at +Z here) staying upright, and the eye and muzzle
  // reach out with it; blended out, they are back at the standing spot.
  u.coverKind = 3;
  u.coverBlend = 1.0f;
  u.facingYaw = glm::half_pi<float>();
  u.coverYaw = glm::half_pi<float>();
  {
    Unit s = u;
    s.coverKind = 0;
    s.coverBlend = 0.0f;
    const glm::vec3 headStand = Head(s), headLean = Head(u);
    CHECK(headLean.z > headStand.z + 0.1f);
    CHECK(headLean.y > headStand.y - 0.15f);
    CHECK(u.EyePosition().z > s.EyePosition().z + 0.2f);
    CHECK(u.SightPosition().z > s.SightPosition().z + 0.2f);
    CHECK(u.MuzzlePosition().z > s.MuzzlePosition().z + 0.2f);
    u.coverBlend = 0.0f;
    CHECK(glm::length(u.EyePosition() - s.EyePosition()) < 1e-5f);
  }
  u.facingYaw = 0.0f;
  u.coverYaw = glm::half_pi<float>();

  // The attack hold places the elbows on the sill with the weapon level
  // along the facing (issue #370): the wireframe weapon box (the only Box
  // part) must point forward and horizontal from a grip held out at
  // elbow-rest height -- not the pitched low carry of a figure merely bent
  // over the edge.
  {
    Unit a;
    a.alive = true;
    a.position = glm::vec3(0.0f);
    a.facingYaw = 0.0f;  // Facing +X, sill toward +X.
    a.coverYaw = 0.0f;
    a.coverKind = 3;
    a.coverBlend = 1.0f;
    a.weapon = WeaponType::AssaultRifle;
    const auto WeaponBox = [](const Unit& unit) {
      for (const gfx::FigurePart& p : gfx::BuildFigureWireframe(unit)) {
        if (p.primitive == gfx::FigurePrimitive::Box) return p.model;
      }
      return glm::mat4(0.0f);
    };
    const glm::mat4 held = WeaponBox(a);
    const glm::vec3 barrel = glm::normalize(glm::vec3(held[0]));
    CHECK(barrel.x > 0.9f);              // Along the facing...
    CHECK(std::fabs(barrel.y) < 0.15f);  // ...and level over the edge.
    const glm::vec3 center(held * glm::vec4(0.5f, 0.5f, 0.5f, 1.0f));
    CHECK(center.x > 0.3f);                       // Held out over the sill...
    CHECK(center.y > 0.75f && center.y < 1.15f);  // ...at elbow-rest height.
    Unit s = a;
    s.coverKind = 0;
    s.coverBlend = 0.0f;
    const glm::vec3 carry = glm::normalize(glm::vec3(WeaponBox(s)[0]));
    CHECK(std::fabs(carry.y) > 0.4f);  // Standing low carry stays pitched.
  }

  u.coverKind = 2;
  u.coverBlend = 0.0f;
  CHECK(std::fabs(Head(u).z - stand.z) < 1e-4f);

  if (gFailures == 0) std::printf("cover pose tests passed\n");
  return gFailures == 0 ? 0 : 1;
}
