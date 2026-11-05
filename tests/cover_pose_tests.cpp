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

  u.coverBlend = 0.0f;
  CHECK(std::fabs(Head(u).z - stand.z) < 1e-4f);

  if (gFailures == 0) std::printf("cover pose tests passed\n");
  return gFailures == 0 ? 0 : 1;
}
