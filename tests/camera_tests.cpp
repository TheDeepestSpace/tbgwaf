// Headless tests for OrbitCamera::FocusOn (double-click focus). glm only; no
// SDL/GL dependency.

#include <cmath>
#include <cstdio>

#include "gfx/Camera.h"

namespace {

int g_failures = 0;

#define CHECK(expr) \
  do { \
    if (!(expr)) { \
      std::fprintf(stderr, "CHECK FAILED at %s:%d: %s\n", __FILE__, __LINE__, #expr); \
      ++g_failures; \
    } \
  } while (0)

void Settle(gfx::OrbitCamera& cam) {
  for (int i = 0; i < 600; ++i) cam.Update(1.0f / 60.0f);
}

void TestFocusOnEasesToPointAndZoom() {
  gfx::OrbitCamera cam;
  cam.FocusOn(glm::vec3(5.0f, 3.0f, -4.0f), 4.0f);
  // Eased, not snapped.
  cam.Update(1.0f / 60.0f);
  CHECK(std::fabs(cam.target.x - 5.0f) > 0.01f);
  Settle(cam);
  CHECK(std::fabs(cam.target.x - 5.0f) < 0.02f);
  CHECK(std::fabs(cam.target.y) < 1e-6f);  // Projected to the ground.
  CHECK(std::fabs(cam.target.z + 4.0f) < 0.02f);
  CHECK(std::fabs(cam.TargetDistance() - 9.6f) < 1e-4f);
}

void TestPanCancelsFocusGlide() {
  gfx::OrbitCamera cam;
  cam.FocusOn(glm::vec3(10.0f, 0.0f, 10.0f), 4.0f);
  cam.Update(1.0f / 60.0f);
  cam.Pan(0.0f, 0.0f);
  const glm::vec3 held = cam.target;
  Settle(cam);
  CHECK(glm::length(cam.target - held) < 1e-6f);
}

void TestRotateCancelsFocusGlide() {
  gfx::OrbitCamera cam;
  cam.FocusOn(glm::vec3(10.0f, 0.0f, 10.0f), 4.0f);
  cam.Update(1.0f / 60.0f);
  cam.Rotate(0.0f, 0.0f);
  const glm::vec3 held = cam.target;
  Settle(cam);
  CHECK(glm::length(cam.target - held) < 1e-6f);
}

}  // namespace

int main() {
  TestFocusOnEasesToPointAndZoom();
  TestPanCancelsFocusGlide();
  TestRotateCancelsFocusGlide();
  if (g_failures == 0) {
    std::printf("All camera tests passed.\n");
    return 0;
  }
  std::fprintf(stderr, "%d check(s) failed.\n", g_failures);
  return 1;
}
