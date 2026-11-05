// Headless tests for OrbitCamera::FocusOn (double-click focus). glm only; no
// SDL/GL dependency.

#include <cmath>
#include <cstdio>

#include "gfx/Camera.h"
#include "gfx/RotationDome.h"

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

void TestDomeProjection() {
  gfx::OrbitCamera cam;
  const float y = cam.Yaw(), p = cam.Pitch();
  // Camera direction matches OrbitCamera::Position().
  const glm::vec3 d = gfx::DomeDirection(y, p);
  CHECK(glm::length(d * 18.0f - (cam.Position() - cam.target)) < 1e-4f);
  // The point toward the camera projects to the center and faces the viewer.
  CHECK(glm::length(gfx::DomeProject(d, y, p) - glm::vec2(0.0f)) < 1e-5f);
  CHECK(gfx::DomeDepth(d, y, p) > 0.99f);
  CHECK(gfx::DomeDepth(-d, y, p) < -0.99f);
  // Projection matches the real camera's screen axes: world up is screen up.
  CHECK(gfx::DomeProject(glm::vec3(0, 1, 0), y, p).y < 0.0f);
  // Everything on the unit sphere stays within the unit disc.
  CHECK(glm::length(gfx::DomeProject(glm::vec3(0.6f, 0.0f, -0.8f), y, p)) <= 1.0001f);
  // Dragging right yaws the camera the same way as map drag-rotate.
  const glm::vec2 da = gfx::DomeDragDelta(glm::vec2(10.0f, -4.0f), 0.005f);
  CHECK(std::fabs(da.x - (-10.0f * 0.005f)) < 1e-6f);
  CHECK(std::fabs(da.y - (-4.0f * 0.005f)) < 1e-6f);
  // Pitch stays clamped no matter how far the drag pushes.
  cam.Rotate(0.0f, 10.0f);
  CHECK(cam.Pitch() <= 1.4835f + 1e-6f);
  cam.Rotate(0.0f, -10.0f);
  CHECK(cam.Pitch() >= 0.0349f - 1e-6f);
}

}  // namespace

int main() {
  TestDomeProjection();
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
