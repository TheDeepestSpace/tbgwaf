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

void TestDomeAngleMapping() {
  gfx::OrbitCamera cam;
  // Camera position direction matches the dome direction for its angles.
  const glm::vec3 d = gfx::DomeDirection(cam.Yaw(), cam.Pitch());
  CHECK(glm::length(d * 18.0f - (cam.Position() - cam.target)) < 1e-4f);
  // Angles round-trip through the dome direction.
  for (float yaw : {-3.0f, -0.9f, 0.0f, 1.2f, 3.0f}) {
    for (float pitch : {0.05f, 0.5f, 0.96f, 1.4f}) {
      float y = 0, p = 0;
      gfx::DomeAngles(gfx::DomeDirection(yaw, pitch), &y, &p);
      CHECK(std::fabs(y - yaw) < 1e-4f);
      CHECK(std::fabs(p - pitch) < 1e-4f);
    }
  }
  // Zenith projects to the dome-view center line; dot stays within the unit disc.
  CHECK(glm::length(gfx::DomeProjectAngles(0.3f, 1.5707963f)) < 0.9f);
  CHECK(glm::length(gfx::DomeProjectAngles(0.7f, 0.3f)) <= 1.0001f);
  // Front (+z) is viewer-facing, back (-z) is not.
  CHECK(gfx::DomeDepth(gfx::DomeDirection(1.5707963f, 0.3f)) > 0.0f);
  CHECK(gfx::DomeDepth(gfx::DomeDirection(-1.5707963f, 0.3f)) < 0.0f);
  // Dragging the dot moves it by (about) the drag delta, and rotating the
  // camera by the resulting angles is reflected in Yaw()/Pitch().
  const glm::vec2 before = gfx::DomeProjectAngles(cam.Yaw(), cam.Pitch());
  const glm::vec2 delta(0.004f, -0.003f);  // Per-frame sized drag.
  const glm::vec2 da = gfx::DomeDragDelta(cam.Yaw(), cam.Pitch(), delta);
  const float y0 = cam.Yaw();
  cam.Rotate(da.x, da.y);
  CHECK(std::fabs(cam.Yaw() - (y0 + da.x)) < 1e-6f);
  const glm::vec2 after = gfx::DomeProjectAngles(cam.Yaw(), cam.Pitch());
  CHECK(glm::length(after - before - delta) < 3e-4f);
  // Pitch stays clamped no matter how far the drag pushes.
  cam.Rotate(0.0f, 10.0f);
  CHECK(cam.Pitch() <= 1.4835f + 1e-6f);
  cam.Rotate(0.0f, -10.0f);
  CHECK(cam.Pitch() >= 0.0349f - 1e-6f);
}

}  // namespace

int main() {
  TestDomeAngleMapping();
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
