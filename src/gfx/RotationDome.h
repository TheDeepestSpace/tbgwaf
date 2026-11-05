#pragma once

// Math for the camera-rotation dome HUD widget (issue #298): a static
// half-sphere on which the orbit camera's position is a dot. Header-only and
// glm-only so it is unit-testable without SDL/GL/ImGui.

#include <algorithm>
#include <cmath>

#include <glm/glm.hpp>

namespace gfx {

// Fixed elevation (radians) from which the static dome is viewed.
constexpr float kDomeViewTilt = 0.61f;  // ~35 degrees.

// Unit direction from the orbit target to the camera; same convention as
// OrbitCamera::Position() (y up).
inline glm::vec3 DomeDirection(float yaw, float pitch) {
  return glm::vec3(std::cos(pitch) * std::cos(yaw), std::sin(pitch),
                   std::cos(pitch) * std::sin(yaw));
}

// Inverse of DomeDirection for directions on the upper hemisphere.
inline void DomeAngles(const glm::vec3& dir, float* yaw, float* pitch) {
  *pitch = std::asin(std::clamp(dir.y, -1.0f, 1.0f));
  *yaw = std::atan2(dir.z, dir.x);
}

// Orthographic projection of a dome point to dome-view coordinates in
// [-1, 1] (x right, y DOWN as on screen). The dome is viewed from +z,
// elevated by kDomeViewTilt.
inline glm::vec2 DomeProject(const glm::vec3& dir) {
  const float up = dir.y * std::cos(kDomeViewTilt) - dir.z * std::sin(kDomeViewTilt);
  return glm::vec2(dir.x, -up);
}

// > 0 when the point is on the viewer-facing side of the dome.
inline float DomeDepth(const glm::vec3& dir) {
  return dir.y * std::sin(kDomeViewTilt) + dir.z * std::cos(kDomeViewTilt);
}

inline glm::vec2 DomeProjectAngles(float yaw, float pitch) {
  return DomeProject(DomeDirection(yaw, pitch));
}

// Angle change (yaw, pitch) that moves the dot by `deltaView` (dome-view
// units) from its current position, so it tracks the finger/cursor. Newton
// steps on a finite-difference Jacobian; returns zero near singular spots.
inline glm::vec2 DomeDragDelta(float yaw, float pitch, const glm::vec2& deltaView) {
  const glm::vec2 goal = DomeProjectAngles(yaw, pitch) + deltaView;
  float y = yaw, p = pitch;
  constexpr float h = 1e-3f;
  for (int i = 0; i < 4; ++i) {
    const glm::vec2 f = DomeProjectAngles(y, p);
    const glm::vec2 cy = (DomeProjectAngles(y + h, p) - f) / h;
    const glm::vec2 cp = (DomeProjectAngles(y, p + h) - f) / h;
    const float det = cy.x * cp.y - cp.x * cy.y;
    if (std::fabs(det) < 1e-3f) break;
    const glm::vec2 e = goal - f;
    y += (e.x * cp.y - cp.x * e.y) / det;
    p += (cy.x * e.y - e.x * cy.y) / det;
  }
  return glm::vec2(y - yaw, p - pitch);
}

}  // namespace gfx
