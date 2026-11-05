#pragma once

// Math for the camera-rotation dome HUD widget (issues #298): a wireframe
// half-sphere showing the world as the orbit camera sees it. Dragging the dome
// rotates the camera. Header-only and glm-only so it is unit-testable without
// SDL/GL/ImGui.

#include <cmath>

#include <glm/glm.hpp>

namespace gfx {

// Unit direction from the orbit target to the camera; same convention as
// OrbitCamera::Position() (y up).
inline glm::vec3 DomeDirection(float yaw, float pitch) {
  return glm::vec3(std::cos(pitch) * std::cos(yaw), std::sin(pitch),
                   std::cos(pitch) * std::sin(yaw));
}

// Orthographic projection of a world-space dome point (unit sphere around the
// orbit target) as seen by the camera, in [-1, 1] (x right, y DOWN as on
// screen).
inline glm::vec2 DomeProject(const glm::vec3& p, float yaw, float pitch) {
  const glm::vec3 toCam = DomeDirection(yaw, pitch);
  const glm::vec3 right(std::sin(yaw), 0.0f, -std::cos(yaw));  // Camera right.
  const glm::vec3 up = glm::cross(toCam, right);                 // Camera up.
  return glm::vec2(glm::dot(p, right), -glm::dot(p, up));
}

// > 0 when the point is on the camera-facing side of the dome.
inline float DomeDepth(const glm::vec3& p, float yaw, float pitch) {
  return glm::dot(p, DomeDirection(yaw, pitch));
}

// Camera (yaw, pitch) change for a dome drag of `dragPixels`, so the dome
// follows the cursor like a grabbed object (same sense as map drag-rotate).
inline glm::vec2 DomeDragDelta(const glm::vec2& dragPixels, float radiansPerPixel) {
  return glm::vec2(-dragPixels.x * radiansPerPixel, dragPixels.y * radiansPerPixel);
}

}  // namespace gfx
