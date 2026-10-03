#pragma once

#include <glm/glm.hpp>

namespace gfx {

struct Ray {
  glm::vec3 origin{0.0f};
  glm::vec3 direction{0.0f, 0.0f, -1.0f};
};

// Bird's-eye orbit camera: rotates and zooms around a fixed ground-plane
// focus point, and pans that focus point across the ground plane.
class OrbitCamera {
 public:
  void Rotate(float deltaYawRadians, float deltaPitchRadians);
  // Adjusts the zoom *target*; Update() glides the actual distance toward it.
  void Zoom(float deltaDistance);
  // Advances zoom smoothing by dt seconds (frame-rate independent).
  void Update(float dt);
  // Snaps zoom (no glide) to the distance at which a square ground area of
  // the given half-extent fits the view, even in a narrow split-screen pane.
  void FitToExtent(float halfExtent);
  // Starts an eased move that centers on `focusPoint`, tilts to the steepest
  // allowed pitch, and zooms so a square ground area of the given half-extent
  // fits the view. Any manual Pan()/Rotate() cancels the target/pitch glide.
  void FocusOn(const glm::vec3& focusPoint, float halfExtent);
  float TargetDistance() const { return targetDistance_; }

  // Moves `target` along the ground-projected right/forward axes (derived
  // from yaw only, so pitch never tilts the pan plane). Deltas are scaled by
  // the orbit distance so pan speed feels consistent across zoom levels.
  void Pan(float deltaRight, float deltaForward);

  glm::vec3 Position() const;
  glm::mat4 ViewMatrix() const;
  glm::mat4 ProjectionMatrix(float aspectRatio) const;

  // Unprojects a screen-space point (pixels, origin top-left) into a world
  // ray from the camera through that pixel.
  Ray ScreenPointToRay(float screenX, float screenY, float screenWidth, float screenHeight) const;

  // Intersects a ray with the y=0 ground plane. Returns false if the ray is
  // parallel to the plane or points away from it.
  static bool IntersectGroundPlane(const Ray& ray, glm::vec3* outPoint);

  glm::vec3 target{0.0f, 0.0f, 0.0f};

 private:
  float yaw_ = -0.9f;                    // Radians around Y.
  float pitch_ = 0.9599f;                // ~55 degrees above horizon.
  float distance_ = 18.0f;
  float targetDistance_ = 18.0f;
  bool focusing_ = false;
  glm::vec3 focusTarget_{0.0f};
  float focusPitch_ = 0.9599f;
  static constexpr float kMinPitch = 0.1745f;   // ~10 degrees.
  static constexpr float kMaxPitch = 1.4835f;   // ~85 degrees.
  static constexpr float kMinDistance = 3.0f;
  static constexpr float kMaxDistance = 260.0f;
  static constexpr float kZoomDampingRate = 24.0f;  // 1/s; higher = snappier.
};

}  // namespace gfx
